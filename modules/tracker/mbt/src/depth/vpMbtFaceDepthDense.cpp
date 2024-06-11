/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Manage depth dense features for a particular face.
 *
*****************************************************************************/

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/mbt/vpMbtFaceDepthDense.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
#include <pcl/common/point_tests.h>
#endif

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1
#endif

// https://stackoverflow.com/a/40765925
#if !defined(__FMA__) && defined(__AVX2__)
#define __FMA__ 1
#endif

#if defined _WIN32 && defined(_M_ARM64)
#define _ARM64_DISTINCT_NEON_TYPES
#include <Intrin.h>
#include <arm_neon.h>
#define VISP_HAVE_NEON 1
#elif (defined(__ARM_NEON__) || defined (__ARM_NEON)) && defined(__aarch64__)
#include <arm_neon.h>
#define VISP_HAVE_NEON 1
#endif

#define USE_SIMD_CODE 1

#if VISP_HAVE_SSE2 && USE_SIMD_CODE
#define USE_SSE 1
#else
#define USE_SSE 0
#endif

#if VISP_HAVE_NEON && USE_SIMD_CODE
#define USE_NEON 1
#else
#define USE_NEON 0
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x040101 || (VISP_HAVE_OPENCV_VERSION < 0x040000 && VISP_HAVE_OPENCV_VERSION >= 0x030407)) && USE_SIMD_CODE
#define USE_OPENCV_HAL 1
#include <opencv2/core/simd_intrinsics.hpp>
#include <opencv2/core/hal/intrin.hpp>
#endif

#if !USE_OPENCV_HAL && (USE_SSE || USE_NEON)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <cstdint>
#endif

namespace
{
#if USE_SSE
inline void v_load_deinterleave(const uint64_t *ptr, __m128i &a, __m128i &b, __m128i &c)
{
  __m128i t0 = _mm_loadu_si128((const __m128i *)ptr);       // a0, b0
  __m128i t1 = _mm_loadu_si128((const __m128i *)(ptr + 2)); // c0, a1
  __m128i t2 = _mm_loadu_si128((const __m128i *)(ptr + 4)); // b1, c1

  t1 = _mm_shuffle_epi32(t1, 0x4e); // a1, c0

  a = _mm_unpacklo_epi64(t0, t1);
  b = _mm_unpacklo_epi64(_mm_unpackhi_epi64(t0, t0), t2);
  c = _mm_unpackhi_epi64(t1, t2);
}

inline void v_load_deinterleave(const double *ptr, __m128d &a0, __m128d &b0, __m128d &c0)
{
  __m128i a1, b1, c1;
  v_load_deinterleave((const uint64_t *)ptr, a1, b1, c1);
  a0 = _mm_castsi128_pd(a1);
  b0 = _mm_castsi128_pd(b1);
  c0 = _mm_castsi128_pd(c1);
}

inline __m128d v_combine_low(const __m128d &a, const __m128d &b)
{
  __m128i a1 = _mm_castpd_si128(a), b1 = _mm_castpd_si128(b);
  return _mm_castsi128_pd(_mm_unpacklo_epi64(a1, b1));
}

inline __m128d v_combine_high(const __m128d &a, const __m128d &b)
{
  __m128i a1 = _mm_castpd_si128(a), b1 = _mm_castpd_si128(b);
  return _mm_castsi128_pd(_mm_unpackhi_epi64(a1, b1));
}

inline __m128d v_fma(const __m128d &a, const __m128d &b, const __m128d &c)
{
#if __FMA__
  return _mm_fmadd_pd(a, b, c);
#else
  return _mm_add_pd(_mm_mul_pd(a, b), c);
#endif
}
#else
inline void v_load_deinterleave(const double *ptr, float64x2_t &a0, float64x2_t &b0, float64x2_t &c0)
{
  float64x2x3_t v = vld3q_f64(ptr);
  a0 = v.val[0];
  b0 = v.val[1];
  c0 = v.val[2];
}

inline float64x2_t v_combine_low(const float64x2_t &a, const float64x2_t &b)
{
  return vcombine_f64(vget_low_f64(a), vget_low_f64(b));
}

inline float64x2_t v_combine_high(const float64x2_t &a, const float64x2_t &b)
{
  return vcombine_f64(vget_high_f64(a), vget_high_f64(b));
}

inline float64x2_t v_fma(const float64x2_t &a, const float64x2_t &b, const float64x2_t &c)
{
  return vfmaq_f64(c, a, b);
}
#endif
}
#endif // !USE_OPENCV_HAL && (USE_SSE || USE_NEON)

BEGIN_VISP_NAMESPACE
vpMbtFaceDepthDense::vpMbtFaceDepthDense()
  : m_cam(), m_clippingFlag(vpPolygon3D::NO_CLIPPING), m_distFarClip(100), m_distNearClip(0.001), m_hiddenFace(nullptr),
  m_planeObject(), m_polygon(nullptr), m_useScanLine(false),
  m_depthDenseFilteringMethod(DEPTH_OCCUPANCY_RATIO_FILTERING), m_depthDenseFilteringMaxDist(3.0),
  m_depthDenseFilteringMinDist(0.8), m_depthDenseFilteringOccupancyRatio(0.3), m_isTrackedDepthDenseFace(true),
  m_isVisible(false), m_listOfFaceLines(), m_planeCamera(), m_pointCloudFace(), m_polygonLines()
{ }

vpMbtFaceDepthDense::~vpMbtFaceDepthDense()
{
  for (size_t i = 0; i < m_listOfFaceLines.size(); i++) {
    delete m_listOfFaceLines[i];
  }
}

/*!
  Add a line belonging to the \f$ index \f$ the polygon to the list of lines.
  It is defined by its two extremities.

  If the line already exists, the polygon's index is added to the list of
  polygon to which it belongs.

  \param P1 : The first extremity of the line.
  \param P2 : The second extremity of the line.
  \param faces : Pointer to vpMbHiddenFaces.
  \param rand_gen : Random number generator used in vpMbtDistanceLine::buildFrom().
  \param polygon : The index of the polygon to which the line belongs.
  \param name : the optional name of the line
*/
void vpMbtFaceDepthDense::addLine(vpPoint &P1, vpPoint &P2, vpMbHiddenFaces<vpMbtPolygon> *const faces,
                                  vpUniRand &rand_gen, int polygon, std::string name)
{
  // Build a PolygonLine to be able to easily display the lines model
  PolygonLine polygon_line;

  // Add polygon
  polygon_line.m_poly.setNbPoint(2);
  polygon_line.m_poly.addPoint(0, P1);
  polygon_line.m_poly.addPoint(1, P2);

  polygon_line.m_poly.setClipping(m_clippingFlag);
  polygon_line.m_poly.setNearClippingDistance(m_distNearClip);
  polygon_line.m_poly.setFarClippingDistance(m_distFarClip);

  polygon_line.m_p1 = &polygon_line.m_poly.p[0];
  polygon_line.m_p2 = &polygon_line.m_poly.p[1];

  m_polygonLines.push_back(polygon_line);

  // suppress line already in the model
  bool already_here = false;
  vpMbtDistanceLine *l;

  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    l = *it;
    if ((samePoint(*(l->p1), P1) && samePoint(*(l->p2), P2)) || (samePoint(*(l->p1), P2) && samePoint(*(l->p2), P1))) {
      already_here = true;
      l->addPolygon(polygon);
      l->hiddenface = faces;
      l->useScanLine = m_useScanLine;
    }
  }

  if (!already_here) {
    l = new vpMbtDistanceLine;

    l->setCameraParameters(m_cam);
    l->buildFrom(P1, P2, rand_gen);
    l->addPolygon(polygon);
    l->hiddenface = faces;
    l->useScanLine = m_useScanLine;

    l->setIndex((unsigned int)m_listOfFaceLines.size());
    l->setName(name);

    if (m_clippingFlag != vpPolygon3D::NO_CLIPPING)
      l->getPolygon().setClipping(m_clippingFlag);

    if ((m_clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
      l->getPolygon().setNearClippingDistance(m_distNearClip);

    if ((m_clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
      l->getPolygon().setFarClippingDistance(m_distFarClip);

    m_listOfFaceLines.push_back(l);
  }
}

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
bool vpMbtFaceDepthDense::computeDesiredFeatures(const vpHomogeneousMatrix &cMo,
                                                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                                 unsigned int stepX, unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                                                 ,
                                                 vpImage<unsigned char> &debugImage,
                                                 std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                                 ,
                                                 const vpImage<bool> *mask)
{
  unsigned int width = point_cloud->width, height = point_cloud->height;
  m_pointCloudFace.clear();

  if (point_cloud->width == 0 || point_cloud->height == 0)
    return false;

  std::vector<vpImagePoint> roiPts;
  double distanceToFace;
  computeROI(cMo, width, height, roiPts
#if DEBUG_DISPLAY_DEPTH_DENSE
             ,
             roiPts_vec
#endif
             ,
             distanceToFace);

  if (roiPts.size() <= 2) {
#ifndef NDEBUG
    std::cerr << "Error: roiPts.size() <= 2 in computeDesiredFeatures" << std::endl;
#endif
    return false;
  }

  if (((m_depthDenseFilteringMethod & MAX_DISTANCE_FILTERING) && distanceToFace > m_depthDenseFilteringMaxDist) ||
      ((m_depthDenseFilteringMethod & MIN_DISTANCE_FILTERING) && distanceToFace < m_depthDenseFilteringMinDist)) {
    return false;
  }

  vpPolygon polygon_2d(roiPts);
  vpRect bb = polygon_2d.getBoundingBox();

  unsigned int top = (unsigned int)std::max<double>(0.0, bb.getTop());
  unsigned int bottom = (unsigned int)std::min<double>((double)height, std::max<double>(0.0, bb.getBottom()));
  unsigned int left = (unsigned int)std::max<double>(0.0, bb.getLeft());
  unsigned int right = (unsigned int)std::min<double>((double)width, std::max<double>(0.0, bb.getRight()));

  bb.setTop(top);
  bb.setBottom(bottom);
  bb.setLeft(left);
  bb.setRight(right);

  if (bb.getHeight() < 0 || bb.getWidth() < 0) {
    return false;
  }

  m_pointCloudFace.reserve((size_t)(bb.getWidth() * bb.getHeight()));

  int totalTheoreticalPoints = 0, totalPoints = 0;
  for (unsigned int i = top; i < bottom; i += stepY) {
    for (unsigned int j = left; j < right; j += stepX) {
      if ((m_useScanLine ? (i < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
                            j < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
                            m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs()[i][j] == m_polygon->getIndex())
           : polygon_2d.isInside(vpImagePoint(i, j)))) {
        totalTheoreticalPoints++;

        if (vpMeTracker::inRoiMask(mask, i, j) && pcl::isFinite((*point_cloud)(j, i)) && (*point_cloud)(j, i).z > 0) {
          totalPoints++;

          m_pointCloudFace.push_back((*point_cloud)(j, i).x);
          m_pointCloudFace.push_back((*point_cloud)(j, i).y);
          m_pointCloudFace.push_back((*point_cloud)(j, i).z);

#if DEBUG_DISPLAY_DEPTH_DENSE
          debugImage[i][j] = 255;
#endif
        }
      }
    }
  }

  if (totalPoints == 0 || ((m_depthDenseFilteringMethod & DEPTH_OCCUPANCY_RATIO_FILTERING) &&
                           totalPoints / (double)totalTheoreticalPoints < m_depthDenseFilteringOccupancyRatio)) {
    return false;
  }

  return true;
}
#endif

bool vpMbtFaceDepthDense::computeDesiredFeatures(const vpHomogeneousMatrix &cMo, unsigned int width,
                                                 unsigned int height, const std::vector<vpColVector> &point_cloud,
                                                 unsigned int stepX, unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                                                 ,
                                                 vpImage<unsigned char> &debugImage,
                                                 std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                                 ,
                                                 const vpImage<bool> *mask)
{
  m_pointCloudFace.clear();

  if (width == 0 || height == 0)
    return 0;

  std::vector<vpImagePoint> roiPts;
  double distanceToFace;
  computeROI(cMo, width, height, roiPts
#if DEBUG_DISPLAY_DEPTH_DENSE
             ,
             roiPts_vec
#endif
             ,
             distanceToFace);

  if (roiPts.size() <= 2) {
#ifndef NDEBUG
    std::cerr << "Error: roiPts.size() <= 2 in computeDesiredFeatures" << std::endl;
#endif
    return false;
  }

  if (((m_depthDenseFilteringMethod & MAX_DISTANCE_FILTERING) && distanceToFace > m_depthDenseFilteringMaxDist) ||
      ((m_depthDenseFilteringMethod & MIN_DISTANCE_FILTERING) && distanceToFace < m_depthDenseFilteringMinDist)) {
    return false;
  }

  vpPolygon polygon_2d(roiPts);
  vpRect bb = polygon_2d.getBoundingBox();

  unsigned int top = (unsigned int)std::max<double>(0.0, bb.getTop());
  unsigned int bottom = (unsigned int)std::min<double>((double)height, std::max<double>(0.0, bb.getBottom()));
  unsigned int left = (unsigned int)std::max<double>(0.0, bb.getLeft());
  unsigned int right = (unsigned int)std::min<double>((double)width, std::max<double>(0.0, bb.getRight()));

  bb.setTop(top);
  bb.setBottom(bottom);
  bb.setLeft(left);
  bb.setRight(right);

  m_pointCloudFace.reserve((size_t)(bb.getWidth() * bb.getHeight()));

  int totalTheoreticalPoints = 0, totalPoints = 0;
  for (unsigned int i = top; i < bottom; i += stepY) {
    for (unsigned int j = left; j < right; j += stepX) {
      if ((m_useScanLine ? (i < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
                            j < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
                            m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs()[i][j] == m_polygon->getIndex())
           : polygon_2d.isInside(vpImagePoint(i, j)))) {
        totalTheoreticalPoints++;

        if (vpMeTracker::inRoiMask(mask, i, j) && point_cloud[i * width + j][2] > 0) {
          totalPoints++;

          m_pointCloudFace.push_back(point_cloud[i * width + j][0]);
          m_pointCloudFace.push_back(point_cloud[i * width + j][1]);
          m_pointCloudFace.push_back(point_cloud[i * width + j][2]);

#if DEBUG_DISPLAY_DEPTH_DENSE
          debugImage[i][j] = 255;
#endif
        }
      }
    }
  }

  if (totalPoints == 0 || ((m_depthDenseFilteringMethod & DEPTH_OCCUPANCY_RATIO_FILTERING) &&
                           totalPoints / (double)totalTheoreticalPoints < m_depthDenseFilteringOccupancyRatio)) {
    return false;
  }

  return true;
}

bool vpMbtFaceDepthDense::computeDesiredFeatures(const vpHomogeneousMatrix &cMo, unsigned int width,
                                                 unsigned int height, const vpMatrix &point_cloud,
                                                 unsigned int stepX, unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                                                 ,
                                                 vpImage<unsigned char> &debugImage,
                                                 std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                                 ,
                                                 const vpImage<bool> *mask)
{
  m_pointCloudFace.clear();

  if (width == 0 || height == 0)
    return 0;

  std::vector<vpImagePoint> roiPts;
  double distanceToFace;
  computeROI(cMo, width, height, roiPts
#if DEBUG_DISPLAY_DEPTH_DENSE
             ,
             roiPts_vec
#endif
             ,
             distanceToFace);

  if (roiPts.size() <= 2) {
#ifndef NDEBUG
    std::cerr << "Error: roiPts.size() <= 2 in computeDesiredFeatures" << std::endl;
#endif
    return false;
  }

  if (((m_depthDenseFilteringMethod & MAX_DISTANCE_FILTERING) && distanceToFace > m_depthDenseFilteringMaxDist) ||
      ((m_depthDenseFilteringMethod & MIN_DISTANCE_FILTERING) && distanceToFace < m_depthDenseFilteringMinDist)) {
    return false;
  }

  vpPolygon polygon_2d(roiPts);
  vpRect bb = polygon_2d.getBoundingBox();

  unsigned int top = (unsigned int)std::max<double>(0.0, bb.getTop());
  unsigned int bottom = (unsigned int)std::min<double>((double)height, std::max<double>(0.0, bb.getBottom()));
  unsigned int left = (unsigned int)std::max<double>(0.0, bb.getLeft());
  unsigned int right = (unsigned int)std::min<double>((double)width, std::max<double>(0.0, bb.getRight()));

  bb.setTop(top);
  bb.setBottom(bottom);
  bb.setLeft(left);
  bb.setRight(right);

  m_pointCloudFace.reserve((size_t)(bb.getWidth() * bb.getHeight()));

  int totalTheoreticalPoints = 0, totalPoints = 0;
  for (unsigned int i = top; i < bottom; i += stepY) {
    for (unsigned int j = left; j < right; j += stepX) {
      if ((m_useScanLine ? (i < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
                            j < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
                            m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs()[i][j] == m_polygon->getIndex())
           : polygon_2d.isInside(vpImagePoint(i, j)))) {
        totalTheoreticalPoints++;

        if (vpMeTracker::inRoiMask(mask, i, j) && point_cloud[i * width + j][2] > 0) {
          totalPoints++;

          m_pointCloudFace.push_back(point_cloud[i * width + j][0]);
          m_pointCloudFace.push_back(point_cloud[i * width + j][1]);
          m_pointCloudFace.push_back(point_cloud[i * width + j][2]);

#if DEBUG_DISPLAY_DEPTH_DENSE
          debugImage[i][j] = 255;
#endif
        }
      }
    }
  }

  if (totalPoints == 0 || ((m_depthDenseFilteringMethod & DEPTH_OCCUPANCY_RATIO_FILTERING) &&
                           totalPoints / (double)totalTheoreticalPoints < m_depthDenseFilteringOccupancyRatio)) {
    return false;
  }

  return true;
}

void vpMbtFaceDepthDense::computeVisibility() { m_isVisible = m_polygon->isVisible(); }

void vpMbtFaceDepthDense::computeVisibilityDisplay()
{
  // Compute lines visibility, only for display
  vpMbtDistanceLine *line;
  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    line = *it;
    bool isvisible = false;

    for (std::list<int>::const_iterator itindex = line->Lindex_polygon.begin(); itindex != line->Lindex_polygon.end();
         ++itindex) {
      int index = *itindex;
      if (index == -1) {
        isvisible = true;
      }
      else {
        if (line->hiddenface->isVisible((unsigned int)index)) {
          isvisible = true;
        }
      }
    }

    // Si la ligne n'appartient a aucune face elle est tout le temps visible
    if (line->Lindex_polygon.empty())
      isvisible = true; // Not sure that this can occur

    if (isvisible) {
      line->setVisible(true);
    }
    else {
      line->setVisible(false);
    }
  }
}

void vpMbtFaceDepthDense::computeInteractionMatrixAndResidu(const vpHomogeneousMatrix &cMo, vpMatrix &L,
                                                            vpColVector &error)
{
  if (m_pointCloudFace.empty()) {
    L.resize(0, 0);
    error.resize(0);
    return;
  }

  L.resize(getNbFeatures(), 6, false, false);
  error.resize(getNbFeatures(), false);

  // Transform the plane equation for the current pose
  m_planeCamera = m_planeObject;
  m_planeCamera.changeFrame(cMo);

  double nx = m_planeCamera.getA();
  double ny = m_planeCamera.getB();
  double nz = m_planeCamera.getC();
  double D = m_planeCamera.getD();

#if defined(VISP_HAVE_SIMDLIB)
  bool useSIMD = vpCPUFeatures::checkSSE2() || vpCPUFeatures::checkNeon();
#else
  bool useSIMD = vpCPUFeatures::checkSSE2();
#endif
#if USE_OPENCV_HAL
  useSIMD = true;
#endif
#if !USE_SSE && !USE_NEON && !USE_OPENCV_HAL
  useSIMD = false;
#endif

  if (useSIMD) {
#if USE_SSE || USE_NEON || USE_OPENCV_HAL
    size_t cpt = 0;
    if (getNbFeatures() >= 2) {
      double *ptr_point_cloud = &m_pointCloudFace[0];
      double *ptr_L = L.data;
      double *ptr_error = error.data;

#if USE_OPENCV_HAL
      const cv::v_float64x2 vnx = cv::v_setall_f64(nx);
      const cv::v_float64x2 vny = cv::v_setall_f64(ny);
      const cv::v_float64x2 vnz = cv::v_setall_f64(nz);
      const cv::v_float64x2 vd = cv::v_setall_f64(D);
#elif USE_SSE
      const __m128d vnx = _mm_set1_pd(nx);
      const __m128d vny = _mm_set1_pd(ny);
      const __m128d vnz = _mm_set1_pd(nz);
      const __m128d vd = _mm_set1_pd(D);
#else
      const float64x2_t vnx = vdupq_n_f64(nx);
      const float64x2_t vny = vdupq_n_f64(ny);
      const float64x2_t vnz = vdupq_n_f64(nz);
      const float64x2_t vd = vdupq_n_f64(D);
#endif

      for (; cpt <= m_pointCloudFace.size() - 6; cpt += 6, ptr_point_cloud += 6) {
#if USE_OPENCV_HAL
        cv::v_float64x2 vx, vy, vz;
        cv::v_load_deinterleave(ptr_point_cloud, vx, vy, vz);

#if (VISP_HAVE_OPENCV_VERSION >= 0x040900)
        cv::v_float64x2 va1 = cv::v_sub(cv::v_mul(vnz, vy), cv::v_mul(vny, vz)); // vnz*vy - vny*vz
        cv::v_float64x2 va2 = cv::v_sub(cv::v_mul(vnx, vz), cv::v_mul(vnz, vx)); // vnx*vz - vnz*vx
        cv::v_float64x2 va3 = cv::v_sub(cv::v_mul(vny, vx), cv::v_mul(vnx, vy)); // vny*vx - vnx*vy
#else
        cv::v_float64x2 va1 = vnz*vy - vny*vz;
        cv::v_float64x2 va2 = vnx*vz - vnz*vx;
        cv::v_float64x2 va3 = vny*vx - vnx*vy;
#endif

        cv::v_float64x2 vnxy = cv::v_combine_low(vnx, vny);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = cv::v_combine_low(vnz, va1);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = cv::v_combine_low(va2, va3);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;

        vnxy = cv::v_combine_high(vnx, vny);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = cv::v_combine_high(vnz, va1);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = cv::v_combine_high(va2, va3);
        cv::v_store(ptr_L, vnxy);
        ptr_L += 2;

#if (VISP_HAVE_OPENCV_VERSION >= 0x040900)
        cv::v_float64x2 verr = cv::v_add(vd, cv::v_muladd(vnx, vx, cv::v_muladd(vny, vy, cv::v_mul(vnz, vz))));
#else
        cv::v_float64x2 verr = vd + cv::v_muladd(vnx, vx, cv::v_muladd(vny, vy, vnz*vz));
#endif

        cv::v_store(ptr_error, verr);
        ptr_error += 2;
#elif USE_SSE
        __m128d vx, vy, vz;
        v_load_deinterleave(ptr_point_cloud, vx, vy, vz);

        __m128d va1 = _mm_sub_pd(_mm_mul_pd(vnz, vy), _mm_mul_pd(vny, vz));
        __m128d va2 = _mm_sub_pd(_mm_mul_pd(vnx, vz), _mm_mul_pd(vnz, vx));
        __m128d va3 = _mm_sub_pd(_mm_mul_pd(vny, vx), _mm_mul_pd(vnx, vy));

        __m128d vnxy = v_combine_low(vnx, vny);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_low(vnz, va1);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_low(va2, va3);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;

        vnxy = v_combine_high(vnx, vny);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_high(vnz, va1);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_high(va2, va3);
        _mm_storeu_pd(ptr_L, vnxy);
        ptr_L += 2;

        const __m128d verror = _mm_add_pd(vd, v_fma(vnx, vx, v_fma(vny, vy, _mm_mul_pd(vnz, vz))));
        _mm_storeu_pd(ptr_error, verror);
        ptr_error += 2;
#else
        float64x2_t vx, vy, vz;
        v_load_deinterleave(ptr_point_cloud, vx, vy, vz);

        float64x2_t va1 = vsubq_f64(vmulq_f64(vnz, vy), vmulq_f64(vny, vz));
        float64x2_t va2 = vsubq_f64(vmulq_f64(vnx, vz), vmulq_f64(vnz, vx));
        float64x2_t va3 = vsubq_f64(vmulq_f64(vny, vx), vmulq_f64(vnx, vy));

        float64x2_t vnxy = v_combine_low(vnx, vny);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_low(vnz, va1);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_low(va2, va3);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;

        vnxy = v_combine_high(vnx, vny);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_high(vnz, va1);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;
        vnxy = v_combine_high(va2, va3);
        vst1q_f64(ptr_L, vnxy);
        ptr_L += 2;

        const float64x2_t verror = vaddq_f64(vd, v_fma(vnx, vx, v_fma(vny, vy, vmulq_f64(vnz, vz))));
        vst1q_f64(ptr_error, verror);
        ptr_error += 2;
#endif
      }
    }

    for (; cpt < m_pointCloudFace.size(); cpt += 3) {
      double x = m_pointCloudFace[cpt];
      double y = m_pointCloudFace[cpt + 1];
      double z = m_pointCloudFace[cpt + 2];

      double _a1 = (nz * y) - (ny * z);
      double _a2 = (nx * z) - (nz * x);
      double _a3 = (ny * x) - (nx * y);

      // L
      L[(unsigned int)(cpt / 3)][0] = nx;
      L[(unsigned int)(cpt / 3)][1] = ny;
      L[(unsigned int)(cpt / 3)][2] = nz;
      L[(unsigned int)(cpt / 3)][3] = _a1;
      L[(unsigned int)(cpt / 3)][4] = _a2;
      L[(unsigned int)(cpt / 3)][5] = _a3;

      vpColVector normal(3);
      normal[0] = nx;
      normal[1] = ny;
      normal[2] = nz;

      vpColVector pt(3);
      pt[0] = x;
      pt[1] = y;
      pt[2] = z;

      // Error
      error[(unsigned int)(cpt / 3)] = D + (normal.t() * pt);
    }
#endif
  }
  else {
    vpColVector normal(3);
    normal[0] = nx;
    normal[1] = ny;
    normal[2] = nz;
    vpColVector pt(3);

    unsigned int idx = 0;
    for (size_t i = 0; i < m_pointCloudFace.size(); i += 3, idx++) {
      double x = m_pointCloudFace[i];
      double y = m_pointCloudFace[i + 1];
      double z = m_pointCloudFace[i + 2];

      double _a1 = (nz * y) - (ny * z);
      double _a2 = (nx * z) - (nz * x);
      double _a3 = (ny * x) - (nx * y);

      // L
      L[idx][0] = nx;
      L[idx][1] = ny;
      L[idx][2] = nz;
      L[idx][3] = _a1;
      L[idx][4] = _a2;
      L[idx][5] = _a3;

      pt[0] = x;
      pt[1] = y;
      pt[2] = z;
      // Error
      error[idx] = D + (normal.t() * pt);
    }
  }
}

void vpMbtFaceDepthDense::computeROI(const vpHomogeneousMatrix &cMo, unsigned int width, unsigned int height,
                                     std::vector<vpImagePoint> &roiPts
#if DEBUG_DISPLAY_DEPTH_DENSE
                                     ,
                                     std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                     ,
                                     double &distanceToFace)
{
  if (m_useScanLine || m_clippingFlag > 2)
    m_cam.computeFov(width, height);

  if (m_useScanLine) {
    for (std::vector<PolygonLine>::iterator it = m_polygonLines.begin(); it != m_polygonLines.end(); ++it) {
      it->m_p1->changeFrame(cMo);
      it->m_p2->changeFrame(cMo);

      vpImagePoint ip1, ip2;

      it->m_poly.changeFrame(cMo);
      it->m_poly.computePolygonClipped(m_cam);

      if (it->m_poly.polyClipped.size() == 2 &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::NEAR_CLIPPING) == 0) &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::FAR_CLIPPING) == 0) &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::DOWN_CLIPPING) == 0) &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::UP_CLIPPING) == 0) &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::LEFT_CLIPPING) == 0) &&
          ((it->m_poly.polyClipped[1].second & it->m_poly.polyClipped[0].second & vpPolygon3D::RIGHT_CLIPPING) == 0)) {

        std::vector<std::pair<vpPoint, vpPoint> > linesLst;
        m_hiddenFace->computeScanLineQuery(it->m_poly.polyClipped[0].first, it->m_poly.polyClipped[1].first, linesLst,
                                           true);

        vpPoint faceCentroid;

        for (unsigned int i = 0; i < linesLst.size(); i++) {
          linesLst[i].first.project();
          linesLst[i].second.project();

          vpMeterPixelConversion::convertPoint(m_cam, linesLst[i].first.get_x(), linesLst[i].first.get_y(), ip1);
          vpMeterPixelConversion::convertPoint(m_cam, linesLst[i].second.get_x(), linesLst[i].second.get_y(), ip2);

          it->m_imPt1 = ip1;
          it->m_imPt2 = ip2;

          roiPts.push_back(ip1);
          roiPts.push_back(ip2);

          faceCentroid.set_X(faceCentroid.get_X() + linesLst[i].first.get_X() + linesLst[i].second.get_X());
          faceCentroid.set_Y(faceCentroid.get_Y() + linesLst[i].first.get_Y() + linesLst[i].second.get_Y());
          faceCentroid.set_Z(faceCentroid.get_Z() + linesLst[i].first.get_Z() + linesLst[i].second.get_Z());

#if DEBUG_DISPLAY_DEPTH_DENSE
          std::vector<vpImagePoint> roiPts_;
          roiPts_.push_back(ip1);
          roiPts_.push_back(ip2);
          roiPts_vec.push_back(roiPts_);
#endif
        }

        if (linesLst.empty()) {
          distanceToFace = std::numeric_limits<double>::max();
        }
        else {
          faceCentroid.set_X(faceCentroid.get_X() / (2 * linesLst.size()));
          faceCentroid.set_Y(faceCentroid.get_Y() / (2 * linesLst.size()));
          faceCentroid.set_Z(faceCentroid.get_Z() / (2 * linesLst.size()));

          distanceToFace =
            sqrt(faceCentroid.get_X() * faceCentroid.get_X() + faceCentroid.get_Y() * faceCentroid.get_Y() +
                 faceCentroid.get_Z() * faceCentroid.get_Z());
        }
      }
    }
  }
  else {
 // Get polygon clipped
    m_polygon->getRoiClipped(m_cam, roiPts, cMo);

    // Get 3D polygon clipped
    std::vector<vpPoint> polygonsClipped;
    m_polygon->getPolygonClipped(polygonsClipped);

    if (polygonsClipped.empty()) {
      distanceToFace = std::numeric_limits<double>::max();
    }
    else {
      vpPoint faceCentroid;

      for (size_t i = 0; i < polygonsClipped.size(); i++) {
        faceCentroid.set_X(faceCentroid.get_X() + polygonsClipped[i].get_X());
        faceCentroid.set_Y(faceCentroid.get_Y() + polygonsClipped[i].get_Y());
        faceCentroid.set_Z(faceCentroid.get_Z() + polygonsClipped[i].get_Z());
      }

      faceCentroid.set_X(faceCentroid.get_X() / polygonsClipped.size());
      faceCentroid.set_Y(faceCentroid.get_Y() / polygonsClipped.size());
      faceCentroid.set_Z(faceCentroid.get_Z() / polygonsClipped.size());

      distanceToFace = sqrt(faceCentroid.get_X() * faceCentroid.get_X() + faceCentroid.get_Y() * faceCentroid.get_Y() +
                            faceCentroid.get_Z() * faceCentroid.get_Z());
    }

#if DEBUG_DISPLAY_DEPTH_DENSE
    roiPts_vec.push_back(roiPts);
#endif
  }
}

void vpMbtFaceDepthDense::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &cam, const vpColor &col, unsigned int thickness,
                                  bool displayFullModel)
{
  std::vector<std::vector<double> > models =
    getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);
    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

void vpMbtFaceDepthDense::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &cam, const vpColor &col, unsigned int thickness,
                                  bool displayFullModel)
{
  std::vector<std::vector<double> > models =
    getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);
    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

void vpMbtFaceDepthDense::displayFeature(const vpImage<unsigned char> & /*I*/, const vpHomogeneousMatrix & /*cMo*/,
                                         const vpCameraParameters & /*cam*/, const double /*scale*/,
                                         const unsigned int /*thickness*/)
{ }

void vpMbtFaceDepthDense::displayFeature(const vpImage<vpRGBa> & /*I*/, const vpHomogeneousMatrix & /*cMo*/,
                                         const vpCameraParameters & /*cam*/, const double /*scale*/,
                                         const unsigned int /*thickness*/)
{ }

/*!
  Return a list of line parameters to display the primitive at a given pose and camera parameters.
  - Parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`

  \param width : Image width.
  \param height : Image height.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbtFaceDepthDense::getModelForDisplay(unsigned int width, unsigned int height,
                                                                          const vpHomogeneousMatrix &cMo,
                                                                          const vpCameraParameters &cam,
                                                                          bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  if ((m_polygon->isVisible() && m_isTrackedDepthDenseFace) || displayFullModel) {
    computeVisibilityDisplay();

    for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
         ++it) {
      vpMbtDistanceLine *line = *it;
      std::vector<std::vector<double> > lineModels =
        line->getModelForDisplay(width, height, cMo, cam, displayFullModel);
      models.insert(models.end(), lineModels.begin(), lineModels.end());
    }
  }

  return models;
}

/*!
  Check if two vpPoints are similar.

  To be similar : \f$ (X_1 - X_2)^2 + (Y_1 - Y_2)^2 + (Z_1 - Z_2)^2 < epsilon
  \f$.

  \param P1 : The first point to compare
  \param P2 : The second point to compare
*/
bool vpMbtFaceDepthDense::samePoint(const vpPoint &P1, const vpPoint &P2) const
{
  double dx = fabs(P1.get_oX() - P2.get_oX());
  double dy = fabs(P1.get_oY() - P2.get_oY());
  double dz = fabs(P1.get_oZ() - P2.get_oZ());

  if (dx <= std::numeric_limits<double>::epsilon() && dy <= std::numeric_limits<double>::epsilon() &&
      dz <= std::numeric_limits<double>::epsilon())
    return true;
  else
    return false;
}

void vpMbtFaceDepthDense::setCameraParameters(const vpCameraParameters &camera)
{
  m_cam = camera;

  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    (*it)->setCameraParameters(camera);
  }
}

void vpMbtFaceDepthDense::setScanLineVisibilityTest(bool v)
{
  m_useScanLine = v;

  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    (*it)->useScanLine = v;
  }
}
END_VISP_NAMESPACE
