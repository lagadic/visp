/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Manage depth normal features for a particular face.
 *
 *****************************************************************************/

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>
#include <visp3/mbt/vpMbtTukeyEstimator.h>

#ifdef VISP_HAVE_PCL
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#endif

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1
#endif

#define USE_SSE_CODE 1
#if VISP_HAVE_SSE2 && USE_SSE_CODE
#define USE_SSE 1
#else
#define USE_SSE 0
#endif

vpMbtFaceDepthNormal::vpMbtFaceDepthNormal()
  : m_cam(), m_clippingFlag(vpPolygon3D::NO_CLIPPING), m_distFarClip(100), m_distNearClip(0.001), m_hiddenFace(NULL),
    m_planeObject(), m_polygon(NULL), m_useScanLine(false), m_faceActivated(false),
    m_faceCentroidMethod(GEOMETRIC_CENTROID), m_faceDesiredCentroid(), m_faceDesiredNormal(),
    m_featureEstimationMethod(ROBUST_FEATURE_ESTIMATION), m_isTrackedDepthNormalFace(true), m_isVisible(false),
    m_listOfFaceLines(), m_planeCamera(),
    m_pclPlaneEstimationMethod(2), // SAC_MSAC, see pcl/sample_consensus/method_types.h
    m_pclPlaneEstimationRansacMaxIter(200), m_pclPlaneEstimationRansacThreshold(0.001), m_polygonLines()
{
}

vpMbtFaceDepthNormal::~vpMbtFaceDepthNormal()
{
  for (size_t i = 0; i < m_listOfFaceLines.size(); i++) {
    delete m_listOfFaceLines[i];
  }
}

/*!
  Add a line belonging to the \f$ index \f$ the polygon to the list of lines.
  It is defined by its two extremities.

  If the line already exists, the ploygone's index is added to the list of
  polygon to which it belongs.

  \param P1 : The first extremity of the line.
  \param P2 : The second extremity of the line.
  \param faces : Pointer to vpMbHiddenFaces.
  \param polygon : The index of the polygon to which the line belongs.
  \param name : the optional name of the line
*/
void vpMbtFaceDepthNormal::addLine(vpPoint &P1, vpPoint &P2, vpMbHiddenFaces<vpMbtPolygon> *const faces, int polygon,
                                   std::string name)
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
    l->buildFrom(P1, P2);
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

#ifdef VISP_HAVE_PCL
bool vpMbtFaceDepthNormal::computeDesiredFeatures(const vpHomogeneousMatrix &cMo, const unsigned int width,
                                                  const unsigned int height,
                                                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                                  vpColVector &desired_features, const unsigned int stepX,
                                                  const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                                                  ,
                                                  vpImage<unsigned char> &debugImage,
                                                  std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                                  , const vpImage<bool> *mask
)
{
  m_faceActivated = false;

  if (width == 0 || height == 0)
    return false;

  std::vector<vpImagePoint> roiPts;
  vpColVector desired_normal(3);

  computeROI(cMo, width, height, roiPts
#if DEBUG_DISPLAY_DEPTH_NORMAL
             ,
             roiPts_vec
#endif
  );

  if (roiPts.size() <= 2) {
#ifndef NDEBUG
    std::cerr << "Error: roiPts.size() <= 2 in computeDesiredFeatures" << std::endl;
#endif
    return false;
  }

  vpPolygon polygon_2d(roiPts);
  vpRect bb = polygon_2d.getBoundingBox();

  unsigned int top = (unsigned int)std::max(0.0, bb.getTop());
  unsigned int bottom = (unsigned int)std::min((double)height, std::max(0.0, bb.getBottom()));
  unsigned int left = (unsigned int)std::max(0.0, bb.getLeft());
  unsigned int right = (unsigned int)std::min((double)width, std::max(0.0, bb.getRight()));

  bb.setTop(top);
  bb.setBottom(bottom);
  bb.setLeft(left);
  bb.setRight(right);

  // Keep only 3D points inside the projected polygon face
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_face(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<double> point_cloud_face_vec, point_cloud_face_custom;

  if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
    point_cloud_face_custom.reserve((size_t)(3 * bb.getWidth() * bb.getHeight()));
    point_cloud_face_vec.reserve((size_t)(3 * bb.getWidth() * bb.getHeight()));
  } else if (m_featureEstimationMethod == ROBUST_SVD_PLANE_ESTIMATION) {
    point_cloud_face_vec.reserve((size_t)(3 * bb.getWidth() * bb.getHeight()));
  } else if (m_featureEstimationMethod == PCL_PLANE_ESTIMATION) {
    point_cloud_face->reserve((size_t)(bb.getWidth() * bb.getHeight()));
  }

  bool checkSSE2 = vpCPUFeatures::checkSSE2();
#if !USE_SSE
  checkSSE2 = false;
#else
  bool push = false;
  double prev_x, prev_y, prev_z;
#endif

  double x = 0.0, y = 0.0;
  for (unsigned int i = top; i < bottom; i += stepY) {
    for (unsigned int j = left; j < right; j += stepX) {
      if (vpMeTracker::inMask(mask, i, j) && pcl::isFinite((*point_cloud)(j, i)) && (*point_cloud)(j, i).z > 0 &&
          (m_useScanLine ? (i < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
                            j < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
                            m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs()[i][j] == m_polygon->getIndex())
                         : polygon_2d.isInside(vpImagePoint(i, j)))) {

        if (m_featureEstimationMethod == PCL_PLANE_ESTIMATION) {
          point_cloud_face->push_back((*point_cloud)(j, i));
        } else if (m_featureEstimationMethod == ROBUST_SVD_PLANE_ESTIMATION ||
                   m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
          point_cloud_face_vec.push_back((*point_cloud)(j, i).x);
          point_cloud_face_vec.push_back((*point_cloud)(j, i).y);
          point_cloud_face_vec.push_back((*point_cloud)(j, i).z);

          if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
            // Add point for custom method for plane equation estimation
            vpPixelMeterConversion::convertPoint(m_cam, j, i, x, y);

            if (checkSSE2) {
#if USE_SSE
              if (!push) {
                push = true;
                prev_x = x;
                prev_y = y;
                prev_z = (*point_cloud)(j, i).z;
              } else {
                push = false;
                point_cloud_face_custom.push_back(prev_x);
                point_cloud_face_custom.push_back(x);

                point_cloud_face_custom.push_back(prev_y);
                point_cloud_face_custom.push_back(y);

                point_cloud_face_custom.push_back(prev_z);
                point_cloud_face_custom.push_back((*point_cloud)(j, i).z);
              }
#endif
            } else {
              point_cloud_face_custom.push_back(x);
              point_cloud_face_custom.push_back(y);
              point_cloud_face_custom.push_back((*point_cloud)(j, i).z);
            }
          }
        }

#if DEBUG_DISPLAY_DEPTH_NORMAL
        debugImage[i][j] = 255;
#endif
      }
    }
  }

#if USE_SSE
  if (checkSSE2 && push) {
    point_cloud_face_custom.push_back(prev_x);
    point_cloud_face_custom.push_back(prev_y);
    point_cloud_face_custom.push_back(prev_z);
  }
#endif

  if (point_cloud_face->empty() && point_cloud_face_custom.empty() && point_cloud_face_vec.empty()) {
    return false;
  }

  // Face centroid computed by the different methods
  vpColVector centroid_point(3);

  if (m_featureEstimationMethod == PCL_PLANE_ESTIMATION) {
    if (!computeDesiredFeaturesPCL(point_cloud_face, desired_features, desired_normal, centroid_point)) {
      return false;
    }
  } else if (m_featureEstimationMethod == ROBUST_SVD_PLANE_ESTIMATION) {
    computeDesiredFeaturesSVD(point_cloud_face_vec, cMo, desired_features, desired_normal, centroid_point);
  } else if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
    computeDesiredFeaturesRobustFeatures(point_cloud_face_custom, point_cloud_face_vec, cMo, desired_features,
                                         desired_normal, centroid_point);
  } else {
    throw vpException(vpException::badValue, "Unknown feature estimation method!");
  }

  computeDesiredNormalAndCentroid(cMo, desired_normal, centroid_point);

  m_faceActivated = true;

  return true;
}
#endif

bool vpMbtFaceDepthNormal::computeDesiredFeatures(const vpHomogeneousMatrix &cMo, const unsigned int width,
                                                  const unsigned int height,
                                                  const std::vector<vpColVector> &point_cloud,
                                                  vpColVector &desired_features, const unsigned int stepX,
                                                  const unsigned int stepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                                                  ,
                                                  vpImage<unsigned char> &debugImage,
                                                  std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
                                                  , const vpImage<bool> *mask
)
{
  m_faceActivated = false;

  if (width == 0 || height == 0)
    return false;

  std::vector<vpImagePoint> roiPts;
  vpColVector desired_normal(3);

  computeROI(cMo, width, height, roiPts
#if DEBUG_DISPLAY_DEPTH_NORMAL
             ,
             roiPts_vec
#endif
  );

  if (roiPts.size() <= 2) {
#ifndef NDEBUG
    std::cerr << "Error: roiPts.size() <= 2 in computeDesiredFeatures" << std::endl;
#endif
    return false;
  }

  vpPolygon polygon_2d(roiPts);
  vpRect bb = polygon_2d.getBoundingBox();

  unsigned int top = (unsigned int)std::max(0.0, bb.getTop());
  unsigned int bottom = (unsigned int)std::min((double)height, std::max(0.0, bb.getBottom()));
  unsigned int left = (unsigned int)std::max(0.0, bb.getLeft());
  unsigned int right = (unsigned int)std::min((double)width, std::max(0.0, bb.getRight()));

  bb.setTop(top);
  bb.setBottom(bottom);
  bb.setLeft(left);
  bb.setRight(right);

  // Keep only 3D points inside the projected polygon face
  std::vector<double> point_cloud_face, point_cloud_face_custom;

  point_cloud_face.reserve((size_t)(3 * bb.getWidth() * bb.getHeight()));
  if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
    point_cloud_face_custom.reserve((size_t)(3 * bb.getWidth() * bb.getHeight()));
  }

  bool checkSSE2 = vpCPUFeatures::checkSSE2();
#if !USE_SSE
  checkSSE2 = false;
#else
  bool push = false;
  double prev_x, prev_y, prev_z;
#endif

  double x = 0.0, y = 0.0;
  for (unsigned int i = top; i < bottom; i += stepY) {
    for (unsigned int j = left; j < right; j += stepX) {
      if (vpMeTracker::inMask(mask, i, j) && point_cloud[i * width + j][2] > 0 &&
          (m_useScanLine ? (i < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
                            j < m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
                            m_hiddenFace->getMbScanLineRenderer().getPrimitiveIDs()[i][j] == m_polygon->getIndex())
                         : polygon_2d.isInside(vpImagePoint(i, j)))) {
        // Add point
        point_cloud_face.push_back(point_cloud[i * width + j][0]);
        point_cloud_face.push_back(point_cloud[i * width + j][1]);
        point_cloud_face.push_back(point_cloud[i * width + j][2]);

        if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
          // Add point for custom method for plane equation estimation
          vpPixelMeterConversion::convertPoint(m_cam, j, i, x, y);

          if (checkSSE2) {
#if USE_SSE
            if (!push) {
              push = true;
              prev_x = x;
              prev_y = y;
              prev_z = point_cloud[i * width + j][2];
            } else {
              push = false;
              point_cloud_face_custom.push_back(prev_x);
              point_cloud_face_custom.push_back(x);

              point_cloud_face_custom.push_back(prev_y);
              point_cloud_face_custom.push_back(y);

              point_cloud_face_custom.push_back(prev_z);
              point_cloud_face_custom.push_back(point_cloud[i * width + j][2]);
            }
#endif
          } else {
            point_cloud_face_custom.push_back(x);
            point_cloud_face_custom.push_back(y);
            point_cloud_face_custom.push_back(point_cloud[i * width + j][2]);
          }
        }

#if DEBUG_DISPLAY_DEPTH_NORMAL
        debugImage[i][j] = 255;
#endif
      }
    }
  }

#if USE_SSE
  if (checkSSE2 && push) {
    point_cloud_face_custom.push_back(prev_x);
    point_cloud_face_custom.push_back(prev_y);
    point_cloud_face_custom.push_back(prev_z);
  }
#endif

  if (point_cloud_face.empty() && point_cloud_face_custom.empty()) {
    return false;
  }

  // Face centroid computed by the different methods
  vpColVector centroid_point(3);

#ifdef VISP_HAVE_PCL
  if (m_featureEstimationMethod == PCL_PLANE_ESTIMATION) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_face_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud_face_pcl->reserve(point_cloud_face.size() / 3);

    for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
      point_cloud_face_pcl->push_back(
          pcl::PointXYZ(point_cloud_face[3 * i], point_cloud_face[3 * i + 1], point_cloud_face[3 * i + 2]));
    }

    computeDesiredFeaturesPCL(point_cloud_face_pcl, desired_features, desired_normal, centroid_point);
  } else
#endif
      if (m_featureEstimationMethod == ROBUST_SVD_PLANE_ESTIMATION) {
    computeDesiredFeaturesSVD(point_cloud_face, cMo, desired_features, desired_normal, centroid_point);
  } else if (m_featureEstimationMethod == ROBUST_FEATURE_ESTIMATION) {
    computeDesiredFeaturesRobustFeatures(point_cloud_face_custom, point_cloud_face, cMo, desired_features,
                                         desired_normal, centroid_point);
  } else {
    throw vpException(vpException::badValue, "Unknown feature estimation method!");
  }

  computeDesiredNormalAndCentroid(cMo, desired_normal, centroid_point);

  m_faceActivated = true;

  return true;
}

#ifdef VISP_HAVE_PCL
bool vpMbtFaceDepthNormal::computeDesiredFeaturesPCL(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud_face,
                                                     vpColVector &desired_features, vpColVector &desired_normal,
                                                     vpColVector &centroid_point)
{
  try {
    // Compute plane equation for this subset of point cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(m_pclPlaneEstimationMethod);
    seg.setDistanceThreshold(m_pclPlaneEstimationRansacThreshold);
    seg.setMaxIterations(m_pclPlaneEstimationRansacMaxIter);

    seg.setInputCloud(point_cloud_face);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_face_extracted(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Extract the inliers
    extract.setInputCloud(point_cloud_face);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*point_cloud_face_extracted);

    pcl::PointXYZ centroid_point_pcl;
    if (pcl::computeCentroid(*point_cloud_face_extracted, centroid_point_pcl)) {
      pcl::PointXYZ face_normal;
      computeNormalVisibility(coefficients->values[0], coefficients->values[1], coefficients->values[2],
                              centroid_point_pcl, face_normal);

      desired_features.resize(3, false);
      desired_features[0] = -coefficients->values[0] / coefficients->values[3];
      desired_features[1] = -coefficients->values[1] / coefficients->values[3];
      desired_features[2] = -coefficients->values[2] / coefficients->values[3];

      desired_normal[0] = face_normal.x;
      desired_normal[1] = face_normal.y;
      desired_normal[2] = face_normal.z;

      centroid_point[0] = centroid_point_pcl.x;
      centroid_point[1] = centroid_point_pcl.y;
      centroid_point[2] = centroid_point_pcl.z;
    } else {
      std::cerr << "Cannot compute centroid!" << std::endl;
      return false;
    }
  } catch (const pcl::PCLException &e) {
    std::cerr << "Catch a PCL exception: " << e.what() << std::endl;
    throw;
  }

  return true;
}
#endif

void vpMbtFaceDepthNormal::computeDesiredFeaturesRobustFeatures(const std::vector<double> &point_cloud_face_custom,
                                                                const std::vector<double> &point_cloud_face,
                                                                const vpHomogeneousMatrix &cMo,
                                                                vpColVector &desired_features,
                                                                vpColVector &desired_normal,
                                                                vpColVector &centroid_point)
{
  std::vector<double> weights;
  double den = 0.0;
  estimateFeatures(point_cloud_face_custom, cMo, desired_features, weights);

  // Compute face centroid
  for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
    centroid_point[0] += weights[i] * point_cloud_face[3 * i];
    centroid_point[1] += weights[i] * point_cloud_face[3 * i + 1];
    centroid_point[2] += weights[i] * point_cloud_face[3 * i + 2];

    den += weights[i];
  }

  centroid_point[0] /= den;
  centroid_point[1] /= den;
  centroid_point[2] /= den;

  computeNormalVisibility(-desired_features[0], -desired_features[1], -desired_features[2], centroid_point,
                          desired_normal);
}

void vpMbtFaceDepthNormal::computeDesiredFeaturesSVD(const std::vector<double> &point_cloud_face,
                                                     const vpHomogeneousMatrix &cMo, vpColVector &desired_features,
                                                     vpColVector &desired_normal, vpColVector &centroid_point)
{
  vpColVector plane_equation_SVD;
  estimatePlaneEquationSVD(point_cloud_face, cMo, plane_equation_SVD, centroid_point);

  desired_features.resize(3, false);
  desired_features[0] = -plane_equation_SVD[0] / plane_equation_SVD[3];
  desired_features[1] = -plane_equation_SVD[1] / plane_equation_SVD[3];
  desired_features[2] = -plane_equation_SVD[2] / plane_equation_SVD[3];

  computeNormalVisibility(-desired_features[0], -desired_features[1], -desired_features[2], centroid_point,
                          desired_normal);
}

void vpMbtFaceDepthNormal::computeDesiredNormalAndCentroid(const vpHomogeneousMatrix &cMo,
                                                           const vpColVector &desired_normal,
                                                           const vpColVector &centroid_point)
{
  // Compute desired centroid in the object frame
  vpColVector centroid_cam(4);
  centroid_cam[0] = centroid_point[0];
  centroid_cam[1] = centroid_point[1];
  centroid_cam[2] = centroid_point[2];
  centroid_cam[3] = 1;

  vpColVector centroid_obj = cMo.inverse() * centroid_cam;
  m_faceDesiredCentroid.setWorldCoordinates(centroid_obj[0], centroid_obj[1], centroid_obj[2]);

  // Compute desired face normal in the object frame
  vpColVector face_normal_cam(4);
  face_normal_cam[0] = desired_normal[0];
  face_normal_cam[1] = desired_normal[1];
  face_normal_cam[2] = desired_normal[2];
  face_normal_cam[3] = 1;

  vpColVector face_normal_obj = cMo.inverse() * face_normal_cam;
  m_faceDesiredNormal.setWorldCoordinates(face_normal_obj[0], face_normal_obj[1], face_normal_obj[2]);
}

bool vpMbtFaceDepthNormal::computePolygonCentroid(const std::vector<vpPoint> &points_, vpPoint &centroid)
{
  if (points_.empty()) {
    return false;
  }

  if (points_.size() < 2) {
    centroid = points_[0];
    return true;
  }

  std::vector<vpPoint> points = points_;
  points.push_back(points_.front());

  double A1 = 0.0, A2 = 0.0, c_x1 = 0.0, c_x2 = 0.0, c_y = 0.0, c_z = 0.0;

  for (size_t i = 0; i < points.size() - 1; i++) {
    // projection onto xy plane
    c_x1 += (points[i].get_X() + points[i + 1].get_X()) *
            (points[i].get_X() * points[i + 1].get_Y() - points[i + 1].get_X() * points[i].get_Y());
    c_y += (points[i].get_Y() + points[i + 1].get_Y()) *
           (points[i].get_X() * points[i + 1].get_Y() - points[i + 1].get_X() * points[i].get_Y());
    A1 += points[i].get_X() * points[i + 1].get_Y() - points[i + 1].get_X() * points[i].get_Y();

    // projection onto xz plane
    c_x2 += (points[i].get_X() + points[i + 1].get_X()) *
            (points[i].get_X() * points[i + 1].get_Z() - points[i + 1].get_X() * points[i].get_Z());
    c_z += (points[i].get_Z() + points[i + 1].get_Z()) *
           (points[i].get_X() * points[i + 1].get_Z() - points[i + 1].get_X() * points[i].get_Z());
    A2 += points[i].get_X() * points[i + 1].get_Z() - points[i + 1].get_X() * points[i].get_Z();
  }

  c_x1 /= 3.0 * A1;
  c_y /= 3.0 * A1;
  c_x2 /= 3.0 * A2;
  c_z /= 3.0 * A2;

  if (A1 > A2) {
    centroid.set_X(c_x1);
  } else {
    centroid.set_X(c_x2);
  }

  centroid.set_Y(c_y);
  centroid.set_Z(c_z);

  return true;
}

void vpMbtFaceDepthNormal::computeROI(const vpHomogeneousMatrix &cMo, const unsigned int width,
                                      const unsigned int height, std::vector<vpImagePoint> &roiPts
#if DEBUG_DISPLAY_DEPTH_NORMAL
                                      ,
                                      std::vector<std::vector<vpImagePoint> > &roiPts_vec
#endif
)
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

        for (unsigned int i = 0; i < linesLst.size(); i++) {
          linesLst[i].first.project();
          linesLst[i].second.project();

          vpMeterPixelConversion::convertPoint(m_cam, linesLst[i].first.get_x(), linesLst[i].first.get_y(), ip1);
          vpMeterPixelConversion::convertPoint(m_cam, linesLst[i].second.get_x(), linesLst[i].second.get_y(), ip2);

          it->m_imPt1 = ip1;
          it->m_imPt2 = ip2;

          roiPts.push_back(ip1);
          roiPts.push_back(ip2);

#if DEBUG_DISPLAY_DEPTH_NORMAL
          std::vector<vpImagePoint> roiPts_;
          roiPts_.push_back(ip1);
          roiPts_.push_back(ip2);
          roiPts_vec.push_back(roiPts_);
#endif
        }
      }
    }
  } else {
    // Get polygon clipped
    m_polygon->getRoiClipped(m_cam, roiPts, cMo);

#if DEBUG_DISPLAY_DEPTH_NORMAL
    roiPts_vec.push_back(roiPts);
#endif
  }
}

void vpMbtFaceDepthNormal::computeVisibility() { m_isVisible = m_polygon->isVisible(); }

void vpMbtFaceDepthNormal::computeVisibilityDisplay()
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
      } else {
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
    } else {
      line->setVisible(false);
    }
  }
}

void vpMbtFaceDepthNormal::computeNormalVisibility(const double nx, const double ny, const double nz,
                                                   const vpHomogeneousMatrix &cMo, const vpCameraParameters &camera,
                                                   vpColVector &correct_normal, vpPoint &centroid)
{
  vpColVector faceNormal(3);
  faceNormal[0] = nx;
  faceNormal[1] = ny;
  faceNormal[2] = nz;
  faceNormal.normalize();

  // Get polygon clipped
  std::vector<vpImagePoint> roiPts;
  m_polygon->getRoiClipped(camera, roiPts, cMo);

  std::vector<vpPoint> polyPts;
  m_polygon->getPolygonClipped(polyPts);

  vpColVector e4(3);
  if (m_faceCentroidMethod == GEOMETRIC_CENTROID) {
    computePolygonCentroid(polyPts, centroid);
    centroid.project();

    e4[0] = -centroid.get_X();
    e4[1] = -centroid.get_Y();
    e4[2] = -centroid.get_Z();
    e4.normalize();
  } else {
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double centroid_z = 0.0;

    for (size_t i = 0; i < polyPts.size(); i++) {
      centroid_x += polyPts[i].get_X();
      centroid_y += polyPts[i].get_Y();
      centroid_z += polyPts[i].get_Z();
    }

    centroid_x /= polyPts.size();
    centroid_y /= polyPts.size();
    centroid_z /= polyPts.size();

    e4[0] = -centroid_x;
    e4[1] = -centroid_y;
    e4[2] = -centroid_z;
    e4.normalize();

    centroid.set_X(centroid_x);
    centroid.set_Y(centroid_y);
    centroid.set_Z(centroid_z);
  }

  correct_normal.resize(3, false);
  double angle = acos(vpColVector::dotProd(e4, faceNormal));
  if (angle < M_PI_2) {
    correct_normal = faceNormal;
  } else {
    correct_normal[0] = -faceNormal[0];
    correct_normal[1] = -faceNormal[1];
    correct_normal[2] = -faceNormal[2];
  }
}

#ifdef VISP_HAVE_PCL
void vpMbtFaceDepthNormal::computeNormalVisibility(const float nx, const float ny, const float nz,
                                                   const pcl::PointXYZ &centroid_point, pcl::PointXYZ &face_normal)
{
  vpColVector faceNormal(3);
  faceNormal[0] = nx;
  faceNormal[1] = ny;
  faceNormal[2] = nz;
  faceNormal.normalize();

  vpColVector e4(3);
  e4[0] = -centroid_point.x;
  e4[1] = -centroid_point.y;
  e4[2] = -centroid_point.z;
  e4.normalize();

  double angle = acos(vpColVector::dotProd(e4, faceNormal));
  if (angle < M_PI_2) {
    face_normal = pcl::PointXYZ(faceNormal[0], faceNormal[1], faceNormal[2]);
  } else {
    face_normal = pcl::PointXYZ(-faceNormal[0], -faceNormal[1], -faceNormal[2]);
  }
}
#endif

void vpMbtFaceDepthNormal::computeNormalVisibility(const double nx, const double ny, const double nz,
                                                   const vpColVector &centroid_point, vpColVector &face_normal)
{
  face_normal.resize(3, false);
  face_normal[0] = nx;
  face_normal[1] = ny;
  face_normal[2] = nz;
  face_normal.normalize();

  vpColVector e4 = -centroid_point;
  e4.normalize();

  double angle = acos(vpColVector::dotProd(e4, face_normal));
  if (angle >= M_PI_2) {
    face_normal[0] = -face_normal[0];
    face_normal[1] = -face_normal[1];
    face_normal[2] = -face_normal[2];
  }
}

void vpMbtFaceDepthNormal::computeInteractionMatrix(const vpHomogeneousMatrix &cMo, vpMatrix &L, vpColVector &features)
{
  L.resize(3, 6, false, false);

  // Transform the plane equation for the current pose
  m_planeCamera = m_planeObject;
  m_planeCamera.changeFrame(cMo);

  double ux = m_planeCamera.getA();
  double uy = m_planeCamera.getB();
  double uz = m_planeCamera.getC();
  double D = m_planeCamera.getD();
  double D2 = D * D;

  // Features
  features.resize(3, false);
  features[0] = -ux / D;
  features[1] = -uy / D;
  features[2] = -uz / D;

  // L_A
  L[0][0] = ux * ux / D2;
  L[0][1] = ux * uy / D2;
  L[0][2] = ux * uz / D2;
  L[0][3] = 0.0;
  L[0][4] = uz / D;
  L[0][5] = -uy / D;

  // L_B
  L[1][0] = ux * uy / D2;
  L[1][1] = uy * uy / D2;
  L[1][2] = uy * uz / D2;
  L[1][3] = -uz / D;
  L[1][4] = 0.0;
  L[1][5] = ux / D;

  // L_C
  L[2][0] = ux * uz / D2;
  L[2][1] = uy * uz / D2;
  L[2][2] = uz * uz / D2;
  L[2][3] = uy / D;
  L[2][4] = -ux / D;
  L[2][5] = 0.0;
}

void vpMbtFaceDepthNormal::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                   const vpCameraParameters &cam, const vpColor &col, const unsigned int thickness,
                                   const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);
    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

void vpMbtFaceDepthNormal::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                   const vpCameraParameters &cam, const vpColor &col, const unsigned int thickness,
                                   const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);
    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

void vpMbtFaceDepthNormal::displayFeature(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                          const vpCameraParameters &cam, const double scale,
                                          const unsigned int thickness)
{
  if (m_faceActivated && m_isTrackedDepthNormalFace && m_isVisible) {
    // Desired feature
    vpPoint pt_centroid = m_faceDesiredCentroid;
    pt_centroid.changeFrame(cMo);
    pt_centroid.project();

    vpImagePoint im_centroid;
    vpMeterPixelConversion::convertPoint(cam, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    vpPoint pt_normal = m_faceDesiredNormal;
    pt_normal.changeFrame(cMo);
    pt_normal.project();

    vpPoint pt_extremity;
    pt_extremity.set_X(pt_centroid.get_X() + pt_normal.get_X() * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + pt_normal.get_Y() * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + pt_normal.get_Z() * scale);
    pt_extremity.project();

    vpImagePoint im_extremity;
    vpMeterPixelConversion::convertPoint(cam, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    vpDisplay::displayArrow(I, im_centroid, im_extremity, vpColor::blue, 4, 2, thickness);

    // Current feature
    // Transform the plane equation for the current pose
    m_planeCamera = m_planeObject;
    m_planeCamera.changeFrame(cMo);

    double ux = m_planeCamera.getA();
    double uy = m_planeCamera.getB();
    double uz = m_planeCamera.getC();

    vpColVector correct_normal;
    vpCameraParameters cam_copy = cam;
    cam_copy.computeFov(I.getWidth(), I.getHeight());
    computeNormalVisibility(ux, uy, uz, cMo, cam_copy, correct_normal, pt_centroid);

    pt_centroid.project();
    vpMeterPixelConversion::convertPoint(cam_copy, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    pt_extremity.set_X(pt_centroid.get_X() + correct_normal[0] * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + correct_normal[1] * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + correct_normal[2] * scale);
    pt_extremity.project();

    vpMeterPixelConversion::convertPoint(cam_copy, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    vpDisplay::displayArrow(I, im_centroid, im_extremity, vpColor::red, 4, 2, thickness);
  }
}

void vpMbtFaceDepthNormal::displayFeature(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                          const vpCameraParameters &cam, const double scale,
                                          const unsigned int thickness)
{
  if (m_faceActivated && m_isTrackedDepthNormalFace && m_isVisible) {
    // Desired feature
    vpPoint pt_centroid = m_faceDesiredCentroid;
    pt_centroid.changeFrame(cMo);
    pt_centroid.project();

    vpImagePoint im_centroid;
    vpMeterPixelConversion::convertPoint(cam, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    vpPoint pt_normal = m_faceDesiredNormal;
    pt_normal.changeFrame(cMo);
    pt_normal.project();

    vpPoint pt_extremity;
    pt_extremity.set_X(pt_centroid.get_X() + pt_normal.get_X() * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + pt_normal.get_Y() * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + pt_normal.get_Z() * scale);
    pt_extremity.project();

    vpImagePoint im_extremity;
    vpMeterPixelConversion::convertPoint(cam, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    vpDisplay::displayArrow(I, im_centroid, im_extremity, vpColor::blue, 4, 2, thickness);

    // Current feature
    // Transform the plane equation for the current pose
    m_planeCamera = m_planeObject;
    m_planeCamera.changeFrame(cMo);

    double ux = m_planeCamera.getA();
    double uy = m_planeCamera.getB();
    double uz = m_planeCamera.getC();

    vpColVector correct_normal;
    vpCameraParameters cam_copy = cam;
    cam_copy.computeFov(I.getWidth(), I.getHeight());
    computeNormalVisibility(ux, uy, uz, cMo, cam_copy, correct_normal, pt_centroid);

    pt_centroid.project();
    vpMeterPixelConversion::convertPoint(cam_copy, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    pt_extremity.set_X(pt_centroid.get_X() + correct_normal[0] * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + correct_normal[1] * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + correct_normal[2] * scale);
    pt_extremity.project();

    vpMeterPixelConversion::convertPoint(cam_copy, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    vpDisplay::displayArrow(I, im_centroid, im_extremity, vpColor::red, 4, 2, thickness);
  }
}

void vpMbtFaceDepthNormal::estimateFeatures(const std::vector<double> &point_cloud_face, const vpHomogeneousMatrix &cMo,
                                            vpColVector &x_estimated, std::vector<double> &w)
{
  vpMbtTukeyEstimator<double> tukey_robust;
  std::vector<double> residues(point_cloud_face.size() / 3);

  w.resize(point_cloud_face.size() / 3, 1.0);

  unsigned int max_iter = 30, iter = 0;
  double error = 0.0, prev_error = -1.0;
  double A = 0.0, B = 0.0, C = 0.0;

  Mat33<double> ATA_3x3;

  bool checkSSE2 = vpCPUFeatures::checkSSE2();
#if !USE_SSE
  checkSSE2 = false;
#endif

  if (checkSSE2) {
#if USE_SSE
    while (std::fabs(error - prev_error) > 1e-6 && (iter < max_iter)) {
      if (iter == 0) {
        // Transform the plane equation for the current pose
        m_planeCamera = m_planeObject;
        m_planeCamera.changeFrame(cMo);

        double ux = m_planeCamera.getA();
        double uy = m_planeCamera.getB();
        double uz = m_planeCamera.getC();
        double D = m_planeCamera.getD();

        // Features
        A = -ux / D;
        B = -uy / D;
        C = -uz / D;

        size_t cpt = 0;
        if (point_cloud_face.size() / 3 >= 2) {
          const double *ptr_point_cloud = &point_cloud_face[0];
          const __m128d vA = _mm_set1_pd(A);
          const __m128d vB = _mm_set1_pd(B);
          const __m128d vC = _mm_set1_pd(C);
          const __m128d vones = _mm_set1_pd(1.0);

          double *ptr_residues = &residues[0];

          for (; cpt <= point_cloud_face.size() - 6; cpt += 6, ptr_point_cloud += 6, ptr_residues += 2) {
            const __m128d vxi = _mm_loadu_pd(ptr_point_cloud);
            const __m128d vyi = _mm_loadu_pd(ptr_point_cloud + 2);
            const __m128d vZi = _mm_loadu_pd(ptr_point_cloud + 4);
            const __m128d vinvZi = _mm_div_pd(vones, vZi);

            const __m128d tmp =
                _mm_add_pd(_mm_add_pd(_mm_mul_pd(vA, vxi), _mm_mul_pd(vB, vyi)), _mm_sub_pd(vC, vinvZi));
            _mm_storeu_pd(ptr_residues, tmp);
          }
        }

        for (; cpt < point_cloud_face.size(); cpt += 3) {
          double xi = point_cloud_face[cpt];
          double yi = point_cloud_face[cpt + 1];
          double Zi = point_cloud_face[cpt + 2];

          residues[cpt / 3] = (A * xi + B * yi + C - 1 / Zi);
        }
      }

      tukey_robust.MEstimator(residues, w, 1e-2);

      __m128d vsum_wi2_xi2 = _mm_setzero_pd();
      __m128d vsum_wi2_yi2 = _mm_setzero_pd();
      __m128d vsum_wi2 = _mm_setzero_pd();
      __m128d vsum_wi2_xi_yi = _mm_setzero_pd();
      __m128d vsum_wi2_xi = _mm_setzero_pd();
      __m128d vsum_wi2_yi = _mm_setzero_pd();

      __m128d vsum_wi2_xi_Zi = _mm_setzero_pd();
      __m128d vsum_wi2_yi_Zi = _mm_setzero_pd();
      __m128d vsum_wi2_Zi = _mm_setzero_pd();

      // Estimate A, B, C
      size_t cpt = 0;
      if (point_cloud_face.size() / 3 >= 2) {
        const double *ptr_point_cloud = &point_cloud_face[0];
        double *ptr_w = &w[0];

        const __m128d vones = _mm_set1_pd(1.0);

        for (; cpt <= point_cloud_face.size() - 6; cpt += 6, ptr_point_cloud += 6, ptr_w += 2) {
          const __m128d vwi2 = _mm_mul_pd(_mm_loadu_pd(ptr_w), _mm_loadu_pd(ptr_w));

          const __m128d vxi = _mm_loadu_pd(ptr_point_cloud);
          const __m128d vyi = _mm_loadu_pd(ptr_point_cloud + 2);
          const __m128d vZi = _mm_loadu_pd(ptr_point_cloud + 4);
          const __m128d vinvZi = _mm_div_pd(vones, vZi);

          vsum_wi2_xi2 = _mm_add_pd(vsum_wi2_xi2, _mm_mul_pd(vwi2, _mm_mul_pd(vxi, vxi)));
          vsum_wi2_yi2 = _mm_add_pd(vsum_wi2_yi2, _mm_mul_pd(vwi2, _mm_mul_pd(vyi, vyi)));
          vsum_wi2 = _mm_add_pd(vsum_wi2, vwi2);
          vsum_wi2_xi_yi = _mm_add_pd(vsum_wi2_xi_yi, _mm_mul_pd(vwi2, _mm_mul_pd(vxi, vyi)));
          vsum_wi2_xi = _mm_add_pd(vsum_wi2_xi, _mm_mul_pd(vwi2, vxi));
          vsum_wi2_yi = _mm_add_pd(vsum_wi2_yi, _mm_mul_pd(vwi2, vyi));

          const __m128d vwi2_invZi = _mm_mul_pd(vwi2, vinvZi);
          vsum_wi2_xi_Zi = _mm_add_pd(vsum_wi2_xi_Zi, _mm_mul_pd(vxi, vwi2_invZi));
          vsum_wi2_yi_Zi = _mm_add_pd(vsum_wi2_yi_Zi, _mm_mul_pd(vyi, vwi2_invZi));
          vsum_wi2_Zi = _mm_add_pd(vsum_wi2_Zi, vwi2_invZi);
        }
      }

      double vtmp[2];
      _mm_storeu_pd(vtmp, vsum_wi2_xi2);
      double sum_wi2_xi2 = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_yi2);
      double sum_wi2_yi2 = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2);
      double sum_wi2 = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_xi_yi);
      double sum_wi2_xi_yi = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_xi);
      double sum_wi2_xi = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_yi);
      double sum_wi2_yi = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_xi_Zi);
      double sum_wi2_xi_Zi = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_yi_Zi);
      double sum_wi2_yi_Zi = vtmp[0] + vtmp[1];

      _mm_storeu_pd(vtmp, vsum_wi2_Zi);
      double sum_wi2_Zi = vtmp[0] + vtmp[1];

      for (; cpt < point_cloud_face.size(); cpt += 3) {
        double wi2 = w[cpt / 3] * w[cpt / 3];

        double xi = point_cloud_face[cpt];
        double yi = point_cloud_face[cpt + 1];
        double Zi = point_cloud_face[cpt + 2];
        double invZi = 1.0 / Zi;

        sum_wi2_xi2 += wi2 * xi * xi;
        sum_wi2_yi2 += wi2 * yi * yi;
        sum_wi2 += wi2;
        sum_wi2_xi_yi += wi2 * xi * yi;
        sum_wi2_xi += wi2 * xi;
        sum_wi2_yi += wi2 * yi;

        sum_wi2_xi_Zi += wi2 * xi * invZi;
        sum_wi2_yi_Zi += wi2 * yi * invZi;
        sum_wi2_Zi += wi2 * invZi;
      }

      ATA_3x3[0] = sum_wi2_xi2;
      ATA_3x3[1] = sum_wi2_xi_yi;
      ATA_3x3[2] = sum_wi2_xi;
      ATA_3x3[3] = sum_wi2_xi_yi;
      ATA_3x3[4] = sum_wi2_yi2;
      ATA_3x3[5] = sum_wi2_yi;
      ATA_3x3[6] = sum_wi2_xi;
      ATA_3x3[7] = sum_wi2_yi;
      ATA_3x3[8] = sum_wi2;

      Mat33<double> minv = ATA_3x3.inverse();

      A = minv[0] * sum_wi2_xi_Zi + minv[1] * sum_wi2_yi_Zi + minv[2] * sum_wi2_Zi;
      B = minv[3] * sum_wi2_xi_Zi + minv[4] * sum_wi2_yi_Zi + minv[5] * sum_wi2_Zi;
      C = minv[6] * sum_wi2_xi_Zi + minv[7] * sum_wi2_yi_Zi + minv[8] * sum_wi2_Zi;

      cpt = 0;

      // Compute error
      prev_error = error;
      error = 0.0;

      __m128d verror = _mm_set1_pd(0.0);
      if (point_cloud_face.size() / 3 >= 2) {
        const double *ptr_point_cloud = &point_cloud_face[0];
        const __m128d vA = _mm_set1_pd(A);
        const __m128d vB = _mm_set1_pd(B);
        const __m128d vC = _mm_set1_pd(C);
        const __m128d vones = _mm_set1_pd(1.0);

        double *ptr_residues = &residues[0];

        for (; cpt <= point_cloud_face.size() - 6; cpt += 6, ptr_point_cloud += 6, ptr_residues += 2) {
          const __m128d vxi = _mm_loadu_pd(ptr_point_cloud);
          const __m128d vyi = _mm_loadu_pd(ptr_point_cloud + 2);
          const __m128d vZi = _mm_loadu_pd(ptr_point_cloud + 4);
          const __m128d vinvZi = _mm_div_pd(vones, vZi);

          const __m128d tmp = _mm_add_pd(_mm_add_pd(_mm_mul_pd(vA, vxi), _mm_mul_pd(vB, vyi)), _mm_sub_pd(vC, vinvZi));
          verror = _mm_add_pd(verror, _mm_mul_pd(tmp, tmp));

          _mm_storeu_pd(ptr_residues, tmp);
        }
      }

      _mm_storeu_pd(vtmp, verror);
      error = vtmp[0] + vtmp[1];

      for (size_t idx = cpt; idx < point_cloud_face.size(); idx += 3) {
        double xi = point_cloud_face[idx];
        double yi = point_cloud_face[idx + 1];
        double Zi = point_cloud_face[idx + 2];

        error += vpMath::sqr(A * xi + B * yi + C - 1 / Zi);
        residues[idx / 3] = (A * xi + B * yi + C - 1 / Zi);
      }

      error /= point_cloud_face.size() / 3;

      iter++;
    } // while ( std::fabs(error - prev_error) > 1e-6 && (iter < max_iter) )
#endif
  } else {
    while (std::fabs(error - prev_error) > 1e-6 && (iter < max_iter)) {
      if (iter == 0) {
        // Transform the plane equation for the current pose
        m_planeCamera = m_planeObject;
        m_planeCamera.changeFrame(cMo);

        double ux = m_planeCamera.getA();
        double uy = m_planeCamera.getB();
        double uz = m_planeCamera.getC();
        double D = m_planeCamera.getD();

        // Features
        A = -ux / D;
        B = -uy / D;
        C = -uz / D;

        for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
          double xi = point_cloud_face[3 * i];
          double yi = point_cloud_face[3 * i + 1];
          double Zi = point_cloud_face[3 * i + 2];

          residues[i] = (A * xi + B * yi + C - 1 / Zi);
        }
      }

      tukey_robust.MEstimator(residues, w, 1e-2);

      // Estimate A, B, C
      double sum_wi2_xi2 = 0.0, sum_wi2_yi2 = 0.0, sum_wi2 = 0.0;
      double sum_wi2_xi_yi = 0.0, sum_wi2_xi = 0.0, sum_wi2_yi = 0.0;

      double sum_wi2_xi_Zi = 0.0, sum_wi2_yi_Zi = 0.0, sum_wi2_Zi = 0.0;

      for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
        double wi2 = w[i] * w[i];

        double xi = point_cloud_face[3 * i];
        double yi = point_cloud_face[3 * i + 1];
        double Zi = point_cloud_face[3 * i + 2];
        double invZi = 1 / Zi;

        sum_wi2_xi2 += wi2 * xi * xi;
        sum_wi2_yi2 += wi2 * yi * yi;
        sum_wi2 += wi2;
        sum_wi2_xi_yi += wi2 * xi * yi;
        sum_wi2_xi += wi2 * xi;
        sum_wi2_yi += wi2 * yi;

        sum_wi2_xi_Zi += wi2 * xi * invZi;
        sum_wi2_yi_Zi += wi2 * yi * invZi;
        sum_wi2_Zi += wi2 * invZi;
      }

      ATA_3x3[0] = sum_wi2_xi2;
      ATA_3x3[1] = sum_wi2_xi_yi;
      ATA_3x3[2] = sum_wi2_xi;
      ATA_3x3[3] = sum_wi2_xi_yi;
      ATA_3x3[4] = sum_wi2_yi2;
      ATA_3x3[5] = sum_wi2_yi;
      ATA_3x3[6] = sum_wi2_xi;
      ATA_3x3[7] = sum_wi2_yi;
      ATA_3x3[8] = sum_wi2;

      Mat33<double> minv = ATA_3x3.inverse();

      A = minv[0] * sum_wi2_xi_Zi + minv[1] * sum_wi2_yi_Zi + minv[2] * sum_wi2_Zi;
      B = minv[3] * sum_wi2_xi_Zi + minv[4] * sum_wi2_yi_Zi + minv[5] * sum_wi2_Zi;
      C = minv[6] * sum_wi2_xi_Zi + minv[7] * sum_wi2_yi_Zi + minv[8] * sum_wi2_Zi;

      prev_error = error;
      error = 0.0;

      // Compute error
      for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
        double xi = point_cloud_face[3 * i];
        double yi = point_cloud_face[3 * i + 1];
        double Zi = point_cloud_face[3 * i + 2];

        error += vpMath::sqr(A * xi + B * yi + C - 1 / Zi);
        residues[i] = (A * xi + B * yi + C - 1 / Zi);
      }

      error /= point_cloud_face.size() / 3;

      iter++;
    } // while ( std::fabs(error - prev_error) > 1e-6 && (iter < max_iter) )
  }

  x_estimated.resize(3, false);
  x_estimated[0] = A;
  x_estimated[1] = B;
  x_estimated[2] = C;
}

void vpMbtFaceDepthNormal::estimatePlaneEquationSVD(const std::vector<double> &point_cloud_face,
                                                    const vpHomogeneousMatrix &cMo,
                                                    vpColVector &plane_equation_estimated, vpColVector &centroid)
{
  const unsigned int max_iter = 10;
  double prev_error = 1e3;
  double error = 1e3 - 1;

  std::vector<double> weights(point_cloud_face.size() / 3, 1.0);
  std::vector<double> residues(point_cloud_face.size() / 3);
  vpMatrix M((unsigned int)(point_cloud_face.size() / 3), 3);
  vpMbtTukeyEstimator<double> tukey;
  vpColVector normal;

  for (unsigned int iter = 0; iter < max_iter && std::fabs(error - prev_error) > 1e-6; iter++) {
    if (iter != 0) {
      tukey.MEstimator(residues, weights, 1e-4);
    } else {
      // Transform the plane equation for the current pose
      m_planeCamera = m_planeObject;
      m_planeCamera.changeFrame(cMo);

      double A = m_planeCamera.getA();
      double B = m_planeCamera.getB();
      double C = m_planeCamera.getC();
      double D = m_planeCamera.getD();

      // Compute distance point to estimated plane
      for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
        residues[i] = std::fabs(A * point_cloud_face[3 * i] + B * point_cloud_face[3 * i + 1] +
                                C * point_cloud_face[3 * i + 2] + D) /
                      sqrt(A * A + B * B + C * C);
      }

      tukey.MEstimator(residues, weights, 1e-4);
      plane_equation_estimated.resize(4, false);
    }

    // Compute centroid
    double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
    double total_w = 0.0;

    for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
      centroid_x += weights[i] * point_cloud_face[3 * i];
      centroid_y += weights[i] * point_cloud_face[3 * i + 1];
      centroid_z += weights[i] * point_cloud_face[3 * i + 2];
      total_w += weights[i];
    }

    centroid_x /= total_w;
    centroid_y /= total_w;
    centroid_z /= total_w;

    // Minimization
    for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
      M[(unsigned int)i][0] = weights[i] * (point_cloud_face[3 * i] - centroid_x);
      M[(unsigned int)i][1] = weights[i] * (point_cloud_face[3 * i + 1] - centroid_y);
      M[(unsigned int)i][2] = weights[i] * (point_cloud_face[3 * i + 2] - centroid_z);
    }

    vpMatrix J = M.t() * M;

    vpColVector W;
    vpMatrix V;
    J.svd(W, V);

    double smallestSv = W[0];
    unsigned int indexSmallestSv = 0;
    for (unsigned int i = 1; i < W.size(); i++) {
      if (W[i] < smallestSv) {
        smallestSv = W[i];
        indexSmallestSv = i;
      }
    }

    normal = V.getCol(indexSmallestSv);

    // Compute plane equation
    double A = normal[0], B = normal[1], C = normal[2];
    double D = -(A * centroid_x + B * centroid_y + C * centroid_z);

    // Update plane equation
    plane_equation_estimated[0] = A;
    plane_equation_estimated[1] = B;
    plane_equation_estimated[2] = C;
    plane_equation_estimated[3] = D;

    // Compute error points to estimated plane
    prev_error = error;
    error = 0.0;
    for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
      residues[i] = std::fabs(A * point_cloud_face[3 * i] + B * point_cloud_face[3 * i + 1] +
                              C * point_cloud_face[3 * i + 2] + D) /
                    sqrt(A * A + B * B + C * C);
      error += residues[i] * residues[i];
    }
    error /= sqrt(error / total_w);
  }

  // Update final weights
  tukey.MEstimator(residues, weights, 1e-4);

  // Update final centroid
  centroid.resize(3, false);
  double total_w = 0.0;

  for (size_t i = 0; i < point_cloud_face.size() / 3; i++) {
    centroid[0] += weights[i] * point_cloud_face[3 * i];
    centroid[1] += weights[i] * point_cloud_face[3 * i + 1];
    centroid[2] += weights[i] * point_cloud_face[3 * i + 2];
    total_w += weights[i];
  }

  centroid[0] /= total_w;
  centroid[1] /= total_w;
  centroid[2] /= total_w;

  // Compute final plane equation
  double A = normal[0], B = normal[1], C = normal[2];
  double D = -(A * centroid[0] + B * centroid[1] + C * centroid[2]);

  // Update final plane equation
  plane_equation_estimated[0] = A;
  plane_equation_estimated[1] = B;
  plane_equation_estimated[2] = C;
  plane_equation_estimated[3] = D;
}

/*!
  Return a list of features parameters for display.
  - Parameters are: `<feature id (here 2 for depth normal)>`, `<centroid.i()>`, `<centroid.j()>`,
  `<extremity.i()>`, `extremity.j()`
*/
std::vector<std::vector<double> > vpMbtFaceDepthNormal::getFeaturesForDisplay(const vpHomogeneousMatrix &cMo,
                                                                              const vpCameraParameters &cam,
                                                                              const double scale)
{
  std::vector<std::vector<double> > features;

  if (m_faceActivated && m_isTrackedDepthNormalFace && m_isVisible) {
    // Desired feature
    vpPoint pt_centroid = m_faceDesiredCentroid;
    pt_centroid.changeFrame(cMo);
    pt_centroid.project();

    vpImagePoint im_centroid;
    vpMeterPixelConversion::convertPoint(cam, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    vpPoint pt_normal = m_faceDesiredNormal;
    pt_normal.changeFrame(cMo);
    pt_normal.project();

    vpPoint pt_extremity;
    pt_extremity.set_X(pt_centroid.get_X() + pt_normal.get_X() * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + pt_normal.get_Y() * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + pt_normal.get_Z() * scale);
    pt_extremity.project();

    vpImagePoint im_extremity;
    vpMeterPixelConversion::convertPoint(cam, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    {
#ifdef VISP_HAVE_CXX11
      std::vector<double> params = {2, //desired normal
                                    im_centroid.get_i(),
                                    im_centroid.get_j(),
                                    im_extremity.get_i(),
                                    im_extremity.get_j()};
#else   
      std::vector<double> params;
      params.push_back(2); //desired normal
      params.push_back(im_centroid.get_i());
      params.push_back(im_centroid.get_j());
      params.push_back(im_extremity.get_i());
      params.push_back(im_extremity.get_j());
#endif
      features.push_back(params);
    }

    // Current feature
    // Transform the plane equation for the current pose
    m_planeCamera = m_planeObject;
    m_planeCamera.changeFrame(cMo);

    double ux = m_planeCamera.getA();
    double uy = m_planeCamera.getB();
    double uz = m_planeCamera.getC();

    vpColVector correct_normal;
    computeNormalVisibility(ux, uy, uz, cMo, cam, correct_normal, pt_centroid);

    pt_centroid.project();
    vpMeterPixelConversion::convertPoint(cam, pt_centroid.get_x(), pt_centroid.get_y(), im_centroid);

    pt_extremity.set_X(pt_centroid.get_X() + correct_normal[0] * scale);
    pt_extremity.set_Y(pt_centroid.get_Y() + correct_normal[1] * scale);
    pt_extremity.set_Z(pt_centroid.get_Z() + correct_normal[2] * scale);
    pt_extremity.project();

    vpMeterPixelConversion::convertPoint(cam, pt_extremity.get_x(), pt_extremity.get_y(), im_extremity);

    {
#ifdef VISP_HAVE_CXX11
      std::vector<double> params = {3, //normal at current pose
                                    im_centroid.get_i(),
                                    im_centroid.get_j(),
                                    im_extremity.get_i(),
                                    im_extremity.get_j()};
#else   
      std::vector<double> params;
      params.push_back(3); //normal at current pose
      params.push_back(im_centroid.get_i());
      params.push_back(im_centroid.get_j());
      params.push_back(im_extremity.get_i());
      params.push_back(im_extremity.get_j());
#endif
      features.push_back(params);
    }
  }

  return features;
}

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
std::vector<std::vector<double> > vpMbtFaceDepthNormal::getModelForDisplay(unsigned int width, unsigned int height,
                                                                           const vpHomogeneousMatrix &cMo,
                                                                           const vpCameraParameters &cam,
                                                                           const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  if ((m_polygon->isVisible() && m_isTrackedDepthNormalFace) || displayFullModel) {
    computeVisibilityDisplay();

    for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
         ++it) {
      vpMbtDistanceLine *line = *it;
      std::vector<std::vector<double> > lineModels = line->getModelForDisplay(width, height, cMo, cam, displayFullModel);
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
bool vpMbtFaceDepthNormal::samePoint(const vpPoint &P1, const vpPoint &P2) const
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

void vpMbtFaceDepthNormal::setCameraParameters(const vpCameraParameters &camera)
{
  m_cam = camera;

  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    (*it)->setCameraParameters(camera);
  }
}

void vpMbtFaceDepthNormal::setScanLineVisibilityTest(const bool v)
{
  m_useScanLine = v;

  for (std::vector<vpMbtDistanceLine *>::const_iterator it = m_listOfFaceLines.begin(); it != m_listOfFaceLines.end();
       ++it) {
    (*it)->useScanLine = v;
  }
}
