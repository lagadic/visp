/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 */

#include <visp3/core/vpConfig.h>
#include <visp3/rbt/vpRBSilhouetteCCDTracker.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

#define VISP_DEBUG_CCD_TRACKER 0

BEGIN_VISP_NAMESPACE

void sampleWithoutReplacement(size_t count, size_t vectorSize, std::vector<size_t> &indices, vpUniRand &random)
{
  count = std::min(count, vectorSize);
  indices.resize(count);
  size_t added = 0;
  for (size_t i = 0; i < vectorSize; ++i) {
    double randomVal = random.uniform(0.0, 1.0);
    if ((vectorSize - i) * randomVal < (count - added)) {
      indices[added++] = i;
    }
    if (added == count) {
      break;
    }
  }
}

template <class T> class FastMat33
{
public:
  std::array<T, 9> data;

  FastMat33() { }

  inline T operator[](const size_t i) const { return data[i]; }

  inline T &operator[](const size_t i) { return data[i]; }

  void inverse(FastMat33<T> &minv) const
  {
    // determinant
    T det = data[0] * (data[4] * data[8] - data[7] * data[5]) - data[1] * (data[3] * data[8] - data[5] * data[6]) +
      data[2] * (data[3] * data[7] - data[4] * data[6]);
    T invdet = 1 / det;

    minv[0] = (data[4] * data[8] - data[7] * data[5]) * invdet;
    minv[1] = (data[2] * data[7] - data[1] * data[8]) * invdet;
    minv[2] = (data[1] * data[5] - data[2] * data[4]) * invdet;
    minv[3] = (data[5] * data[6] - data[3] * data[8]) * invdet;
    minv[4] = (data[0] * data[8] - data[2] * data[6]) * invdet;
    minv[5] = (data[3] * data[2] - data[0] * data[5]) * invdet;
    minv[6] = (data[3] * data[7] - data[6] * data[4]) * invdet;
    minv[7] = (data[6] * data[1] - data[0] * data[7]) * invdet;
    minv[8] = (data[0] * data[4] - data[3] * data[1]) * invdet;
  }

  static void multiply(const vpMatrix &A, const FastMat33<double> &B, vpMatrix &C)
  {
    C.resize(A.getRows(), 3, false, false);
    for (unsigned int i = 0; i < A.getRows(); ++i) {
      C[i][0] = A[i][0] * B.data[0] + A[i][1] * B.data[3] + A[i][2] * B.data[6];
      C[i][1] = A[i][0] * B.data[1] + A[i][1] * B.data[4] + A[i][2] * B.data[7];
      C[i][2] = A[i][0] * B.data[2] + A[i][1] * B.data[5] + A[i][2] * B.data[8];
    }
  }
};

template <class T> class FastMat63
{
public:
  std::array<T, 18> data;

  FastMat63() { }

  inline T operator[](const size_t i) const { return data[i]; }

  inline T &operator[](const size_t i) { return data[i]; }

  static void multiply(const FastMat63<T> &A, const FastMat33<T> &B, FastMat63 &C)
  {
    for (unsigned int i = 0; i < 6; ++i) {
      const T *d = &A.data[i * 3];
      T *c = &C.data[i * 3];
      c[0] = d[0] * B.data[0] + d[1] * B.data[3] + d[2] * B.data[6];
      c[1] = d[0] * B.data[1] + d[1] * B.data[4] + d[2] * B.data[7];
      c[2] = d[0] * B.data[2] + d[1] * B.data[5] + d[2] * B.data[8];
    }
  }

  static void multiplyBTranspose(const FastMat63<double> &A, const FastMat63<double> &B, vpMatrix &C)
  {
    for (unsigned int i = 0; i < 6; ++i) {
      const double *a = &A.data[i * 3];
      double *c = C[i];

      c[0] = a[0] * B[0] + a[1] * B[1] + a[2] * B[2];
      c[1] = a[0] * B[3] + a[1] * B[4] + a[2] * B[5];
      c[2] = a[0] * B[6] + a[1] * B[7] + a[2] * B[8];

      c[3] = a[0] * B[9] + a[1] * B[10] + a[2] * B[11];
      c[4] = a[0] * B[12] + a[1] * B[13] + a[2] * B[14];
      c[5] = a[0] * B[15] + a[1] * B[16] + a[2] * B[17];
    }
  }
};

template <class T> class FastVec3
{
public:
  std::array<T, 3> data;

  inline T operator[](const size_t i) const { return data[i]; }
  inline T &operator[](const size_t i) { return data[i]; }

  static void multiply(const FastMat63<double> &A, const FastVec3<double> &B, vpColVector &C)
  {
    C[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    C[1] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
    C[2] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
    C[3] = A[9] * B[0] + A[10] * B[1] + A[11] * B[2];
    C[4] = A[12] * B[0] + A[13] * B[1] + A[14] * B[2];
    C[5] = A[15] * B[0] + A[16] * B[1] + A[17] * B[2];
  }
};

const unsigned int vpRBSilhouetteCCDTracker::BASE_SEED = 421;

vpRBSilhouetteCCDTracker::vpRBSilhouetteCCDTracker() : vpRBFeatureTracker(), m_vvsConvergenceThreshold(0.0),
m_temporalSmoothingFac(0.0), m_useMask(false), m_minMaskConfidence(0.0), m_maxPoints(0), m_random(vpRBSilhouetteCCDTracker::BASE_SEED), m_displayType(DT_SIMPLE)
{ }

void vpRBSilhouetteCCDTracker::onTrackingIterStart(const vpHomogeneousMatrix & /*cMo*/)
{
  m_ccdParameters.h = m_ccdParameters.start_h;
  m_ccdParameters.delta_h = m_ccdParameters.start_delta_h;
  m_ccdParameters.iters_since_scale_change = 0;
  m_controlPoints.clear();
}

void vpRBSilhouetteCCDTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{


  //m_controlPoints.reserve(frame.silhouettePoints.size());
  const vpHomogeneousMatrix cMo = frame.renders.cMo;
  const vpHomogeneousMatrix oMc = cMo.inverse();

  std::vector<std::vector<vpRBSilhouetteControlPoint>> pointsPerThread;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
#ifdef VISP_HAVE_OPENMP
#pragma omp single
    {
      unsigned int numThreads = omp_get_num_threads();
      pointsPerThread.resize(numThreads);
    }
#else
    {
      pointsPerThread.resize(1);
    }
#endif
#ifdef VISP_HAVE_OPENMP
    unsigned int threadIdx = omp_get_thread_num();
#else
    unsigned int threadIdx = 0;
#endif
    std::vector<vpRBSilhouetteControlPoint> localControlPoints;
#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
    for (int i = 0; i < static_cast<int>(frame.silhouettePoints.size()); ++i) {
      const vpRBSilhouettePoint sp = frame.silhouettePoints[i];
      int ii = sp.i, jj = sp.j;

      if (!sp.isSilhouette) {
        continue;
      }

      if (ii <= m_ccdParameters.h || jj <= m_ccdParameters.h ||
        static_cast<unsigned int>(ii) >= frame.I.getHeight() - m_ccdParameters.h ||
        static_cast<unsigned int>(jj) >= frame.I.getWidth() - m_ccdParameters.h) {
        continue;
      }
      vpRBSilhouetteControlPoint pccd;

      pccd.buildSilhouettePoint(ii, jj, sp.Z, sp.orientation, sp.normal, cMo, oMc, frame.cam);

      if (std::isnan(sp.orientation) || !pccd.isValid()) {
        continue;
      }

      if (frame.hasMask() && m_useMask) {
        double maskGradValue = pccd.getMaxMaskGradientAlongLine(frame.mask, m_ccdParameters.h);
        if (maskGradValue < m_minMaskConfidence) {
          continue;
        }
      }
      localControlPoints.push_back(std::move(pccd));
    }

    {
      pointsPerThread[threadIdx] = std::move(localControlPoints);
    }
  }
  unsigned int numElements = 0;
  for (const std::vector<vpRBSilhouetteControlPoint> &points: pointsPerThread) {
    numElements += points.size();
  }

  m_controlPoints.reserve(numElements);
  for (const std::vector<vpRBSilhouetteControlPoint> &points: pointsPerThread) {
    m_controlPoints.insert(m_controlPoints.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
  }

  if (m_maxPoints > 0 && m_controlPoints.size() > m_maxPoints) {
    std::vector<size_t> keptIndices(m_maxPoints);
    sampleWithoutReplacement(m_maxPoints, m_controlPoints.size(), keptIndices, m_random);

    std::vector<vpRBSilhouetteControlPoint> finalPoints;
    finalPoints.reserve(m_maxPoints);
    for (unsigned int index : keptIndices) {
      finalPoints.emplace_back(std::move(m_controlPoints[index]));
    }
    m_controlPoints = std::move(finalPoints);
  }
}

void vpRBSilhouetteCCDTracker::initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix & /*cMo*/)
{
  // Reinit all variables
  m_sigma = vpMatrix(m_ccdParameters.phi_dim, m_ccdParameters.phi_dim, 0.0);
  m_cov.resize(6, 6);

  m_vvsConverged = false;

  unsigned int resolution = m_controlPoints.size();
  int normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  unsigned nerror_ccd = 2 * normal_points_number * 3 * resolution;
  m_numFeatures = nerror_ccd;

  m_stats.reinit(resolution, normal_points_number);
  m_prevStats.reinit(resolution, normal_points_number);
  m_gradient = vpMatrix(m_ccdParameters.phi_dim, 1, 0.0);
  m_hessian = vpMatrix(m_ccdParameters.phi_dim, m_ccdParameters.phi_dim, 0.0);
  // m_gradientData.clear();
  // m_hessianData.clear();
  if (m_gradients.size() != m_controlPoints.size() * 2 * normal_points_number) {
    // Resize only if we need additional data
    if (m_gradientData.size() <  m_controlPoints.size() * 2 * normal_points_number * 6) {
      m_gradientData.resize(m_controlPoints.size() * 2 * normal_points_number * 6);
      m_hessianData.resize(m_controlPoints.size() * 2 * normal_points_number * 6 * 6);
    }
    m_gradients.clear();
    m_hessians.clear();
    m_gradients.resize(m_controlPoints.size() * 2 * normal_points_number);
    m_hessians.resize(m_controlPoints.size() * 2 * normal_points_number);
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(m_gradients.size()); ++i) {
      vpColVector::view(m_gradients[i], m_gradientData.data() + i * 6, 6);
      vpMatrix::view(m_hessians[i], m_hessianData.data() + i * 6 * 6, 6, 6);
    }
  }
  m_weights.resize(m_numFeatures, false);
  if (m_temporalSmoothingFac > 0.0) {
    computeLocalStatistics(previousFrame.IRGB, m_prevStats);
  }
  else {
    m_prevStats.zero();
  }
  m_previousFrame = &previousFrame;
}

void vpRBSilhouetteCCDTracker::changeScale()
{
  m_sigma = vpMatrix(m_ccdParameters.phi_dim, m_ccdParameters.phi_dim, 0.0);
  m_cov.resize(6, 6);
  unsigned int resolution = m_controlPoints.size();
  int normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  unsigned nerror_ccd = 2 * normal_points_number * 3 * resolution;
  m_numFeatures = nerror_ccd;

  m_prevStats.reinit(resolution, normal_points_number);
  m_stats.reinit(resolution, normal_points_number);

  if (m_gradientData.size() != m_controlPoints.size() * 2 * normal_points_number * 6) {
    if (m_gradientData.size() <  m_controlPoints.size() * 2 * normal_points_number * 6) {
      m_gradientData.resize(m_controlPoints.size() * 2 * normal_points_number * 6);
      m_hessianData.resize(m_controlPoints.size() * 2 * normal_points_number * 6 * 6);
    }
    m_gradients.clear();
    m_hessians.clear();
    m_gradients.resize(m_controlPoints.size() * 2 * normal_points_number);
    m_hessians.resize(m_controlPoints.size() * 2 * normal_points_number);
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < m_gradients.size(); ++i) {
      vpColVector::view(m_gradients[i], m_gradientData.data() + i * 6, 6);
      vpMatrix::view(m_hessians[i], m_hessianData.data() + i * 6 * 6, 6, 6);
    }
  }
  m_weights.resize(m_numFeatures, false);
  if (m_temporalSmoothingFac > 0.0) {
    computeLocalStatistics(m_previousFrame->IRGB, m_prevStats);
  }
  else {
    m_prevStats = m_stats;
  }
}

void vpRBSilhouetteCCDTracker::computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration)
{
  if (m_numFeatures == 0) {
    m_vvsConverged = false;
    return;
  }

  vpColVector oldPoints(m_controlPoints.size() * 2);
  for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
    oldPoints[i * 2] = m_controlPoints[i].icpoint.get_u();
    oldPoints[i * 2 + 1] = m_controlPoints[i].icpoint.get_v();
  }

  updateCCDPoints(cMo);

  // Compute motion between current and previous frames, in pixels
  tol = 0.0;
  for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
    tol += abs(oldPoints[i * 2] - m_controlPoints[i].icpoint.get_u());
    tol += abs(oldPoints[i * 2 + 1] - m_controlPoints[i].icpoint.get_v());
  }
  tol /= m_controlPoints.size();
  // If enabled and if motion between consecutive frame is small enough, halve contour size
  if (m_ccdParameters.iters_since_scale_change > m_ccdParameters.min_iters_before_scale_change && tol < sqrt(static_cast<double>(m_ccdParameters.h) / 8.0)) {
    int previousH = m_ccdParameters.h;
    m_ccdParameters.h = std::max(m_ccdParameters.min_h, previousH / 2);
    if (m_ccdParameters.h != previousH) {
      m_ccdParameters.delta_h = static_cast<int>(std::max(1, m_ccdParameters.delta_h / 2));
      changeScale();
    }
    m_ccdParameters.iters_since_scale_change = 0;
  }
  else {
    ++m_ccdParameters.iters_since_scale_change;
  }
  computeLocalStatistics(frame.IRGB, m_stats);
  // Update interaction matrix, and gauss newton left and right side terms
  if (m_temporalSmoothingFac > 0.0) {
    computeErrorAndInteractionMatrix<true>();
  }
  else {
    computeErrorAndInteractionMatrix<false>();
  }

  m_vvsConverged = false;
  if (iteration > 0 && tol < m_vvsConvergenceThreshold) {
    m_vvsConverged = true;
  }
}

void vpRBSilhouetteCCDTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &/*I*/,
                                       const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &/*depth*/) const
{
  unsigned normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  unsigned nerror_per_point = 2 * normal_points_number * 3;
  if (m_displayType == DT_SIMPLE) {

    for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
      const vpRBSilhouetteControlPoint &p = m_controlPoints[i];
      vpDisplay::displayPoint(IRGB, p.icpoint.get_i(), p.icpoint.get_j(), vpColor::green, 2);
    }
  }
  else if (m_displayType == DT_ERROR) {
    vpColVector errorPerPoint(m_controlPoints.size());
    double maxPointError = 0.0;
    for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
      double sum = 0.0;
      for (unsigned int j = 0; j < nerror_per_point; ++j) {
        sum += m_error[i * nerror_per_point + j];
      }
      if (sum > maxPointError) {
        maxPointError = sum;
      }
      errorPerPoint[i] = sum;
    }
    const vpColor bestColor = vpColor::green;
    const vpColor worstColor = vpColor::red;
    const double diffR = static_cast<double>(worstColor.R) - static_cast<double>(bestColor.R);
    const double diffG = static_cast<double>(worstColor.G) - static_cast<double>(bestColor.G);
    const double diffB = static_cast<double>(worstColor.B) - static_cast<double>(bestColor.B);
    unsigned idx = 0;
    for (const vpRBSilhouetteControlPoint &p : m_controlPoints) {
      const double weight = errorPerPoint[idx] / maxPointError;

      vpColor c;
      c.R = static_cast<unsigned char>(static_cast<double>(bestColor.R) + diffR * weight);
      c.G = static_cast<unsigned char>(static_cast<double>(bestColor.G) + diffG * weight);
      c.B = static_cast<unsigned char>(static_cast<double>(bestColor.B) + diffB * weight);

      vpDisplay::displayCross(IRGB, static_cast<int>(p.icpoint.get_i()), static_cast<int>(p.icpoint.get_j()), 3, c, 1);
      ++idx;
    }
  }
  else if (m_displayType == DT_WEIGHT) {
    vpColVector weightPerPoint(static_cast<unsigned int>(m_controlPoints.size()));
    for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
      double sum = 0.0;
      for (unsigned int j = 0; j < nerror_per_point; ++j) {
        sum += m_weights[i * nerror_per_point + j];
      }

      weightPerPoint[i] = sum / nerror_per_point;
    }
    const vpColor bestColor = vpColor::green;
    unsigned idx = 0;
    for (const vpRBSilhouetteControlPoint &p : m_controlPoints) {
      const double weight = weightPerPoint[idx];
      vpColor c;
      c.R = 0;
      c.G = static_cast<unsigned char>(255.f * weight);
      c.B = 0;

      vpDisplay::displayCross(IRGB, static_cast<int>(p.icpoint.get_i()), static_cast<int>(p.icpoint.get_j()), 3, c, 1);
      idx++;
    }
  }
  else if (m_displayType == DT_WEIGHT_AND_ERROR) {
    vpColVector weightPerPoint(static_cast<unsigned int>(m_controlPoints.size()));
    vpColVector errorPerPoint(static_cast<unsigned int>(m_controlPoints.size()));
    for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
      double sum = 0.0;
      double sumError = 0.0;
      for (unsigned int j = 0; j < nerror_per_point; ++j) {
        sum += m_weights[i * nerror_per_point + j];
        sumError += m_error[i * nerror_per_point + j];
      }
      weightPerPoint[i] = sum / nerror_per_point;
      errorPerPoint[i] = sumError / nerror_per_point;

    }
    double maxError = errorPerPoint.getMaxValue();
    unsigned idx = 0;
    for (const vpRBSilhouetteControlPoint &p : m_controlPoints) {
      vpColor c;
      c.R = static_cast<unsigned char>((errorPerPoint[idx] / maxError) * 255.0);
      c.G = 0;
      c.B = static_cast<unsigned char>(255.0 * weightPerPoint[idx]);
      // vpImagePoint diff(m_stats.nv[idx][1] * m_ccdParameters.h, m_stats.nv[idx][0] * m_ccdParameters.h);
      // vpImagePoint ip1 = p.icpoint - diff;
      // vpImagePoint ip2 = p.icpoint + diff;

      // vpDisplay::displayLine(IRGB, ip1, ip2, c, 1);
      vpDisplay::displayCross(IRGB, static_cast<int>(p.icpoint.get_i()), static_cast<int>(p.icpoint.get_j()), 3, c, 1);
      idx++;
    }
  }
  else {
    throw vpException(vpException::badValue, "Unknown display type");
  }
}

void vpRBSilhouetteCCDTracker::updateCCDPoints(const vpHomogeneousMatrix &cMo)
{
  for (vpRBSilhouetteControlPoint &p : m_controlPoints) {
    p.updateSilhouettePoint(cMo);
  }
}

bool extremitiesOutsideOfImage(int h, int w, unsigned int l, const double *nv_ptr, double i, double j)
{
  int id, jd;

  jd = static_cast<int>(round(j - l * nv_ptr[0]));
  id = static_cast<int>(round(i - l * nv_ptr[1]));

  if (id < 0 || jd < 0 || id >= h || jd >= w) {
    return true;
  }

  jd = static_cast<int>(round(j + l * nv_ptr[0]));
  id = static_cast<int>(round(i + l * nv_ptr[1]));

  if (id < 0 || jd < 0 || id >= h || jd >= w) {
    return true;
  }
  return false;
}

void vpRBSilhouetteCCDTracker::computeLocalStatistics(const vpImage<vpRGBa> &I, vpCCDStatistics &stats)
{

  const double minus_exp_gamma2 = exp(-m_ccdParameters.gamma_2);

  const double sigma = m_ccdParameters.h / (m_ccdParameters.alpha * m_ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = m_ccdParameters.gamma_3 * sigma + m_ccdParameters.gamma_4;
  unsigned int resolution = static_cast<unsigned int>(m_controlPoints.size());
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  vpMatrix normalized_param = vpMatrix(resolution, 2, 0.0);

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int kk = 0; kk < static_cast<int>(m_controlPoints.size()); kk++) {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    std::array<double, 2> pt1, pt2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    std::array<double, 2> dist1, dist2;

    vpRBSilhouetteControlPoint &p = m_controlPoints[kk];

    double *nv_ptr = stats.nv[kk];
    nv_ptr[0] = p.nxs;
    nv_ptr[1] = p.nys;

    int ccdh = m_ccdParameters.h;
    if (extremitiesOutsideOfImage(static_cast<int>(I.getHeight()), static_cast<int>(I.getWidth()), ccdh, nv_ptr, p.icpoint.get_i(), p.icpoint.get_j())) {
      p.setValid(false); // invalidate points that are too close to image border
    }

    if (!p.isValid()) {
      continue;
    }
#if VISP_DEBUG_CCD_TRACKER
    if (std::isnan(nv_ptr[0]) || std::isnan(nv_ptr[1])) {
      throw vpException(vpException::fatalError, "x: %f, theta = %f", p.xs, p.getTheta());
    }
#endif

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = stats.vic[kk];
    for (int j = m_ccdParameters.delta_h; j <= m_ccdParameters.h; j += m_ccdParameters.delta_h, k++) {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      pt1[0] = round(p.icpoint.get_u() + j * nv_ptr[0]);
      // y_{k,l}
      pt1[1] = round(p.icpoint.get_v() + j * nv_ptr[1]);
      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      dist1[0] = (pt1[0] - p.icpoint.get_u()) * nv_ptr[0] + (pt1[1] - p.icpoint.get_v()) * nv_ptr[1];
      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      dist1[1] = (pt1[0] - p.icpoint.get_u()) * nv_ptr[1] - (pt1[1] - p.icpoint.get_v()) * nv_ptr[0];
      vic_ptr[10 * k + 0] = pt1[1];
      vic_ptr[10 * k + 1] = pt1[0];
      vic_ptr[10 * k + 2] = dist1[0];
      vic_ptr[10 * k + 3] = dist1[1];

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10 * k + 4] = 0.5 * (erf((dist1[0]) / (sqrt(2) * sigma)) + 1.0);
      //vic_ptr[10*k + 4] = logistic(dist1[0]/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10 * k + 4] - m_ccdParameters.gamma_1) / (1 - m_ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10 * k + 5] = wp1 * wp1 * wp1 * wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1 - vic_ptr[10 * k + 4] - 0.25);
      vic_ptr[10 * k + 6] = -64 * wp2 * wp2 * wp2 * wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10 * k + 7] = std::max((exp(-0.5 * dist1[0] * dist1[0] / (sigma_hat * sigma_hat)) - minus_exp_gamma2), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[10 * k + 8] = 0.5 * exp(-abs(dist1[1]) / alpha) / alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[10 * k + 9] = exp(-dist1[0] * dist1[0] / (2 * sigma * sigma)) / (sqrt(2.0 * M_PI) * sigma);


      // calculate the normalization parameter c
      normalized_param[kk][0] += vic_ptr[10 * k + 7];

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      pt2[0] = round(p.icpoint.get_u() - j * nv_ptr[0]);
      pt2[1] = round(p.icpoint.get_v() - j * nv_ptr[1]);

      // cv::circle(canvas_tmp, cv::Point2d(pt2[0], pt2[1]), 1, CV_RGB(255,0,0), 1);#ifdef DEBUG

      // start compute the size in the direction of -(n_x, n_y)
      dist2[0] = (pt2[0] - p.icpoint.get_u()) * nv_ptr[0] + (pt2[1] - p.icpoint.get_v()) * nv_ptr[1];
      dist2[1] = (pt2[0] - p.icpoint.get_u()) * nv_ptr[1] - (pt2[1] - p.icpoint.get_v()) * nv_ptr[0];
      int negative_normal = k + static_cast<int>(floor(m_ccdParameters.h / m_ccdParameters.delta_h));
      vic_ptr[10 * negative_normal + 0] = pt2[1];
      vic_ptr[10 * negative_normal + 1] = pt2[0];
      vic_ptr[10 * negative_normal + 2] = dist2[0];
      vic_ptr[10 * negative_normal + 3] = dist2[1];
      vic_ptr[10 * negative_normal + 4] = 0.5 * (erf(dist2[0] / (sqrt(2) * sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(dist2[0]/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10 * negative_normal + 4] - 0.25);
      vic_ptr[10 * negative_normal + 5] = -64 * wp1 * wp1 * wp1 * wp1 + 0.25;
      wp2 = (1 - vic_ptr[10 * negative_normal + 4] - m_ccdParameters.gamma_1) / (1 - m_ccdParameters.gamma_1);
      vic_ptr[10 * negative_normal + 6] = wp2 * wp2 * wp2 * wp2;
      vic_ptr[10 * negative_normal + 7] = std::max((exp(-0.5 * dist2[0] * dist2[0] / (sigma_hat * sigma_hat)) - minus_exp_gamma2), 0.0);
      vic_ptr[10 * negative_normal + 8] = 0.5 * exp(-abs(dist2[0]) / alpha) / alpha;
      vic_ptr[10 * negative_normal + 9] = exp(-dist2[0] * dist2[0] / (2 * sigma * sigma)) / (sqrt(2 * M_PI) * sigma);
      normalized_param[kk][1] += vic_ptr[10 * negative_normal + 7];
    }
  }

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(resolution); ++i) {
    if (!m_controlPoints[i].isValid()) {
      continue;
    }

    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 = 0.0, w2 = 0.0;

    // store mean value near the curve
    std::array<double, 3> m1 { 0.0, 0.0, 0.0 }, m2 { 0.0, 0.0, 0.0 };

    // store the second mean value near the curve
    std::array<double, 9> m1_o2 { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::array<double, 9> m2_o2 { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = stats.vic[i];
    double *mean_vic_ptr = stats.mean_vic[i];
    double *cov_vic_ptr = stats.cov_vic[i];
    double *pix_ptr = stats.imgPoints[i];

    for (int j = m_ccdParameters.delta_h; j <= m_ccdParameters.h; j += m_ccdParameters.delta_h, k++) {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + static_cast<int>(floor(m_ccdParameters.h / m_ccdParameters.delta_h));
      const double *vic_k = vic_ptr + 10 * k;

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = (vic_k[5] * vic_k[7] / normalized_param[i][0]);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = (vic_k[6] * vic_k[7] / normalized_param[i][1]);
      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}
      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2
#if VISP_DEBUG_CCD_TRACKER
      if (vic_k[0] >= I.getHeight() || vic_k[1] >= I.getWidth()) {
        throw vpException(vpException::badValue, "Reading out of image");
      }
#endif

      const vpRGBa pixelRGBa = I(vic_k[0], vic_k[1]);
      double *pixel = pix_ptr + k * 3;
      pixel[0] = static_cast<double>(pixelRGBa.R);
      pixel[1] = static_cast<double>(pixelRGBa.G);
      pixel[2] = static_cast<double>(pixelRGBa.B);

      m1[0] += wp1 * pixel[0];
      m1[1] += wp1 * pixel[1];
      m1[2] += wp1 * pixel[2];

      m2[0] += wp2 * pixel[0];
      m2[1] += wp2 * pixel[1];
      m2[2] += wp2 * pixel[2];

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T
      for (unsigned int m = 0; m < 3; ++m) {
        for (unsigned int n = 0; n < 3; ++n) {
          m1_o2[m * 3 + n] += wp1 * pixel[m] * pixel[n];
          m2_o2[m * 3 + n] += wp2 * pixel[m] * pixel[n];
        }
      }
      const double *vic_neg = vic_ptr + 10 * negative_normal;
      const vpRGBa pixelNegRGBa = I(vic_neg[0], vic_neg[1]);
      double *pixelNeg = pix_ptr + negative_normal * 3;

      pixelNeg[0] = static_cast<double>(pixelNegRGBa.R);
      pixelNeg[1] = static_cast<double>(pixelNegRGBa.G);
      pixelNeg[2] = static_cast<double>(pixelNegRGBa.B);
      wp1 = (vic_neg[5] * vic_neg[7] / normalized_param[i][0]);
      wp2 = (vic_neg[6] * vic_neg[7] / normalized_param[i][1]);
      w1 += wp1;
      w2 += wp2;

      m1[0] += wp1 * pixelNeg[0];
      m1[1] += wp1 * pixelNeg[1];
      m1[2] += wp1 * pixelNeg[2];

      m2[0] += wp2 * pixelNeg[0];
      m2[1] += wp2 * pixelNeg[1];
      m2[2] += wp2 * pixelNeg[2];

      for (unsigned int m = 0; m < 3; ++m) {
        for (unsigned int n = 0; n < 3; ++n) {
          m1_o2[m * 3 + n] += wp1 * pixelNeg[m] * pixelNeg[n];
          m2_o2[m * 3 + n] += wp2 * pixelNeg[m] * pixelNeg[n];
        }
      }
    }
    mean_vic_ptr[0] = m1[0] / w1;
    mean_vic_ptr[1] = m1[1] / w1;
    mean_vic_ptr[2] = m1[2] / w1;

    mean_vic_ptr[3] = m2[0] / w2;
    mean_vic_ptr[4] = m2[1] / w2;
    mean_vic_ptr[5] = m2[2] / w2;

    for (unsigned int m = 0; m < 3; ++m) {
      for (unsigned int n = 0; n < 3; ++n) {
        cov_vic_ptr[m * 3 + n] = m1_o2[m * 3 + n] / w1 - m1[m] * m1[n] / (w1 * w1);
        cov_vic_ptr[9 + m * 3 + n] = m2_o2[m * 3 + n] / w2 - m2[m] * m2[n] / (w2 * w2);
      }
      cov_vic_ptr[m * 3 + m] += m_ccdParameters.kappa;
      cov_vic_ptr[9 + m * 3 + m] += m_ccdParameters.kappa;
    }
  }
}
template<bool hasTemporalSmoothing>
void vpRBSilhouetteCCDTracker::computeErrorAndInteractionMatrix()
{
  const unsigned int npointsccd = static_cast<unsigned int>(m_controlPoints.size());
  const unsigned int normal_points_number = static_cast<unsigned int>(floor(m_ccdParameters.h / m_ccdParameters.delta_h));
  const unsigned int nerror_ccd = 2 * normal_points_number * 3 * npointsccd;
  m_error.resize(nerror_ccd, false);
  m_weighted_error.resize(nerror_ccd, false);
  vpColVector errorPerPoint(m_controlPoints.size(), 0.0);
  m_L.resize(nerror_ccd, 6, false, false);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    // vpMatrix tmp_cov(3, 3);
    // vpMatrix tmp_cov_inv(3, 3);
    FastMat33<double> tmp_cov, tmp_cov_inv;
    FastMat63<double> tmp_jacobian;
    FastMat63<double> tmp_jacobian_x_tmp_cov_inv;
    FastVec3<double> tmp_pixel_diff;
    double Lnvp[6];

#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
    for (int i = 0; i < static_cast<int>(m_controlPoints.size()); i++) {
      const unsigned int ui = static_cast<unsigned int>(i);
      const vpRBSilhouetteControlPoint &p = m_controlPoints[ui];
      errorPerPoint[ui] = 0.0;
      if (!p.isValid()) {
        for (unsigned int j = 0; j < 2 * normal_points_number; ++j) {
          for (unsigned int m = 0; m < 3; ++m) {
            m_error[ui * 2 * normal_points_number * 3 + j * 3 + m] = 0.0;
          }
        }
        continue;
      }

      const double *vic_ptr = m_stats.vic[ui];
      const double *nv_ptr = m_stats.nv[ui];
      const double *mean_vic_ptr = m_stats.mean_vic[ui];
      const double *cov_vic_ptr = m_stats.cov_vic[ui];
      const double *pix_ptr = m_stats.imgPoints[ui];

      const double *mean_vic_ptr_prev = m_prevStats.mean_vic[ui];
      const double *cov_vic_ptr_prev = m_prevStats.cov_vic[ui];

      const vpCameraParameters &cam = p.getCameraParameters();

      Lnvp[0] = (-nv_ptr[0] / p.Zs);
      Lnvp[1] = (-nv_ptr[1] / p.Zs);
      Lnvp[2] = ((nv_ptr[0] * p.xs + nv_ptr[1] * p.ys) / p.Zs);
      Lnvp[3] = (nv_ptr[0] * p.xs * p.ys + nv_ptr[1] * (1.0 + p.ys * p.ys));
      Lnvp[4] = (-nv_ptr[1] * p.xs * p.ys - nv_ptr[0] * (1.0 + p.xs * p.xs));
      Lnvp[5] = (nv_ptr[0] * p.ys - nv_ptr[1] * p.xs);

      for (unsigned int j = 0; j < 2 * normal_points_number; ++j) {
        const double *vic_j = vic_ptr + 10 * j;
        const double *pix_j = pix_ptr + j * 3;
        const double errf = vic_j[4];
        const double smooth2 = m_temporalSmoothingFac * m_temporalSmoothingFac;
        double *error_ccd_j = m_error.data + i * 2 * normal_points_number * 3 + j * 3;

        for (unsigned n = 0; n < 9; ++n) {
          //double *tmp_cov_ptr = tmp_cov[m];
          tmp_cov[n] = errf * cov_vic_ptr[n] + (1.0 - errf) * cov_vic_ptr[n + 9];
          if constexpr (hasTemporalSmoothing) {
            tmp_cov[n] += smooth2 * (errf * cov_vic_ptr_prev[n] + (1.0 - errf) * cov_vic_ptr_prev[n + 9]);
          }
        }

        tmp_cov.inverse(tmp_cov_inv);

        //compute the difference between I_{kl} and \hat{I_{kl}}
        for (int m = 0; m < 3; ++m) {
          double err = (pix_j[m] - errf * mean_vic_ptr[m] - (1.0 - errf) * mean_vic_ptr[m + 3]);
          if constexpr (hasTemporalSmoothing) {
            err += m_temporalSmoothingFac * (pix_j[m] - errf * mean_vic_ptr_prev[m] - (1.0 - errf) * mean_vic_ptr_prev[m + 3]);
          }
          //error_ccd[i*2*normal_points_number*3 + j*3 + m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- errf * mean_vic_ptr[m]- (1-errf)* mean_vic_ptr[m+3];
          tmp_pixel_diff[m] = err;
          error_ccd_j[m] = err;
          errorPerPoint[ui] += err;
        }

        //compute jacobian matrix
        //memset(tmp_jacobian.data, 0, 3 * m_ccdParameters.phi_dim * sizeof(double));
        for (unsigned int n = 0; n < 3; ++n) {
          const double f = -cam.get_px() * (vic_j[9] * (mean_vic_ptr[n] - mean_vic_ptr[n + 3]));
          const double facPrev = hasTemporalSmoothing ? -cam.get_px() * m_temporalSmoothingFac * (vic_j[9] * (mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n + 3])) : 0.0;
          for (unsigned int dof = 0; dof < 6; ++dof) {
            tmp_jacobian.data[dof * 3 + n] = f * Lnvp[dof];
            if constexpr (hasTemporalSmoothing) {
              tmp_jacobian.data[dof * 3 + n] += facPrev * Lnvp[dof];
            }
          }
        }

        FastMat63<double>::multiply(tmp_jacobian, tmp_cov_inv, tmp_jacobian_x_tmp_cov_inv);
        //vpMatrix::mult2Matrices(tmp_jacobian, tmp_cov_inv, tmp_jacobian_x_tmp_cov_inv);

        FastVec3<double>::multiply(tmp_jacobian_x_tmp_cov_inv, tmp_pixel_diff, m_gradients[ui * 2 * normal_points_number + j]);
        FastMat63<double>::multiplyBTranspose(tmp_jacobian_x_tmp_cov_inv, tmp_jacobian, m_hessians[ui * 2 * normal_points_number + j]);
        // vpMatrix::mult2Matrices(tmp_jacobian_x_tmp_cov_inv, tmp_jacobian.t(), m_hessians[i * 2 * normal_points_number + j]);
      }
    }
  }

  //m_robust.setMinMedianAbsoluteDeviation(1.0);
  vpColVector weightPerPoint(errorPerPoint.getRows());

  m_robust.MEstimator(vpRobust::vpRobustEstimatorType::TUKEY, errorPerPoint, weightPerPoint);
  for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
    double w = m_controlPoints[i].isValid() ? weightPerPoint[i] : 0.0;
    for (unsigned int j = 0; j < static_cast<unsigned int>(2 * normal_points_number * 3); ++j) {
      m_weights[i * 2 * normal_points_number * 3 + j] = w;
    }
  }
  std::vector<vpColVector> gradientPerThread;
  std::vector<vpMatrix> hessianPerThread;
  m_gradient = 0.0;
  m_hessian = 0.0;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
#ifdef VISP_HAVE_OPENMP
#pragma omp single
    {
      unsigned int numThreads = omp_get_num_threads();
      gradientPerThread.resize(numThreads);
      hessianPerThread.resize(numThreads);
    }
#else
    {
      gradientPerThread.resize(1);
      hessianPerThread.resize(1);
    }
#endif

#ifdef VISP_HAVE_OPENMP
    unsigned int threadIdx = omp_get_thread_num();
#else
    unsigned int threadIdx = 0;
#endif
    vpColVector localGradient(m_gradient.getRows(), 0.0);
    vpMatrix localHessian(m_hessian.getRows(), m_hessian.getCols(), 0.0);

#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
    for (int ii = 0; ii < static_cast<int>(m_gradients.size()); ++ii) {
      const unsigned int i = static_cast<unsigned int>(ii);
      vpColVector &g = m_gradients[i];
      vpMatrix &h = m_hessians[i];
      double w = m_weights[i];
      if (w == 0.0) {
        for (unsigned int j = 0; j < 6; ++j) {
          m_L[i][j] = 0;
        }
      }
      else {
        for (unsigned int j = 0; j < 6; ++j) {
          g[j] *= w;
          m_L[i][j] *= w;
          localGradient[j] += g[j];
          for (unsigned int k = 0; k < 6; ++k) {
            h[j][k] *= w;
            localHessian[j][k] += h[j][k];
          }
        }
      }
    }
    {
      gradientPerThread[threadIdx] = localGradient;
      hessianPerThread[threadIdx] = localHessian;
    }
  }

  for (unsigned int i = 0; i < gradientPerThread.size(); ++i) {
    m_gradient += gradientPerThread[i];
    m_hessian += hessianPerThread[i];
  }

  m_LTL = m_hessian;
  m_LTR = -m_gradient;

  try {
    vpMatrix hessian_E_inv = m_hessian.inverseByCholesky();
    //m_sigma = /*m_sigma +*/ 2*hessian_E_inv;
    m_sigma = m_ccdParameters.covarianceIterDecreaseFactor * m_sigma + 2.0 * (1.0 - m_ccdParameters.covarianceIterDecreaseFactor) * hessian_E_inv;
  }
  catch (vpException &e) {
    std::cerr << "Inversion issues in CCD tracker" << std::endl;
    unsigned int nanGradients = 0, nanHessians = 0;
    for (unsigned int i = 0; i < m_gradients.size(); ++i) {
      nanGradients += static_cast<unsigned int>(!vpArray2D<double>::isFinite(m_gradients[i]));
      nanHessians += static_cast<unsigned int>(!vpArray2D<double>::isFinite(m_hessians[i]));

    }
    std::cerr << "Nan gradients: " << nanGradients << std::endl;
    std::cerr << "Nan hessians: " << nanHessians << std::endl;
    std::cerr << vpArray2D<double>::isFinite(m_hessian) << std::endl;
    std::cerr << vpArray2D<double>::isFinite(m_gradient) << std::endl;
    std::cerr << vpArray2D<double>::isFinite(m_error) << std::endl;
    std::cerr << vpArray2D<double>::isFinite(m_weights) << std::endl;


    m_numFeatures = 0;
    m_weighted_error = 0;
    m_LTL = 0;
    m_LTR = 0;

    std::cerr << e.what() << std::endl;
  }
}

END_VISP_NAMESPACE
