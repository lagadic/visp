
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

vpRBSilhouetteCCDTracker::vpRBSilhouetteCCDTracker() : vpRBFeatureTracker(), m_vvsConvergenceThreshold(0.0), m_temporalSmoothingFac(0.1)
{ }

void vpRBSilhouetteCCDTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix &cMo)
{
  m_controlPoints.clear();
  //m_controlPoints.reserve(frame.silhouettePoints.size());
  const vpHomogeneousMatrix oMc = cMo.inverse();
  for (const vpRBSilhouettePoint &sp : frame.silhouettePoints) {
    // std::cout << m_ccdParameters.h << std::endl;
    // std::cout << sp.j << ", " << sp.i << std::endl;
    int ii = sp.i, jj = sp.j;

    if (ii < 4 || jj < 4 || static_cast<unsigned int>(ii) > frame.I.getHeight() - 4 || static_cast<unsigned int>(jj) > frame.I.getWidth() - 4) {
      continue;
    }
    vpRBSilhouetteControlPoint pccd;
    pccd.buildSilhouettePoint(ii, jj, sp.Z, sp.orientation, sp.normal, cMo, oMc, frame.cam);

    pccd.detectSilhouette(frame.renders.depth);
    if (pccd.isSilhouette() && !std::isnan(sp.orientation) && pccd.isValid()) {
      m_controlPoints.push_back(std::move(pccd));
    }
  }
}



void vpRBSilhouetteCCDTracker::initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix & /*cMo*/)
{
  // Reinit all variables
  m_sigma = vpMatrix(m_ccdParameters.phi_dim, m_ccdParameters.phi_dim, 0.0);
  m_cov.resize(6, 6);
  tol = 0.0;
  m_vvsConverged = false;

  unsigned int resolution = m_controlPoints.size();
  int normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  unsigned nerror_ccd = 2 * normal_points_number * 3 * resolution;
  m_numFeatures = nerror_ccd;

  m_stats.reinit(resolution, normal_points_number);
  m_prevStats.reinit(resolution, normal_points_number);
  m_gradient = vpMatrix(m_ccdParameters.phi_dim, 1, 0.0);
  m_hessian = vpMatrix(m_ccdParameters.phi_dim, m_ccdParameters.phi_dim, 0.0);
  m_gradients.resize(m_controlPoints.size() * 2 * normal_points_number, vpColVector(m_gradient.getRows()));
  m_hessians.resize(m_controlPoints.size() * 2 * normal_points_number, vpMatrix(m_hessian.getRows(), m_hessian.getCols()));


  //m_weights.resize(nerror_ccd);
  m_weights.resize(m_numFeatures, false);
  // m_weights = 1;
  // computeMask(frame.renders.color, m_stats);
  // computeMask(frame.renders.color, m_prevStats);
  // m_stats.weight = 1.0;
  // m_prevStats.weight = 1.0;
  computeLocalStatistics(previousFrame.IRGB, m_prevStats);
  //computeLocalStatistics(image, m_stats);
}

void vpRBSilhouetteCCDTracker::computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration)
{
  vpColVector oldPoints(m_controlPoints.size() * 2);
  for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
    oldPoints[i * 2] = m_controlPoints[i].icpoint.get_u();
    oldPoints[i * 2 + 1] = m_controlPoints[i].icpoint.get_v();
  }
  updateCCDPoints(cMo);

  tol = 0.0;
  for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
    tol += abs(oldPoints[i * 2] - m_controlPoints[i].icpoint.get_u());
    tol += abs(oldPoints[i * 2 + 1] - m_controlPoints[i].icpoint.get_v());
  }
  tol /= m_controlPoints.size();
  computeLocalStatistics(frame.IRGB, m_stats);
  computeErrorAndInteractionMatrix(); // Update interaction matrix, and gauss newton left and right side terms

  m_vvsConverged = false;
  if (iteration > 0 && tol < m_vvsConvergenceThreshold) {
    m_vvsConverged = true;
  }
}

void vpRBSilhouetteCCDTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &/*I*/, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &/*depth*/, const vpRBFeatureDisplayType type) const
{
  unsigned normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  unsigned nerror_per_point = 2 * normal_points_number * 3;
  if (type == vpRBFeatureDisplayType::SIMPLE) {

    for (unsigned int i = 0; i < m_controlPoints.size(); ++i) {
      const vpRBSilhouetteControlPoint &p = m_controlPoints[i];
      vpDisplay::displayCross(IRGB, p.icpoint.get_i(), p.icpoint.get_j(), 3, vpColor::green, 1);
      vpImagePoint diff(m_stats.nv[i][1] * m_ccdParameters.h, m_stats.nv[i][0] * m_ccdParameters.h);

      // vpImagePoint ip2 = p.icpoint + diff;
      // vpDisplay::displayArrow(IRGB, p.icpoint, ip2, p.invnormal ? vpColor::red : vpColor::lightBlue);
    }
  }
  else if (type == vpRBFeatureDisplayType::ERROR) {
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
    unsigned idx = 0;
    for (const vpRBSilhouetteControlPoint &p : m_controlPoints) {
      const double weight = errorPerPoint[idx] / maxPointError;
      const double diffR = (double)(worstColor.R) - (double)(bestColor.R);
      const double diffG = (double)(worstColor.G) - (double)(bestColor.G);
      const double diffB = (double)(worstColor.B) - (double)(bestColor.B);

      vpColor c;
      c.R = (unsigned char)((double)(bestColor.R) + diffR * weight);
      c.G = (unsigned char)((double)(bestColor.G) + diffG * weight);
      c.B = (unsigned char)((double)(bestColor.B) + diffB * weight);

      vpDisplay::displayCross(IRGB, p.icpoint.get_i(), p.icpoint.get_j(), 3, c, 1);
      ++idx;
    }
  }
  else if (type == vpRBFeatureDisplayType::IMPORTANCE) {
    vpColVector weightPerPoint(m_controlPoints.size());
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
      c.G = (unsigned char)(255.f * weight);
      c.B = 0;

      vpDisplay::displayCross(IRGB, p.icpoint.get_i(), p.icpoint.get_j(), 3, c, 1);
      idx++;
    }
  }
  else {
    throw vpException(vpException::badValue, "Unknown display type");
  }
}

void vpRBSilhouetteCCDTracker::updateCCDPoints(const vpHomogeneousMatrix &cMo)
{
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (vpRBSilhouetteControlPoint &p : m_controlPoints) {
    p.updateSilhouettePoint(cMo);
  }
}





void vpRBSilhouetteCCDTracker::computeLocalStatistics(const vpImage<vpRGBa> &I, vpCCDStatistics &stats)
{

  const double minus_exp_gamma2 = exp(-m_ccdParameters.gamma_2);

  const double sigma = m_ccdParameters.h / (m_ccdParameters.alpha * m_ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = m_ccdParameters.gamma_3 * sigma + m_ccdParameters.gamma_4;
  unsigned int resolution = m_controlPoints.size();
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  vpMatrix normalized_param = vpMatrix(resolution, 2, 0.0);

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int kk = 0; kk < m_controlPoints.size(); kk++) {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    std::array<double, 2> pt1, pt2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    std::array<double, 2> dist1, dist2;

    vpRBSilhouetteControlPoint &p = m_controlPoints[kk];
    int ccdh = m_ccdParameters.h;
    if (p.icpoint.get_i() <= ccdh || p.icpoint.get_i() > (I.getHeight() - ccdh) || p.icpoint.get_j() <= ccdh || p.icpoint.get_j() > (I.getWidth() - ccdh)) {
      p.setValid(false); // invalidate points that are too close to image border
    }

    if (!p.isValid()) {
      continue;
    }
    double *nv_ptr = stats.nv[kk];
    nv_ptr[0] = p.nxs;
    nv_ptr[1] = p.nys;
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
      //std::cout << tmp1 << std::endl;

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
      int negative_normal = k + (int)floor(m_ccdParameters.h / m_ccdParameters.delta_h);
      vic_ptr[10 * negative_normal + 0] = pt2[1];
      vic_ptr[10 * negative_normal + 1] = pt2[0];
      vic_ptr[10 * negative_normal + 2] = dist2[0];
      vic_ptr[10 * negative_normal + 3] = dist2[1];
      //std::cout << " u " <<  p.icpoint.get_u() <<  " v " << p.icpoint.get_v() << " dist " << dist2[0] << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p.get_theta() << std::endl;
      vic_ptr[10 * negative_normal + 4] = 0.5 * (erf(dist2[0] / (sqrt(2) * sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(dist2[0]/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10 * negative_normal + 4] - 0.25);
      vic_ptr[10 * negative_normal + 5] = -64 * wp1 * wp1 * wp1 * wp1 + 0.25;
      wp2 = (1 - vic_ptr[10 * negative_normal + 4] - m_ccdParameters.gamma_1) / (1 - m_ccdParameters.gamma_1);
      vic_ptr[10 * negative_normal + 6] = wp2 * wp2 * wp2 * wp2;
      vic_ptr[10 * negative_normal + 7] = std::max((exp(-0.5 * dist2[0] * dist2[0] / (sigma_hat * sigma_hat)) - minus_exp_gamma2), 0.0);
      vic_ptr[10 * negative_normal + 8] = 0.5 * exp(-abs(dist2[0]) / alpha) / alpha;
      vic_ptr[10 * negative_normal + 9] = exp(-dist2[0] * dist2[0] / (2 * sigma * sigma)) / (sqrt(2 * CV_PI) * sigma);
      normalized_param[kk][1] += vic_ptr[10 * negative_normal + 7];
    }

  }
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < resolution; ++i) {
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
      int negative_normal = k + (int)floor(m_ccdParameters.h / m_ccdParameters.delta_h);
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
      const vpRGBa pixelRGBa = I(vic_k[0], vic_k[1]);
      double *pixel = pix_ptr + k * 3;
      pixel[0] = pixelRGBa.R;
      pixel[1] = pixelRGBa.G;
      pixel[2] = pixelRGBa.B;

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

      pixelNeg[0] = pixelNegRGBa.R;
      pixelNeg[1] = pixelNegRGBa.G;
      pixelNeg[2] = pixelNegRGBa.B;
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

void vpRBSilhouetteCCDTracker::computeErrorAndInteractionMatrix()
{
  const int npointsccd = m_controlPoints.size();
  const int normal_points_number = floor(m_ccdParameters.h / m_ccdParameters.delta_h);
  const int nerror_ccd = 2 * normal_points_number * 3 * npointsccd;
  m_error.resize(nerror_ccd, false);
  m_weighted_error.resize(nerror_ccd, false);
  m_L.resize(nerror_ccd, 6, false, false);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {

    // vpMatrix tmp_cov(3, 3);
    // vpMatrix tmp_cov_inv(3, 3);
    FastMat33<double> tmp_cov, tmp_cov_inv;
    vpMatrix tmp_jacobian(m_ccdParameters.phi_dim, 3);
    vpMatrix tmp_jacobian_x_tmp_cov_inv(tmp_jacobian.getRows(), 3);
    vpColVector tmp_pixel_diff(3);
    double Lnvp[6];
    unsigned int normal_points_number = static_cast<unsigned int>(floor(m_ccdParameters.h / m_ccdParameters.delta_h));

#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
    for (unsigned int kk = 0; kk < m_controlPoints.size(); kk++) {
      const int i = kk;
      const vpRBSilhouetteControlPoint &p = m_controlPoints[kk];

      if (!p.isValid()) {
        for (unsigned int j = 0; j < 2 * normal_points_number; ++j) {
          for (unsigned int m = 0; m < 3; ++m) {
            m_error[i * 2 * normal_points_number * 3 + j * 3 + m] = 0.0;
          }
        }
        continue;
      }

      const double *vic_ptr = m_stats.vic[i];
      const double *nv_ptr = m_stats.nv[i];
      const double *mean_vic_ptr = m_stats.mean_vic[i];
      const double *cov_vic_ptr = m_stats.cov_vic[i];
      const double *pix_ptr = m_stats.imgPoints[i];

      const double *mean_vic_ptr_prev = m_prevStats.mean_vic[i];
      const double *cov_vic_ptr_prev = m_prevStats.cov_vic[i];
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
          tmp_cov[n] = errf * cov_vic_ptr[n] + (1.0 - errf) * cov_vic_ptr[n + 9]
            + smooth2 * (errf * cov_vic_ptr_prev[n] + (1.0 - errf) * cov_vic_ptr_prev[n + 9]);
        }

        tmp_cov.inverse(tmp_cov_inv);

        //compute the difference between I_{kl} and \hat{I_{kl}}
        for (int m = 0; m < 3; ++m) {
          double err = (pix_j[m] - errf * mean_vic_ptr[m] - (1.0 - errf) * mean_vic_ptr[m + 3])
            + m_temporalSmoothingFac * (pix_j[m] - errf * mean_vic_ptr_prev[m] - (1.0 - errf) * mean_vic_ptr_prev[m + 3]);
          tmp_pixel_diff[m] = err;
            //error_ccd[i*2*normal_points_number*3 + j*3 + m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- errf * mean_vic_ptr[m]- (1-errf)* mean_vic_ptr[m+3];
          error_ccd_j[m] = err;
        }

        //compute jacobian matrix
        //memset(tmp_jacobian.data, 0, 3 * m_ccdParameters.phi_dim * sizeof(double));
        for (int n = 0; n < 3; ++n) {
          const double f = -cam.get_px() * (vic_j[9] * (mean_vic_ptr[n] - mean_vic_ptr[n + 3]));
          const double facPrev = -cam.get_px() * m_temporalSmoothingFac * (vic_j[9] * (mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n + 3]));
          for (unsigned int dof = 0; dof < 6; ++dof) {
            tmp_jacobian[dof][n] = f * Lnvp[dof] + facPrev * Lnvp[dof];
          }
        }

        FastMat33<double>::multiply(tmp_jacobian, tmp_cov_inv, tmp_jacobian_x_tmp_cov_inv);
        //vpMatrix::mult2Matrices(tmp_jacobian, tmp_cov_inv, tmp_jacobian_x_tmp_cov_inv);
        vpMatrix::mult2Matrices(tmp_jacobian_x_tmp_cov_inv, tmp_pixel_diff, m_gradients[i * 2 * normal_points_number + j]);
        vpMatrix::mult2Matrices(tmp_jacobian_x_tmp_cov_inv, tmp_jacobian.t(), m_hessians[i * 2 * normal_points_number + j]);

      }
    }
  }

  m_gradient = 0.0;
  m_hessian = 0.0;
  //m_robust.setMinMedianAbsoluteDeviation(1.0);
  m_robust.MEstimator(vpRobust::vpRobustEstimatorType::TUKEY, m_error, m_weights);

  for (unsigned int i = 0; i < m_L.getRows(); ++i) {
    m_weighted_error[i] = m_error[i] * m_weights[i];
    for (unsigned int j = 0; j < 6; ++j) {
      m_L[i][j] *= m_weights[i];
    }
  }

  // Store all the gradients and hessians and then sum them up after the parallel region. This ensures that computation is determinist
  std::vector<vpColVector> localGradients;
  std::vector<vpMatrix> localHessians;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    vpColVector localGradient(m_gradient.getRows(), 0.0);
    vpMatrix localHessian(m_hessian.getRows(), m_hessian.getCols(), 0.0);
#ifdef VISP_HAVE_OPENMP
#pragma omp single
#endif
    {
#ifdef VISP_HAVE_OPENMP
      unsigned int threads = omp_get_num_threads();
#else
      unsigned int threads = 1;
#endif
      localGradients.resize(threads, localGradient);
      localHessians.resize(threads, localHessian);
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp for schedule(static)
#endif
    for (unsigned int i = 0; i < m_gradients.size(); ++i) {
      m_gradients[i] *= m_weights[i];
      m_hessians[i] *= m_weights[i];
      localHessian += m_hessians[i];
      localGradient += m_gradients[i];
    }
#ifdef VISP_HAVE_OPENMP
    unsigned int currentThread = omp_get_thread_num();
#else
    unsigned int currentThread = 0;
#endif
    localGradients[currentThread] = localGradient;
    localHessians[currentThread] = localHessian;
  }
  for (unsigned int i = 0; i < localGradients.size(); ++i) {
    m_gradient += localGradients[i];
    m_hessian += localHessians[i];
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
  }

}

END_VISP_NAMESPACE
