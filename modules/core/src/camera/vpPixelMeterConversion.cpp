/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Pixel to meter conversion.
 */

/*!
  \file vpPixelMeterConversion.cpp
  \brief Pixel to meter conversion.
*/
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPixelMeterConversion.h>

#if defined(HAVE_OPENCV_IMGPROC) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_CALIB3D)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_CALIB) && defined(HAVE_OPENCV_3D)))

#if (VISP_HAVE_OPENCV_VERSION < 0x050000)
#include <opencv2/calib3d/calib3d.hpp>
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x050000)
#include <opencv2/calib.hpp>
#include <opencv2/3d.hpp>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * Convert ellipse parameters (ie ellipse center and normalized centered moments)
 * from pixels \f$(u_c, v_c, n_{{20}_p}, n_{{11}_p}, n_{{02}_p})\f$
 * to meters \f$(x_c, y_c, n_{{20}_m}, n_{{11}_m}, n_{{02}_m})\f$ in the image plane.
 * \param[in] cam : Camera parameters.
 * \param[in] center_p : Center of the ellipse (uc, vc) with pixel coordinates.
 * \param[in] n20_p : Normalized second order moments of the ellipse in pixels.
 * \param[in] n11_p : Normalized second order moments of the ellipse in pixels.
 * \param[in] n02_p : Normalized second order moments of the ellipse in pixels.
 * \param[out] xc_m : Center of the ellipse along x-axis with coordinates in meters in the image plane.
 * \param[out] yc_m : Center of the ellipse along y-axis with coordinates in meters in the image plane.
 * \param[out] n20_m : Normalized second order moments of the ellipse in meters in the image plane.
 * \param[out] n11_m : Normalized second order moments of the ellipse in meters in the image plane.
 * \param[out] n02_m : Normalized second order moments of the ellipse in meters in the image plane.
 */
  void vpPixelMeterConversion::convertEllipse(const vpCameraParameters &cam, const vpImagePoint &center_p, double n20_p,
                                              double n11_p, double n02_p, double &xc_m, double &yc_m, double &n20_m,
                                              double &n11_m, double &n02_m)
{
  vpPixelMeterConversion::convertPoint(cam, center_p, xc_m, yc_m);
  double px = cam.get_px();
  double py = cam.get_py();

  n20_m = n20_p / (px * px);
  n11_m = n11_p / (px * py);
  n02_m = n02_p / (py * py);
}

/*!
  Line parameters conversion from pixel \f$(\rho_p,\theta_p)\f$ to normalized coordinates \f$(\rho_m,\theta_m)\f$
  in meter using ViSP camera parameters. This function doesn't use distortion coefficients.

  \param[in] cam : camera parameters.
  \param[in] rho_p : Line \f$ \rho \f$ parameter expressed in pixels.
  \param[in] theta_p : Line \f$ \theta \f$ parameter expressed in pixels.
  \param[out] rho_m : Line \f$ \rho \f$ parameter expressed in meters in the image plane.
  \param[out] theta_m : Line \f$ \theta \f$ parameter expressed in meters in the image plane.

*/
void vpPixelMeterConversion::convertLine(const vpCameraParameters &cam, const double &rho_p, const double &theta_p,
                                         double &rho_m, double &theta_m)
{
  double co = cos(theta_p);
  double si = sin(theta_p);
  double d = vpMath::sqr(cam.m_px * co) + vpMath::sqr(cam.m_py * si);

  if (fabs(d) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }
  theta_m = atan2(si * cam.m_py, co * cam.m_px);
  rho_m = (rho_p - (cam.m_u0 * co) - (cam.m_v0 * si)) / sqrt(d);
}

/*!
  Moments conversion from pixel to normalized coordinates in meter using ViSP camera parameters.
  This function doesn't use distortion coefficients.

  \param[in] cam : camera parameters.
  \param[in] order : Moment order.
  \param[in] moment_pixel : Moment values in pixels.
  \param[out] moment_meter : Moment values in meters in the image plane.

  The following example show how to use this function.
  \code
  unsigned int order = 3;
  vpMatrix M_p(order, order); // 3-by-3 matrix with mij moments expressed in pixels
  M_p = 0;
  vpMatrix M_m(order, order); // 3-by-3 matrix with mij moments expressed in meters
  M_m = 0;

  // Fill input Matrix with mij moments in pixels
  M_p[0][0] = m00_p;
  M_p[1][0] = m10_p;
  M_p[0][1] = m01_p;
  M_p[2][0] = m20_p;
  M_p[1][1] = m11_p;
  M_p[0][2] = m02_p;

  vpPixelMeterConversion::convertMoment(cam, order, M_p, M_m);

  // Moments mij in meters
  double m00 = M_m[0][0];
  double m01 = M_m[0][1];
  double m10 = M_m[1][0];
  double m02 = M_m[0][2];
  double m11 = M_m[1][1];
  double m20 = M_m[2][0];
  \endcode

*/
void vpPixelMeterConversion::convertMoment(const vpCameraParameters &cam, unsigned int order,
                                           const vpMatrix &moment_pixel, vpMatrix &moment_meter)
{
  vpMatrix m(order, order);
  double yc = -cam.m_v0;
  double xc = -cam.m_u0;

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {  // iteration en X
      for (unsigned int q = 0; q < order; ++q) { // iteration en Y
        if ((p + q) == k) {                       // on est bien dans la matrice triangulaire superieure
          m[p][q] = 0;                            // initialisation e 0
          for (unsigned int r = 0; r <= p; ++r) {  // somme externe
            for (unsigned int t = 0; t <= q; ++t) { // somme interne
              m[p][q] += static_cast<double>(vpMath::comb(p, r)) * static_cast<double>(vpMath::comb(q, t)) *
                pow(xc, static_cast<int>(p - r)) * pow(yc, static_cast<int>(q - t)) * moment_pixel[r][t];
            }
          }
        }
      }
    }
  }

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {
      for (unsigned int q = 0; q < order; ++q) {
        if ((p + q) == k) {
          m[p][q] *= pow(cam.m_inv_px, static_cast<int>(1 + p)) * pow(cam.m_inv_py, static_cast<int>(1 + q));
        }
      }
    }
  }

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {
      for (unsigned int q = 0; q < order; ++q) {
        if ((p + q) == k) {
          moment_meter[p][q] = m[p][q];
        }
      }
    }
  }
}

#if defined(HAVE_OPENCV_IMGPROC) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_CALIB3D)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_CALIB) && defined(HAVE_OPENCV_3D)))
/*!
 * Convert ellipse parameters (ie ellipse center and normalized centered moments)
 * from pixels \f$(u_c, v_c, n_{{20}_p}, n_{{11}_p}, n_{{02}_p})\f$
 * to meters \f$(x_c, y_c, n_{{20}_m}, n_{{11}_m}, n_{{02}_m})\f$ in the image plane.
 * \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
 * \param[in] distCoeffs : Input vector of distortion coefficients \f$(k_1, k_2, p_1, p_2[, k_3[, k_4,
 * k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of 4, 5, 8, 12 or 14 elements. If the vector is nullptr/empty,
 * the zero distortion coefficients are assumed.
 * \param[in] center_p : Center of the ellipse (uc, vc) with pixel coordinates.
 * \param[in] n20_p : Normalized second order moments of the ellipse in pixels.
 * \param[in] n11_p : Normalized second order moments of the ellipse in pixels.
 * \param[in] n02_p : Normalized second order moments of the ellipse in pixels.
 * \param[out] xc_m : Center of the ellipse along x-axis with coordinates in meters in the image plane.
 * \param[out] yc_m : Center of the ellipse along y-axis with coordinates in meters in the image plane.
 * \param[out] n20_m : Normalized second order moments of the ellipse in meters in the image plane.
 * \param[out] n11_m : Normalized second order moments of the ellipse in meters in the image plane.
 * \param[out] n02_m : Normalized second order moments of the ellipse in meters in the image plane.
 */
void vpPixelMeterConversion::convertEllipse(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                            const vpImagePoint &center_p, double n20_p, double n11_p, double n02_p,
                                            double &xc_m, double &yc_m, double &n20_m, double &n11_m, double &n02_m)
{
  vpPixelMeterConversion::convertPoint(cameraMatrix, distCoeffs, center_p, xc_m, yc_m);
  double px = cameraMatrix.at<double>(0, 0);
  double py = cameraMatrix.at<double>(1, 1);

  n20_m = n20_p / (px * px);
  n11_m = n11_p / (px * py);
  n02_m = n02_p / (py * py);
}

/*!
  Line parameters conversion from pixel \f$(\rho_p,\theta_p)\f$ to normalized coordinates \f$(\rho_m,\theta_m)\f$
  in meter using OpenCV camera parameters. This function doesn't use distortion coefficients.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] rho_p : Line \f$ \rho \f$ parameter expressed in pixels.
  \param[in] theta_p : Line \f$ \theta \f$ parameter expressed in pixels.
  \param[out] rho_m : Line \f$ \rho \f$ parameter expressed in meters in the image plane.
  \param[out] theta_m : Line \f$ \theta \f$ parameter expressed in meters in the image plane.

*/
void vpPixelMeterConversion::convertLine(const cv::Mat &cameraMatrix, const double &rho_p, const double &theta_p,
                                         double &rho_m, double &theta_m)
{
  double co = cos(theta_p);
  double si = sin(theta_p);
  double px = cameraMatrix.at<double>(0, 0);
  double py = cameraMatrix.at<double>(1, 1);
  double u0 = cameraMatrix.at<double>(0, 2);
  double v0 = cameraMatrix.at<double>(1, 2);

  double d = vpMath::sqr(px * co) + vpMath::sqr(py * si);

  if (fabs(d) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }
  theta_m = atan2(si * py, co * px);
  rho_m = (rho_p - u0 * co - v0 * si) / sqrt(d);
}

/*!
  Moments conversion from pixel to normalized coordinates in meter using OpenCV camera parameters.
  This function doesn't use distortion coefficients.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] order : Moment order.
  \param[in] moment_pixel : Moment values in pixels.
  \param[out] moment_meter : Moment values in meters in the image plane.

*/
void vpPixelMeterConversion::convertMoment(const cv::Mat &cameraMatrix, unsigned int order,
                                           const vpMatrix &moment_pixel, vpMatrix &moment_meter)
{
  double inv_px = 1. / cameraMatrix.at<double>(0, 0);
  double inv_py = 1. / cameraMatrix.at<double>(1, 1);
  double u0 = cameraMatrix.at<double>(0, 2);
  double v0 = cameraMatrix.at<double>(1, 2);

  vpMatrix m(order, order);
  double yc = -v0;
  double xc = -u0;

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {  // iteration en X
      for (unsigned int q = 0; q < order; ++q) { // iteration en Y
        if (p + q == k) {                        // on est bien dans la matrice triangulaire superieure
          m[p][q] = 0;                            // initialisation e 0
          for (unsigned int r = 0; r <= p; ++r) {  // somme externe
            for (unsigned int t = 0; t <= q; ++t) { // somme interne
              m[p][q] += static_cast<double>(vpMath::comb(p, r)) * static_cast<double>(vpMath::comb(q, t)) *
                pow(xc, static_cast<int>(p - r)) * pow(yc, static_cast<int>(q - t)) * moment_pixel[r][t];
            }
          }
        }
      }
    }
  }

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {
      for (unsigned int q = 0; q < order; ++q) {
        if (p + q == k) {
          m[p][q] *= pow(inv_px, static_cast<int>(1 + p)) * pow(inv_py, static_cast<int>(1 + q));
        }
      }
    }
  }

  for (unsigned int k = 0; k < order; ++k) { // iteration corresponding to moment order
    for (unsigned int p = 0; p < order; ++p) {
      for (unsigned int q = 0; q < order; ++q) {
        if (p + q == k) {
          moment_meter[p][q] = m[p][q];
        }
      }
    }
  }
}

/*!
  Point coordinates conversion from pixel coordinates
  \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is nullptr/empty, the zero distortion coefficients are assumed.
  \param[in] u : input coordinate in pixels along image horizontal axis.
  \param[in] v : input coordinate in pixels along image vertical axis.
  \param[out] x : output coordinate in meter along image plane x-axis.
  \param[out] y : output coordinate in meter along image plane y-axis.

*/
void vpPixelMeterConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const double &u,
                                          const double &v, double &x, double &y)
{
  std::vector<cv::Point2d> imagePoints_vec;
  imagePoints_vec.push_back(cv::Point2d(u, v));
  std::vector<cv::Point2d> objectPoints_vec;
  cv::undistortPoints(imagePoints_vec, objectPoints_vec, cameraMatrix, distCoeffs);
  x = objectPoints_vec[0].x;
  y = objectPoints_vec[0].y;
}

/*!
  Point coordinates conversion from pixel coordinates
  \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is nullptr/empty, the zero distortion coefficients are assumed.
  \param[in] iP : input coordinates in pixels.
  \param[out] x : output coordinate in meter along image plane x-axis.
  \param[out] y : output coordinate in meter along image plane y-axis.

*/
void vpPixelMeterConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                          const vpImagePoint &iP, double &x, double &y)
{
  std::vector<cv::Point2d> imagePoints_vec;
  imagePoints_vec.push_back(cv::Point2d(iP.get_u(), iP.get_v()));
  std::vector<cv::Point2d> objectPoints_vec;
  cv::undistortPoints(imagePoints_vec, objectPoints_vec, cameraMatrix, distCoeffs);
  x = objectPoints_vec[0].x;
  y = objectPoints_vec[0].y;
}

#endif
END_VISP_NAMESPACE
