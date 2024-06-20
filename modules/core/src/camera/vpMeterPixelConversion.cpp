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
 *
 * Description:
 * Meter to pixel conversion.
 *
*****************************************************************************/

/*!
  \file vpMeterPixelConversion.cpp
  \brief meter to pixel conversion
*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>

BEGIN_VISP_NAMESPACE
/*!
  Line parameters conversion from normalized coordinates \f$(\rho_m,\theta_m)\f$ expressed in the image plane
  to pixel coordinates \f$(\rho_p,\theta_p)\f$ using ViSP camera parameters. This function doesn't use distortion
  coefficients.

  \param[in] cam : camera parameters.
  \param[in] rho_p, theta_p : Line parameters expressed in pixels.
  \param[out] rho_m, theta_m : Line parameters expressed in meters in the image plane.
*/
void vpMeterPixelConversion::convertLine(const vpCameraParameters &cam, const double &rho_m, const double &theta_m,
                                         double &rho_p, double &theta_p)
{
  double co = cos(theta_m);
  double si = sin(theta_m);
  double d = sqrt(vpMath::sqr(cam.m_py * co) + vpMath::sqr(cam.m_px * si));

  if (fabs(d) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }

  theta_p = atan2(cam.m_px * si, cam.m_py * co);
  rho_p = ((cam.m_px * cam.m_py * rho_m) + (cam.m_u0 * cam.m_py * co) + (cam.m_v0 * cam.m_px * si));
  rho_p /= d;
}

/*!
  Noting that the perspective projection of a 3D circle is usually an ellipse, using the camera intrinsic parameters
  converts the parameters of the 3D circle expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D circle) in the image with values in pixels using ViSP camera parameters.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cam : Intrinsic camera parameters.
  \param[in] circle : 3D circle with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internally updated after perspective projection of the sphere.
  \param[out] center_p : Center \f$(u_c, v_c)\f$ of the corresponding ellipse in the image with coordinates expressed in
  pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its area (i.e.,
  such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in pixels.

  The following code shows how to use this function:
  \code
  vpCircle circle;
  double n20_p, n11_p, n02_p;
  circle.changeFrame(cMo);
  circle.projection();
  vpMeterPixelConversion::convertEllipse(cam, circle, center_p, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center_p, n20_p, n11_p, n02_p, true, vpColor::red);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam, const vpCircle &circle,
                                            vpImagePoint &center_p, double &n20_p, double &n11_p, double &n02_p)
{
  // Get the parameters of the ellipse in the image plane
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  double xc_m = circle.p[index_0];
  double yc_m = circle.p[index_1];
  double n20_m = circle.p[index_2];
  double n11_m = circle.p[index_3];
  double n02_m = circle.p[index_4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cam, xc_m, yc_m, center_p);
  n20_p = n20_m * vpMath::sqr(cam.get_px());
  n11_p = n11_m * cam.get_px() * cam.get_py();
  n02_p = n02_m * vpMath::sqr(cam.get_py());
}

/*!
  Noting that the perspective projection of a 3D sphere is usually an ellipse, using the camera intrinsic parameters
  converts the parameters of the 3D sphere expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D sphere) in the image with values in pixels.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cam : Intrinsic camera parameters.
  \param[in] sphere : 3D sphere with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internally updated after perspective projection of the sphere.
  \param[out] center_p : Center \f$(u_c, v_c)\f$ of the corresponding ellipse in the image with coordinates expressed in
  pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its area (i.e.,
  such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in pixels.

  The following code shows how to use this function:
  \code
  vpSphere sphere;
  double n20_p, n11_p, n02_p;
  sphere.changeFrame(cMo);
  sphere.projection();
  vpMeterPixelConversion::convertEllipse(cam, sphere, center_p, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center_p, n20_p, n11_p, n02_p, true, vpColor::red);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam, const vpSphere &sphere,
                                            vpImagePoint &center_p, double &n20_p, double &n11_p, double &n02_p)
{
  // Get the parameters of the ellipse in the image plane
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  double xc_m = sphere.p[index_0];
  double yc_m = sphere.p[index_1];
  double n20_m = sphere.p[index_2];
  double n11_m = sphere.p[index_3];
  double n02_m = sphere.p[index_4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cam, xc_m, yc_m, center_p);
  n20_p = n20_m * vpMath::sqr(cam.get_px());
  n11_p = n11_m * cam.get_px() * cam.get_py();
  n02_p = n02_m * vpMath::sqr(cam.get_py());
}

/*!
  Convert parameters of an ellipse expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D sphere) in the image with values in pixels using ViSP intrinsic camera parameters.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cam : Intrinsic camera parameters.
  \param[in] xc_m, yc_m : Center of the ellipse in the image plane with normalized coordinates expressed in meters.
  \param[in] n20_m, n11_m, n02_m : Second order centered moments of the ellipse normalized by its area
  (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in
  meter.
  \param[out] center_p : Center \f$(u_c, v_c)\f$ of the corresponding ellipse in the image with coordinates
  expressed in pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its
  area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area)
  expressed in pixels.

 */
void vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam, double xc_m, double yc_m, double n20_m,
                                            double n11_m, double n02_m, vpImagePoint &center_p, double &n20_p,
                                            double &n11_p, double &n02_p)
{
  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cam, xc_m, yc_m, center_p);
  n20_p = n20_m * vpMath::sqr(cam.get_px());
  n11_p = n11_m * cam.get_px() * cam.get_py();
  n02_p = n02_m * vpMath::sqr(cam.get_py());
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_CALIB3D)
/*!
  Line parameters conversion from normalized coordinates \f$(\rho_m,\theta_m)\f$ expressed in the image plane
  to pixel coordinates \f$(\rho_p,\theta_p)\f$ using OpenCV camera parameters. This function doesn't use distortion
  coefficients.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 &
  1\end{bmatrix}\f$ \param[in] rho_p, theta_p : Line parameters expressed in pixels.
  \param[out] rho_m, theta_m : Line parameters expressed in meters in the image plane.

*/
void vpMeterPixelConversion::convertLine(const cv::Mat &cameraMatrix, const double &rho_m, const double &theta_m,
                                         double &rho_p, double &theta_p)
{
  double co = cos(theta_m);
  double si = sin(theta_m);
  double px = cameraMatrix.at<double>(0, 0);
  double py = cameraMatrix.at<double>(1, 1);
  double u0 = cameraMatrix.at<double>(0, 2);
  double v0 = cameraMatrix.at<double>(1, 2);
  double d = sqrt(vpMath::sqr(py * co) + vpMath::sqr(px * si));

  if (fabs(d) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }

  theta_p = atan2(px * si, py * co);
  rho_p = (px * py * rho_m + u0 * py * co + v0 * px * si);
  rho_p /= d;
}

/*!
  Noting that the perspective projection of a 3D circle is usually an ellipse, using the camera intrinsic parameters
  converts the parameters of the 3D circle expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D circle) in the image with values in pixels using OpenCV camera parameters.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] circle : 3D circle with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internally updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its area
  (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in
  pixels.

  The following code shows how to use this function:
  \code
  vpCircle circle;
  double n20_p, n11_p, n02_p;
  circle.changeFrame(cMo);
  circle.projection();
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                    0, py, v0,
                                                    0,  0,  1);
  vpMeterPixelConversion::convertEllipse(cameraMatrix, circle, center_p, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center_p, n20_p, n11_p, n02_p, true, vpColor::red);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const cv::Mat &cameraMatrix, const vpCircle &circle, vpImagePoint &center,
                                            double &n20_p, double &n11_p, double &n02_p)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  double px = cameraMatrix.at<double>(index_0, index_0);
  double py = cameraMatrix.at<double>(index_1, index_1);
  cv::Mat distCoeffs = cv::Mat::zeros(index_5, index_1, CV_64FC1);
  // Get the parameters of the ellipse in the image plane
  double xc_m = circle.p[index_0];
  double yc_m = circle.p[index_1];
  double n20_m = circle.p[index_2];
  double n11_m = circle.p[index_3];
  double n02_m = circle.p[index_4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, xc_m, yc_m, center);
  n20_p = n20_m * vpMath::sqr(px);
  n11_p = n11_m * px * py;
  n02_p = n02_m * vpMath::sqr(py);
}

/*!
  Noting that the perspective projection of a 3D sphere is usually an ellipse, using the camera intrinsic parameters
  converts the parameters of the 3D sphere expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D sphere) in the image with values in pixels using OpenCV camera parameters.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] sphere : 3D sphere with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internally updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its area
  (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in
  pixels.

  The following code shows how to use this function:
  \code
  vpSphere sphere;
  double n20_p, n11_p, n02_p;
  sphere.changeFrame(cMo);
  sphere.projection();
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                    0, py, v0,
                                                    0,  0,  1);
  vpMeterPixelConversion::convertEllipse(cameraMatrix, sphere, center_p, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center_p, n20_p, n11_p, n02_p, true, vpColor::red);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const cv::Mat &cameraMatrix, const vpSphere &sphere, vpImagePoint &center,
                                            double &n20_p, double &n11_p, double &n02_p)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  double px = cameraMatrix.at<double>(index_0, index_0);
  double py = cameraMatrix.at<double>(index_1, index_1);
  cv::Mat distCoeffs = cv::Mat::zeros(index_5, index_1, CV_64FC1);
  // Get the parameters of the ellipse in the image plane
  double xc_m = sphere.p[index_0];
  double yc_m = sphere.p[index_1];
  double n20_m = sphere.p[index_2];
  double n11_m = sphere.p[index_3];
  double n02_m = sphere.p[index_4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, xc_m, yc_m, center);
  n20_p = n20_m * vpMath::sqr(px);
  n11_p = n11_m * px * py;
  n02_p = n02_m * vpMath::sqr(py);
}

/*!
  Convert parameters of an ellipse expressed in the image plane (these parameters are obtained after perspective
  projection of the 3D sphere) in the image with values in pixels using ViSP intrinsic camera parameters.

  The ellipse resulting from the conversion is here represented by its parameters \f$u_c,v_c,n_{20},
  n_{11}, n_{02}\f$ corresponding to its center coordinates in pixel and the centered moments normalized by its area.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] xc_m, yc_m : Center of the ellipse in the image plane with normalized coordinates expressed in meters.
  \param[in] n20_m, n11_m, n02_m : Second order centered moments of the ellipse normalized by its area
  (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in
  meter.
  \param[out] center_p : Center \f$(u_c, v_c)\f$ of the corresponding ellipse in the image with coordinates
  expressed in pixels.
  \param[out] n20_p, n11_p, n02_p : Second order centered moments of the ellipse normalized by its
  area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a the area)
  expressed in pixels.
 */
void vpMeterPixelConversion::convertEllipse(const cv::Mat &cameraMatrix, double xc_m, double yc_m, double n20_m,
                                            double n11_m, double n02_m, vpImagePoint &center_p, double &n20_p,
                                            double &n11_p, double &n02_p)
{
  double px = cameraMatrix.at<double>(0, 0);
  double py = cameraMatrix.at<double>(1, 1);
  cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, xc_m, yc_m, center_p);
  n20_p = n20_m * vpMath::sqr(px);
  n11_p = n11_m * px * py;
  n02_p = n02_m * vpMath::sqr(py);
}

/*!

  Point coordinates conversion from normalized coordinates \f$(x,y)\f$ in meter
  in the image plane to pixel coordinates \f$(u,v)\f$ in the image using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is nullptr/empty, the zero distortion coefficients are assumed.
  \param[in] x : input coordinate in meter along image plane x-axis.
  \param[in] y : input coordinate in meter along image plane y-axis.
  \param[out] u : output coordinate in pixels along image horizontal axis.
  \param[out] v : output coordinate in pixels along image vertical axis.

*/
void vpMeterPixelConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const double &x,
                                          const double &y, double &u, double &v)
{
  std::vector<cv::Point3d> objectPoints_vec;
  objectPoints_vec.push_back(cv::Point3d(x, y, 1.0));
  std::vector<cv::Point2d> imagePoints_vec;
  cv::projectPoints(objectPoints_vec, cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), cameraMatrix,
                    distCoeffs, imagePoints_vec);
  u = imagePoints_vec[0].x;
  v = imagePoints_vec[0].y;
}

/*!

  Point coordinates conversion from normalized coordinates \f$(x,y)\f$ in meter
  in the image plane to pixel coordinates \f$(u,v)\f$ in the image using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is nullptr/empty, the zero distortion coefficients are assumed.
  \param[in] x : input coordinate in meter along image plane x-axis.
  \param[in] y : input coordinate in meter along image plane y-axis.
  \param[out] iP : output coordinates in pixels.

*/
void vpMeterPixelConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const double &x,
                                          const double &y, vpImagePoint &iP)
{
  std::vector<cv::Point3d> objectPoints_vec;
  objectPoints_vec.push_back(cv::Point3d(x, y, 1.0));
  std::vector<cv::Point2d> imagePoints_vec;
  cv::projectPoints(objectPoints_vec, cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), cameraMatrix,
                    distCoeffs, imagePoints_vec);
  iP.set_u(imagePoints_vec[0].x);
  iP.set_v(imagePoints_vec[0].y);
}
#endif
END_VISP_NAMESPACE
