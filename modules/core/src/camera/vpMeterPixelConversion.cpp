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
 * Meter to pixel conversion.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpMeterPixelConversion.cpp
  \brief meter to pixel conversion
*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>

/*!
   Line parameters conversion from normalized coordinates \f$(\rho_m,\theta_m)\f$ expressed in the image plane
   to pixel coordinates \f$(\rho_p,\theta_p)\f$ using ViSP camera parameters. This function doesn't use distorsion coefficients.

   \param[in] cam : camera parameters.
   \param[in] rho_p, theta_p : Line parameters expressed in pixels.
   \param[out] rho_m, theta_m : Line parameters expressed in meters in the image plane.

*/
void vpMeterPixelConversion::convertLine(const vpCameraParameters &cam,
                                         const double &rho_m, const double &theta_m,
                                         double &rho_p, double &theta_p)
{
  double co = cos(theta_m);
  double si = sin(theta_m);
  double d = sqrt(vpMath::sqr(cam.py * co) + vpMath::sqr(cam.px * si));

  if (fabs(d) < 1e-6) {
    vpERROR_TRACE("division by zero");
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }

  theta_p = atan2(cam.px * si, cam.py * co);
  rho_p = (cam.px * cam.py * rho_m + cam.u0 * cam.py * co + cam.v0 * cam.px * si);
  rho_p /= d;
}

/*!
  Noting that the perspective projection of a 3D circle is usually an ellipse, using the camera intrinsic parameters converts the
  parameters of the 3D circle expressed in the image plane (these parameters are obtained after perspective projection
  of the 3D circle) in the image with values in pixels using ViSP camera parameters.

  The ellipse resulting from the perspective projection is here represented by its parameters \f$x_c, y_c, \mu_{20},
  \mu_{11}, \mu_{02}\f$ corresponding to its center coordinates in pixel and the centered moments.

  \param[in] cam : Intrinsic camera parameters.
  \param[in] circle : 3D circle with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internaly updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] mu20_p, mu11_p, mu02_p : Centered moments expressed in pixels.

  The following code shows how to use this function:
  \code
  vpCircle circle;
  double mu20_p, mu11_p, mu02_p;
  circle.changeFrame(cMo);
  circle.projection();
  vpMeterPixelConversion::convertEllipse(cam, circle, center_p, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center_p, mu20_p, mu11_p, mu02_p);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam,
                                            const vpCircle &circle, vpImagePoint &center,
                                            double &mu20_p, double &mu11_p, double &mu02_p)
{
  // Get the parameters of the ellipse in the image plane
  double xc_m = circle.p[0];
  double yc_m = circle.p[1];
  double mu20_m = circle.p[2];
  double mu11_m = circle.p[3];
  double mu02_m = circle.p[4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cam, xc_m, yc_m, center);
  mu20_p = mu20_m * vpMath::sqr(cam.get_px());
  mu11_p = mu11_m * cam.get_px() * cam.get_py();
  mu02_p = mu02_m * vpMath::sqr(cam.get_py());
}

/*!
  Noting that the perspective projection of a 3D sphere is usually an ellipse, using the camera intrinsic parameters converts the
  parameters of the 3D sphere expressed in the image plane (these parameters are obtained after perspective projection
  of the 3D sphere) in the image with values in pixels using ViSP camera parameters.

  The ellipse resulting from the perspective projection is here represented by its parameters \f$x_c,y_c,\mu_{20},
  \mu_{11}, \mu_{02}\f$ corresponding to its center coordinates in pixel and the centered moments.

  \param[in] cam : Intrinsic camera parameters.
  \param[in] sphere : 3D sphere with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internaly updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] mu20_p, mu11_p, mu02_p : Centered moments expressed in pixels.

  The following code shows how to use this function:
  \code
  vpSphere sphere;
  double mu20_p, mu11_p, mu02_p;
  sphere.changeFrame(cMo);
  sphere.projection();
  vpMeterPixelConversion::convertEllipse(cam, sphere, center_p, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center_p, mu20_p, mu11_p, mu02_p);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam, const vpSphere &sphere, vpImagePoint &center,
                                            double &mu20_p, double &mu11_p, double &mu02_p)
{
  // Get the parameters of the ellipse in the image plane
  double xc_m = sphere.p[0];
  double yc_m = sphere.p[1];
  double mu20_m = sphere.p[2];
  double mu11_m = sphere.p[3];
  double mu02_m = sphere.p[4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cam, xc_m, yc_m, center);
  mu20_p = mu20_m * vpMath::sqr(cam.get_px());
  mu11_p = mu11_m * cam.get_px() * cam.get_py();
  mu02_p = mu02_m * vpMath::sqr(cam.get_py());
}

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
/*!
   Line parameters conversion from normalized coordinates \f$(\rho_m,\theta_m)\f$ expressed in the image plane
   to pixel coordinates \f$(\rho_p,\theta_p)\f$ using OpenCV camera parameters. This function doesn't use distorsion coefficients.

   \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
   \param[in] rho_p, theta_p : Line parameters expressed in pixels.
   \param[out] rho_m, theta_m : Line parameters expressed in meters in the image plane.

*/
void vpMeterPixelConversion::convertLine(const cv::Mat &cameraMatrix,
                                         const double &rho_m, const double &theta_m,
                                         double &rho_p, double &theta_p)
{
  double co = cos(theta_m);
  double si = sin(theta_m);
  double px = cameraMatrix.at<double>(0,0);
  double py = cameraMatrix.at<double>(1,1);
  double u0 = cameraMatrix.at<double>(0,2);
  double v0 = cameraMatrix.at<double>(1,2);
  double d = sqrt(vpMath::sqr(py * co) + vpMath::sqr(px * si));

  if (fabs(d) < 1e-6) {
    vpERROR_TRACE("division by zero");
    throw(vpException(vpException::divideByZeroError, "division by zero"));
  }

  theta_p = atan2(px * si, py * co);
  rho_p = (px * py * rho_m + u0 * py * co + v0 * px * si);
  rho_p /= d;
}

/*!
  Noting that the perspective projection of a 3D circle is usually an ellipse, using the camera intrinsic parameters converts the
  parameters of the 3D circle expressed in the image plane (these parameters are obtained after perspective projection
  of the 3D circle) in the image with values in pixels using OpenCV camera parameters.

  The ellipse resulting from the perspective projection is here represented by its parameters \f$x_c, y_c, \mu_{20},
  \mu_{11}, \mu_{02}\f$ corresponding to its center coordinates in pixel and the centered moments.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] circle : 3D circle with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internaly updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] mu20_p, mu11_p, mu02_p : Centered moments expressed in pixels.

  The following code shows how to use this function:
  \code
  vpCircle circle;
  double mu20_p, mu11_p, mu02_p;
  circle.changeFrame(cMo);
  circle.projection();
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                    0, py, v0,
                                                    0,  0,  1);
  vpMeterPixelConversion::convertEllipse(cameraMatrix, circle, center_p, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center_p, mu20_p, mu11_p, mu02_p);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const cv::Mat &cameraMatrix,
                                            const vpCircle &circle, vpImagePoint &center,
                                            double &mu20_p, double &mu11_p, double &mu02_p)
{
  double px = cameraMatrix.at<double>(0,0);
  double py = cameraMatrix.at<double>(1,1);
  cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64FC1);
  // Get the parameters of the ellipse in the image plane
  double xc_m = circle.p[0];
  double yc_m = circle.p[1];
  double mu20_m = circle.p[2];
  double mu11_m = circle.p[3];
  double mu02_m = circle.p[4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, xc_m, yc_m, center);
  mu20_p = mu20_m * vpMath::sqr(px);
  mu11_p = mu11_m * px * py;
  mu02_p = mu02_m * vpMath::sqr(py);
}

/*!
  Noting that the perspective projection of a 3D sphere is usually an ellipse, using the camera intrinsic parameters converts the
  parameters of the 3D sphere expressed in the image plane (these parameters are obtained after perspective projection
  of the 3D sphere) in the image with values in pixels using OpenCV camera parameters.

  The ellipse resulting from the perspective projection is here represented by its parameters \f$x_c,y_c,\mu_{20},
  \mu_{11}, \mu_{02}\f$ corresponding to its center coordinates in pixel and the centered moments.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] sphere : 3D sphere with internal vector `circle.p[]` that contains the ellipse parameters expressed
  in the image plane. These parameters are internaly updated after perspective projection of the sphere.
  \param[out] center : Center of the corresponding ellipse in the image with coordinates expressed in pixels.
  \param[out] mu20_p, mu11_p, mu02_p : Centered moments expressed in pixels.

  The following code shows how to use this function:
  \code
  vpSphere sphere;
  double mu20_p, mu11_p, mu02_p;
  sphere.changeFrame(cMo);
  sphere.projection();
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                    0, py, v0,
                                                    0,  0,  1);
  vpMeterPixelConversion::convertEllipse(cameraMatrix, sphere, center_p, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center_p, mu20_p, mu11_p, mu02_p);
  \endcode
 */
void vpMeterPixelConversion::convertEllipse(const cv::Mat &cameraMatrix,
                                            const vpSphere &sphere, vpImagePoint &center,
                                            double &mu20_p, double &mu11_p, double &mu02_p)
{
  double px = cameraMatrix.at<double>(0,0);
  double py = cameraMatrix.at<double>(1,1);
  cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64FC1);
  // Get the parameters of the ellipse in the image plane
  double xc_m = sphere.p[0];
  double yc_m = sphere.p[1];
  double mu20_m = sphere.p[2];
  double mu11_m = sphere.p[3];
  double mu02_m = sphere.p[4];

  // Convert from meter to pixels
  vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, xc_m, yc_m, center);
  mu20_p = mu20_m * vpMath::sqr(px);
  mu11_p = mu11_m * px * py;
  mu02_p = mu02_m * vpMath::sqr(py);
}

/*!

  Point coordinates conversion from normalized coordinates \f$(x,y)\f$ in meter
  in the image plane to pixel coordinates \f$(u,v)\f$ in the image using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
  \param[in] x : input coordinate in meter along image plane x-axis.
  \param[in] y : input coordinate in meter along image plane y-axis.
  \param[out] u : output coordinate in pixels along image horizontal axis.
  \param[out] v : output coordinate in pixels along image vertical axis.

*/
void vpMeterPixelConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                          const double &x, const double &y, double &u, double &v)
{
  std::vector<cv::Point3d> objectPoints_vec;
  objectPoints_vec.push_back(cv::Point3d(x, y, 1.0));
  std::vector<cv::Point2d> imagePoints_vec;
  cv::projectPoints(objectPoints_vec, cv::Mat::eye(3,3,CV_64FC1), cv::Mat::zeros(3,1,CV_64FC1), cameraMatrix, distCoeffs, imagePoints_vec);
  u = imagePoints_vec[0].x;
  v = imagePoints_vec[0].y;
}

/*!

  Point coordinates conversion from normalized coordinates \f$(x,y)\f$ in meter
  in the image plane to pixel coordinates \f$(u,v)\f$ in the image using OpenCV camera parameters.

  \param[in] cameraMatrix : Camera Matrix \f$\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}\f$
  \param[in] distCoeffs : Input vector of distortion coefficients
  \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
  4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
  \param[in] x : input coordinate in meter along image plane x-axis.
  \param[in] y : input coordinate in meter along image plane y-axis.
  \param[out] iP : output coordinates in pixels.

*/
void vpMeterPixelConversion::convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                          const double &x, const double &y, vpImagePoint &iP)
{
  std::vector<cv::Point3d> objectPoints_vec;
  objectPoints_vec.push_back(cv::Point3d(x, y, 1.0));
  std::vector<cv::Point2d> imagePoints_vec;
  cv::projectPoints(objectPoints_vec, cv::Mat::eye(3,3,CV_64FC1), cv::Mat::zeros(3,1,CV_64FC1), cameraMatrix, distCoeffs, imagePoints_vec);
  iP.set_u(imagePoints_vec[0].x);
  iP.set_v(imagePoints_vec[0].y);
}
#endif
