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

#ifndef vpMeterPixelConversion_H
#define vpMeterPixelConversion_H

/*!
  \file vpMeterPixelConversion.h
  \brief Meter to pixel conversion.

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpSphere.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
#  include <opencv2/calib3d/calib3d.hpp>
#endif

/*!
  \class vpMeterPixelConversion

  \ingroup group_core_camera

  Various conversion functions to transform primitives (2D ellipse, 2D line, 2D point) from normalized
  coordinates in meter in the image plane into pixel coordinates.

  Tranformation relies either on ViSP camera parameters implemented in vpCameraParameters or on OpenCV camera parameters
  that are set from a projection matrix and a distorsion coefficients vector.

*/
class VISP_EXPORT vpMeterPixelConversion
{
public:
  /** @name Using ViSP camera parameters  */
  //@{
  static void convertEllipse(const vpCameraParameters &cam, const vpSphere &sphere, vpImagePoint &center_p,
                             double &n20_p, double &n11_p, double &n02_p);
  static void convertEllipse(const vpCameraParameters &cam, const vpCircle &circle, vpImagePoint &center_p,
                             double &n20_p, double &n11_p, double &n02_p);
  static void convertEllipse(const vpCameraParameters &cam,
                             double xc_m, double yc_m, double n20_m, double n11_m, double n02_m,
                             vpImagePoint &center_p, double &n20_p, double &n11_p, double &n02_p);
  static void convertLine(const vpCameraParameters &cam, const double &rho_m, const double &theta_m, double &rho_p,
                          double &theta_p);

  /*!

    Point coordinates conversion from normalized coordinates
    \f$(x,y)\f$ in meter in the image plane to pixel coordinates \f$(u,v)\f$ in the image using ViSP camera parameters.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param[in] cam : camera parameters.
    \param[in] x : input coordinate in meter along image plane x-axis.
    \param[in] y : input coordinate in meter along image plane y-axis.
    \param[out] u : output coordinate in pixels along image horizontal axis.
    \param[out] v : output coordinate in pixels along image vertical axis.

    \f$ u = x*p_x + u_0 \f$ and  \f$ v = y*p_y + v_0 \f$ in the case of
    perspective projection without distortion.

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0
    \f$ with \f$ r^2 = x^2+y^2 \f$ in the  case of perspective projection with
    distortion.

    In the case of a projection with Kannala-Brandt distortion, refer to
    \cite KannalaBrandt.
  */
  inline static void convertPoint(const vpCameraParameters &cam,
                                  const double &x, const double &y, double &u, double &v)
  {
    switch (cam.projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      convertPointWithoutDistortion(cam, x, y, u, v);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      convertPointWithDistortion(cam, x, y, u, v);
      break;
    case vpCameraParameters::ProjWithKannalaBrandtDistortion:
      convertPointWithKannalaBrandtDistortion(cam, x, y, u, v);
      break;
    }
  }

  /*!

    Point coordinates conversion from normalized coordinates
    \f$(x,y)\f$ in meter in the image plane to pixel coordinates in the image using ViSP camera parameters.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param[in] cam : camera parameters.
    \param[in] x : input coordinate in meter along image plane x-axis.
    \param[in] y : input coordinate in meter along image plane y-axis.
    \param[out] iP : output coordinates in pixels.

    In the frame (u,v) the result is given by:

    \f$ u = x*p_x + u_0 \f$ and  \f$ v = y*p_y + v_0 \f$ in the case of
    perspective projection without distortion.

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0
    \f$ with \f$ r^2 = x^2+y^2 \f$ in the  case of perspective projection with
    distortion.

    In the case of a projection with Kannala-Brandt distortion, refer to
    \cite KannalaBrandt.
  */

  inline static void convertPoint(const vpCameraParameters &cam, const double &x, const double &y, vpImagePoint &iP)
  {
    switch (cam.projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      convertPointWithoutDistortion(cam, x, y, iP);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      convertPointWithDistortion(cam, x, y, iP);
      break;
    case vpCameraParameters::ProjWithKannalaBrandtDistortion:
      convertPointWithKannalaBrandtDistortion(cam, x, y, iP);
      break;
    }
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!

    Point coordinates conversion without distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates
    \f$(u,v)\f$.

    \f$ u = x*p_x+u_0 \f$ and  \f$ v = y*p_y+v_0  \f$
  */

  inline static void convertPointWithoutDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                   double &u, double &v)
  {
    u = x * cam.px + cam.u0;
    v = y * cam.py + cam.v0;
  }

  /*!

    Point coordinates conversion without distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates.

    In the frame (u,v) the result is given by:

    \f$ u = x*p_x+u_0 \f$ and  \f$ v = y*p_y+v_0  \f$
  */

  inline static void convertPointWithoutDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                   vpImagePoint &iP)
  {
    iP.set_u(x * cam.px + cam.u0);
    iP.set_v(y * cam.py + cam.v0);
  }

  /*!

    Point coordinates conversion with distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates
    \f$(u,v)\f$.

    \param[in] cam : camera parameters.
    \param[in] x : input coordinate in meter along image plane x-axis.
    \param[in] y : input coordinate in meter along image plane y-axis.
    \param[out] u : output coordinate in pixels along image horizontal axis.
    \param[out] v : output coordinate in pixels along image vertical axis.

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and
    \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0 \f$
    with \f$ r^2 = x^2+y^2 \f$
  */
  inline static void convertPointWithDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                double &u, double &v)
  {
    double r2 = 1. + cam.kud * (x * x + y * y);
    u = cam.u0 + cam.px * x * r2;
    v = cam.v0 + cam.py * y * r2;
  }

  /*!

    Point coordinates conversion with distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates.

    \param[in] cam : camera parameters.
    \param[in] x : input coordinate in meter along image plane x-axis.
    \param[in] y : input coordinate in meter along image plane y-axis.
    \param[out] iP : output coordinates in pixels.

    In the frame (u,v) the result is given by:

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and
    \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0 \f$
    with \f$ r^2 = x^2+y^2 \f$
  */
  inline static void convertPointWithDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                vpImagePoint &iP)
  {
    double r2 = 1. + cam.kud * (x * x + y * y);
    iP.set_u(cam.u0 + cam.px * x * r2);
    iP.set_v(cam.v0 + cam.py * y * r2);
  }

  /*!
    Point coordinates conversion with Kannala-Brandt distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates
    \f$(u,v)\f$.

    \param[in] cam : camera parameters.
    \param[in] x   : input coordinate in meter along image plane x-axis.
    \param[in] y   : input coordinate in meter along image plane y-axis.
    \param[out] u  : output coordinate in pixels along image horizontal axis.
    \param[out] v  : output coordinate in pixels along image vertical axis.

    \f$ r = sqrt{x^2 + y^2} \f$
    \f$ \theta = \arctan{r} \f$
    Calculate \f$ r_d \f$ knowing ditorsion coefficients as follows:
    \f$ r_d = \theta + k_1 \theta^3 + k_2 \theta^5 + k_3 \theta^7 + k_4 \theta^9 \f$
    \f$ scale = r_d / r \f$
    \f$ x_d = x * scale \f$
    \f$ y_d = y * scale \f$
    \f$ u = x_d*p_x+u_0 \f$ and
    \f$ v = y_d*p_y+v_0 \f$
    with \f$ r^2 = x^2+y^2 \f$
  */
  inline static void convertPointWithKannalaBrandtDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                             double &u, double &v)
  {
    double r = sqrt(vpMath::sqr(x) + vpMath::sqr(y));
    double theta = atan(r);

    std::vector<double> k = cam.getKannalaBrandtDistortionCoefficients();

    double theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta,
          theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;

    double r_d = theta + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;

    double scale = (std::fabs(r) < std::numeric_limits<double>::epsilon()) ? 1.0 : r_d / r;

    double x_d = x * scale;
    double y_d = y * scale;

    u = cam.px * x_d + cam.u0;
    v = cam.py * y_d + cam.v0;
  }

  /*!
    Point coordinates conversion with Kannala-Brandt distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates
    \f$(u,v)\f$.

    \param[in] cam : camera parameters.
    \param[in] x : input coordinate in meter along image plane x-axis.
    \param[in] y : input coordinate in meter along image plane y-axis.
    \param[out] iP : output coordinates in pixels.

    \f$ r = sqrt{x^2 + y^2} \f$
    \f$ \theta = \arctan{r} \f$
    Calculate \f$ r_d \f$ knowing ditorsion coefficients as follows:
    \f$ r_d = \theta + k_1 \theta^3 + k_2 \theta^5 + k_3 \theta^7 + k_4 \theta^9 \f$
    \f$ scale = r_d / r \f$
    \f$ x_d = x * scale \f$
    \f$ y_d = y * scale \f$
    \f$ u = x_d*p_x+u_0 \f$ and
    \f$ v = y_d*p_y+v_0 \f$
    with \f$ r^2 = x^2+y^2 \f$
  */
  inline static void convertPointWithKannalaBrandtDistortion(const vpCameraParameters &cam, const double &x, const double &y,
                                                             vpImagePoint &iP)
  {
    double r = sqrt(vpMath::sqr(x) + vpMath::sqr(y));
    double theta = atan(r);

    std::vector<double> k = cam.getKannalaBrandtDistortionCoefficients();

    double theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta,
          theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;

    double r_d = theta + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;

    double scale = (std::fabs(r) < std::numeric_limits<double>::epsilon()) ? 1.0 : r_d / r;

    double x_d = x * scale;
    double y_d = y * scale;

    iP.set_u(cam.px * x_d + cam.u0);
    iP.set_v(cam.py * y_d + cam.v0);
  }

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
  //@}

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
  /** @name Using OpenCV camera parameters  */
  //@{
  static void convertEllipse(const cv::Mat &cameraMatrix,
                             const vpCircle &circle, vpImagePoint &center,
                             double &n20_p, double &n11_p, double &n02_p);
  static void convertEllipse(const cv::Mat &cameraMatrix,
                             const vpSphere &sphere, vpImagePoint &center,
                             double &n20_p, double &n11_p, double &n02_p);
  static void convertEllipse(const cv::Mat &cameraMatrix,
                             double xc_m, double yc_m, double n20_m, double n11_m, double n02_m,
                             vpImagePoint &center_p, double &n20_p, double &n11_p, double &n02_p);
  static void convertLine(const cv::Mat &cameraMatrix,
                          const double &rho_m, const double &theta_m,
                          double &rho_p, double &theta_p);
  static void convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                           const double &x, const double &y, double &u, double &v);
  static void convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                           const double &x, const double &y, vpImagePoint &iP);
  //@}
#endif
};

#endif
