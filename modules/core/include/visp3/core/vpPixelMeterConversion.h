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
 * Pixel to meter conversion.
 */

/*!
  \file vpPixelMeterConversion.h
  \brief pixel to meter conversion
*/

#ifndef VP_PIXEL_METER_CONVERSION_H
#define VP_PIXEL_METER_CONVERSION_H

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpPixelMeterConversion

  \ingroup group_core_camera

  Various conversion functions to transform primitives (2D line, moments, 2D point) from pixel to normalized
  coordinates in meter in the image plane.

  Transformation relies either on ViSP camera parameters implemented in vpCameraParameters or on OpenCV camera parameters
  that are set from a projection matrix and a distortion coefficients vector.

*/
class VISP_EXPORT vpPixelMeterConversion
{
public:
  /** @name Using ViSP camera parameters  */
  //@{
  static void convertEllipse(const vpCameraParameters &cam, const vpImagePoint &center_p, double n20_p, double n11_p,
                             double n02_p, double &xc_m, double &yc_m, double &n20_m, double &n11_m, double &n02_m);
  static void convertLine(const vpCameraParameters &cam, const double &rho_p, const double &theta_p, double &rho_m,
                          double &theta_m);

  static void convertMoment(const vpCameraParameters &cam, unsigned int order, const vpMatrix &moment_pixel,
                            vpMatrix &moment_meter);
  /*!
    Point coordinates conversion from pixel coordinates
    \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter using ViSP camera parameters.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param[in] cam : camera parameters.
    \param[in] u : input coordinate in pixels along image horizontal axis.
    \param[in] v : input coordinate in pixels along image vertical axis.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    \f$ x = (u-u_0)/p_x \f$ and \f$ y = (v-v_0)/p_y  \f$ in the case of
    perspective projection without distortion.

    \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
    \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y  \f$
    with \f$ r^2=((u - u_0)/p_x)^2+((v-v_0)/p_y)^2 \f$ in the case of
    perspective projection with distortion.

    In the case of a projection with Kannala-Brandt distortion, refer to
    \cite KannalaBrandt.
  */
  inline static void convertPoint(const vpCameraParameters &cam, const double &u, const double &v, double &x, double &y)
  {
    switch (cam.m_projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      convertPointWithoutDistortion(cam, u, v, x, y);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      convertPointWithDistortion(cam, u, v, x, y);
      break;
    case vpCameraParameters::ProjWithKannalaBrandtDistortion:
      convertPointWithKannalaBrandtDistortion(cam, u, v, x, y);
      break;
    default:
      std::cerr << "projection model not identified" << std::endl;
    }
  }

  /*!
    Point coordinates conversion from pixel coordinates
    Coordinates in pixel to normalized coordinates \f$(x,y)\f$ in meter using ViSP camera parameters.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param[in] cam : camera parameters.
    \param[in] iP : input coordinates in pixels.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    Thanks to the pixel coordinates in the frame (u,v), the meter coordinates
    are given by :

    \f$ x = (u-u_0)/p_x \f$ and \f$ y = (v-v_0)/p_y  \f$ in the case of
    perspective projection without distortion.

    \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
    \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y  \f$
    with \f$ r^2=((u - u_0)/p_x)^2+((v-v_0)/p_y)^2 \f$ in the case of
    perspective projection with distortion.

    In the case of a projection with Kannala-Brandt distortion, refer to
    \cite KannalaBrandt.
  */
  inline static void convertPoint(const vpCameraParameters &cam, const vpImagePoint &iP, double &x, double &y)
  {
    switch (cam.m_projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      convertPointWithoutDistortion(cam, iP, x, y);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      convertPointWithDistortion(cam, iP, x, y);
      break;
    case vpCameraParameters::ProjWithKannalaBrandtDistortion:
      convertPointWithKannalaBrandtDistortion(cam, iP, x, y);
      break;
    default:
      std::cerr << "projection model not identified" << std::endl;
    }
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!
    Point coordinates conversion without distortion from pixel
    coordinates \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter.

    \param[in] cam : camera parameters.
    \param[in] u : input coordinate in pixels along image horizontal axis.
    \param[in] v : input coordinate in pixels along image vertical axis.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    \f$ x = (u-u_0)/p_x \f$ and  \f$ y = (v-v_0)/p_y  \f$
  */
  inline static void convertPointWithoutDistortion(const vpCameraParameters &cam, const double &u, const double &v,
                                                   double &x, double &y)
  {
    x = (u - cam.m_u0) * cam.m_inv_px;
    y = (v - cam.m_v0) * cam.m_inv_py;
  }

  /*!
    Point coordinates conversion without distortion from pixel
    coordinates Coordinates in pixel to normalized coordinates \f$(x,y)\f$ in
    meter.

    \param[in] cam : camera parameters.
    \param[in] iP : input coordinates in pixels.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    Thanks to the pixel coordinates in the frame (u,v), the meter coordinates
    are given by :

    \f$ x = (u-u_0)/p_x \f$ and  \f$ y = (v-v_0)/p_y  \f$
  */
  inline static void convertPointWithoutDistortion(const vpCameraParameters &cam, const vpImagePoint &iP, double &x,
                                                   double &y)
  {
    x = (iP.get_u() - cam.m_u0) * cam.m_inv_px;
    y = (iP.get_v() - cam.m_v0) * cam.m_inv_py;
  }

  /*!
    Point coordinates conversion with distortion from pixel coordinates
    \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter.

    \param[in] cam : camera parameters.
    \param[in] u : input coordinate in pixels along image horizontal axis.
    \param[in] v : input coordinate in pixels along image vertical axis.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
    \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y \f$
    with \f$ r^2=((u - u_0)/p_x)^2 + ((v-v_0)/p_y)^2 \f$
  */
  inline static void convertPointWithDistortion(const vpCameraParameters &cam, const double &u, const double &v,
                                                double &x, double &y)
  {
    double r2 = 1. + (cam.m_kdu * (vpMath::sqr((u - cam.m_u0) * cam.m_inv_px) + vpMath::sqr((v - cam.m_v0) * cam.m_inv_py)));
    x = (u - cam.m_u0) * r2 * cam.m_inv_px;
    y = (v - cam.m_v0) * r2 * cam.m_inv_py;
  }

  /*!
    Point coordinates conversion with distortion from pixel coordinates
    Coordinates in pixel to normalized coordinates \f$(x,y)\f$ in meter.

    \param[in] cam : camera parameters.
    \param[in] iP : input coordinates in pixels.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    Thanks to the pixel coordinates in the frame (u,v), the meter coordinates
    are given by :

    \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
    \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y \f$
    with \f$ r^2=((u - u_0)/p_x)^2 + ((v-v_0)/p_y)^2 \f$
  */
  inline static void convertPointWithDistortion(const vpCameraParameters &cam, const vpImagePoint &iP, double &x,
                                                double &y)
  {
    double r2 = 1. + (cam.m_kdu * (vpMath::sqr((iP.get_u() - cam.m_u0) * cam.m_inv_px) +
                                   vpMath::sqr((iP.get_v() - cam.m_v0) * cam.m_inv_py)));
    x = (iP.get_u() - cam.m_u0) * r2 * cam.m_inv_px;
    y = (iP.get_v() - cam.m_v0) * r2 * cam.m_inv_py;
  }

  /*!
    Point coordinates conversion with Kannala-Brandt distortion from pixel
    coordinates \f$(u,v)\f$ to normalized coordinates \f$(x,y)\f$ in meter.

    \param[in]  cam : Camera parameters.
    \param[in]  u   : Input coordinate in pixels along image horizontal axis.
    \param[in]  v   : Input coordinate in pixels along image vertical axis.
    \param[out] x   : Output coordinate in meter along image plane x-axis.
    \param[out] y   : Output coordinate in meter along image plane y-axis.

    \f$ x_d = (u-u_0)/px \f$
    \f$ y_d = (v-v_0)/py \f$
    \f$ r_d = \sqrt{x^2_d + y^2_d} \f$
    Solve for \f$ \theta \f$ knowing that:
    \f$ r_d = \theta + k_1 \theta^3 + k_2 \theta^5 + k_3 \theta^7 + k_4 \theta^5 \f$
    Calculate the distortion scale \f$ scale \f$:
    \f$ scale = \tan(\theta) / r_d \f$
    \f$ x   = x_d * scale \f$
    \f$ y   = y_d * scale \f$
  */
  inline static void convertPointWithKannalaBrandtDistortion(const vpCameraParameters &cam, const double &u,
                                                             const double &v, double &x, double &y)
  {
    double x_d = (u - cam.m_u0) / cam.m_px, y_d = (v - cam.m_v0) / cam.m_py;
    double scale = 1.0;
    double r_d = sqrt(vpMath::sqr(x_d) + vpMath::sqr(y_d));
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int val_1 = 1;
    const unsigned int val_3 = 3;
    const unsigned int val_5 = 5;
    const unsigned int val_7 = 7;
    const unsigned int val_9 = 9;
    const unsigned int val_10 = 10;

    r_d = std::min<double>(std::max<double>(-M_PI, r_d), M_PI); // FOV restricted to 180degrees.

    std::vector<double> k = cam.getKannalaBrandtDistortionCoefficients();

    const double EPS = 1e-8;
    // Use Newton-Raphson method to solve for the angle theta
    if (r_d > EPS) {
      // compensate distortion iteratively
      double theta = r_d;

      for (unsigned int j = 0; j < val_10; ++j) {
        double theta2 = theta * theta;
        double theta4 = theta2 * theta2;
        double theta6 = theta4 * theta2;
        double theta8 = theta6 * theta2;
        double k0_theta2 = k[index_0] * theta2;
        double k1_theta4 = k[index_1] * theta4;
        double k2_theta6 = k[index_2] * theta6,
          k3_theta8 = k[index_3] * theta8;
          /*
          // new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta)
          */
        double theta_fix = ((theta * (val_1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8)) - r_d)
          / (val_1 + (val_3 * k0_theta2) + (val_5 * k1_theta4) + (val_7 * k2_theta6) + (val_9 * k3_theta8));
        theta = theta - theta_fix;
        if (fabs(theta_fix) < EPS) {
          break;
        }
      }

      scale = std::tan(theta) / r_d; // Scale of norm of (x,y) and (x_d, y_d)
    }

    x = x_d * scale;
    y = y_d * scale;
  }

  /*!
    Point coordinates conversion with Kannala-Brandt distortion from pixel
    coordinates in pixel to normalized coordinates \f$(x,y)\f$ in meter.

    \param[in] cam : camera parameters.
    \param[in] iP : input coordinates in pixels.
    \param[out] x : output coordinate in meter along image plane x-axis.
    \param[out] y : output coordinate in meter along image plane y-axis.

    \f$ x_d = (u-u_0)/px \f$
    \f$ y_d = (v-v_0)/py \f$
    \f$ r_d = \sqrt{x^2_d + y^2_d} \f$
    Solve for \f$ \theta \f$ knowing that:
    \f$ r_d = \theta + k_1 \theta^3 + k_2 \theta^5 + k_3 \theta^7 + k_4 \theta^5 \f$
    Calculate the distortion scale \f$ scale \f$:
    \f$ scale = \tan(\theta) / r_d \f$
    \f$ x   = x_d * scale \f$
    \f$ y   = y_d * scale \f$
   */
  inline static void convertPointWithKannalaBrandtDistortion(const vpCameraParameters &cam, const vpImagePoint &iP,
                                                             double &x, double &y)
  {
    double x_d = (iP.get_u() - cam.m_u0) / cam.m_px, y_d = (iP.get_v() - cam.m_v0) / cam.m_py;
    double scale = 1.0;
    double r_d = sqrt(vpMath::sqr(x_d) + vpMath::sqr(y_d));
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int val_1 = 1;
    const unsigned int val_3 = 3;
    const unsigned int val_5 = 5;
    const unsigned int val_7 = 7;
    const unsigned int val_9 = 9;
    const unsigned int val_10 = 10;

    r_d = std::min<double>(std::max<double>(-M_PI, r_d), M_PI); // FOV restricted to 180degrees.

    std::vector<double> k = cam.getKannalaBrandtDistortionCoefficients();

    const double EPS = 1e-8;
    // Use Newton-Raphson method to solve for the angle theta
    if (r_d > EPS) {
      // compensate distortion iteratively
      double theta = r_d;

      for (unsigned int j = 0; j < val_10; ++j) {
        double theta2 = theta * theta;
        double theta4 = theta2 * theta2;
        double theta6 = theta4 * theta2;
        double theta8 = theta6 * theta2;
        double k0_theta2 = k[index_0] * theta2;
        double k1_theta4 = k[index_1] * theta4;
        double k2_theta6 = k[index_2] * theta6;
        double k3_theta8 = k[index_3] * theta8;
        /*
          // new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta)
        */
        double theta_fix = ((theta * (val_1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8)) - r_d) /
          (val_1 + (val_3 * k0_theta2) + (val_5 * k1_theta4) + (val_7 * k2_theta6) + (val_9 * k3_theta8));
        theta = theta - theta_fix;
        if (fabs(theta_fix) < EPS) {
          break;
        }
      }

      scale = std::tan(theta) / r_d; // Scale of norm of (x,y) and (x_d, y_d)
    }

    x = x_d * scale;
    y = y_d * scale;
  }
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
  //@}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_IMGPROC)
  /** @name Using OpenCV camera parameters  */
  //@{
  static void convertEllipse(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const vpImagePoint &center_p,
                             double n20_p, double n11_p, double n02_p, double &xc_m, double &yc_m, double &n20_m,
                             double &n11_m, double &n02_m);
  static void convertLine(const cv::Mat &cameraMatrix, const double &rho_p, const double &theta_p, double &rho_m,
                          double &theta_m);
  static void convertMoment(const cv::Mat &cameraMatrix, unsigned int order, const vpMatrix &moment_pixel,
                            vpMatrix &moment_meter);
  static void convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const double &u, const double &v,
                           double &x, double &y);
  static void convertPoint(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const vpImagePoint &iP, double &x,
                           double &y);
  //@}
#endif
};
END_VISP_NAMESPACE
#endif
