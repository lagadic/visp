/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
  \brief meter to pixel  conversion

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>

/*!
  \class vpMeterPixelConversion

  \ingroup group_core_camera

  \brief Conversion from normalized coordinates \f$(x,y)\f$ in meter
  to pixel coordinates \f$(u,v)\f$.

  This class relates to vpCameraParameters.

*/
class VISP_EXPORT vpMeterPixelConversion
{
public:
  static void convertEllipse(const vpCameraParameters &cam, const vpCircle &circle, vpImagePoint &center,
                             double &mu20_p, double &mu11_p, double &mu02_p);

  static void convertLine(const vpCameraParameters &cam, const double &rho_m, const double &theta_m, double &rho_p,
                          double &theta_p);

  /*!

    \brief Point coordinates conversion from normalized coordinates
    \f$(x,y)\f$ in meter to pixel coordinates \f$(u,v)\f$.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param cam : camera parameters.
    \param x : input coordinate in meter along image plane x-axis.
    \param y : input coordinate in meter along image plane y-axis.
    \param u : output coordinate in pixels along image horizontal axis.
    \param v : output coordinate in pixels along image vertical axis.

    \f$ u = x*p_x + u_0 \f$ and  \f$ v = y*p_y + v_0 \f$ in the case of
    perspective projection without distortion.

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0
    \f$ with \f$ r^2 = x^2+y^2 \f$ in the  case of perspective projection with
    distortion.
  */

  inline static void convertPoint(const vpCameraParameters &cam, const double &x, const double &y, double &u, double &v)
  {
    switch (cam.projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      convertPointWithoutDistortion(cam, x, y, u, v);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      convertPointWithDistortion(cam, x, y, u, v);
      break;
    }
  }

  /*!

    \brief Point coordinates conversion from normalized coordinates
    \f$(x,y)\f$ in meter to pixel coordinates.

    The used formula depends on the projection model of the camera. To
    know the currently used projection model use
    vpCameraParameter::get_projModel()

    \param cam : camera parameters.
    \param x : input coordinate in meter along image plane x-axis.
    \param y : input coordinate in meter along image plane y-axis.
    \param iP : output coordinates in pixels.

    In the frame (u,v) the result is given by:

    \f$ u = x*p_x + u_0 \f$ and  \f$ v = y*p_y + v_0 \f$ in the case of
    perspective projection without distortion.

    \f$ u = x*p_x*(1+k_{ud}*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_{ud}*r^2)+v_0
    \f$ with \f$ r^2 = x^2+y^2 \f$ in the  case of perspective projection with
    distortion.
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
    }
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!

    \brief Point coordinates conversion without distortion from
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

    \brief Point coordinates conversion without distortion from
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

    \brief Point coordinates conversion with distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates
    \f$(u,v)\f$.

    \param cam : camera parameters.
    \param x : input coordinate in meter along image plane x-axis.
    \param y : input coordinate in meter along image plane y-axis.
    \param u : output coordinate in pixels along image horizontal axis.
    \param v : output coordinate in pixels along image vertical axis.

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

    \brief Point coordinates conversion with distortion from
    normalized coordinates \f$(x,y)\f$ in meter to pixel coordinates.

    \param cam : camera parameters.
    \param x : input coordinate in meter along image plane x-axis.
    \param y : input coordinate in meter along image plane y-axis.
    \param iP : output coordinates in pixels.

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
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
};

#endif
