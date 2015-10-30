/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpDebug.h>

//! Line coordinates conversion (rho,theta).
void
vpMeterPixelConversion::convertLine(const vpCameraParameters &cam,
				    const double &rho_m, const double &theta_m,
				    double &rho_p, double &theta_p)
{
  double co = cos(theta_m) ;
  double si = sin(theta_m) ;
  double d = sqrt(vpMath::sqr(cam.py*co) + vpMath::sqr(cam.px*si)) ;

  if (fabs(d)<1e-6)
  {
    vpERROR_TRACE("division by zero") ;
    throw(vpException(vpException::divideByZeroError,
		      "division by zero")) ;
  }

  theta_p = atan2(cam.px*si, cam.py*co) ;
  rho_p = (cam.px*cam.py*rho_m + cam.u0*cam.py*co + cam.v0*cam.px*si) ;
  rho_p /= d ;
}

/*!
  Converts an ellipse with parameters expressed in meter in the image
  plane in parameters expressed in the image in pixels.

  The ellipse is here represented by its parameters \f$x_c,y_c,\mu_{20}, \mu_{11}, \mu_{02}\f$.

  \param cam [in]: Intrinsic camera parameters.
  \param circle [in]: 3D circle with parameters circle.p[] expressed in meters in the image plane.
  \param center [out]: Center of the corresponding ellipse in the image with coordinates
  expressed in pixels.
  \param mu20_p,mu11_p,mu02_p [out]: Centered moments expressed in pixels.

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
void
vpMeterPixelConversion::convertEllipse(const vpCameraParameters &cam,
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
  mu20_p = mu20_m*vpMath::sqr(cam.get_px());
  mu11_p = mu11_m*cam.get_px()*cam.get_py();
  mu02_p = mu02_m*vpMath::sqr(cam.get_py());
}

