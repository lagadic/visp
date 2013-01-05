/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpMeterPixelConversion.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>
#include <visp/vpMath.h>
#include <visp/vpDebug.h>

//! line coordinates conversion (rho,theta)
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


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

