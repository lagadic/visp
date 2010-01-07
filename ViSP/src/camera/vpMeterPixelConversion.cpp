/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

