/****************************************************************************
 *
 * $Id: vpMeterPixelConversion.cpp,v 1.6 2006-06-23 14:45:05 brenier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 *
 *****************************************************************************/

/*!
  \file vpMeterPixelConversion.h
  \brief meter to pixel  conversion

*/

#include <visp/vpMeterPixelConversion.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpException.h>
#include <visp/vpMath.h>
#include <visp/vpDebug.h>

  //! point coordinates conversion (u,v)->(x,y)
void
vpMeterPixelConversion::convertPoint(const vpCameraParameters &cam,
				     const double x, const double y,
				     double &u, double &v)

{
  u = x * cam.get_px() + cam.get_u0() ;
  v = y * cam.get_py() + cam.get_v0() ;
}
//! line coordinates conversion (rho,theta)
void
vpMeterPixelConversion::convertLine(const vpCameraParameters &cam,
				    const double rho_m, const double theta_m,
				    double &rho_p, double &theta_p)
{


  double u0 = cam.get_u0() ;  double v0 = cam.get_v0() ;
  double px = cam.get_px() ;  double py = cam.get_py() ;


  double co = cos(theta_m) ;
  double si = sin(theta_m) ;
  double d = sqrt(vpMath::sqr(py*co) + vpMath::sqr(px*si)) ;

  if (fabs(d)<1e-6)
  {
    vpERROR_TRACE("division by zero") ;
    throw(vpException(vpException::divideByZeroError,
		      "division by zero")) ;
  }

  theta_p = atan2(px*si, py*co) ;
  rho_p = (px*py*rho_m + u0*py*co + v0*px*si) ;
  rho_p /= d ;


}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

