
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMeterPixelConversion.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpMeterPixelConversion.cpp,v 1.2 2005-06-28 13:26:11 marchand Exp $
 *
 * Description
 * ============
 *  meter to pixel  conversion
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpMeterPixelConversion.h
  \brief meter to pixel  conversion

*/

#include<visp/vpMeterPixelConversion.h>
#include<visp/vpCameraParameters.h>
#include<visp/vpException.h>
#include<visp/vpMath.h>
#include<visp/vpDebug.h>

  //! point coordinates conversion (u,v)->(x,y)
void
vpMeterPixelConversion::convertPoint(const vpCameraParameters &cam,
				     const double x, const double y,
				     double &u, double &v)

{
  u = x * cam.px + cam.u0 ;
  v = y * cam.py + cam.v0 ;
}
//! line coordinates conversion (rho,theta)
void
vpMeterPixelConversion::convertLine(const vpCameraParameters &cam,
				    const double rho_m, const double theta_m,
				    double &rho_p, double &theta_p)
{


  double u0 = cam.u0 ;  double v0 = cam.v0 ;
  double px = cam.px ;  double py = cam.py ;


  double co = cos(theta_m) ;
  double si = sin(theta_m) ;
  double d = sqrt(vpMath::sqr(py*co) + vpMath::sqr(px*si)) ;

  if (fabs(d)<1e-6)
  {
    ERROR_TRACE("division by zero") ;
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

