
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPixelMeterConversion.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPixelMeterConversion.cpp,v 1.3 2005-06-28 13:26:11 marchand Exp $
 *
 * Description
 * ============
 * pixel to meter conversion
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpPixelMeterConversion.h
  \brief pixel to meter conversion

*/
#include<visp/vpPixelMeterConversion.h>
#include<visp/vpCameraParameters.h>
#include<visp/vpException.h>
#include<visp/vpMath.h>
#include<visp/vpDebug.h>


/*!
  \brief point coordinates conversion (u,v)->(x,y)

  \f$ x = (u-u_0)/px \f$ and  \f$ y = (v-v_0)/p_y  \f$
*/
void
vpPixelMeterConversion::convertPoint(const vpCameraParameters &cam,
				     const double u, const double v,
				     double &x, double &y)
{

  if (fabs(cam.px)<1e-6)
  {
    ERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
		      "Camera parameter px = 0")) ;
  }
  if (fabs(cam.py)<1e-6)
  {
    ERROR_TRACE("Camera parameter py = 0") ;
    throw(vpException(vpException::divideByZeroError,
		      "Camera parameter px = 0")) ;
  }
  x = (u - cam.u0)/cam.px ;
  y = (v - cam.v0)/cam.py ;
}
//! line coordinates conversion (rho,theta)
void
vpPixelMeterConversion::convertLine(const vpCameraParameters &cam,
				    const double rho_p, const double theta_p,
				    double &rho_m, double &theta_m)
{

  double u0 = cam.u0 ;  double v0 = cam.v0 ;
  double px = cam.px ;  double py = cam.py ;

  double co = cos(theta_p) ;
  double si = sin(theta_p) ;
  double d = vpMath::sqr(px*co)+vpMath::sqr(py*si) ;

  if (fabs(d)<1e-6)
  {
    ERROR_TRACE("division by zero") ;
    throw(vpException(vpException::divideByZeroError,
		      "division by zero")) ;
  }
  theta_m = atan2(si*py, co*px) ;
  rho_m = (rho_p - u0*co-v0*si)/sqrt(d) ;
}


void
vpPixelMeterConversion::convertMoment(const vpCameraParameters &cam,
				      int order,
				      const vpMatrix &moment_pixel,
				      vpMatrix &moment_meter)
{

  long double m[order][order];
  int p, r, q, t;
  int k;
  double yc = -cam.v0 ;
  double xc = -cam.u0 ;
  double my = 1/cam.py ;
  double mx = 1/cam.px ;


  for (k=0; k<order; k++) // itération correspondant à l'ordre du moment
  {
    for (p=0 ; p<order; p++) // itération en X
      for (q=0; q<order; q++) // itération en Y
	if (p+q==k) // on est bien dans la matrice triangulaire supérieure
	{
	  m[p][q] = 0; // initialisation à 0
	  for( r=0; r<=p; r++) // somme externe
	    for( t=0; t<=q; t++) // somme interne
	    {
	      m[p][q] +=
		vpMath::comb(p, r) * vpMath::comb(q, t)
		* vpMath::exp(xc, p-r) * vpMath::exp(yc, q-t)
		* moment_pixel[r][t];

	    }
	}

  }

  for (k=0; k<order; k++) // itération correspondant à l'ordre du moment
    for (p=0 ; p<order; p++)
      for (q=0; q<order; q++)
	if (p+q==k)
	{
	  m[p][q] *= vpMath::exp(mx,1+p) * vpMath::exp(my,1+q);
	}

  for (k=0; k<order; k++) // itération correspondant à l'ordre du moment
    for (p=0 ; p<order; p++)
      for (q=0; q<order; q++)
	if (p+q==k)
	{
	  moment_meter[p][q] = m[p][q];
	}

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

