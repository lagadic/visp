/****************************************************************************
 *
 * $Id: vpPixelMeterConversion.cpp,v 1.11 2008-01-31 14:43:50 asaunier Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Pixel to meter conversion.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpPixelMeterConversion.cpp
  \brief Pixel to meter conversion.
*/
#include<visp/vpPixelMeterConversion.h>
#include<visp/vpCameraParameters.h>
#include<visp/vpException.h>
#include<visp/vpMath.h>
#include<visp/vpDebug.h>


//! line coordinates conversion (rho,theta)
void
vpPixelMeterConversion::convertLine(const vpCameraParameters &cam,
				    const double &rho_p, const double &theta_p,
				    double &rho_m, double &theta_m)
{
  double co = cos(theta_p) ;
  double si = sin(theta_p) ;
  double d = vpMath::sqr(cam.px*co)+vpMath::sqr(cam.py*si) ;

  if (fabs(d)<1e-6)
  {
    vpERROR_TRACE("division by zero") ;
    throw(vpException(vpException::divideByZeroError,
		      "division by zero")) ;
  }
  theta_m = atan2(si*cam.py, co*cam.px) ;
  rho_m = (rho_p - cam.u0*co-cam.v0*si)/sqrt(d) ;
}


void
vpPixelMeterConversion::convertMoment(const vpCameraParameters &cam,
				      int order,
				      const vpMatrix &moment_pixel,
				      vpMatrix &moment_meter)
{

  vpMatrix m(order, order);
  int p, r, q, t;
  int k;
  double yc = -cam.v0 ;
  double xc = -cam.u0 ;

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
		* pow(xc, p-r) * pow(yc, q-t)
		* moment_pixel[r][t];

	    }
	}

  }

  for (k=0; k<order; k++) // itération correspondant à l'ordre du moment
    for (p=0 ; p<order; p++)
      for (q=0; q<order; q++)
	if (p+q==k)
	{
	  m[p][q] *= pow(cam.inv_px,1+p) * pow(cam.inv_py,1+q);
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

