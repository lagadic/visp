/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
				      unsigned int order,
				      const vpMatrix &moment_pixel,
				      vpMatrix &moment_meter)
{

  vpMatrix m(order, order);
  double yc = -cam.v0 ;
  double xc = -cam.u0 ;

  for (unsigned int k=0; k<order; k++) // itération correspondant à l'ordre du moment
  {
    for (unsigned int p=0 ; p<order; p++) // itération en X
      for (unsigned int q=0; q<order; q++) // itération en Y
	if (p+q==k) // on est bien dans la matrice triangulaire supérieure
	{
	  m[p][q] = 0; // initialisation à 0
	  for(unsigned int r=0; r<=p; r++) // somme externe
	    for(unsigned int t=0; t<=q; t++) // somme interne
	    {
	      m[p][q] +=
		vpMath::comb(p, r) * vpMath::comb(q, t)
		* pow(xc, (int)(p-r)) * pow(yc, (int)(q-t))
		* moment_pixel[r][t];

	    }
	}

  }

  for (unsigned int k=0; k<order; k++) // itération correspondant à l'ordre du moment
    for (unsigned int p=0 ; p<order; p++)
      for (unsigned int q=0; q<order; q++)
	if (p+q==k)
	{
	  m[p][q] *= pow(cam.inv_px,(int)(1+p)) * pow(cam.inv_py,(int)(1+q));
	}

  for (unsigned int k=0; k<order; k++) // itération correspondant à l'ordre du moment
    for (unsigned int p=0 ; p<order; p++)
      for (unsigned int q=0; q<order; q++)
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

