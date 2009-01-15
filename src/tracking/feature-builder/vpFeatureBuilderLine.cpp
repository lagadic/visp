/****************************************************************************
 *
 * $Id: vpFeatureBuilderLine.cpp,v 1.6 2009-01-15 15:42:46 nmelchio Exp $
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
 * Conversion between tracker and visual feature line.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilderLine.cpp
  \brief  conversion between tracker
  and visual feature Line
*/

#include <visp/vpFeatureBuilder.h>


#include <visp/vpMath.h>

// create vpFeatureLine feature
void vpFeatureBuilder::create(vpFeatureLine &s, const vpLine &t )
{
  try
  {
    double A,B,C,D ;
    s.setRhoTheta(t.getRho(),t.getTheta()) ;

    if (fabs(t.cP[3]) > fabs(t.cP[7])) // |D1| > |D2|
    {
      A = t.cP[0] ;
      B = t.cP[1] ;
      C = t.cP[2] ;
      D = t.cP[3] ;
    }
    else
    {
      A = t.cP[4] ;
      B = t.cP[5] ;
      C = t.cP[6] ;
      D = t.cP[7] ;
    }


    s.setABCD(A,B,C,D) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

}
void vpFeatureBuilder::create(vpFeatureLine &s,
			      const vpCylinder &t,
			      const int line)
{
  try
  {

    double a = t.getA() ;
    double b = t.getB() ;
    double c = t.getC() ;

    double x0 = t.getX() ;
    double y0 = t.getY() ;
    double z0 = t.getZ() ;

    double R = t.getR() ;

    double D =
      vpMath::sqr(x0) + vpMath::sqr(y0) + vpMath::sqr(z0)
      - vpMath::sqr(R)
      - vpMath::sqr(a*x0 + b*y0 + c*z0);

    double alpha1 = (1 - a*a)*x0 - a*b*y0  -   a*c*z0;
    double beta1 = -a*b*x0  +  (1 - b*b)*y0  - b*c*z0;
    double gamma1 = -a*c*x0  - b*c*y0   + (1 - c*c)*z0;

    D*=-1 ;

    if (D<0)
    {
      alpha1*=-1 ;
      beta1*=-1 ;
      gamma1*=-1 ;
      D*=-1 ;
    }

    s.setABCD(alpha1,beta1,gamma1,D) ;

    if (line==vpCylinder::line1)
    {

      s.setRhoTheta(t.getRho1(),t.getTheta1()) ;

    }
    else
    {

      s.setRhoTheta(t.getRho2(),t.getTheta2()) ;
    }
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}


void
vpFeatureBuilder::create(vpFeatureLine &s,
			 const vpCameraParameters &cam,
			 const vpMeLine &t)
{
  double rhop ;
  double thetap ;

  double rho ;
  double theta ;

  try{
    rhop = t.getRho() ;
    thetap = t.getTheta();

    //Gives the rho and theta coordinates in the (u,v) coordinate system.
    if (thetap >= 0 && thetap < M_PI/2)
    {
      thetap = M_PI/2 - thetap;
    }

    else if (thetap >= M_PI/2 && thetap < 3*M_PI/2)
    {
      thetap = 3*M_PI/2 + M_PI - thetap;
    }

    else if (thetap >= 3*M_PI/2 && thetap <= 2*M_PI)
    {
      thetap = M_PI/2 + 2*M_PI - thetap;
    }

    //  vpTRACE("pixel %f %f",rhop, thetap) ;
    vpPixelMeterConversion::convertLine(cam,rhop,thetap, rho,theta) ;
    //   vpTRACE("meter %f %f",rho, theta) ;
    /*

    while(theta < -M_PI)	theta += 2*M_PI ;
    while(theta >= M_PI)	theta -= 2*M_PI ;

    // If theta is between -90 and -180 get the equivalent
    // between 0 and 90
    if(theta <-M_PI/2)
    {
      theta += M_PI ;
      rho *= -1 ;
    }
    // If theta is between 90 and 180 get the equivalent
    // between 0 and -90
    if(theta >M_PI/2)
    {
      theta -= M_PI ;
      rho *= -1 ;
    }
    */
    s.buildFrom(rho,theta) ;
    //   vpTRACE("meter %f %f",rho, theta) ;

  }
  catch(...)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
