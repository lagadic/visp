

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilderLine.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilderLine.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     conversion between tracker and visual feature
 *     and visual feature Line
 *
 * ++++++++++++
 */

/*!
  \file vpFeatureBuilderLine.cpp
  \brief  conversion between tracker
  and visual feature Line
*/

#include<visp/vpFeatureBuilder.h>


#include<visp/vpMath.h>

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
    ERROR_TRACE(" ") ;
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
    ERROR_TRACE(" ") ;
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

    //  TRACE("pixel %f %f",rhop, thetap) ;
    vpPixelMeterConversion::convertLine(cam,rhop,thetap, rho,theta) ;
    //   TRACE("meter %f %f",rho, theta) ;
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
    //   TRACE("meter %f %f",rho, theta) ;

  }
  catch(...)
    {
      ERROR_TRACE(" ") ;
      throw ;
    }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
