

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilderEllipse.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilderEllipse.cpp,v 1.4 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 *     conversion between tracker and visual feature
 *     and visual feature Ellipse
 *
 * ++++++++++++
 */

/*!
  \file vpFeatureBuilderEllipse.cpp
  \brief  conversion between tracker
  and visual feature Ellipse
*/

#include <visp/vpFeatureBuilder.h>


#include <visp/vpMath.h>

// create vpFeatureEllipse feature
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCircle &t )
{
  try
  {

    // 3D data
    double alpha = t.cP[0] ;
    double beta = t.cP[1] ;
    double gamma = t.cP[2] ;

    double X0 = t.cP[3] ;
    double Y0 = t.cP[4] ;
    double Z0 = t.cP[5] ;

    // equation p 318 prior eq (39)
    double d = alpha*X0 + beta*Y0 + gamma*Z0 ;

    double A = alpha / d ;
    double B = beta / d ;
    double C = gamma / d ;

    s.setABC(A,B,C) ;


    //2D data
    s.buildFrom( t.p[0],  t.p[1],  t.p[2],  t.p[3],  t.p[4] ) ;

  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
void vpFeatureBuilder::create(vpFeatureEllipse &s,  const vpSphere &t)
{
  try
  {

    // 3D data
    double X0 = t.cP[0] ;
    double Y0 = t.cP[1] ;
    double Z0 = t.cP[2] ;
    double R = t.cP[3] ;

    double d = vpMath::sqr(X0) + vpMath::sqr(Y0) + vpMath::sqr(Z0) -
      vpMath::sqr(R) ;


    double A = X0 / d ;
    double B = Y0 / d ;
    double C = Z0 / d ;

    s.setABC(A,B,C) ;

    //2D data
    s.buildFrom( t.p[0],  t.p[1],  t.p[2],  t.p[3],  t.p[4] ) ;


  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

// create vpFeatureEllipse feature
void vpFeatureBuilder::create(vpFeatureEllipse &s,
			      const vpCameraParameters &cam,
			      const vpDot &t )
{
  try
  {

    int order = 3 ;
    vpMatrix mp(order,order) ; mp =0 ;
    vpMatrix m(order,order) ; m = 0 ;

    mp[0][0] = t.m00 ;
    mp[1][0] = t.m10;
    mp[0][1] = t.m01 ;
    mp[2][0] = t.m20 ;
    mp[1][1] = t.m11 ;
    mp[0][2] = t.m02 ;

    vpPixelMeterConversion::convertMoment(cam,order,mp,m) ;

    double  m00 = m[0][0] ;
    double  m01 = m[0][1] ;
    double  m10 = m[1][0] ;
    double  m02 = m[0][2] ;
    double  m11 = m[1][1] ;
    double  m20 = m[2][0] ;

    double xc = m10/m00 ; // sum j /S
    double yc = m01/m00 ; // sum i /S

    double mu20 = 4*(m20 - m00*vpMath::sqr(xc))/(m00) ;
    double mu02 = 4*(m02 - m00*vpMath::sqr(yc))/(m00) ;
    double mu11 = 4*(m11 - m00*xc*yc)/(m00) ;

    s.buildFrom(xc, yc,  mu20, mu11, mu02  ) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}

// create vpFeatureEllipse feature
void vpFeatureBuilder::create(vpFeatureEllipse &s,
			      const vpCameraParameters &cam,
			      const vpDot2 &t )
{
  try
  {

    int order = 3 ;
    vpMatrix mp(order,order) ; mp =0 ;
    vpMatrix m(order,order) ; m = 0 ;

    mp[0][0] = t.m00 ;
    mp[1][0] = t.m10;
    mp[0][1] = t.m01 ;
    mp[2][0] = t.m20 ;
    mp[1][1] = t.m11 ;
    mp[0][2] = t.m02 ;

    vpPixelMeterConversion::convertMoment(cam,order,mp,m) ;

    double  m00 = m[0][0] ;
    double  m01 = m[0][1] ;
    double  m10 = m[1][0] ;
    double  m02 = m[0][2] ;
    double  m11 = m[1][1] ;
    double  m20 = m[2][0] ;

    double xc = m10/m00 ; // sum j /S
    double yc = m01/m00 ; // sum i /S

    double mu20 = 4*(m20 - m00*vpMath::sqr(xc))/(m00) ;
    double mu02 = 4*(m02 - m00*vpMath::sqr(yc))/(m00) ;
    double mu11 = 4*(m11 - m00*xc*yc)/(m00) ;

    s.buildFrom(xc, yc,  mu20, mu11, mu02  ) ;
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
