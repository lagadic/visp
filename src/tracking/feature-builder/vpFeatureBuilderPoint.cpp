

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilderPoint.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilderPoint.cpp,v 1.3 2005-07-22 09:37:21 fspindle Exp $
 *
 * Description
 * ============
 *     conversion between tracker and visual feature
 *     and visual feature Point
 *
 * ++++++++++++
 */

/*!
  \file vpFeatureBuilderPoint.cpp
  \brief  conversion between tracker
  and visual feature Point
*/
#include<visp/vpFeatureBuilder.h>
#include<visp/vpFeatureException.h>
#include<visp/vpException.h>

void vpFeatureBuilder::create(vpFeaturePoint &s,
			      const vpCameraParameters &cam,
			      const vpDot &t)
{
  try
  {
    double x,y ;

    double u = t.J() ;
    double v = t.I() ;

    vpPixelMeterConversion::convertPoint(cam,u,v,x,y) ;

    s.set_x(x) ;
    s.set_y(y) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
void vpFeatureBuilder::create(vpFeaturePoint &s,
			      const vpCameraParameters &cam,
			      const vpDot2 &t)
{
  try
  {
    double x,y ;

    double u = t.J() ;
    double v = t.I() ;

    vpPixelMeterConversion::convertPoint(cam,u,v,x,y) ;

    s.set_x(x) ;
    s.set_y(y) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

void
vpFeatureBuilder::create(vpFeaturePoint &s, const vpPoint &t)
{
  try
  {



    s.set_x( t.get_x()) ;
    s.set_y( t.get_y()) ;

    s.set_Z( t.cP[2]/t.cP[3])  ;

    if (s.get_Z() < 0)
    {
      ERROR_TRACE("Point is behind the camera ") ;
      cout <<"Z = " << s.get_Z() << endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
			       "Point is behind the camera ")) ;
    }

    if (fabs(s.get_Z()) < 1e-6)
    {
      ERROR_TRACE("Point Z coordinates is null ") ;
      cout <<"Z = " << s.get_Z() << endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
			       "Point Z coordinates is null")) ;
    }

  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

//! introduce noise in the transfert
void
vpFeatureBuilder::create(vpFeaturePoint &s,
		     const vpCameraParameters &goodCam,
		     const vpCameraParameters &wrongCam,
		     const vpPoint &t)
{
  try
  {
    double x = t.p[0] ;
    double y = t.p[1] ;

     s.set_Z( t.cP[2]/t.cP[3])  ;


    double u,v;
    vpMeterPixelConversion::convertPoint(goodCam,x,y,u,v) ;
    vpPixelMeterConversion::convertPoint(wrongCam,u,v,x,y) ;


    s.set_x(x) ;
    s.set_y(y) ;
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
