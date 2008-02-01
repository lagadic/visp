/****************************************************************************
 *
 * $Id: vpFeatureBuilderPoint.cpp,v 1.13 2008-02-01 15:11:39 fspindle Exp $
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
 * Conversion between tracker and visual feature point.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilderPoint.cpp
  \brief  conversion between tracker
  and visual feature Point
*/
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureException.h>
#include <visp/vpException.h>

void vpFeatureBuilder::create(vpFeaturePoint &s,
			      const vpCameraParameters &cam,
			      const vpDot &t)
{
  try
  {
    double x=0, y=0;

    double u = t.get_u() ;
    double v = t.get_v() ;

    vpPixelMeterConversion::convertPoint(cam,u,v,x,y) ;

    s.set_x(x) ;
    s.set_y(y) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}
void vpFeatureBuilder::create(vpFeaturePoint &s,
			      const vpCameraParameters &cam,
			      const vpDot2 &t)
{
  try
  {
    double x=0, y=0;

    double u = t.get_u() ;
    double v = t.get_v() ;

    vpPixelMeterConversion::convertPoint(cam,u,v,x,y) ;

    s.set_x(x) ;
    s.set_y(y) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
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
      vpERROR_TRACE("Point is behind the camera ") ;
      std::cout <<"Z = " << s.get_Z() << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
			       "Point is behind the camera ")) ;
    }

    if (fabs(s.get_Z()) < 1e-6)
    {
      vpERROR_TRACE("Point Z coordinates is null ") ;
      std::cout <<"Z = " << s.get_Z() << std::endl ;

      throw(vpFeatureException(vpFeatureException::badInitializationError,
			       "Point Z coordinates is null")) ;
    }

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
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


    double u=0, v=0;
    vpMeterPixelConversion::convertPoint(goodCam,x,y,u,v) ;
    vpPixelMeterConversion::convertPoint(wrongCam,u,v,x,y) ;


    s.set_x(x) ;
    s.set_y(y) ;
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
