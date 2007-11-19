/****************************************************************************
 *
 * $Id: vpFeaturePoint.cpp,v 1.9 2007-11-19 16:00:58 asaunier Exp $
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
 * 2D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeaturePoint.cpp
  \brief Class that defines 2D point visual feature
*/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeaturePoint.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>

// math
#include <visp/vpMath.h>

#include <visp/vpFeatureDisplay.h>



/*



attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory





*/

void
vpFeaturePoint::init()
{
    //feature dimension
    dim_s = 2 ;

    // memory allocation
    s.resize(dim_s) ;

    //default value Z (1 meters)
    set_Z(1) ;

}

vpFeaturePoint::vpFeaturePoint() : vpBasicFeature()
{
    init() ;
}


//! set the point depth
void
vpFeaturePoint::set_Z(const double _Z)
{
    Z = _Z ;
}

#define WARNING
//! get the point depth
double
vpFeaturePoint::get_Z() const
{
    return Z ;
}

//! set the point x-coordinates
void
vpFeaturePoint::set_x(const double _x)
{
    s[0] = _x ;
}
//! get the point x-coordinates
double
vpFeaturePoint::get_x() const
{
    return s[0] ;
}

//! set the point y-coordinates
void
vpFeaturePoint::set_y(const double _y)
{
    s[1] = _y ;
}
//! get the point y-coordinates
double
vpFeaturePoint::get_y() const
{
    return s[1] ;
}

//! set the point xy and Z-coordinates
void
vpFeaturePoint::set_xyZ(const double _x,
			const double _y,
			const double _Z)
{
  set_x(_x) ;
  set_y(_y) ;
  set_Z(_Z) ;
}

//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpFeaturePoint::interaction(const int select) const
{
  vpMatrix L ;

  L.resize(0,6) ;

  double x = get_x() ;
  double y = get_y() ;
  double Z = get_Z() ;

  if (Z < 0)
  {
    vpERROR_TRACE("Point is behind the camera ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    vpERROR_TRACE("Point Z coordinates is null ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point Z coordinates is null")) ;
  }

  if (vpFeaturePoint::selectX() & select )
  {
    vpMatrix Lx(1,6) ; Lx = 0;

    Lx[0][0] = -1/Z  ;
    Lx[0][1] = 0 ;
    Lx[0][2] = x/Z ;
    Lx[0][3] = x*y ;
    Lx[0][4] = -(1+x*x) ;
    Lx[0][5] = y ;

    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeaturePoint::selectY() & select )
  {
    vpMatrix Ly(1,6) ; Ly = 0;

    Ly[0][0] = 0 ;
    Ly[0][1]  = -1/Z ;
    Ly[0][2] = y/Z ;
    Ly[0][3] = 1+y*y ;
    Ly[0][4] = -x*y ;
    Ly[0][5] = -x ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }
  return L ;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeaturePoint::error(const vpBasicFeature &s_star,
		      const int select)
{
  vpColVector e(0) ;

  try{
    if (vpFeaturePoint::selectX() & select )
    {
      vpColVector ex(1) ;
      ex[0] = s[0] - s_star[0] ;

      e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeaturePoint::selectY() & select )
    {
      vpColVector ey(1) ;
      ey[0] = s[1] - s_star[1] ;
      e =  vpMatrix::stackMatrices(e,ey) ;
    }
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("caught a Matrix related error") ;
    std::cout <<std::endl << me << std::endl ;
    throw(me) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("caught another error") ;
    std::cout <<std::endl << me << std::endl ;
    throw(me) ;
  }


  return e ;

}


void
vpFeaturePoint::print(const int select ) const
{

  std::cout <<"Point:  Z=" << get_Z() ;
  if (vpFeaturePoint::selectX() & select )
    std::cout << " x=" << get_x() ;
  if (vpFeaturePoint::selectY() & select )
    std::cout << " y=" << get_y() ;
  std::cout <<std::endl ;
}


void
vpFeaturePoint::buildFrom(const double _x, const double _y, const double _Z)
{

  s[0] = _x ;
  s[1] = _y ;

  Z = _Z  ;

  if (Z < 0)
  {
    vpERROR_TRACE("Point is behind the camera ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    vpERROR_TRACE("Point Z coordinates is null ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point Z coordinates is null")) ;
  }

}


//! display point feature
void
vpFeaturePoint::display(const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			vpColor::vpColorType color) const
{
  try{
    double x,y ;
    x = get_x() ;
    y = get_y() ;

    vpFeatureDisplay::displayPoint(x,y, cam, I, color) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}
//! display point feature
void
vpFeaturePoint::display(const vpCameraParameters &cam,
      vpImage<unsigned char> &I,
      const bool usedistortion,
      vpColor::vpColorType color) const
{
  try{
    double x,y ;
    x = get_x() ;
    y = get_y() ;

    vpFeatureDisplay::displayPoint(x,y, cam, I, color, usedistortion) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}


//! for memory issue (used by the vpServo class only)
vpFeaturePoint *vpFeaturePoint::duplicate() const
{
  vpFeaturePoint *feature = new vpFeaturePoint ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
