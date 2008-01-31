/****************************************************************************
 *
 * $Id: vpFeatureVanishingPoint.cpp,v 1.11 2008-01-31 14:59:35 asaunier Exp $
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
 * 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/


/*!  \file vpFeatureVanishingPoint.cpp
  \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/
#include <visp/vpBasicFeature.h>
#include <visp/vpFeatureVanishingPoint.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>

// math
#include <visp/vpMath.h>

#include <visp/vpFeatureDisplay.h>

void
vpFeatureVanishingPoint::init()
{
    //feature dimension
    dim_s = 2 ;

    // memory allocation
    s.resize(dim_s) ;

    //Z not required  (infinity)
    //set_Z(1) ;

}
vpFeatureVanishingPoint::vpFeatureVanishingPoint() : vpBasicFeature()
{
    init() ;
}


//! set the point x-coordinates
void
vpFeatureVanishingPoint::set_x(const double _x)
{
    s[0] = _x ;
}
//! get the point x-coordinates
double
vpFeatureVanishingPoint::get_x() const
{
    return s[0] ;
}

//! set the point y-coordinates
void
vpFeatureVanishingPoint::set_y(const double _y)
{
    s[1] = _y ;
}
//! get the point y-coordinates
double
vpFeatureVanishingPoint::get_y() const
{
    return s[1] ;
}


//! set the point xy coordinates
void
vpFeatureVanishingPoint::set_xy(const double _x,
			const double _y)
{
  set_x(_x) ;
  set_y(_y) ;
}


//! compute the interaction matrix from a subset of the possible features
vpMatrix
vpFeatureVanishingPoint::interaction(const int select) const
{
  vpMatrix L ;

  L.resize(0,6) ;

  double x = get_x() ;
  double y = get_y() ;

  if (vpFeatureVanishingPoint::selectX() & select )
  {
    vpMatrix Lx(1,6) ; Lx = 0;

    Lx[0][0] = 0.  ;
    Lx[0][1] = 0. ;
    Lx[0][2] = 0. ;
    Lx[0][3] = x*y ;
    Lx[0][4] = -(1+x*x) ;
    Lx[0][5] = y ;

    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeatureVanishingPoint::selectY() & select )
  {
    vpMatrix Ly(1,6) ; Ly = 0;

    Ly[0][0] = 0 ;
    Ly[0][1] = 0. ;
    Ly[0][2] = 0. ;
    Ly[0][3] = 1+y*y ;
    Ly[0][4] = -x*y ;
    Ly[0][5] = -x ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }
  return L ;
}


/*! compute the error between two visual features from a subset of the possible
  features
 */
vpColVector
vpFeatureVanishingPoint::error(const vpBasicFeature &s_star,
		      const int select)
{
  vpColVector e(0) ;

  try{
    if (vpFeatureVanishingPoint::selectX() & select )
    {
      vpColVector ex(1) ;
      ex[0] = s[0] - s_star[0] ;

      e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeatureVanishingPoint::selectY() & select )
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
vpFeatureVanishingPoint::print(const int select ) const
{

  std::cout <<"Point: " <<std::endl;
  if (vpFeatureVanishingPoint::selectX() & select )
    std::cout << " x=" << get_x() ;
  if (vpFeatureVanishingPoint::selectY() & select )
    std::cout << " y=" << get_y() ;
  std::cout <<std::endl ;
}


void
vpFeatureVanishingPoint::buildFrom(const double _x, const double _y)
{
  s[0] = _x ;
  s[1] = _y ;
}


//! display VanishingPoint feature
void
vpFeatureVanishingPoint::display(const vpCameraParameters &cam,
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


/*! for memory issue (used by the vpServo class only)
 */
vpFeatureVanishingPoint *vpFeatureVanishingPoint::duplicate() const
{
  vpFeatureVanishingPoint *feature = new vpFeatureVanishingPoint ;
  return feature ;
}
