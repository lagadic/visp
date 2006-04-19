/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureVanishingPoint.cpp
 * Project:   ViSP2
 * Author:    Odile Bourquardez
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureVanishingPoint.cpp,v 1.2 2006-04-19 09:01:23 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
    ERROR_TRACE("caught a Matrix related error") ;
    cout <<endl << me << endl ;
    throw(me) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("caught another error") ;
    cout <<endl << me << endl ;
    throw(me) ;
  }
  return e ;
}



void
vpFeatureVanishingPoint::print(const int select ) const
{

  cout <<"Point: " <<endl;
  if (vpFeatureVanishingPoint::selectX() & select )
    cout << " x=" << get_x() ;
  if (vpFeatureVanishingPoint::selectY() & select )
    cout << " y=" << get_y() ;
  cout <<endl ;
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
			int color ) const
{
  try{
    double x,y ;
    x = get_x() ;
    y = get_y() ;

    vpFeatureDisplay::displayPoint(x,y, cam, I, color) ;

  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
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
