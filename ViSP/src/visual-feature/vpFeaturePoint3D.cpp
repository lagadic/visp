/****************************************************************************
 *
 * $Id: vpFeaturePoint3D.cpp,v 1.13 2008-02-26 10:32:11 asaunier Exp $
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
 * 3D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeaturePoint3D.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>





/*


attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory





*/

void
vpFeaturePoint3D::init()
{
    //feature dimension
    dim_s = 3 ;

    // memory allocation
    s.resize(dim_s) ;

    //default value XYZ
    set_XYZ(0,0,1) ;

}

vpFeaturePoint3D::vpFeaturePoint3D() : vpBasicFeature()
{
    init() ;
}


//! set the point X coordinates
void
vpFeaturePoint3D::set_X(const double X)
{
    s[0] = X ;
}

//! get the point X coordinates
double
vpFeaturePoint3D::get_X() const
{
    return s[0] ;
}

//! set the point Y coordinates
void
vpFeaturePoint3D::set_Y(const double Y)
{
    s[1] = Y ;
}

//! get the point Y coordinates
double
vpFeaturePoint3D::get_Y() const
{
    return s[1] ;
}

//! set the point depth
void
vpFeaturePoint3D::set_Z(const double Z)
{
    s[2] = Z ;
}

//! get the point depth
double
vpFeaturePoint3D::get_Z() const
{
    return s[2] ;
}

//! set the point XY and Z-coordinates
void
vpFeaturePoint3D::set_XYZ(const double X,
			  const double Y,
			  const double Z)
{
  set_X(X) ;
  set_Y(Y) ;
  set_Z(Z) ;
}

//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpFeaturePoint3D::interaction(const int select) const
{
  vpMatrix L ;

  L.resize(0,6) ;

  double X = get_X() ;
  double Y = get_Y() ;
  double Z = get_Z() ;

  if (vpFeaturePoint3D::selectX() & select )
  {
    vpMatrix Lx(1,6) ; Lx = 0;

    Lx[0][0] = -1  ;
    Lx[0][1] = 0 ;
    Lx[0][2] = 0 ;
    Lx[0][3] = 0 ;
    Lx[0][4] = -Z ;
    Lx[0][5] = Y ;

    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeaturePoint3D::selectY() & select )
  {
    vpMatrix Ly(1,6) ; Ly = 0;

    Ly[0][0] = 0 ;
    Ly[0][1] = -1 ;
    Ly[0][2] = 0 ;
    Ly[0][3] = Z ;
    Ly[0][4] = 0 ;
    Ly[0][5] = -X ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }
  if (vpFeaturePoint3D::selectZ() & select )
  {
    vpMatrix Lz(1,6) ; Lz = 0;

    Lz[0][0] = 0 ;
    Lz[0][1] = 0 ;
    Lz[0][2] = -1 ;
    Lz[0][3] = -Y ;
    Lz[0][4] = X ;
    Lz[0][5] = 0 ;

    L = vpMatrix::stackMatrices(L,Lz) ;
  }
  return L ;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeaturePoint3D::error(const vpBasicFeature &s_star,
		      const int select)
{
  vpColVector e(0) ;

  try{
    if (vpFeaturePoint3D::selectX() & select )
    {
      vpColVector ex(1) ;
      ex[0] = s[0] - s_star[0] ;

      e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeaturePoint3D::selectY() & select )
    {
      vpColVector ey(1) ;
      ey[0] = s[1] - s_star[1] ;
      e =  vpMatrix::stackMatrices(e,ey) ;
    }

    if (vpFeaturePoint3D::selectZ() & select )
    {
      vpColVector ez(1) ;
      ez[0] = s[2] - s_star[2] ;
      e =  vpMatrix::stackMatrices(e,ez) ;
    }
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("caught a Matric related error") ;
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

/*!
  This function has no meaning related to 3D point. It is not implemented.

  \exception vpException::notImplementedError : This function has no meaning
  related to 3D point.

*/
vpColVector
vpFeaturePoint3D::error(const int /* select */)
{

  vpERROR_TRACE("This function has no meaning related to 3D point ") ;
  vpERROR_TRACE("It is not implemented ") ;

  throw(vpException(vpException::notImplementedError,
			   "This function has no meaning related to 3D point")) ;
}

void
vpFeaturePoint3D::buildFrom(const vpPoint &p)
{

  // cP is expressed in homogeneous coordinates
  // we devide by the fourth coordinate
  s[0] = p.cP[0]/p.cP[3]  ;
  s[1] = p.cP[1]/p.cP[3]  ;
  s[2] = p.cP[2]/p.cP[3]  ;

  double Z = s[2] ;
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

void
vpFeaturePoint3D::buildFrom(const double X, const double Y, const double Z)
{

  s[0] = X ;
  s[1] = Y ;
  s[2] = Z  ;

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


void
vpFeaturePoint3D::print(const int select ) const
{

  std::cout <<"Point3D:  "  ;
  if (vpFeaturePoint3D::selectX() & select )
    std::cout << " X=" << get_X() ;
  if (vpFeaturePoint3D::selectY() & select )
    std::cout << " Y=" << get_Y() ;
  if (vpFeaturePoint3D::selectZ() & select )
    std::cout << " Z=" << get_Z() ;
  std::cout <<std::endl ;
}


vpFeaturePoint3D *vpFeaturePoint3D::duplicate() const
{
  vpFeaturePoint3D *feature = new vpFeaturePoint3D ;
  return feature ;
}

/*!

  Not implemented.
*/
void
vpFeaturePoint3D::display(const vpCameraParameters &/*cam*/,
			  vpImage<unsigned char> &/* I */,
			  vpColor::vpColorType /* color */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.
 */
void
vpFeaturePoint3D::display(const vpCameraParameters &/*cam*/,
                          vpImage<vpRGBa> &/* I */,
                          vpColor::vpColorType /* color */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
