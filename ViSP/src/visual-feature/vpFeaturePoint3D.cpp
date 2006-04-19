
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeaturePoint3D.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeaturePoint3D.cpp,v 1.4 2006-04-19 09:01:23 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines the 3D point visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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
    ERROR_TRACE("caught a Matric related error") ;
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

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeaturePoint3D::error(const int select)
{

  ERROR_TRACE("This function has no meaning related to 3D point ") ;
  ERROR_TRACE("It is not implemented ") ;

  throw(vpException(vpException::notImplementedError,
			   "This function has no meaning related to 3D point")) ;

  return vpColVector() ;

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
    ERROR_TRACE("Point is behind the camera ") ;
    cout <<"Z = " << Z << endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    ERROR_TRACE("Point Z coordinates is null ") ;
    cout <<"Z = " << Z << endl ;

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
    ERROR_TRACE("Point is behind the camera ") ;
    cout <<"Z = " << Z << endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    ERROR_TRACE("Point Z coordinates is null ") ;
    cout <<"Z = " << Z << endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point Z coordinates is null")) ;
  }

}


void
vpFeaturePoint3D::print(const int select ) const
{

  cout <<"Point3D:  "  ;
  if (vpFeaturePoint3D::selectX() & select )
    cout << " X=" << get_X() ;
  if (vpFeaturePoint3D::selectY() & select )
    cout << " Y=" << get_Y() ;
  if (vpFeaturePoint3D::selectZ() & select )
    cout << " Z=" << get_Z() ;
  cout <<endl ;
}


vpFeaturePoint3D *vpFeaturePoint3D::duplicate() const
{
  vpFeaturePoint3D *feature = new vpFeaturePoint3D ;
  return feature ;
}


void
vpFeaturePoint3D::display(const vpCameraParameters &cam,
			  vpImage<unsigned char> &I,
			  int color) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    ERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
