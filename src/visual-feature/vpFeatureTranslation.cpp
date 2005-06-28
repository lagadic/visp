
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureTranslation.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureTranslation.cpp,v 1.2 2005-06-28 13:05:13 marchand Exp $
 *
 * Description
 * ============
 *     class that defines the thetaU visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeatureTranslation.h>

#include <visp/vpMath.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \file vpFeatureTranslation.cpp
  \brief class that defines 3D translation visual feature
*/
/*

attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory

*/

void
vpFeatureTranslation::init()
{
    //feature dimension
    dim_s = 3 ;

    // memory allocation
    s.resize(dim_s) ;

}

vpFeatureTranslation::vpFeatureTranslation() : vpBasicFeature()
{
    init() ;
}


vpFeatureTranslation::vpFeatureTranslation(vpHomogeneousMatrix &_cdMc) : vpBasicFeature()
{
    init() ;

    buildFrom(_cdMc) ;
}

void
vpFeatureTranslation::buildFrom(const vpHomogeneousMatrix &_cdMc)
{
    cdMc = _cdMc ;
    s[0] = cdMc[0][3] ;
    s[1] = cdMc[1][3] ;
    s[2] = cdMc[2][3] ;
}


double vpFeatureTranslation::get_Tx()  const
{
    return s[0] ;
}


double vpFeatureTranslation::get_Ty()   const
{
    return s[1] ;
}


double
vpFeatureTranslation::get_Tz() const
{
    return  s[2]  ;
}


//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpFeatureTranslation::interaction(const int select) const
{

  vpMatrix L ;
  L.resize(0,6) ;

  //This version is a simplification
  if (vpFeatureTranslation::selectTx() & select )
  {
    vpMatrix Lx(1,6) ;

    for (int i=0 ; i < 3 ; i++)
      Lx[0][i] = cdMc[0][i] ;
    Lx[0][3] = 0 ;    Lx[0][4] = 0 ;    Lx[0][5] = 0 ;

    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeatureTranslation::selectTy() & select )
  {
    vpMatrix Ly(1,6) ;

    for (int i=0 ; i < 3 ; i++)
      Ly[0][i] = cdMc[1][i] ;
    Ly[0][3] = 0 ;    Ly[0][4] = 0 ;    Ly[0][5] = 0 ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }

  if (vpFeatureTranslation::selectTz() & select )
  {
    vpMatrix Lz(1,6) ;

    for (int i=0 ; i < 3 ; i++)
      Lz[0][i] = cdMc[2][i] ;
    Lz[0][3] = 0 ;    Lz[0][4] = 0 ;    Lz[0][5] = 0 ;

    L = vpMatrix::stackMatrices(L,Lz) ;
  }

  return L ;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeatureTranslation::error(const vpBasicFeature &s_star,
	       const int select)
{
    vpColVector e(0) ;


    if (s_star.get_s().sumSquare() > 1e-6)
    {
      ERROR_TRACE("\n\t\t s* should be 0") ;
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
			       "\n\t\t s* should be 0")) ;
    }


    if (vpFeatureTranslation::selectTx() & select )
    {
	vpColVector ex(1) ;
	ex[0] = s[0]  ;
	e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeatureTranslation::selectTy() & select )
    {
	vpColVector ey(1) ;
	ey[0] = s[1] ;
	e = vpMatrix::stackMatrices(e,ey) ;
    }

    if (vpFeatureTranslation::selectTz() & select )
    {
	vpColVector ez(1) ;
	ez[0] = s[2] ;
	e = vpMatrix::stackMatrices(e,ez) ;
    }


    return e ;

}

void
vpFeatureTranslation::print(const int select) const
{
  cout <<"Translation 3D: " << s.t() ;
}

vpFeatureTranslation *vpFeatureTranslation::duplicate() const
{
  vpFeatureTranslation *feature = new vpFeatureTranslation ;
  return feature ;
}



void
vpFeatureTranslation::display(const vpCameraParameters &cam,
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
