
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureThetaU.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureThetaU.cpp,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines the thetaU visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpMath.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \file vpFeatureThetaU.cpp
  \brief class that defines the thetaU visual feature
*/
/*

attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory

*/

void
vpFeatureThetaU::init()
{
    //feature dimension
    dim_s = 3 ;

    // memory allocation
    s.resize(dim_s) ;

}

vpFeatureThetaU::vpFeatureThetaU() : vpBasicFeature()
{
    init() ;
}

vpFeatureThetaU::vpFeatureThetaU(vpRotationMatrix &cdRc) : vpBasicFeature()
{
    init() ;

    vpThetaUVector tu(cdRc) ;
    buildFrom(tu) ;
}

vpFeatureThetaU::vpFeatureThetaU(vpHomogeneousMatrix &cdMc) : vpBasicFeature()
{
    init() ;
    vpRotationMatrix cdRc ;
    cdMc.extract(cdRc)  ;
    vpThetaUVector tu(cdRc) ;
    buildFrom(tu) ;
}

void
vpFeatureThetaU::buildFrom(vpThetaUVector &tu)
{
  s[0] = tu[0] ;
  s[1] = tu[1] ;
  s[2] = tu[2] ;
}

void vpFeatureThetaU::set_TUx(const double _TUx)
{
    s[0] = _TUx ;
}

double vpFeatureThetaU::get_TUx()  const
{
    return s[0] ;
}

void vpFeatureThetaU::set_TUy(const double _TUy)
{
    s[1] = _TUy ;
}

double vpFeatureThetaU::get_TUy()   const
{
    return s[1] ;
}

void
vpFeatureThetaU::set_TUz(const double _TUz)
{
    s[2] = _TUz ;
}

double
vpFeatureThetaU::get_TUz() const
{
    return  s[2]  ;
}

void
vpFeatureThetaU::buildFrom(const vpRotationMatrix &cdRc)
{
    vpThetaUVector tu(cdRc) ;
    buildFrom(tu) ;
}

void
vpFeatureThetaU::buildFrom(const vpHomogeneousMatrix &cdMc)
{
    vpRotationMatrix cdRc ;
    cdMc.extract(cdRc)  ;
    vpThetaUVector tu(cdRc) ;
    buildFrom(tu) ;
}

//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpFeatureThetaU::interaction(const int select) const
{

  vpMatrix L ;
  L.resize(0,6) ;

  vpMatrix Lw(3,3) ;
  Lw.setIdentity() ;

  double  theta = sqrt(s.sumSquare()) ;

  vpColVector U(3)  ;
  for (int i=0 ; i < 3 ; i++) U[i] = s[i]/theta ;

  vpMatrix skewU ;
  skewU = vpColVector::skew(U) ;

 if (theta < 1e-6)
  {
    Lw.setIdentity() ;
  }
 else
   Lw += -theta/2.0*skewU +
    (1-vpMath::sinc(theta)/vpMath::sqr(vpMath::sinc(theta/2.0)))*skewU*skewU ;



  //This version is a simplification
  if (vpFeatureThetaU::selectTUx() & select )
  {
    vpMatrix Lx(1,6) ;

    Lx[0][0] = 0 ;    Lx[0][1] = 0 ;    Lx[0][2] = 0 ;
    for (int i=0 ; i < 3 ; i++) Lx[0][i+3] = Lw[0][i] ;


    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeatureThetaU::selectTUy() & select )
  {
    vpMatrix Ly(1,6) ;

    Ly[0][0] = 0 ;    Ly[0][1] = 0 ;    Ly[0][2] = 0 ;
    for (int i=0 ; i < 3 ; i++) Ly[0][i+3] = Lw[1][i] ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }

  if (vpFeatureThetaU::selectTUz() & select )
  {
    vpMatrix Lz(1,6) ;

    Lz[0][0] = 0 ;    Lz[0][1] = 0 ;    Lz[0][2] = 0 ;
    for (int i=0 ; i < 3 ; i++) Lz[0][i+3] = Lw[2][i] ;

    L = vpMatrix::stackMatrices(L,Lz) ;
  }

  return L ;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeatureThetaU::error(const vpBasicFeature &s_star,
	       const int select)
{

  if (fabs(s_star.get_s().sumSquare()) > 1e-6)
  {

    ERROR_TRACE("s* should be zero ! ") ;
        throw(vpFeatureException(vpFeatureException::badInitializationERR,
				 "s* should be zero !")) ;

  }

    vpColVector e(0) ;


    if (vpFeatureThetaU::selectTUx() & select )
    {
	vpColVector ex(1) ;
	ex[0] = s[0]  ;
	e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeatureThetaU::selectTUy() & select )
    {
	vpColVector ey(1) ;
	ey[0] = s[1] ;
	e = vpMatrix::stackMatrices(e,ey) ;
    }

    if (vpFeatureThetaU::selectTUz() & select )
    {
	vpColVector ez(1) ;
	ez[0] = s[2] ;
	e = vpMatrix::stackMatrices(e,ez) ;
    }


    return e ;

}


void
vpFeatureThetaU::print(const int select) const
{
  cout <<"ThetaU: " << s.t() ;
}

vpFeatureThetaU *vpFeatureThetaU::duplicate() const
{
  vpFeatureThetaU *feature  = new vpFeatureThetaU ;
  return feature ;
}

void
vpFeatureThetaU::display(const vpCameraParameters &cam,
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
