/****************************************************************************
 *
 * $Id: vpFeatureTranslation.cpp,v 1.8 2007-12-18 15:03:18 fspindle Exp $
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
 * 3D translation visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


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
      vpERROR_TRACE("\n\t\t s* should be 0") ;
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
vpFeatureTranslation::print(const int /* select */) const
{
  std::cout <<"Translation 3D: " << s.t() ;
}

vpFeatureTranslation *vpFeatureTranslation::duplicate() const
{
  vpFeatureTranslation *feature = new vpFeatureTranslation ;
  return feature ;
}


/*!

Not implemented.
*/
void
vpFeatureTranslation::display(const vpCameraParameters &/* cam */,
			      vpImage<unsigned char> &/* I */,
			      bool /*useDistortion*/,
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
