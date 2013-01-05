/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Homography transformation.
 *
 * Authors:
 * Muriel Pressigout
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file vpHomography.cpp
  \brief Definition de la classe vpHomography. Class that consider
  the particular case of homography
*/

#include <visp/vpHomography.h>
#include <visp/vpDebug.h>
#include <visp/vpMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#include <stdio.h>



/*!
  \brief initialiaze a 4x4 matrix as identity
*/

void
vpHomography::init()
{
  unsigned int i,j ;

  try {
    vpMatrix::resize(3,3) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }


  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      if (i==j)
	(*this)[i][j] = 1.0 ;
      else
 	(*this)[i][j] = 0.0;

}

/*!
  \brief initialize an homography as Identity
*/
vpHomography::vpHomography() : vpMatrix()
{
  init() ;
}


/*!
  \brief initialize an homography from another homography
*/

vpHomography::vpHomography(const vpHomography &aHb) : vpMatrix()
{
  init() ;
  *this = aHb ;
}

/*!
  \brief initialize an homography from another homography
*/

vpHomography::vpHomography(const vpHomogeneousMatrix &aMb,
			   const vpPlane &_bP) : vpMatrix()
{


  init() ;

  buildFrom(aMb,_bP) ;


}

vpHomography::vpHomography(const vpThetaUVector &tu,
			   const vpTranslationVector &atb,
			   const vpPlane &_bP) : vpMatrix()
{
  init() ;
  buildFrom(tu,atb,_bP) ;
}

vpHomography::vpHomography(const vpRotationMatrix &aRb,
			   const vpTranslationVector &atb,
			   const vpPlane &_bP) : vpMatrix()
{
  init() ;
  buildFrom(aRb,atb,_bP) ;
 }

vpHomography::vpHomography(const vpPoseVector &arb,
			   const vpPlane &_bP) : vpMatrix()
{

  init() ;
  buildFrom(arb,_bP) ;
}



void
vpHomography::buildFrom(const vpHomogeneousMatrix &aMb,
			const vpPlane &_bP)
{


  insert(aMb) ;
  insert(_bP) ;
  build() ;


}

void
vpHomography::buildFrom(const vpThetaUVector &tu,
			const vpTranslationVector &atb,
			const vpPlane &_bP)
{

  insert(tu) ;
  insert(atb) ;
  insert(_bP) ;
  build() ;
}

void
vpHomography::buildFrom(const vpRotationMatrix &aRb,
			const vpTranslationVector &atb,
			const vpPlane &_bP)
{
  init() ;
  insert(aRb) ;
  insert(atb) ;
  insert(_bP) ;
  build() ;
}

void
vpHomography::buildFrom(const vpPoseVector &arb,
			const vpPlane &_bP)
{

  aMb.buildFrom(arb[0],arb[1],arb[2],arb[3],arb[4],arb[5]) ;
  insert(_bP) ;
  build() ;
}



/*********************************************************************/


/*!
  \brief insert the rotational component and
  recompute the homography
*/
void
vpHomography::insert(const vpRotationMatrix &aRb)
{
  aMb.insert(aRb) ;
  //build() ;
}
/*!
  \brief insert the rotational component and
  recompute the homography
*/
void
vpHomography::insert(const vpHomogeneousMatrix &_aMb)
{

  aMb = _aMb ;
  //build() ;
}


/*!  \brief insert the rotational component, insert a
  theta u vector (transformation into a rotation matrix) and
  recompute the homography

*/
void
vpHomography::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix aRb(tu) ;
  aMb.insert(aRb) ;
  //build() ;
}


/*!
  \brief  insert the translational component in a homography and
  recompute the homography
*/
void
vpHomography::insert(const vpTranslationVector &atb)
{
  aMb.insert(atb) ;
  //build() ;
}

/*!
  \brief  insert the reference plane and
  recompute the homography
*/
void
vpHomography::insert(const vpPlane &_bP)
{

  bP= _bP ;
  //build() ;
}


/*!
  \brief invert the homography


  \return   [H]^-1
*/
vpHomography
vpHomography::inverse() const
{
  vpHomography bHa ;


  vpMatrix::pseudoInverse(bHa,1e-16) ;

  return  bHa;
}

/*!
  \brief invert the homography


  \param bHa : [H]^-1
*/
void
vpHomography::inverse(vpHomography &bHa) const
{
  bHa = inverse() ;
}


void
vpHomography::save(std::ofstream &f) const
{
  if (f != NULL)
  {
    f << *this ;
  }
  else
  {
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}

/*!
 
  Multiplication by an homography.

  \param H : Homography to multiply with.

  \code 
  vpHomography aHb, bHc; 
  // Initialize aHb and bHc homographies
  vpHomography aHc = aHb * bHc;  
  \endcode

*/
vpHomography vpHomography::operator*(const vpHomography &H) const
{
  vpHomography Hp;
  for(unsigned int i = 0; i < 3; i++) {
    for(unsigned int j = 0; j < 3; j++) {
      double s = 0.;
      for(unsigned int k = 0; k < 3; k ++) {
	s += (*this)[i][k] * H[k][j]; 
      }
      Hp[i][j] = s;
    }
  }
  return Hp;
}

/*!
 
  Multiply an homography by a scalar.

  \param v : Value of the scalar.

  \code 
  double v = 1.1;
  vpHomography aHb; 
  // Initialize aHb
  vpHomography H = aHb * v;  
  \endcode

*/
vpHomography vpHomography::operator*(const double &v) const
{
  vpHomography H;
  	
  for (unsigned int i=0; i < 9; i ++) {
    H.data[i] = this->data[i] * v;
  }

  return H;
}

/*!
 
  Divide an homography by a scalar.

  \param v : Value of the scalar.

  \code 
  vpHomography aHb; 
  // Initialize aHb
  vpHomography H = aHb / aHb[2][2];  
  \endcode

*/
vpHomography vpHomography::operator/(const double &v) const
{
  vpHomography H;
  double one_over_v = 1. / v;
 
  for (unsigned int i=0; i < 9; i ++) {
    H.data[i] = this->data[i] * one_over_v;
  }

  return H;
}
/*!
  Read an homography in a file, verify if it is really an homogeneous
  matrix.

  \param f : the file.
*/
void
vpHomography::load(std::ifstream &f)
{
  if (f != NULL)
  {
    for (unsigned int i=0 ; i < 3 ; i++)
      for (unsigned int j=0 ; j < 3 ; j++)
      {
	f>>   (*this)[i][j] ;
      }
  }
  else
  {
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}



//! Print the matrix as a vector [T thetaU]
void
vpHomography::print()
{
  std::cout <<*this << std::endl ;
}

/*!
  \brief Compute aHb such that

  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]
*/
void
vpHomography::build()
{
  unsigned int i,j ;

  vpColVector n(3) ;
  vpColVector atb(3) ;
  for (i=0 ; i < 3 ; i++)
  {
    atb[i] = aMb[i][3] ;
    for (j=0 ; j < 3 ; j++) (*this)[i][j] = aMb[i][j];
  }

  bP.getNormal(n) ;

  double d = bP.getD() ;
  *this -= atb*n.t()/d ; // the d used in the equation is such as nX=d is the
			 // plane equation. So if the plane is described by
			 // Ax+By+Cz+D=0, d=-D

}

/*!
  \brief Compute aHb such that

  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]
  //note d => -d verifier
*/
void
vpHomography::build(vpHomography &aHb,
		    const vpHomogeneousMatrix &aMb,
		    const vpPlane &bP)
{
  unsigned int i,j ;

  vpColVector n(3) ;
  vpColVector atb(3) ;
  for (i=0 ; i < 3 ; i++)
  {
    atb[i] = aMb[i][3] ;
    for (j=0 ; j < 3 ; j++) aHb[i][j] = aMb[i][j];
  }

  bP.getNormal(n) ;

  double d = bP.getD() ;
  aHb -= atb*n.t()/d ; // the d used in the equation is such as nX=d is the
		       // plane equation. So if the plane is described by
		       // Ax+By+Cz+D=0, d=-D

}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
