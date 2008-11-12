/****************************************************************************
 *
 * $Id: vpFeatureTranslation.cpp,v 1.12 2008-11-12 17:36:26 fspindle Exp $
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
 * Fabien Spindler
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

/*! 

  Initialise the memory space requested for 3D translation visual
  feature.
*/
void
vpFeatureTranslation::init()
{
  //feature dimension
  dim_s = 3 ;

  // memory allocation
  s.resize(dim_s) ;

}

/*! 
  Default constructor that build a visual feature and initialize it to zero.

*/
vpFeatureTranslation::vpFeatureTranslation() : vpBasicFeature()
{
  init() ;
}


/*!

  Constructor that build a 3D visual feature from an homogeneous
  matrix that represent the 3D transformation between the desired
  camera frame and the current camera frame \f$ ^{c^*}M_c \f$.

  \param cdMc [in] : 3D displacement that the camera has to achieve to
  move from the desired camera frame to the current one (\f$
  ^{c^*}M_c\f$).

*/
vpFeatureTranslation::vpFeatureTranslation(vpHomogeneousMatrix &cdMc) : vpBasicFeature()
{
  init() ;

  buildFrom(cdMc) ;
}

/*!
  Build a 3D translation visual feature from an homogeneous
  matrix that represent the 3D transformation between the desired
  camera frame and the current camera frame \f$ ^{c^*}M_c \f$.

  \param cdMc [in] : 3D displacement that the camera has to achieve to
  move from the desired camera frame to the current one (\f$
  ^{c^*}M_c\f$).
*/
void
vpFeatureTranslation::buildFrom(const vpHomogeneousMatrix &cdMc)
{
  this->cdMc = cdMc ;
  s[0] = cdMc[0][3] ;
  s[1] = cdMc[1][3] ;
  s[2] = cdMc[2][3] ;
}

/*!

  Initialise the \f$t_x \f$ subset value of the current 3D
  visual feature \f$ s\f$.

  \param t_x : \f$t_x \f$ subset value to initialize.
  \sa get_Tx()

*/
void vpFeatureTranslation::set_Tx(const double t_x)
{
    s[0] = t_x ;
}
/*!

  Initialise the \f$t_y \f$ subset value of the current 3D
  visual feature \f$ s\f$.

  \param t_y : \f$t_y \f$ subset value to initialize.
  \sa get_Ty()

*/
void vpFeatureTranslation::set_Ty(const double t_y)
{
    s[1] = t_y ;
}
/*!

  Initialise the \f$t_z \f$ subset value of the current 3D
  visual feature \f$ s\f$.

  \param t_z : \f$t_z \f$ subset value to initialize.
  \sa get_Tz()

*/
void
vpFeatureTranslation::set_Tz(const double t_z)
{
    s[2] = t_z ;
}

/*!
  Return the \f$t_x \f$ subset value of the current visual feature 
  \f$s\f$.

*/
double vpFeatureTranslation::get_Tx()  const
{
  return s[0] ;
}


/*!
  Return the \f$t_y \f$ subset value of the current visual feature 
  \f$s\f$.

*/
double vpFeatureTranslation::get_Ty()   const
{
  return s[1] ;
}


/*!
  Return the \f$t_z \f$ subset value of the current visual feature 
  \f$s\f$.

*/
double
vpFeatureTranslation::get_Tz() const
{
  return  s[2]  ;
}


/*!

  Compute and return the interaction matrix \f$ L \f$ from a subset
  \f$(t_x, t_y, t_z)\f$ of the possible translation features that
  represent the 3D transformation \f$^{c^*}M_c\f$, with

  \f[ L = [ ^{c^*}R_c \;\; 0_3] \f] 

  where \f$^{c^*}R_c\f$ is the rotation the camera has to achieve to
  move from the desired camera frame and the current camera frame.

  

  \param select : Selection of a subset of the possible translation
  features. 
  - To compute the interaction matrix for all the three translation
    subset features \f$(t_x,t_y,t_y)\f$ use vpBasicFeature::FEATURE_ALL. In
    that case the dimention of the interaction matrix is \f$ [3 \times
    6] \f$
  - To compute the interaction matrix for only one of the translation
    subset (\f$t_x, t_y, t_z\f$) use
    one of the corresponding function selectTx(), selectTy() or
    selectTz(). In that case the returned interaction matrix is \f$ [1
    \times 6] \f$ dimension.

  \return The interaction matrix computed from the translation
  features.

  The code below shows how to compute the interaction matrix
  associated to the visual feature \f$s = t_x \f$. 

  \code
  vpHomogeneousMatrix cdMc;
  ... 
  // Creation of the current feature s
  vpFeatureTranslation s;
  s.buildFrom(cdMc);

  vpMatrix L_x = s.interaction( vpFeatureTranslation::selectTx() );
  \endcode

  The code below shows how to compute the interaction matrix
  associated to the \f$s = (t_x, t_y) \f$
  subset visual feature:

  \code
  vpMatrix L_xy = s.interaction( vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy() );
  \endcode

  L_xy is here now a 2 by 6 matrix. The first line corresponds to
  the \f$ t_x \f$ visual feature while the second one to the \f$
  t_y \f$ visual feature.

  It is also possible to build the interaction matrix from all the
  translation components by:

  \code
  vpMatrix L_xyz = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode

  In that case, L_xyz is a 3 by 6 interaction matrix where the last
  line corresponds to the \f$ t_z \f$ visual feature.

*/
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

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  Since this visual feature \f$ s \f$ represent the 3D translation from the desired
  camera frame to the current one \f$^{c^*}t_{c} \f$, the desired
  visual feature \f$ s^* \f$ should be zero. Thus, the error is here
  equal to the current visual feature \f$ s \f$.

  \param s_star : Desired visual visual feature that should be equal to zero.

  \param select : The error can be computed for a selection of a
  subset of the possible translation features.
  - To compute the error for all the three translation vector coordinates use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 3 
    dimention column vector.
  - To compute the error for only one of the translation vector coordinate
    feature \f$(t_x, t_y, t_z)\f$ use one of the
    corresponding function selectTx(), selectTy() or selectTz(). In
    that case the error vector is a 1 dimention column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  \exception vpFeatureException::badInitializationError : If the
  desired visual feature \f$ s^* \f$ is not equal to zero.

  The code below shows how to use this method to manipulate the \f$
  t_z \f$ subset:

  \code
  // Creation of the current feature s
  vpFeatureTranslation s;
  s.set_TUz(0.3); // Initialization of the feature

  // Creation of the desired feature s*. By default this feature is 
  // initialized to zero
  vpFeatureTranslation s_star; 

  // Compute the interaction matrix for the t_z translation feature
  vpMatrix L_z = s.interaction( vpFeatureTranslation::selectTz() );

  // Compute the error vector (s-s*) for the t_z feature
  s.error(s_star, vpFeatureTranslation::selectTz());
  \endcode

  To manipulate the subset features \f$s=(t_y, t_z)\f$,
  the code becomes:
  \code
  // Compute the interaction matrix for the t_y, t_z features
  vpMatrix L_yz = s.interaction( vpFeatureTranslation::selectTy() | vpFeatureTranslation::selectTz() );

  // Compute the error vector (s-s*) for the t_y, t_z feature
  s.error(s_star, vpFeatureTranslation::selectTy() | vpFeatureTranslation::selectTz());
  \endcode

*/
vpColVector
vpFeatureTranslation::error(const vpBasicFeature &s_star,
			    const int select)
{
  vpColVector e(0) ;


  if (s_star.get_s().sumSquare() > 1e-6)
    {
      vpERROR_TRACE("s* should be zero ! ") ;
      throw(vpFeatureException(vpFeatureException::badInitializationError,
			       "s* should be zero !")) ;
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

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible translation features.
  - To print all the three translation vector coordinates used as features use
  vpBasicFeature::FEATURE_ALL. 
  - To print only one of the translation coordinate
  feature \f$(t_x, t_y, t_z)\f$ use one of the
  corresponding function selectTx(), selectTy() or selectTz().

  \code
  vpHomogeneousMatrix cdMc; // Homogenous transformation between desired camera frame and current camera frame
  ...
  
  // Creation of the current feature s
  vpFeatureTranslation s;
  s.buildFrom(cdMc);

  s.print(); // print all the 3 components of the translation feature
  s.print(vpBasicFeature::FEATURE_ALL); // same behavior then previous line
  s.print(vpFeatureTranslation::selectTz()); // print only the t_z component
  \endcode
*/
void
vpFeatureTranslation::print(const int select) const
{
  std::cout <<"Translation 3D: ";
  if (vpFeatureTranslation::selectTx() & select ) {
    std::cout << s[0] << " ";
  }
  if (vpFeatureTranslation::selectTy() & select ) {
    std::cout << s[1] << " ";
  }
  if (vpFeatureTranslation::selectTz() & select ) {
    std::cout << s[2] << " ";
  }
  std::cout << std::endl;
}


/*!
  
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureTranslation s;
  s_star = s.duplicate(); // s_star is now a vpFeatureTranslation
  \endcode

*/
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
vpFeatureTranslation::display(const vpCameraParameters &/* cam */,
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
