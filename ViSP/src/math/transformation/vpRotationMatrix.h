/****************************************************************************
 *
 * $Id: vpRotationMatrix.h,v 1.9 2008-11-18 21:55:41 fspindle Exp $
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
 * Rotation matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpROTATIONMATRIX_H
#define vpROTATIONMATRIX_H

/*!
  \file vpRotationMatrix.h
  \brief Class that consider the particular case of rotation matrix
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRzyxVector.h>
#include <visp/vpRzyzVector.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpTranslationVector.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp/vpEulerVector.h>
#endif

/*!
  \class vpRotationMatrix

  \ingroup RotTransformation

  \brief The vpRotationMatrix considers the particular case of
  a rotation matrix.
  
  It is derived from vpMatrix. 

  \author  Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

*/
class VISP_EXPORT vpRotationMatrix : public vpMatrix
{
  friend class vpMatrix;
  friend class vpRotationVector;
  friend class vpRxyzVector;
  friend class vpRzyzVector;
  friend class vpRzyxVector;
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  friend class vpEulerVector;
#endif
  friend class vpThetaUVector;
  friend class vpTranslationVector;
public:
  //! Basic initialisation (identity)
  void init() ;

  //! Basic initialisation (identity)
  void setIdentity() ;
  //! basic constructor
  vpRotationMatrix()   ;
  //! copy constructor
  vpRotationMatrix(const vpRotationMatrix &R) ;
  //! Construction from  rotation (theta U parameterization)
  vpRotationMatrix(const vpThetaUVector &r) ;
  //! Construction from  rotation (Euler parameterization)
  vpRotationMatrix(const vpRzyzVector &r) ;
  //! Construction from  rotation Rxyz
  vpRotationMatrix(const vpRxyzVector &r) ;
  //! Construction from  rotation Rzyx
  vpRotationMatrix(const vpRzyxVector &r) ;
  //! Construction from  rotation (theta U parameterization)
  vpRotationMatrix(const double tux, const  double tuy, const double tuz) ;


  //! copy operator from vpRotationMatrix
  vpRotationMatrix &operator=(const vpRotationMatrix &R);
  //! copy operator from vpMatrix (handle with care)
  vpRotationMatrix &operator=(const vpMatrix &m) ;
  //! operation c = A * b (A is unchanged)
  vpTranslationVector operator*(const vpTranslationVector &mat) const ;
  //! operation C = A * B (A is unchanged)
  vpRotationMatrix operator*(const vpRotationMatrix &B) const ;
  // operation v2 = A * v1 (A is unchanged)
  vpColVector operator*(const vpColVector &v) const ;
   //! overload + operator (to say it forbidden operation, throw exception)
  vpRotationMatrix operator+(const vpRotationMatrix &B) const ;
   //! overload - operator (to say it forbidden operation, throw exception)
  vpRotationMatrix operator-(const vpRotationMatrix &B) const ;

  //! transpose
  vpRotationMatrix t() const ;

  //! invert the rotation matrix
  vpRotationMatrix inverse() const ;
  //! invert the rotation matrix
  void inverse(vpRotationMatrix &M) const;

  //! test if the  matrix is an rotation matrix
  bool isARotationMatrix() const  ;

  //! Print the matrix as a vector [T thetaU]
  void printVector() ;
  friend VISP_EXPORT std::ostream &operator << (std::ostream &s, const vpRotationMatrix &m);

  //! Transform a vector vpThetaUVector into an rotation matrix
  vpRotationMatrix buildFrom(const vpThetaUVector &v) ;
  //! Transform a vector reprensenting the euler (Rzyz) angle
  //! into a rotation matrix
  vpRotationMatrix buildFrom(const vpRzyzVector &v) ;
  //! Transform a vector reprensenting the Rxyz angle into a rotation matrix
  vpRotationMatrix buildFrom(const vpRxyzVector &v) ;
  //! Transform a vector reprensenting the Rzyx angle into a rotation matrix
  vpRotationMatrix buildFrom(const vpRzyxVector &v) ;
  //! Construction from  rotation (theta U parameterization)
  vpRotationMatrix buildFrom(const double tux,
			     const double tuy,
			     const double tuz) ;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  // Construction from  rotation Rzyz
  vpRotationMatrix(const vpEulerVector &r) ;
  // Transform a vector representing the euler (Rzyz) angle
  // into a rotation matrix
  vpRotationMatrix buildFrom(const vpEulerVector &v) ;
#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

private:
  static const double threshold;
  static const double minimum; // usefull only for debug
  };

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
