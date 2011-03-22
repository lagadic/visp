/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
  friend class vpThetaUVector;
  friend class vpTranslationVector;
public:
  //! Basic initialisation (identity)
  void init() ;

  //! Basic initialisation (identity)
  void setIdentity() ;
  void eye();
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
  //! operation C = A * B (A is unchanged)
  vpMatrix operator*(const vpMatrix &B) const ;
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

private:
  static const double threshold;
  static const double minimum; // useful only for debug
  };

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
