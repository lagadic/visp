

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRotationMatrix.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRotationMatrix.h,v 1.2 2005-06-28 08:33:02 marchand Exp $
 *
 * Description
 * ============
 *     Class that consider the particular case of rotation matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* Modifications
   june, 8 2005      |  Add operator=(vpMatrix &m)
   EM                |  with a test to know is m is rotation matrix
*/


#ifndef vpROTATIONMATRIX_H
#define vpROTATIONMATRIX_H

/*!
  \file vpRotationMatrix.h
  \brief Class that consider the particular case of rotation matrix
*/

#include <visp/vpMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRzyxVector.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpTranslationVector.h>

class vpRotationMatrix : public vpMatrix
{
  friend class vpMatrix;
  friend class vpRotationVector;
  friend class vpRxyzVector;
  friend class vpRzyzVector;
  friend class vpRzyxVector;
  friend class vpEulerVector;
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
  //! Construction from  rotation Rzyz
  vpRotationMatrix(const vpEulerVector &r) ;
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
  friend ostream &operator << (ostream &s, const vpRotationMatrix &m);

  //! Transform a vector vpThetaUVector into an rotation matrix
  vpRotationMatrix buildFrom(const vpThetaUVector &v) ;
  //! Transform a vector reprensenting the euler (Rzyz) angle
  //! into a rotation matrix
  vpRotationMatrix buildFrom(const vpEulerVector &v) ;
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
  };

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
