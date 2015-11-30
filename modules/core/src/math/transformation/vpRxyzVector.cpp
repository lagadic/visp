/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Rxyz angle parameterization for the rotation.
 * Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi).
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <math.h>

#include <visp3/core/vpRxyzVector.h>

/*!
  \file vpRxyzVector.cpp
  \brief class that consider the case of the  Rxyz angle parameterization for the rotation :
  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

/*! Default constructor that initialize all the 3 angles to zero. */
vpRxyzVector::vpRxyzVector()
  : vpRotationVector(3)
{}

/*! Copy constructor. */
vpRxyzVector::vpRxyzVector(const vpRxyzVector &rxyz)
  : vpRotationVector(rxyz)
{}

/*!
  Constructor from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
vpRxyzVector::vpRxyzVector(const double phi, const double theta, const double psi)
  : vpRotationVector (3)
{
  buildFrom(phi, theta, psi);
}

/*! 
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpRotationMatrix& R)
  : vpRotationVector (3)
{
  buildFrom(R) ;
}

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpThetaUVector& tu)
  : vpRotationVector (3)
{
  buildFrom(tu) ;
}

/*! 
  Convert a rotation matrix into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.
  
  \param R : Rotation matrix used as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRxyzVector
vpRxyzVector::buildFrom(const vpRotationMatrix& R)
{
  double COEF_MIN_ROT = 1e-6;
  double phi ;

  if ((fabs(R[1][2]) < COEF_MIN_ROT) && (fabs(R[2][2]) < COEF_MIN_ROT)) phi = 0 ;
  else phi = atan2(-R[1][2], R[2][2]) ;

  double si = sin(phi) ;
  double co = cos(phi) ;
  double theta = atan2(R[0][2], -si*R[1][2] + co*R[2][2]) ;
  double psi = atan2(co*R[1][0] + si*R[2][0], co*R[1][1] + si*R[2][1]);

  buildFrom(phi, theta, psi);

  return *this ;
}

/*! 
  Convert a \f$\theta {\bf u}\f$ vector into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$
  Euler angles vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRxyzVector
vpRxyzVector::buildFrom(const vpThetaUVector& tu)
{
  vpRotationMatrix R ;
  R.buildFrom(tu) ;
  buildFrom(R) ;

  return *this ;
}

/*!
  Construction from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
void
vpRxyzVector::buildFrom(const double phi, const double theta, const double psi)
{
  data[0] = phi ;
  data[1] = theta ;
  data[2] = psi ;
}

/*!

  Initialize each element of the vector to the same angle value \e v.

  \param v : Angle value to set for each element of the vector.

\code
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRxyzVector.h>

int main()
{
  vpRxyzVector v;

  // Initialise the rotation vector
  v = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
}
\endcode
*/
vpRxyzVector &vpRxyzVector::operator=(double v)
{
  for (unsigned int i=0; i< dsize; i++)
    data[i] = v;

  return *this;
}
