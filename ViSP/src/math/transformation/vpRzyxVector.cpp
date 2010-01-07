/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Rzyx angle parameterization for the rotation.
 * Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <math.h>
#include <visp/vpRzyxVector.h>

/*!
  \file vpRzyxVector.cpp
  \brief class that consider the case of the  Rzyx angle parameterization for the  rotation :
  Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta,Rot(x,psi)

*/


/*!
  Affectation of two vectors.
*/
vpRzyxVector &
vpRzyxVector::operator=(const vpRzyxVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! Copy constructor.
vpRzyxVector::vpRzyxVector(const vpRzyxVector &m) : vpRotationVector()
{
  *this = m ;
}


/*! 
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpRotationMatrix& R)
{
  buildFrom(R) ;
}


/*!
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta u\f$ vector.
  \param tu : \f$\theta u\f$ representation of a rotation used here as 
  input to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpThetaUVector& tu)
{
  buildFrom(tu) ;
}

/*! 
  Convert a rotation matrix into a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.
  
  Source: R. Paul, Robot Manipulators: Mathematics, Programming, and Control.
  MIT Press, 1981, p. 71

  \param R : Rotation matrix used as input.
  \return \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRzyxVector
vpRzyxVector::buildFrom(const vpRotationMatrix& R)
{
  double nx = R[0][0];
  double ny = R[1][0];

  double phi = atan2(ny,nx) ;
  double si = sin(phi) ;
  double co = cos(phi) ;

  double nz = R[2][0];
  double theta = atan2(-nz, co*nx+si*ny) ;

  double ax = R[0][2];
  double ay = R[1][2];
  double ox = R[0][1];
  double oy = R[1][1];

  double psi = atan2(si*ax-co*ay,-si*ox+co*oy);

  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;

  return *this ;
}


/*! 
  Convert a \f$\theta u\f$ vector into a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ 
  Euler angles vector.
  \param tu : \f$\theta u\f$ representation of a rotation used here as 
  input.
  \return \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRzyxVector
vpRzyxVector::buildFrom(const vpThetaUVector& tu)
{
  vpRotationMatrix R ;
  R.buildFrom(tu) ;
  buildFrom(R) ;
  
  return *this ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
