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
