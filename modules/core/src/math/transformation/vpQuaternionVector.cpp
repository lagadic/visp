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
 * Quaternion vector.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/


#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpMath.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>


// minimum value of sine
const double vpQuaternionVector::minimum = 0.0001;

/*!
  \file vpQuaternionVector.cpp
  \brief Defines a quaternion and common operations on it.
*/

//! Constructor from doubles.
vpQuaternionVector::vpQuaternionVector(const double x_, const double y_,
               const double z_,const double w_)
  : vpRotationVector(4) 
{
  set(x_, y_, z_, w_);
}

/*! 
  Constructs a quaternion from a rotation matrix.

  \param R : Matrix containing a rotation.
*/
vpQuaternionVector::vpQuaternionVector(const vpRotationMatrix &R)  
  : vpRotationVector(4) 
{	
  buildFrom(R);
}

/*! 
  Manually change values of a quaternion.
  \param x_ : x quaternion parameter.
  \param y_ : y quaternion parameter.
  \param z_ : z quaternion parameter.
  \param w_ : w quaternion parameter.
*/
void vpQuaternionVector::set(const double x_, const double y_,
           const double z_,const double w_)
{
  r[0]=x_;
  r[1]=y_;
  r[2]=z_;
  r[3]=w_;
}

    
/*! 
  Quaternion addition.

  Adds two quaternions. Addition is component-wise.

  \param q : quaternion to add.
*/
vpQuaternionVector vpQuaternionVector::operator+( vpQuaternionVector &q)  
{	
  return vpQuaternionVector(x()+q.x(), y()+q.y(), z()+q.z(), w()+q.w());
}
/*! 
  Quaternion substraction.

  Substracts a quaternion from another. Substraction is component-wise.

  \param q : quaternion to substract.
*/
vpQuaternionVector vpQuaternionVector::operator-( vpQuaternionVector &q)  
{
  return vpQuaternionVector(x()-q.x(), y()-q.y(), z()-q.z(), w()-q.w());
}

//! Negate operator. Returns a quaternion defined by (-x,-y,-z-,-w).
vpQuaternionVector vpQuaternionVector::operator-()  
{
  return vpQuaternionVector(-x(), -y(), -z(), -w());
}

//! Multiplication by scalar. Returns a quaternion defined by (lx,ly,lz,lw).
vpQuaternionVector vpQuaternionVector::operator*( double l) 
{
  return vpQuaternionVector(l*x(),l*y(),l*z(),l*w());
}

//! Multiply two quaternions.
vpQuaternionVector vpQuaternionVector::operator* ( vpQuaternionVector &rq) {	
  return vpQuaternionVector(w() * rq.x() + x() * rq.w() + y() * rq.z() - z() * rq.y(),
			    w() * rq.y() + y() * rq.w() + z() * rq.x() - x() * rq.z(),
			    w() * rq.z() + z() * rq.w() + x() * rq.y() - y() * rq.x(),
			    w() * rq.w() - x() * rq.x() - y() * rq.y() - z() * rq.z());
}

/*! 
  Constructs a quaternion from a rotation matrix.
  
  \param R : Rotation matrix.
*/
void vpQuaternionVector::buildFrom(const vpRotationMatrix &R)
{
  vpThetaUVector tu(R);
  vpColVector u;
  double theta;
  tu.extract(theta, u);

  theta *= 0.5;

  double sinTheta_2 = sin(theta);
  set( u[0] * sinTheta_2, u[1] * sinTheta_2, u[2] * sinTheta_2, cos(theta) );
}
