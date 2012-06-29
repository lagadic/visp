/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Quaternion vector.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/


#include <visp/vpQuaternionVector.h>
#include <visp/vpMath.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>


// minimum value of sine
const double vpQuaternionVector::minimum = 0.0001;

/*!
  \file vpQuaternionVector.cpp
  \brief Defines a quaternion and common operations on it.
*/
vpQuaternionVector::vpQuaternionVector() : vpRotationVector(4) {  }

//! Constructor from doubles.
vpQuaternionVector::vpQuaternionVector(const double x, const double y, 
				       const double z,const double w) 
  : vpRotationVector(4) 
{
  set(x, y, z, w);
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
  Copy constructor.
  \param q : quaternion to construct from.
*/
vpQuaternionVector::vpQuaternionVector(const vpQuaternionVector &q) 
  : vpRotationVector(4) 
{  
  for(unsigned int i=0;i<size();i++) (*this)[i]=q.r[i];   
}
    
/*! 
  Manually change values of a quaternion.
  \param x : x quaternion parameter.
  \param y : y quaternion parameter.
  \param z : z quaternion parameter.
  \param w : w quaternion parameter.
*/
void vpQuaternionVector::set(const double x, const double y, 
			     const double z,const double w) 
{
  r[0]=x;
  r[1]=y;
  r[2]=z;
  r[3]=w;
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

//! Copy operator.   Allow operation such as Q = q.
vpQuaternionVector &vpQuaternionVector::operator=( vpQuaternionVector &q)
{ 
  for(unsigned int i=0;i<size();i++) (*this)[i]=q.r[i];
  return *this;
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
  set( cos(theta), u[0] * sinTheta_2, u[1] * sinTheta_2, u[2] * sinTheta_2);
}
