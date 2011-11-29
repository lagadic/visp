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

  \param R: Matrix containing a rotation.
*/
vpQuaternionVector::vpQuaternionVector(const vpRotationMatrix &R)  
  : vpRotationVector(4) 
{	
  buildFrom(R);
}
/*! 
  Copy constructor.
  \param q: quaternion to construct from. 
*/
vpQuaternionVector::vpQuaternionVector(const vpQuaternionVector &q) 
  : vpRotationVector(4) 
{
  std::copy(q.r, q.r+size(), this->r);
}
    
/*! 
  Manually change values of a quaternion.
  \param x: x quaternion parameter.
  \param y: y quaternion parameter.
  \param z: z quaternion parameter.
  \param w: w quaternion parameter.
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

  \param q: quaternion to add.
*/
vpQuaternionVector vpQuaternionVector::operator+( vpQuaternionVector &q)  
{	
  return vpQuaternionVector(x()+q.x(), y()+q.y(), z()+q.z(), w()+q.w());
}
/*! 
  Quaternion substraction.

  Substracts a quaternion from another. Substraction is component-wise.

  \param q: quaternion to substract.
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
  std::copy(q.r, q.r+size(), this->r);

  return *this;
} 

/*! 
  Constructs a quaternion from a rotation matrix.
  
  \param R: Rotation matrix.
*/
void vpQuaternionVector::buildFrom(const vpRotationMatrix &R)
{
  double s,c,theta,sinc;
  double axis_x,axis_y,axis_z;

  s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
      + (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
      + (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
  s = sqrt(s)/2.0;
  c = (R[0][0]+R[1][1]+R[2][2]-1.0)/2.0;
  theta=atan2(s,c);  /* theta in [0, PI] since s > 0 */
  
  if ((s > minimum) || (c > 0.0)) { /* general case */	
    sinc = vpMath::sinc(s,theta);
      
    axis_x = (R[2][1]-R[1][2])/(2*sinc);
    axis_y = (R[0][2]-R[2][0])/(2*sinc);
    axis_z = (R[1][0]-R[0][1])/(2*sinc);
  } else { /* theta near PI */	
    axis_x = theta*(sqrt((R[0][0]-c)/(1-c)));
    if ((R[2][1]-R[1][2]) < 0) axis_x = -axis_x;
    axis_y = theta*(sqrt((R[1][1]-c)/(1-c)));
    if ((R[0][2]-R[2][0]) < 0) axis_y = -axis_y;
    axis_z = theta*(sqrt((R[2][2]-c)/(1-c)));
    if ((R[1][0]-R[0][1]) < 0) axis_z = -axis_z;
  }
	
  theta *= 0.5;
  double norm = sqrt(axis_x*axis_x+axis_y*axis_y+axis_z*axis_z);	
  if(fabs(norm)<minimum) norm = 1.;
  double sinTheta_2 = sin(theta);
  set(cos(theta),
      (axis_x * sinTheta_2)/norm,
      (axis_y * sinTheta_2)/norm,
      (axis_z * sinTheta_2)/norm);

}
