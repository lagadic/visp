/****************************************************************************
 *
 * $Id: vpTranslationVector.cpp 3057 2011-02-11 13:17:26Z fspindle $
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
 * Translation vector.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/


#include <visp/vpQuaternion.h>
#include <visp/vpMath.h>
#include <stdio.h>
#include <string.h>


// minimum value of sine
const double vpQuaternion::minimum = 0.0001;

/*!
  \file vpQuaternion.cpp
  \brief Defines a quaternion and common operations on it.
*/

//! initialize a null quaternion    
void vpQuaternion::init() {
	x = y = z = 0.;
	w = 1.;
}
vpQuaternion::vpQuaternion() { init() ; }
    //! constructor from doubles
vpQuaternion::vpQuaternion(const double x, const double y, const double z,const double w) {
	set(x,y,z,w);
}

/*! constrcuts a quaternion from a matrix.
	This method assumes the 3x3 upper left submatrix is a rotation matrix.
	The passed parameter may therefore be a rotation matrix, a homogenous matrix or any kind of other matrix containing a rotation submatrix in it's upper left corner.

	\param Matrix containing a rotation
*/
vpQuaternion::vpQuaternion(const vpMatrix& R){
	init();
	buildFrom(R);
}
    //! copy constructor
vpQuaternion::vpQuaternion(const vpQuaternion &q){
	set(q.x,q.y,q.z,q.w);
}
    
	//! manually change values of a quaternion
void vpQuaternion::set(const double x, const double y, const double z,const double w) {
	this->x=x;
	this->y=y;
	this->z=z;
	this->w=w;
}

    
    //! quaternion addition
vpQuaternion vpQuaternion::operator+(const vpQuaternion &q) const {
	return vpQuaternion(x+q.x,y+q.y,z+q.z,w+q.w);
}
    //! quaternion substraction
vpQuaternion vpQuaternion::operator-(const vpQuaternion &q) const {
	return vpQuaternion(x-q.x,y-q.y,z-q.z,w-q.w);
}
    //! negate operator. Returns a quaternion defined by (-x,-y,-z-,-w)
vpQuaternion vpQuaternion::operator-() const {
	return vpQuaternion(-x,-y,-z,-w);
}
    //! multiplication by scalar. Returns a quaternion defined by (lx,ly,lz,lw)
vpQuaternion vpQuaternion::operator*(const double l) const{
	return vpQuaternion(l*x,l*y,l*z,l*w);
}

	//! multiply two quaternions
vpQuaternion vpQuaternion::operator* (const vpQuaternion &rq) const{	
	return vpQuaternion(w * rq.x + x * rq.w + y * rq.z - z * rq.y,
	                  w * rq.y + y * rq.w + z * rq.x - x * rq.z,
	                  w * rq.z + z * rq.w + x * rq.y - y * rq.x,
	                  w * rq.w - x * rq.x - y * rq.y - z * rq.z);
}
    //! Copy operator.   Allow operation such as Q = q
vpQuaternion &vpQuaternion::operator=(const vpQuaternion &q){
	set(q.x,q.y,q.z,q.w);

	return *this;
} 
/*! constrcuts a quaternion from a matrix.
	This method assumes the 3x3 upper left submatrix is a rotation matrix.
	The passed parameter may therefore be a rotation matrix, a homogenous matrix or any kind of other matrix containing a rotation submatrix in it's upper left corner.

	\param Matrix containing a rotation
*/
void vpQuaternion::buildFrom(const vpMatrix& R){
	double s,c,theta,sinc;
	double axis_x,axis_y,axis_z;

	s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
	+ (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
	+ (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
	s = sqrt(s)/2.0;
	c = (R[0][0]+R[1][1]+R[2][2]-1.0)/2.0;
	theta=atan2(s,c);  /* theta in [0, PI] since s > 0 */

	if ((s > minimum) || (c > 0.0)){ /* general case */	
		sinc = vpMath::sinc(s,theta);

		axis_x = (R[2][1]-R[1][2])/(2*sinc);
		axis_y = (R[0][2]-R[2][0])/(2*sinc);
		axis_z = (R[1][0]-R[0][1])/(2*sinc);
	}else{ /* theta near PI */	
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
	x = cos(theta);
	y = (axis_x * sinTheta_2)/norm;
	z = (axis_y * sinTheta_2)/norm;
	w = (axis_z * sinTheta_2)/norm;

}


/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
