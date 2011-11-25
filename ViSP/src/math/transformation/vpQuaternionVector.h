/****************************************************************************
 *
 * $Id: vpTranslationVector.h 3460 2011-11-14 17:11:26Z fspindle $
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
 * Quaternion definition.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/



#ifndef __QUATERNIONVECTOR_H__
#define __QUATERNIONVECTOR_H__

/*!
  \file vpQuaternionVector.h
  \brief Class that consider the case of a quaternion and basic operations on it.


*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpRotationVector.h>

/*!
  \class vpQuaternionVector

  \ingroup RotTransformation
  
  \brief defines a quaternion and its basic operations.

  The class allows to convert between a rotation defined by a quaternion and a homogenous matrix.
  It also defines common operations on a quaternion such as:
	- multiplication (scalar and quaternion)
	- addition
	- substraction

  */
class VISP_EXPORT vpQuaternionVector : public vpRotationVector
{
private:        
	static const double minimum;
public:
	//! returns x-component of the quaternion
	inline double x(){return r[0];}
	//! returns y-component of the quaternion
	inline double y(){return r[1];}
	//! returns z-component of the quaternion
	inline double z(){return r[2];}
	//! returns w-component of the quaternion
	inline double w(){return r[3];}
    
    vpQuaternionVector() ;    
    vpQuaternionVector(const double x, const double y, const double z,const double w) ;    
    vpQuaternionVector(const vpQuaternionVector &q);
	vpQuaternionVector(const vpMatrix& R);
    void set(const double x, const double y, const double z,const double w) ;

    
    vpQuaternionVector operator+( vpQuaternionVector &q)  ;
    vpQuaternionVector operator-( vpQuaternionVector &q)  ;
    vpQuaternionVector operator-()  ;
    vpQuaternionVector operator*(const double l) ;
	vpQuaternionVector operator* ( vpQuaternionVector &rq) ;
    vpQuaternionVector &operator=( vpQuaternionVector &q);

	void buildFrom(const vpMatrix& R);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
