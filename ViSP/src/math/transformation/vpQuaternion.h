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



#ifndef __QUATERNION_H__
#define __QUATERNION_H__

/*!
  \file vpQuaternion.h
  \brief Class that consider the case of a quaternion and basic operations on it.


*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>


/*!
  \class vpQuaternion

  \ingroup TransTransformation
  
  \brief defines a quaternion and its basic operations.

  The class allows to convert between a rotation defined by a quaternion and a homogenous matrix.
  It also defines common operations on a quaternion such as:
	- multiplication (scalar and quaternion)
	- addition
	- substraction

  */
class VISP_EXPORT vpQuaternion
{
private:    
    void init() ;
	static const double minimum;
public:
	double x;
	double y;
	double z;
	double w;
    
    vpQuaternion() ;    
    vpQuaternion(const double x, const double y, const double z,const double w) ;    
    vpQuaternion(const vpQuaternion &q);
	void vpQuaternion(const vpMatrix& R)
    void set(const double x, const double y, const double z,const double w) ;

    
    vpQuaternion operator+(const vpQuaternion &q) const ;
    vpQuaternion operator-(const vpQuaternion &q) const ;
    vpQuaternion operator-() const ;
    vpQuaternion operator*(const double l) const;
	vpQuaternion operator* (const vpQuaternion &rq) const;
    vpQuaternion &operator=(const vpQuaternion &q);

	void buildFrom(const vpMatrix& R);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
