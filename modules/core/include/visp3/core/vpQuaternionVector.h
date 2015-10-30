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

  \brief Class that consider the case of a quaternion and basic
   operations on it.

*/

class vpHomogeneousMatrix;

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>


/*!
  \class vpQuaternionVector

  \ingroup group_core_transformations
  
  \brief Defines a quaternion and its basic operations.

  A quaternion is defined by four values: \f${\bf q} = (x, y, z, w)\f$.

  This class allows to compute a quaternion from a rotation matrix
  using either vpQuaternionVector(const vpRotationMatrix &) constructor
  or buildFrom() method.

  It also defines common operations on a quaternion such as:
	- multiplication (scalar and quaternion)
	- addition
	- substraction.

  */
class VISP_EXPORT vpQuaternionVector : public vpRotationVector
{
private:        
  static const double minimum;
public:
    
  /*! Default constructor that initialize all the angles to zero. */
  vpQuaternionVector() : vpRotationVector(4) {}
  /*! Copy constructor. */
  vpQuaternionVector(const vpQuaternionVector &q) : vpRotationVector(q) {}
  vpQuaternionVector(const double x, const double y, const double z,const double w) ;    
  vpQuaternionVector(const vpRotationMatrix &R);

  void buildFrom(const vpRotationMatrix& R);

  void set(const double x, const double y, const double z,const double w) ;
    
  //! Returns x-component of the quaternion.
  inline double x() const {return r[0];}
  //! Returns y-component of the quaternion.
  inline double y() const {return r[1];}
  //! Returns z-component of the quaternion.
  inline double z() const {return r[2];}
  //! Returns w-component of the quaternion.
  inline double w() const {return r[3];}

  vpQuaternionVector operator+( vpQuaternionVector &q)  ;
  vpQuaternionVector operator-( vpQuaternionVector &q)  ;
  vpQuaternionVector operator-()  ;
  vpQuaternionVector operator*(const double l) ;
  vpQuaternionVector operator*( vpQuaternionVector &rq) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
