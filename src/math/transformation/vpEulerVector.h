/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Euler angles parameterization for the rotation.
 * Euler(phi,theta,psi)= Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpEULERVECTOR_H
#define vpEULERVECTOR_H

/*!
  \file vpEulerVector.h
  \brief class that consider the case of the euler angles  parameterization
  for the  rotation

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

class vpEulerVector;
class vpRotationMatrix;
class vpThetaUVector;

#include <visp/vpConfig.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>

class vpRotationMatrix;
class vpThetaUVector;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
  \class vpEulerVector

  \ingroup RotTransformation

  \brief Class that consider the case of the euler angles  parameterization
  for the  rotation.

  \deprecated This class is deprecated because to ambiguous. You
  should use vpRzyzVector class instead.

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/
class VISP_EXPORT vpEulerVector : public vpRotationVector
{

public:
  //! constructor
  vp_deprecated vpEulerVector() { ; }
  //! copy operator
  vp_deprecated vpEulerVector &operator=(const vpEulerVector &m);
  //! copy constructor
  vpEulerVector(const vpEulerVector &m);

  //! constructor initialize a Theta U vector from a rotation matrix
  vp_deprecated vpEulerVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vp_deprecated vpEulerVector(const vpThetaUVector&  tu) ;

  //! constructor from 3 angles (in radian)
  vp_deprecated vpEulerVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into Euler vector
  vpEulerVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  Theta U vector into Euler vector
  vpEulerVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
