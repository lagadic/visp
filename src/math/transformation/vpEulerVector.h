/****************************************************************************
 *
 * $Id: vpEulerVector.h,v 1.8 2008-11-14 17:43:37 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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

  \deprecated This class is deprecated because to ambiguous. Yous
  should use vpRxyzVector, or vpRzyxVector or vpRzyzVector classes.

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/
class VISP_EXPORT vpEulerVector : public vpRotationVector
{

public:
  //! constructor
  vpEulerVector() { ; }
  //! copy operator
  vpEulerVector &operator=(const vpEulerVector &m);
  //! copy constructor
  vpEulerVector(const vpEulerVector &m);

  //! constructor initialize a Theta U vector from a rotation matrix
  vpEulerVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vpEulerVector(const vpThetaUVector&  tu) ;

  //! constructor from 3 angles (in radian)
  vpEulerVector(const double phi, const double theta, const double psi) :
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
