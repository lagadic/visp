/****************************************************************************
 *
 * $Id: vpRzyxVector.h,v 1.5 2006-05-30 08:40:44 fspindle Exp $
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
 * Rzyx angle parameterization for the rotation.
 * Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRzyxVECTOR_H
#define vpRzyxVECTOR_H

/*!
  \file vpRzyxVector.h
  \brief  class that consider the case of the Rzyx angle parameterization for the
    rotation : Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
*/

#include <visp/vpConfig.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRotationVector.h>

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRzyxVector
  \brief class that consider the case of the Rzyx angle parameterization for the
    rotation
    Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
*/

class VISP_EXPORT vpRzyxVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpThetaUVector;

public:
  //! constructor
  vpRzyxVector() { ; }
public:
  //! copy operator
  vpRzyxVector &operator=(const vpRzyxVector &m);
  //! copy constructor
  vpRzyxVector(const vpRzyxVector &m);
  //! constructor initialize a Theta U vector from a rotation matrix
  vpRzyxVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vpRzyxVector(const vpThetaUVector&  tu) ;

  //! constructor from 3 angles (in radian)
  vpRzyxVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into Rzyx vector
  vpRzyxVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  thetau vector into Rzyx vector
  vpRzyxVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
