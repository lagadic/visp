/****************************************************************************
 *
 * $Id: vpRxyzVector.h,v 1.5 2008-09-26 15:20:55 fspindle Exp $
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
 * Rxyz angle parameterization for the rotation.
 * Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpRxyzVECTOR_H
#define vpRxyzVECTOR_H

/*!
  \file vpRxyzVector.h

  \brief Class that consider the case of the Rxyz angle
  parameterization for the rotation.

  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRxyzVector

  \ingroup RotTransformation

  \brief Class that consider the case of the Rxyz angle
  parameterization for the rotation: Rxyz(phi,theta,psi) =
  Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

class VISP_EXPORT vpRxyzVector : public vpRotationVector
{
    friend class vpRotationMatrix;
    friend class vpThetaUVector;
private:
  //! initialize a size 3 vector
  void init() ;

public:
  //! constructor
  vpRxyzVector() { ; }
  //! copy operator
  vpRxyzVector &operator=(const vpRxyzVector &m);
  //! copy constructor
  vpRxyzVector(const vpRxyzVector &m);

  //! constructor from 3 angles (in radian)
  vpRxyzVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! constructor initialize a Theta U vector from a rotation matrix
  vpRxyzVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vpRxyzVector(const vpThetaUVector&  tu) ;

  //! convert a rotation matrix into Rxyz vector
  vpRxyzVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  Theta U vector into Rxyz vector
  vpRxyzVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
