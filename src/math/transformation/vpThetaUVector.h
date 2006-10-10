/****************************************************************************
 *
 * $Id: vpThetaUVector.h,v 1.6 2006-10-10 16:06:00 fspindle Exp $
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
 * Theta U parameterization for the rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpTHETAUVECTOR_H
#define vpTHETAUVECTOR_H

/*!
  \file vpThetaUVector.h
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

class vpRotationMatrix;
class vpRzyxVector;
class vpRxyzVector;
class vpEulerVector;
class vpRzyzVector;

#include <visp/vpConfig.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRzyxVector.h>

class vpRotationMatrix;
class vpRzyxVector;
class vpRxyzVector;
class vpEulerVector;
class vpRzyzVector;

/*!
  \class vpThetaUVector
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/
class VISP_EXPORT vpThetaUVector : public vpRotationVector
{

private:
  //! initialize a size 3 vector
  void init() ;

  static const double minimum;

public:

  //! constructor
  vpThetaUVector() { ; }
  //! copy operator
  vpThetaUVector &operator=(const vpThetaUVector &m);
  //! copy constructor
  vpThetaUVector(const vpThetaUVector &m) ;

  //! constructor initialize a Theta U vector from a rotation matrix
  vpThetaUVector(const vpRotationMatrix& R) ;

  //! constructor from 3 angles (in radian)
  vpThetaUVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into Theta U vector
  vpThetaUVector buildFrom(const vpRotationMatrix& R) ;
  //! convert an Rzyx vector into Theta U vector
  vpThetaUVector buildFrom(const vpRzyxVector &e) ;
  //! convert an Rzyz vector into Theta U vector
  vpThetaUVector buildFrom(const vpRzyzVector &e) ;
  //! convert an Euler vector into Theta U vector
  vpThetaUVector buildFrom(const vpEulerVector &e) ;
  //! convert an Rxyz vector into Theta U vector
  vpThetaUVector buildFrom(const vpRxyzVector &e) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

