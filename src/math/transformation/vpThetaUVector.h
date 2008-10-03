/****************************************************************************
 *
 * $Id: vpThetaUVector.h,v 1.9 2008-10-03 15:50:16 fspindle Exp $
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

  \ingroup RotTransformation

  \brief Class that consider the case of the Theta U parameterization for the
  rotation.
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
  //! constructor initialize a Theta U vector from a RzyxVector
  vpThetaUVector(const vpRzyxVector& rzyx) ;
  //! constructor initialize a Theta U vector from a RzyzVector
  vpThetaUVector(const vpRzyzVector& rzyz) ;
  //! constructor initialize a Theta U vector from a RxyzVector
  vpThetaUVector(const vpRxyzVector& rxyz) ;

  //! constructor from 3 angles (in radian)
  vpThetaUVector(const double thetaux, const double thetauy, const double thetauz) :
    vpRotationVector (thetaux, thetauy, thetauz) { ; }

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

  // extract the angle and the axis from the ThetaU representation
  void extract( double &theta, vpColVector &u) const;

  
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

