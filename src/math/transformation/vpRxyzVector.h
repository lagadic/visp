/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Fabien Spindler
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

  \brief Class that consider the case of the Euler
  \f$(\varphi,\theta,\psi)\f$ angle using the x-y-z convention, where \f$(\varphi,\theta,\psi)\f$ are respectively the
  rotation angles around the \f$x\f$, \f$y\f$ and \f$z\f$ axis.

  \f[R_{xyz}(\varphi,\theta,\psi) = R_x(\varphi) \; R_y(\theta) \; R_z(\psi)\f]

  with

  \f[R_{x}(\varphi) = \left(
  \begin{array}{ccc}
  1 & 0 & 0 \\
  0 &\cos \varphi & -\sin\varphi \\
  0 &\sin \varphi & \cos\varphi \\
  \end{array}
  \right) \;
  R_{y}(\theta) = \left(
  \begin{array}{ccc}
  \cos \theta & 0 & \sin\theta\\
  0 & 1 & 0 \\
  -\sin\theta & 0 &\cos \theta
  \end{array}
  \right) \;
  R_{z}(\psi) = \left(
  \begin{array}{ccc}
  \cos \psi & -\sin\psi & 0\\
  \sin\psi &\cos \psi& 0 \\
  0 & 0 & 1
  \end{array}
  \right)\f]

  The rotation matrix corresponding to the x-y-z convention is given by:

  \f[
  R_{xyz}(\varphi,\theta,\psi) = \left(
  \begin{array}{ccc}
  \cos\theta \cos\psi & -\cos\theta \sin\psi & \sin\theta \\
  \sin\varphi \sin\theta \cos\psi + \cos\varphi\sin\psi & -\sin\varphi \sin\theta \sin\psi +\cos\varphi\cos\psi & -\sin\varphi \cos\theta \\
  -\cos\varphi \sin\theta \cos\psi + \sin\varphi\sin\psi & \cos\varphi \sin\theta \sin\psi +\sin\varphi\cos\psi & \cos\varphi \cos\theta
  \end{array}
  \right)
  \f]

  The code below shows first how to contruct a rotation matrix from a
  vpRxyzVector and than how to extract the vpRxyzVector Euler angles
  from the build rotation matrix.

  \code
#include <iostream>
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRxyzVector.h>

int main()
{
  vpRxyzVector rxyz;

  // Initialise the Euler angles
  rxyz[0] = vpMath::rad( 45.f); // phi   angle in rad/s around x axis 
  rxyz[1] = vpMath::rad(-30.f); // theta angle in rad/s around y axis
  rxyz[2] = vpMath::rad( 90.f); // psi   angle in rad/s around z axis

  // Construct a rotation matrix from the Euler angles
  vpRotationMatrix R(rxyz);

  // Extract the Euler angles around x,y,z axis from a rotation matrix
  rxyz.buildFrom(R);

  std::cout << rxyz; // Print the Euler angles
}
  \endcode

*/

class VISP_EXPORT vpRxyzVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpThetaUVector;
  
 public:
  //! Default constructor. Initialize the angles to zero.
  vpRxyzVector() { ; }
    
  // Copy constructor.
  vpRxyzVector(const vpRxyzVector &m);
    
  /*!
    Constructor from 3 angles (in radian).
    \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
    \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
    \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
  */
  vpRxyzVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  // initialize a Rxyz vector from a rotation matrix
  vpRxyzVector(const vpRotationMatrix& R) ;

  // initialize a Rxyz vector from a ThetaU vector
  vpRxyzVector(const vpThetaUVector&  tu) ;

  // Affectation of two vectors.
  vpRxyzVector &operator=(const vpRxyzVector &m);

  // convert a rotation matrix into Rxyz vector
  vpRxyzVector buildFrom(const vpRotationMatrix& R) ;

  // convert a ThetaU vector into a Rxyz vector
  vpRxyzVector buildFrom(const vpThetaUVector& tu) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
