/****************************************************************************
 *
 * $Id: vpRzyxVector.h,v 1.7 2008-11-14 17:45:11 fspindle Exp $
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
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRzyxVector_h
#define vpRzyxVector_h

/*!
  \file vpRzyxVector.h

  \brief class that consider the case of the Rzyx angle
  parameterization for the rotation.

  Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
*/

#include <visp/vpConfig.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRotationVector.h>

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRzyxVector

  \ingroup RotTransformation

  \brief Class that consider the case of the Euler
  \f$(\varphi,\theta,\psi)\f$ angle using the z-y-x convention, where \f$(\varphi,\theta,\psi)\f$ are respectively the
  rotation angles around the \f$z\f$, \f$y\f$ and \f$x\f$ axis.

  \f[R_{zyx}(\varphi,\theta,\psi) = R_z(\varphi) \; R_y(\theta) \; R_x(\psi)\f]

  with

  \f[
  R_{z}(\varphi) = \left(
  \begin{array}{ccc}
  \cos \varphi & -\sin\varphi & 0\\
  \sin\varphi &\cos \varphi& 0 \\
  0 & 0 & 1
  \end{array}
  \right) \;
  R_{y}(\theta) = \left(
  \begin{array}{ccc}
  \cos \theta & 0 & \sin\theta\\
  0 & 1 & 0 \\
  -\sin\theta & 0 &\cos \theta
  \end{array}
  \right) \;
  R_{x}(\psi) = \left(
  \begin{array}{ccc}
  1 & 0 & 0 \\
  0 &\cos \psi & -\sin\psi \\
  0 &\sin \psi & \cos\psi \\
  \end{array}
  \right) 
  \f]

  The rotation matrix corresponding to the z-y-x convention is given by:

  \f[
  R_{zyx}(\varphi,\theta,\psi) = \left(
  \begin{array}{ccc}
  \cos\varphi \cos\theta & -\sin\varphi \cos\psi + \cos\varphi\sin\theta\sin\psi & \sin\varphi \sin\psi +\cos\varphi\sin\theta\cos\psi \\
  \sin\varphi \cos\theta & \cos\varphi\cos\psi + \sin\varphi\sin\theta \sin\psi & -\cos\varphi \sin\psi +\sin\varphi\sin\theta\cos\psi \\
  -\sin\theta & \cos\varphi \sin\psi & \cos\theta \cos\psi
  \end{array}
  \right)
  \f]

  The code below shows first how to contruct a rotation matrix from a
  vpRzyxVector and than how to extract the vpRzyxVector Euler angles
  from the build rotation matrix.

  \code
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRzyxVector.h>

int main()
{
  vpRzyxVector rzyx;

  // Initialise the Euler angles
  rzyx[0] = vpMath::rad( 45.f); // phi   angle in rad/s around z axis 
  rzyx[1] = vpMath::rad(-30.f); // theta angle in rad/s around y axis
  rzyx[2] = vpMath::rad( 90.f); // psi   angle in rad/s around x axis

  // Construct a rotation matrix from the Euler angles
  vpRotationMatrix R(rzyx);

  // Extract the Euler angles around z,y,x axis from a rotation matrix
  rzyx.buildFrom(R);

  std::cout << rzyx; // Print the Euler angles
}
  \endcode

*/

class VISP_EXPORT vpRzyxVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpThetaUVector;

public:
  //! Default constructor. Initialize the angles to zero.
  vpRzyxVector() { ; }

  // Copy constructor.
  vpRzyxVector(const vpRzyxVector &m);

  /*!
    Constructor from 3 angles (in radian).
    \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
    \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
    \param psi : \f$\psi\f$ angle around the \f$x\f$ axis.
  */
  vpRzyxVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  // initialize a Rzyx vector from a rotation matrix
  vpRzyxVector(const vpRotationMatrix& R) ;

  // initialize a Rzyx vector from a ThetaU vector
  vpRzyxVector(const vpThetaUVector&  tu) ;

  // Affectation of two vectors.
  vpRzyxVector &operator=(const vpRzyxVector &m);

  // convert a rotation matrix into Rzyx vector
  vpRzyxVector buildFrom(const vpRotationMatrix& R) ;

  // convert a ThetaU vector into a Rzyx vector
  vpRzyxVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
