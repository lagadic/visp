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
 * Theta U parameterization for the rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
\file vpThetaUVector.cpp
\brief class that consider the case of the Theta U parameterization for the
rotation
*/

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#include <visp3/core/vpThetaUVector.h>

const double vpThetaUVector::minimum = 0.0001;

/*! Default constructor that initialize all the 3 angles to zero. */
vpThetaUVector::vpThetaUVector()
  : vpRotationVector(3)
{}
/*! Copy constructor. */
vpThetaUVector::vpThetaUVector(const vpThetaUVector &tu)
  : vpRotationVector(tu)
{}

/*!
Initialize a \f$\theta {\bf u}\f$ vector from an homogeneous matrix.
*/
vpThetaUVector::vpThetaUVector(const vpHomogeneousMatrix& M)
  : vpRotationVector(3)
{
  buildFrom(M) ;
}
/*!
Initialize a \f$\theta {\bf u}\f$ vector from a pose vector.
*/
vpThetaUVector::vpThetaUVector(const vpPoseVector& p)
  : vpRotationVector(3)
{
  buildFrom(p) ;
}
/*!
Initialize a \f$\theta {\bf u}\f$ vector from a rotation matrix.
*/
vpThetaUVector::vpThetaUVector(const vpRotationMatrix& R)
  : vpRotationVector(3)
{
  buildFrom(R) ;
}

/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-x
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyxVector& rzyx)
  : vpRotationVector(3)
{
  buildFrom(rzyx) ;
} 
/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-z
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyzVector& rzyz)
  : vpRotationVector(3)
{
  buildFrom(rzyz) ;
}
/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler x-y-z
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRxyzVector& rxyz)
  : vpRotationVector(3)
{
  buildFrom(rxyz) ;
}
/*!
Initialize a \f$\theta {\bf u}\f$ vector from a quaternion
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpQuaternionVector& q)
  : vpRotationVector(4)
{
  buildFrom(q) ;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
*/
vpThetaUVector::vpThetaUVector(const double tux, const double tuy, const double tuz)
  : vpRotationVector (3)
{
  buildFrom(tux, tuy, tuz);
}

/*!
Converts an homogeneous matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpHomogeneousMatrix& M)
{
  vpRotationMatrix R;

  M.extract(R);
  buildFrom(R);

  return *this ;
}
/*!
Converts a pose vector into a \f$\theta {\bf u}\f$ vector copying
the \f$\theta {\bf u}\f$ values contained in the pose vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpPoseVector& p)
{
  for(unsigned int i=0; i<3; i++)
    data[i] = p[i+3];

  return *this ;
}

/*!
Converts a rotation matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpRotationMatrix& R)
{
  double s,c,theta,sinc;

  s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
    + (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
    + (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
  s = sqrt(s)/2.0;
  c = (R[0][0]+R[1][1]+R[2][2]-1.0)/2.0;
  theta=atan2(s,c);  /* theta in [0, PI] since s > 0 */

  // General case when theta != pi. If theta=pi, c=-1
  if ( (1+c) > minimum) // Since -1 <= c <= 1, no fabs(1+c) is required
  {
    sinc = vpMath::sinc(s,theta);

    data[0] = (R[2][1]-R[1][2])/(2*sinc);
    data[1] = (R[0][2]-R[2][0])/(2*sinc);
    data[2] = (R[1][0]-R[0][1])/(2*sinc);
  }
  else /* theta near PI */
  {
    if ( (R[0][0]-c) < std::numeric_limits<double>::epsilon() )
      data[0] = 0.;
    else
      data[0] = theta*(sqrt((R[0][0]-c)/(1-c)));
    if ((R[2][1]-R[1][2]) < 0) data[0] = -data[0];

    if ( (R[1][1]-c) < std::numeric_limits<double>::epsilon() )
      data[1] = 0.;
    else
      data[1] = theta*(sqrt((R[1][1]-c)/(1-c)));

    if ((R[0][2]-R[2][0]) < 0) data[1] = -data[1];

    if ( (R[2][2]-c) < std::numeric_limits<double>::epsilon() )
      data[2] = 0.;
    else
      data[2] = theta*(sqrt((R[2][2]-c)/(1-c)));

    if ((R[1][0]-R[0][1]) < 0) data[2] = -data[2];
  }

  return *this ;
}
/*!  
Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-x
representation vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpRzyxVector& rzyx)
{
  vpRotationMatrix R(rzyx) ;

  buildFrom(R) ;
  return *this ;
}
/*!  
Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-z
representation vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpRzyzVector& rzyz)
{
  vpRotationMatrix R(rzyz) ;

  buildFrom(R) ;
  return *this ;
}
/*!
Build a \f$\theta {\bf u}\f$ vector from an Euler x-y-z
representation vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpRxyzVector& rxyz)
{
  vpRotationMatrix R(rxyz) ;

  buildFrom(R) ;
  return *this ;
}

/*!
Build a \f$\theta {\bf u}\f$ vector from a quaternion
representation vector.
*/
vpThetaUVector
vpThetaUVector::buildFrom(const vpQuaternionVector& q)
{
  vpRotationMatrix R(q) ;

  buildFrom(R) ;
  return *this ;
}

/*!

  Initialize each element of the \f$\theta {\bf u}\f$ vector to the
  same angle value \e v.

  \param v : Angle value to set for each element of the \f$\theta {\bf
  u}\f$ vector.

\code
#include <visp3/core/vpMath.h>
#include <visp3/core/vpThetaUVector.h>

int main()
{
  vpThetaUVector tu;

  // Initialise the theta U rotation vector
  tu = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
}
\endcode
*/
vpThetaUVector &vpThetaUVector::operator=(double v)
{
  for (unsigned int i=0; i< dsize; i++)
    data[i] = v;

  return *this;
}

/*!

Extract the rotation angle \f$ \theta \f$ and the unit vector
\f$\bf u \f$ from the \f$ \theta {\bf u} \f$ representation.

\param theta : Rotation angle \f$ \theta \f$.

\param u : Unit vector \f${\bf u} = (u_{x},u_{y},u_{z})^{\top} \f$
representing the rotation axis.

*/
void 
vpThetaUVector::extract(double &theta, vpColVector &u) const
{
  u.resize(3);

  theta = sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);
  //if (theta == 0) {
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return;
  }
  for (unsigned int i=0 ; i < 3 ; i++) 
    u[i] = data[i] / theta ;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
*/
void
vpThetaUVector::buildFrom(const double tux, const double tuy, const double tuz)
{
  data[0] = tux;
  data[1] = tuy;
  data[2] = tuz;
}
