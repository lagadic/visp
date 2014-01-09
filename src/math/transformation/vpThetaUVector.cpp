/****************************************************************************
*
* $Id$
*
* This file is part of the ViSP software.
* Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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



#include <visp/vpThetaUVector.h>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#define vpDEBUG_LEVEL1 0

const double vpThetaUVector::minimum = 0.0001;

/*!
Affectation of two \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector &
vpThetaUVector::operator=(const vpThetaUVector &m)
{
  for (int i=0; i<3; i++)
  {
    r[i] = m.r[i] ;
  }
  return *this;
}


/*!
Copy constructor.
*/
vpThetaUVector::vpThetaUVector(const vpThetaUVector &m) : vpRotationVector()
{
  *this = m ;
}

/*!
Initialize a \f$\theta {\bf u}\f$ vector from an homogeneous matrix.
*/
vpThetaUVector::vpThetaUVector(const vpHomogeneousMatrix& M)
{
  buildFrom(M) ;
}
/*!
Initialize a \f$\theta {\bf u}\f$ vector from a rotation matrix.
*/
vpThetaUVector::vpThetaUVector(const vpRotationMatrix& R)
{
  buildFrom(R) ;
}

/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-x
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyxVector& rzyx)
{
  buildFrom(rzyx) ;
} 
/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-z
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyzVector& rzyz)
{
  buildFrom(rzyz) ;
}
/*!  
Initialize a \f$\theta {\bf u}\f$ vector from an Euler x-y-z
representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRxyzVector& rxyz)
{
  buildFrom(rxyz) ;
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

    r[0] = (R[2][1]-R[1][2])/(2*sinc);
    r[1] = (R[0][2]-R[2][0])/(2*sinc);
    r[2] = (R[1][0]-R[0][1])/(2*sinc);
  }
  else /* theta near PI */
  {
    if ( (R[0][0]-c) < std::numeric_limits<double>::epsilon() )
      r[0] = 0.;
    else
      r[0] = theta*(sqrt((R[0][0]-c)/(1-c)));
    if ((R[2][1]-R[1][2]) < 0) r[0] = -r[0];

    if ( (R[1][1]-c) < std::numeric_limits<double>::epsilon() )
      r[1] = 0.;
    else
      r[1] = theta*(sqrt((R[1][1]-c)/(1-c)));

    if ((R[0][2]-R[2][0]) < 0) r[1] = -r[1];

    if ( (R[2][2]-c) < std::numeric_limits<double>::epsilon() )
      r[2] = 0.;
    else
      r[2] = theta*(sqrt((R[2][2]-c)/(1-c)));

    if ((R[1][0]-R[0][1]) < 0) r[2] = -r[2];
  }

#if (vpDEBUG_LEVEL1)  // test new version wrt old version
  {
    // old version
    int i;
    //    double s,c;
    double ang;
    double r2[3]; // has to be replaced by r below if good version

    s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
      + (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
      + (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
    s = sqrt(s)/2.0;
    c = (R[0][0]+R[1][1]+R[2][2]-1)/2.0;
    ang=atan2(s,c);
    if (ang > minimum)
    {
      if (s > minimum)
      {
        r2[0] = (R[2][1]-R[1][2])/(2*s);
        r2[1] = (R[0][2]-R[2][0])/(2*s);
        r2[2] = (R[1][0]-R[0][1])/(2*s);
      }
      else
      {
        r2[0] = (sqrt((R[0][0]-c)/(1-c)));
        if ((R[2][1]-R[1][2]) < 0) r2[0] = -r2[0];
        r2[1] = (sqrt((R[1][1]-c)/(1-c)));
        if ((R[0][2]-R[2][0]) < 0) r2[1] = -r2[1];
        r2[2] = (sqrt((R[2][2]-c)/(1-c)));
        if ((R[1][0]-R[0][1]) < 0) r2[2] = -r2[2];
      }
      for (i=0;i<3;i++) r2[i] = r2[i]*ang;
    }
    else
    {
      r2[0] =   r2[1] =   r2[2] = 0.0;
    }
    // end old version
    // verification of the new version
    int pb = 0;

    for (i=0;i<3;i++)
    {
      if (fabs(r[i] - r2[i]) > 1e-5) pb = 1;
    }
    if (pb == 1)
    {
      printf("vpThetaUVector::buildFrom(const vpRotationMatrix& R)\n");
      printf(" r      : %lf %lf %lf\n",r[0],r[1],r[2]);
      printf(" r2     : %lf %lf %lf\n",r2[0],r2[1],r2[2]);
      printf(" r - r2 : %lf %lf %lf\n",r[0]-r2[0],r[1]-r2[1],r[2]-r2[2]);
    }
    // end of the verification
  }
#endif
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

Initialize each element of the \f$\theta {\bf u}\f$ vector to the
same angle value \e v.

\param v : Angle value to set for each element of the \f$\theta {\bf
u}\f$ vector.

\code
#include <visp/vpMath.h>
#include <visp/vpThetaUVector.h>

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
  for (int i=0; i< 3; i++)
    r[i] = v;

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

  theta = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
  //if (theta == 0) {
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return;
  }
  for (unsigned int i=0 ; i < 3 ; i++) 
    u[i] = r[i] / theta ;
}

#undef vpDEBUG_LEVEL1
/*
* Local variables:
* c-basic-offset: 2
* End:
*/
