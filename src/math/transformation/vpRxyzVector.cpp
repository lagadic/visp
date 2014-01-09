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
 * Rxyz angle parameterization for the rotation.
 * Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi).
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpRxyzVector.h>
#include <math.h>
/*!
  \file vpRxyzVector.cpp
  \brief class that consider the case of the  Rxyz angle parameterization for the rotation :
  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/


/*!
  Affectation of two vectors.
*/
vpRxyzVector &
vpRxyzVector::operator=(const vpRxyzVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! Copy constructor.
vpRxyzVector::vpRxyzVector(const vpRxyzVector &m) : vpRotationVector()
{
  *this = m ;
}

/*! 
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpRotationMatrix& R)
{
  buildFrom(R) ;
}

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta u\f$ vector.
  \param tu : \f$\theta u\f$ representation of a rotation used here as 
  input to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpThetaUVector& tu)
{
  buildFrom(tu) ;
}

/*! 
  Convert a rotation matrix into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.
  
  \param R : Rotation matrix used as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRxyzVector
vpRxyzVector::buildFrom(const vpRotationMatrix& R)
{

  double COEF_MIN_ROT = 1e-6;
  double phi ;

  if ((fabs(R[1][2]) < COEF_MIN_ROT) && (fabs(R[2][2]) < COEF_MIN_ROT)) phi = 0 ;
  else phi = atan2(-R[1][2], R[2][2]) ;

  double si = sin(phi) ;
  double co = cos(phi) ;
  double theta = atan2(R[0][2], -si*R[1][2] + co*R[2][2]) ;
  double psi = atan2(co*R[1][0] + si*R[2][0], co*R[1][1] + si*R[2][1]);

  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;

  if (0)  // test new version wrt old version
  {
    // old version

    double  v1;
    double r2[3];  // has to be replaced by r below if good version

    v1 = R[0][2];
    if (v1 > 1.0 ) v1 = 1.0;
    if (v1 < -1.0 ) v1 = -1.0;
    r2[1] = asin(v1);
    if ( fabs(fabs(r2[1]) - M_PI_2) < 0.00001)
    {
	r2[0] = 0.0;
	r2[2] = atan2(R[1][0],R[1][1]);
    }
    else
    {
	r2[0] = atan2(-R[1][2],R[2][2]);
	r2[2] = atan2(-R[0][1],R[0][0]);
    }
    // verification of the new version
    int pb = 0;
    int i;
    for (i=0;i<3;i++)
    {
      if (fabs(r[i] - r2[i]) > 1e-5) pb = 1;
    }
    if (pb == 1)
    {
      printf("vpRxyzVector::buildFrom(const vpRotationMatrix& R)\n");
      printf(" r      : %lf %lf %lf\n",r[0],r[1],r[2]);
      printf(" r2     : %lf %lf %lf\n",r2[0],r2[1],r2[2]);
      printf(" r - r2 : %lf %lf %lf\n",r[0]-r2[0],r[1]-r2[1],r[2]-r2[2]);
    }
  }

    // What is below corresponds to another representation, but which one???
  /* double phi ;
  if ((fabs(R[1][2]) < 1e-6)&&(fabs(R[2][2]) < 1e-6)) phi = 0 ;
  else phi = atan2(R[1][2], R[2][2]) ;

  double si = sin(phi) ;
  double co = cos(phi) ;
  double theta = atan2(R[0][2], -si*R[1][2] + co*R[2][2]) ;
  double psi = atan2(co*R[0][1]+si*R[0][2], co*R[1][1]+si*R[1][2]);

  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;*/

  return *this ;
}

/*! 
  Convert a \f$\theta u\f$ vector into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ 
  Euler angles vector.
  \param tu : \f$\theta u\f$ representation of a rotation used here as 
  input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.   
*/
vpRxyzVector
vpRxyzVector::buildFrom(const vpThetaUVector& tu)
{
  vpRotationMatrix R ;
  R.buildFrom(tu) ;
  buildFrom(R) ;

  return *this ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
