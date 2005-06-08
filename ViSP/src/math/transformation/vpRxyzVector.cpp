
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRxyzVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpRxyzVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpRxyzVector.cpp,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Rxyz angle parameterization for the
 *   rotation
 *
 *  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta,Rot(z,psi)
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <math.h>
#include <vpRxyzVector.h>

/*!
  \file vpRxyzVector.cpp
  \brief class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation  for the  rotation
*/


/*!
  \brief  affectation of two euler vector matrix
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


//! copy constructor
vpRxyzVector::vpRxyzVector(const vpRxyzVector &m)
{

    *this = m ;
}

//! initialize a Rxyz vector from a rotation matrix
vpRxyzVector::vpRxyzVector(const vpRotationMatrix& R)
{

    buildFrom(R) ;
}


//! initialize a Rxyz vector from a thetau vector
vpRxyzVector::vpRxyzVector(const vpThetaUVector& tu)
{

    buildFrom(tu) ;
}

//! convert a rotation matrix into Rxyz vector
vpRxyzVector
vpRxyzVector::buildFrom(const vpRotationMatrix& R)
{
  /*
    double  v1;
    v1 = R[0][2];
    if (v1 > 1.0 ) v1 = 1.0;
    if (v1 < -1.0 ) v1 = -1.0;
    r[1] = asin(v1);
    if ( fabs(fabs(r[1]) - M_PI_2) < 0.00001)
    {
	r[0] = 0.0;
	r[2] = atan2(R[1][0],R[1][1]);
    }
    else
    {
	r[0] = atan2(-R[1][2],R[2][2]);
	r[2] = atan2(-R[0][1],R[0][0]);
    }
  */

  double phi ;
  if ((fabs(R[1][2]) < 1e-6)&&(fabs(R[2][2]) < 1e-6)) phi = 0 ;
  else phi = atan2(R[1][2], R[2][2]) ;

  double si = sin(phi) ;
  double co = cos(phi) ;
  double theta = atan2(R[0][2], -si*R[1][2] + co*R[2][2]) ;
  double psi = atan2(co*R[0][1]+si*R[0][2], co*R[1][1]+si*R[1][2]);

  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;

  return *this ;
}


//! convert a rotation matrix into thetaU vector
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
