
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpEulerVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpEulerVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpEulerVector.cpp,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Euler angle parameterization for the
 *   rotation
 *
 *  Euler(phi,theta,phi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 *
 *  Rzyz
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <math.h>
#include <visp/vpEulerVector.h>


/*!
  \file vpEulerVector.cpp

  \brief class that consider the case of the euler angles  parameterization
  for the  rotation

  Euler(phi,theta,phi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

/*!
  \brief  affectation of two euler vector matrix
*/
vpEulerVector &
vpEulerVector::operator=(const vpEulerVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! copy constructor
vpEulerVector::vpEulerVector(const vpEulerVector &m)
{
    *this = m ;
}

//! initialize a Euler vector from a rotation matrix
vpEulerVector::vpEulerVector(const vpRotationMatrix& R)
{
    buildFrom(R) ;
}


//! initialize a Euler vector from a thetau vector
vpEulerVector::vpEulerVector(const vpThetaUVector& tu)
{
    buildFrom(tu) ;
}

//! convert a rotation matrix into Euler vector
vpEulerVector
vpEulerVector::buildFrom(const vpRotationMatrix& R)
{
    double phi ;
    if ((fabs(R[1][2]) < 1e-6) &&(fabs(R[0][2]) < 1e-6))
	phi = 0 ;
    else
	phi = atan2(R[1][2],R[0][2]) ;
    double cphi = cos(phi) ;
    double sphi = sin(phi) ;

    double theta = atan2(cphi*R[0][2]+sphi*R[1][2],R[2][2]);

    double psi = atan2(-sphi*R[0][0]+cphi*R[1][0],-sphi*R[0][1]+cphi*R[1][1]) ;

    r[0] = phi ;
    r[1] = theta ;
    r[2] = psi ;

    return *this ;
}


//! convert a rotation matrix into thetaU vector
vpEulerVector
vpEulerVector::buildFrom(const vpThetaUVector& tu)
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

