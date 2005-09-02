
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRzyxVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpRzyxVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpRzyxVector.cpp,v 1.4 2005-09-02 14:35:17 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Rzyx angle parameterization for the
 *   rotation
 *
 *  Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <math.h>
#include <visp/vpRzyxVector.h>

/*!
  \file vpRzyxVector.cpp
  \brief class that consider the case of the  Rzyx angle parameterization for the  rotation :
  Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta,Rot(x,psi)

*/


/*!
  \brief  affectation of two vector
*/
vpRzyxVector &
vpRzyxVector::operator=(const vpRzyxVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! copy constructor
vpRzyxVector::vpRzyxVector(const vpRzyxVector &m)
{

    *this = m ;
}


//! initialize a Rzyx vector from a rotation matrix
vpRzyxVector::vpRzyxVector(const vpRotationMatrix& R)
{
    buildFrom(R) ;
}


//! initialize a Rzyx vector from a Theta U vector
vpRzyxVector::vpRzyxVector(const vpThetaUVector& tu)
{
    buildFrom(tu) ;
}

/*! convert a rotation matrix into Rzyx vector

  source
  R. Paul, Robot Manipulators: Mathematics, Programming, and Control.
  MIT Press, 1981,  p. 71
*/
vpRzyxVector
vpRzyxVector::buildFrom(const vpRotationMatrix& R)
{
  double nx = R[0][0];
  double ny = R[1][0];

  double phi = atan2(ny,nx) ;
  double si = sin(phi) ;
  double co = cos(phi) ;

  double nz = R[2][0];
  double theta = atan2(-nz, co*nx+si*ny) ;

  double ax = R[0][2];
  double ay = R[1][2];
  double ox = R[0][1];
  double oy = R[1][1];

  double psi = atan2(si*ax-co*ay,-si*ox+co*oy);

  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;

  return *this ;
}


//! convert a rotation matrix into Theta U vector
vpRzyxVector
vpRzyxVector::buildFrom(const vpThetaUVector& tu)
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
