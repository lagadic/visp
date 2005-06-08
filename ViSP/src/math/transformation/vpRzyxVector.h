
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
 *  $Id: vpRzyxVector.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpRzyxVECTOR_H
#define vpRzyxVECTOR_H

/*!
  \file vpRzyxVector.h
  \brief class that consider the case of the yow-pitch-roll  parameterization
  for the  rotation
*/

#include <visp/vpRotationMatrix.h>
#include <visp/vpRotationVector.h>


/*!
  \class vpRzyxVector
  \brief class that consider the case of the  yow-pitch-roll parameterization
  for the  rotation

  Rzyx(phi,theta,phi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)

*/

class vpRzyxVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpThetaUVector;

public:
  //! constructor
  vpRzyxVector() { ; }
public:
  //! copy operator
  vpRzyxVector &operator=(const vpRzyxVector &m);
  //! copy constructor
  vpRzyxVector(const vpRzyxVector &m);
  //! constructor initialize a thetaU vector from a rotation matrix
  vpRzyxVector(const vpRotationMatrix& R) ;
  //! constructor initialize a thetaU vector from a rotation matrix
  vpRzyxVector(const vpThetaUVector&  tu) ;

  //! constructor from 3 angles (in radian)
  vpRzyxVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into Ryp vector
  vpRzyxVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  thetau vector into Ryp vector
  vpRzyxVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
