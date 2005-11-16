
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
 *  $Id: vpRxyzVector.h,v 1.3 2005-11-16 09:44:07 fspindle Exp $
 *
 * Description
 * ============
 *    class that consider the case of the  Rxyz angle parameterization for the rotation

 *   Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpRxyzVECTOR_H
#define vpRxyzVECTOR_H

/*!
  \file vpRxyzVector.h
  \brief   class that consider the case of the  Rxyz angle parameterization for the rotation :
  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

#include <visp/vpMatrix.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRxyzVector
  \brief  class that consider the case of the  Rxyz angle parameterization for the rotation
  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

class vpRxyzVector : public vpRotationVector
{
    friend class vpRotationMatrix;
    friend class vpThetaUVector;
private:
  //! initialize a size 3 vector
  void init() ;

public:
  //! constructor
  vpRxyzVector() { ; }
  //! copy operator
  vpRxyzVector &operator=(const vpRxyzVector &m);
  //! copy constructor
  vpRxyzVector(const vpRxyzVector &m);

  //! constructor from 3 angles (in radian)
  vpRxyzVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! constructor initialize a Theta U vector from a rotation matrix
  vpRxyzVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vpRxyzVector(const vpThetaUVector&  tu) ;

  //! convert a rotation matrix into Rxyz vector
  vpRxyzVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  Theta U vector into Rxyz vector
  vpRxyzVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
