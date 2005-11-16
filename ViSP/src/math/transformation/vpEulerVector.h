
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
 *  $Id: vpEulerVector.h,v 1.4 2005-11-16 09:44:07 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Euler angles parameterization for the
 *   rotation
 *   Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpEULERVECTOR_H
#define vpEULERVECTOR_H

/*!
  \file vpEulerVector.h
  \brief class that consider the case of the euler angles  parameterization
  for the  rotation

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpEulerVector
  \brief class that consider the case of the euler angles  parameterization
  for the  rotation

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/
class vpEulerVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpThetaUVector;

public:
  //! constructor
  vpEulerVector() { ; }
  //! copy operator
  vpEulerVector &operator=(const vpEulerVector &m);
  //! copy constructor
  vpEulerVector(const vpEulerVector &m);

  //! constructor initialize a Theta U vector from a rotation matrix
  vpEulerVector(const vpRotationMatrix& R) ;
  //! constructor initialize a Theta U vector from a rotation matrix
  vpEulerVector(const vpThetaUVector&  tu) ;

  //! constructor from 3 angles (in radian)
  vpEulerVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into Euler vector
  vpEulerVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  Theta U vector into Euler vector
  vpEulerVector buildFrom(const vpThetaUVector& R) ;

} ;

/*!
  \class vpRzyzVector
  \brief class that consider the case of the Euler angles  parameterization
  for the  rotation (this is fully equivalent to the vpEulerVector class)


  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/
class vpRzyzVector : public vpEulerVector
{
public:
  //!  constructor
  vpRzyzVector() {  ; }
  //! copy constructor
  vpRzyzVector(const vpRzyzVector &m) : vpEulerVector(m){ ;}

  //! constructor from 3 angles (in radian)
  vpRzyzVector(const double phi, const double theta, const double psi) :
    vpEulerVector(phi, theta, psi) { ; }

  //! constructor initialize a Theta U vector from a rotation matrix
  vpRzyzVector(const vpRotationMatrix& R) : vpEulerVector(R){ ;}
  //! constructor initialize a Theta U vector from a rotation matrix
  vpRzyzVector(const vpThetaUVector&  tu) : vpEulerVector(tu) { ;}

} ;
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
