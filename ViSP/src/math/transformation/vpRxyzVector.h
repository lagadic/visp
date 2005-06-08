
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
 *  $Id: vpRxyzVector.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpRxyzVECTOR_H
#define vpRxyzVECTOR_H

/*!
  \file vpRxyzVector.h
  \brief  class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation  for the  rotation
*/

#include <visp/vpMatrix.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>


/*!
  \class vpRxyzVector
  \brief class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation  for the  rotation
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

  //! constructor initialize a thetaU vector from a rotation matrix
  vpRxyzVector(const vpRotationMatrix& R) ;
  //! constructor initialize a thetaU vector from a rotation matrix
  vpRxyzVector(const vpThetaUVector&  tu) ;

  //! convert a rotation matrix into Ryp vector
  vpRxyzVector buildFrom(const vpRotationMatrix& R) ;
  //! convert a  thetau vector into Ryp vector
  vpRxyzVector buildFrom(const vpThetaUVector& R) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
