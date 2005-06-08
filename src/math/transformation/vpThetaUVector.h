
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpThetaUVector.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpThetaUVector.h, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpThetaUVector.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Theta U parameterization for the
 *   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpTHETAUVECTOR_H
#define vpTHETAUVECTOR_H

/*!
  \file vpThetaUVector.h
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpRxyzVector.h>


/*!
  \class vpThetaUVector
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/
class vpThetaUVector : public vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpEulerVector;
  friend class vpRxyzVector;
  friend class vpRzyzVector;
private:
  //! initialize a size 3 vector
  void init() ;

public:

  //! constructor
  vpThetaUVector() { ; }
  //! copy operator
  vpThetaUVector &operator=(const vpThetaUVector &m);
  //! copy constructor
  vpThetaUVector(const vpThetaUVector &m) ;

  //! constructor initialize a thetaU vector from a rotation matrix
  vpThetaUVector(const vpRotationMatrix& R) ;

  //! constructor from 3 angles (in radian)
  vpThetaUVector(const double phi, const double theta, const double psi) :
    vpRotationVector (phi, theta, psi) { ; }

  //! convert a rotation matrix into thetaU vector
  vpThetaUVector buildFrom(const vpRotationMatrix& R) ;
  //! convert an Rzyz vector into thetaU vector
  vpThetaUVector buildFrom(const vpEulerVector &e) ;
  //! convert an Rzyz vector into thetaU vector
  vpThetaUVector buildFrom(const vpRzyzVector &e) ;
  //! convert an Rxyz vector into thetaU vector
  vpThetaUVector buildFrom(const vpRxyzVector &e) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

