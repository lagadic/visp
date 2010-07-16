/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Theta U parameterization for the rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpTHETAUVECTOR_H
#define vpTHETAUVECTOR_H

/*!
  \file vpThetaUVector.h
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

class vpHomogeneousMatrix;
class vpRotationMatrix;
class vpRzyxVector;
class vpRxyzVector;
class vpEulerVector;
class vpRzyzVector;

#include <visp/vpConfig.h>
#include <visp/vpRotationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRzyxVector.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp/vpEulerVector.h>
#endif

/*!
  \class vpThetaUVector

  \ingroup RotTransformation

  \brief Class that consider the case of the \f$\theta {\bf u}\f$
  parameterization for the rotation.

  The \f$\theta {\bf u}\f$ representation is one of the minimal
  representation of a rotation matrix, where 
  \f${\bf u} = (u_{x} \; u_{y} \; u_{z})^{\top}\f$ 
  is a unit vector representing the rotation
  axis and \f$\theta\f$ is the rotation angle.

  From the \f$\theta {\bf u}\f$ representation it is possible to build the
  rotation matrix \f${\bf R}\f$ using the Rodrigues formula:

  \f[
  {\bf R} =  {\bf I}_{3} + (1 - \cos{ \theta}) \; {\bf u u}^{\top} + \sin{ \theta} \; [{\bf u}]_{\times}
  \f]

  with \f${\bf I}_{3}\f$ the identity matrix of dimension
  \f$3\times3\f$ and \f$[{\bf u}]_{\times}\f$ the skew matrix:

  \f[
  [{\bf u}]_{\times} = \left(
  \begin{array}{ccc}    
  0 & -u_{z} & u_{y} \\
  u_{z} & 0 & -u_{x} \\
  -u_{y} & u_{x} & 0
  \end{array}
  \right)
  \f]
  From the implementation point of view, it is nothing more than an
  array of three floats. 

  The code below shows first how to initialize a \f$\theta {\bf u}\f$
  vector, than how to contruct a rotation matrix from a vpThetaUVector
  and finaly how to extract the theta U angles from the build rotation
  matrix.

  \code
#include <iostream>
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpThetaUVector.h>

int main()
{
  vpThetaUVector tu;

  // Initialise the theta U rotation vector
  tu[0] = vpMath::rad( 45.f); 
  tu[1] = vpMath::rad(-30.f); 
  tu[2] = vpMath::rad( 90.f); 

  // Do the same initialization
  tu.set(vpMath::rad( 45.f), vpMath::rad(-30.f), vpMath::rad( 90.f));

  // Construct a rotation matrix from the theta U angles
  vpRotationMatrix R(tu);

  // Extract the theta U angles from a rotation matrix
  tu.buildFrom(R);

  // Print the extracted theta U angles. Values are the same than the
  // one used for initialization
  std::cout << tu; 

  // Since the rotation vector is 3 values column vector, the
  // transpose operation produce a row vector.
  vpRowVector tu_t = tu.t();
  
  // Print the transpose row vector
  std::cout << tu_t << std::endl;
}
  \endcode
*/
class VISP_EXPORT vpThetaUVector : public vpRotationVector
{

private:
  //! initialize a size 3 vector
  void init() ;

  static const double minimum;

public:

  // constructor
  vpThetaUVector() { ; }
  // copy constructor
  vpThetaUVector(const vpThetaUVector &tu) ;

  // constructor initialize a Theta U vector from a homogeneous matrix
  vpThetaUVector(const vpHomogeneousMatrix & M) ;
  // constructor initialize a Theta U vector from a rotation matrix
  vpThetaUVector(const vpRotationMatrix& R) ;
  // constructor initialize a Theta U vector from a RzyxVector
  vpThetaUVector(const vpRzyxVector& rzyx) ;
  // constructor initialize a Theta U vector from a RzyzVector
  vpThetaUVector(const vpRzyzVector& rzyz) ;
  // constructor initialize a Theta U vector from a RxyzVector
  vpThetaUVector(const vpRxyzVector& rxyz) ;

  /*!
    Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
  */
  vpThetaUVector(const double tux, const double tuy, const double tuz) :
    vpRotationVector (tux, tuy, tuz) { ; }

  // convert an homogeneous matrix into Theta U vector
  vpThetaUVector buildFrom(const vpHomogeneousMatrix& M) ;
  // convert a rotation matrix into Theta U vector
  vpThetaUVector buildFrom(const vpRotationMatrix& R) ;
  // convert an Rzyx vector into Theta U vector
  vpThetaUVector buildFrom(const vpRzyxVector &rzyx) ;
  // convert an Rzyz vector into Theta U vector
  vpThetaUVector buildFrom(const vpRzyzVector &zyz) ;
  // convert an Rxyz vector into Theta U vector
  vpThetaUVector buildFrom(const vpRxyzVector &xyz) ;

  // copy operator
  vpThetaUVector &operator=(const vpThetaUVector &tu);
  vpThetaUVector &operator=(double x) ;

  // extract the angle and the axis from the ThetaU representation
  void extract( double &theta, vpColVector &u) const;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  //! \deprecated Convert an Euler vector into Theta U vector
  vpThetaUVector buildFrom(const vpEulerVector &euler) ;
#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
 
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

