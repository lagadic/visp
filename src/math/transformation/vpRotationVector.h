
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRotationVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpRotationVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpRotationVector.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Roll Yaw pitch angles parameterization for the   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpRotationVECTOR_H
#define vpRotationVECTOR_H

/*!
  \file vpRotationVector.h
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/

#include <visp/vpMath.h>

#include <stdio.h>
#include <iostream>
using namespace std;

#include <math.h>

#include <visp/vpConfig.h>


//#include <vpRotationMatrix.h>

/*!
  \class vpRotationVector
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/

class vpRotationVector
{
  friend class vpRotationMatrix;

protected:
  double r[3] ;


public:
  //! constructor
  vpRotationVector() { ; }


  //! constructor from 3 angles (in radian)
  vpRotationVector(const double phi, const double theta, const double psi) ;

  //! convert a rotation matrix into Ryp vector
  //  virtual vpRotationVector buildFrom(const vpRotationMatrix& R) =0 ;


  //! Access  r[i] = x
  inline double &operator [](int n) {  return *(r + n);  }
  //! Access x = r[i]
  inline const double &operator [](int n) const { return *(r+n);  }

 //---------------------------------
  // Printing
  //---------------------------------
  friend ostream &operator << (ostream &s,const vpRotationVector &m);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
