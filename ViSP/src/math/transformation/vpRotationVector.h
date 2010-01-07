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
 * Generic rotation vector (cannot be used as is !).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRotationVECTOR_H
#define vpRotationVECTOR_H

/*!
  \file vpRotationVector.h
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/


#include <stdio.h>
#include <iostream>


#include <math.h>

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpRowVector.h>


/*!
  \class vpRotationVector

  \ingroup RotTransformation

  \brief Class that consider the case of a generic rotation vector
  (cannot be used as is !) consisting in three angles.

  The code below shows how this class can be used to manipulate a vpRxyzVector.

  \code
#include <iostream>
#include <visp/vpRxyzVector.h>
#include <visp/vpMath.h>

int main() 
{
  vpRxyzVector r;         // By default initialized to zero
  r[0] = vpMath::rad(45); // Rotation arround x set to 45 degres converted in radians
  r[1] = M_PI;            // Rotation arround y set to PI radians
  r[2] = 0;               // Rotation arround z set to 0 radians
  
  std::cout << "Rxyz rotation vector: " << r << std::endl;

  double rx = r[1];       // Get the value of the angle arround x axis
  double ry = r[2];       // Get the value of the angle arround y axis
  double rz = r[3];       // Get the value of the angle arround z axis
}
  
  \endcode

*/

class VISP_EXPORT vpRotationVector
{
  friend class vpRotationMatrix;
  friend class vpColVector;
protected:
  double r[3] ;


public:
  //! Constructor that initialize the three angles to zero.
  vpRotationVector() { 
    r[0] = r[1] = r[2] = 0.;
  }
  // Constructor from 3 angles (in radian)
  vpRotationVector(const double phi, const double theta, const double psi) ;

  // Transpose of the rotation vector.
  vpRowVector t() const;

  /*!
    Operator that allows to set the value of an element of the rotation 
    vector: r[i] = value
  */
  inline double &operator [](int n) {  return *(r + n);  }
  /*!
    Operator that allows to get the value of an element of the rotation 
    vector: value = r[i]
  */
  inline const double &operator [](int n) const { return *(r+n);  }

  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,
						const vpRotationVector &m);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
