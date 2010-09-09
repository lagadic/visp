/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Generic rotation vector (cannot be used as is !).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <math.h>
#include <visp/vpRotationVector.h>

/*!
  \file vpRotationVector.cpp
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/


/*! 
  Constructor from 3 angles \f$\varphi,\theta,\psi\f$ expressed in radians.
  
  \param phi, theta, psi : Respectively the first, the second and the
  third angle of the rotation vector.
*/
vpRotationVector::vpRotationVector(const double phi,
				   const double theta,
				   const double psi)
{
  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;
}
/*!  
  
  Initialize a rotation vector from 3 angles \f$\varphi,\theta,\psi\f$
  expressed in radians.
  
  \param phi, theta, psi : Respectively the first, the second and the
  third angle of the rotation vector.
*/
void
vpRotationVector::set(const double phi,
		      const double theta,
		      const double psi)
{
  r[0] = phi ;
  r[1] = theta ;
  r[2] = psi ;
}

/*!
  Transpose the rotation vector.

  \return Return \f$[r[Ø] r[1] r[2]]\f$.
 */
vpRowVector vpRotationVector::t() const
{
  vpRowVector v(3);

  v[0] = r[0];
  v[1] = r[1];
  v[2] = r[2];

  return v;
}

/*!

  Print the values of the three angles on the output stream. Data are 
  formatted as a column vector. 

  \code
#include <iostream>
#include <visp/vpRxyzVector.h>

int main()
{
  vpRxyzVector r; // By default initialized to zero
  
  std::cout << "Rxyz rotation vector: " << std::endl << r << std::endl;
}
  \endcode

  will lead to the following printing on the standart stream:

  \code
Rxyz rotation vector:
0
0
0
  \endcode
*/
std::ostream &operator <<(std::ostream &s,const vpRotationVector &m)
{
  s.precision(10) ;

  for (int i=0; i<3; i++)
    s <<  m.r[i] << "\n";

  s << std::endl;

  return s;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
