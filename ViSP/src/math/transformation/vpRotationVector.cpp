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
