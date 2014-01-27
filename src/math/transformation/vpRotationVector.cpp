/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
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

#include <visp/vpRotationVector.h>
#include <algorithm>
#include <math.h>
/*!
  \file vpRotationVector.cpp
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/



/*!
  Return the transpose of the rotation vector.

*/
vpRowVector vpRotationVector::t() const
{
  vpRowVector v(_size);

  for (unsigned int i=0; i< _size; i++)
    v[i] = r[i];

  return v;
}

/*!
	Size of the rotation vector: number of double values describing the rotation.
	Common sizes are 4 for a quaternion and 3 for angle-based rotation vectors.
*/
unsigned int vpRotationVector::size() const {
	return _size;
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
VISP_EXPORT std::ostream &operator <<(std::ostream &s,const vpRotationVector &m)
{
  std::ios::fmtflags original_flags( s.flags() );
  s.precision(10) ;

  for (unsigned int i=0; i < m.size(); i++)
    s <<  m.r[i] << "\n";

  s << std::endl;

  // Restore ostream format
  s.flags(original_flags);

  return s;
}


void vpRotationVector::init(const unsigned int size){
	this->_size = size;
	r = new double[this->_size];
	std::fill(r,r+this->_size,0.);
}

vpRotationVector::~vpRotationVector(){
	delete[] r;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
