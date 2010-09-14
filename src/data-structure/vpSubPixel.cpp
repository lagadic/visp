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
 * Sub pixel manipulation.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>

#include <visp/vpSubPixel.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
  \file vpSubPixel.cpp
  \brief Class that defines what is a sub-pixel.
*/

/*!
  Default contructor.
  Sub-pixel coordinates are set to zero.
*/
vpSubPixel::vpSubPixel()
{
  this->u = 0.0;
  this->v = 0.0;
}

/*!
  Contructor.
  
*/
vpSubPixel::vpSubPixel(const double &u, const double &v)
{
  this->u = u;
  this->v = v;
}

/*!
  Copy contructor.
  
*/
vpSubPixel::vpSubPixel(const vpSubPixel &p)
{
  this->u = p.u;
  this->v = p.v;
}

/*!
  Prints the sub-pixel coordinates "u v".

*/
std::ostream &operator << (std::ostream &s, const vpSubPixel &p)
{
  s.precision(10) ;
  
  s <<  p.u << " " <<p.v;

  return s;
}

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
