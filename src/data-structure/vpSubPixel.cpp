/****************************************************************************
 *
 * $Id: vpSubPixel.cpp,v 1.1 2007-05-14 08:37:20 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Sub pixel manipulation.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>

#include <visp/vpSubPixel.h>

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
  *this = p;
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


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
