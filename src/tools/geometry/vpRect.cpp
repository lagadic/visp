/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Defines a rectangle in the plane.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpRect.cpp
  \brief Defines a rectangle in the plane.
  \ingroup libtools
*/


#include <visp/vpRect.h>
#include <visp/vpDebug.h>

/*!
 
  Constructs a default rectangle with the \e top, \e left corner set to (0,0)
  and \e width and \e height set to 1.

*/

vpRect::vpRect()
{
  this->left = 0;
  this->top = 0;
  this->width = 0;
  this->height = 0;
};

/*!
 
  Constructs a rectangle with the \e top, \e left corner and \e width
  and \e height.

*/
vpRect::vpRect(double left, double top, double width, double height)
{
  this->left = left;
  this->top = top;
  this->width = width;
  this->height = height;
};

/*!
 
  Constructs a rectangle with \e topLeft the top-left corner location
  and \e width and \e height the rectangle size.

*/
vpRect::vpRect(const vpImagePoint &topLeft, double width, double height)
{
  this->left = topLeft.get_u();
  this->top = topLeft.get_v();
  this->width = width;
  this->height = height;
};

/*!
 
  Constructs a rectangle with \e topLeft the top-left corner location
  and \e bottomRight the bottom-right corner.

*/
vpRect::vpRect(const vpImagePoint &topLeft, const vpImagePoint &bottomRight)
{
  this->left = topLeft.get_u();
  this->top = topLeft.get_v();
  
  setBottom( bottomRight.get_v() );
  setRight( bottomRight.get_u() );
};

/*!
 
  Constructs a rectangle that is a copy of \e r.

*/
vpRect::vpRect(const vpRect& r)
{
  this->left   = r.left;
  this->top    = r.top;
  this->width  = r.width;
  this->height = r.height;
};

/*!

  Assign \e r to this rectangle and return a reference to this rectangle.
*/  
vpRect &vpRect::operator=(const vpRect& r) 
{
  this->left   = r.left;
  this->top    = r.top;
  this->width  = r.width;
  this->height = r.height;
  return *this;
};
