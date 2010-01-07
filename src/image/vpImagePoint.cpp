/****************************************************************************
 *
 * $Id: vpImagePoint.cpp 2319 2009-10-22 13:45:50Z nmelchio $
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
 * 2D point useful for image processing
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpConfig.h>
#include <visp/vpImagePoint.h>
#include <visp/vpRect.h>

/*!

  Check if an image point belongs to a rectangle.
  
  \param rect : the rectangle.
  
  \return Returns true if the point belongs to the rectangle.

*/
bool vpImagePoint::inRectangle( const vpRect &rect )
{
  return ( this->i <= rect.getBottom() && 
	   this->i >= rect.getTop() &&
	   this->j <= rect.getRight() &&
	   this->j >= rect.getLeft());
}
