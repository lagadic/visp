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
 * GDI based Display for windows 32.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/
/*!
\file vpDisplayGDI.cpp
\brief GDI based Display for windows 32.
*/

#include <visp/vpConfig.h>

#if ( defined(VISP_HAVE_GDI) )

#include <visp/vpDisplayGDI.h>

//A vpDisplayGDI is just a vpDisplayWin32 which uses a vpGDIRenderer to do the drawing.

/*!
  \brief Basic constructor.
*/
vpDisplayGDI::vpDisplayGDI(): vpDisplayWin32(new vpGDIRenderer()){}

/*!

  \brief Constructor : Initialize a display to visualize a RGBa image
  (32 bits).

  \param I : image to be displayed (note that image has to be initialized).
  \param winx, winy The window is set at position x,y (column index, row index).
  \param _title  Window's title.

*/
vpDisplayGDI::vpDisplayGDI(vpImage<vpRGBa> &I,
			   int winx, int winy,
			   const char *_title)
  : vpDisplayWin32(new vpGDIRenderer())
{
  init(I,winx,winy,_title);
}

/*!

  \brief Constructor : Initialize a display to visualize a grayscale image
  (8 bits).

  \param I Image to be displayed (note that image has to be initialized).
  \param winx, winy The window is set at position x,y (column index, row index).
  \param _title  Window's title.

*/
vpDisplayGDI::vpDisplayGDI(vpImage<unsigned char> &I,
			   int winx, int winy,
			   const char *_title)
  : vpDisplayWin32(new vpGDIRenderer())
{
  init(I,winx,winy,_title);
}

/*!
  \brief Basic destructor.
*/
vpDisplayGDI::~vpDisplayGDI(){}


#endif
