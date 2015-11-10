/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpConfig.h>

#if ( defined(VISP_HAVE_GDI) )

#include <visp3/gui/vpDisplayGDI.h>

//A vpDisplayGDI is just a vpDisplayWin32 which uses a vpGDIRenderer to do the drawing.

/*!
  \brief Basic constructor.
*/
vpDisplayGDI::vpDisplayGDI(): vpDisplayWin32(new vpGDIRenderer()){}

/*!

  \brief Constructor : Initialize a display.

  \param winx, winy The window is set at position x,y (column index, row index).
  \param title  Window's title.

*/
vpDisplayGDI::vpDisplayGDI(int winx, int winy, const char *title)
  : vpDisplayWin32(new vpGDIRenderer())
{
  windowXPosition = winx;
  windowYPosition = winy;

  if (title != NULL)
    title_ = std::string(title);
  else
    title_ = std::string(" ");
}

/*!

  \brief Constructor : Initialize a display to visualize a RGBa image
  (32 bits).

  \param I : image to be displayed (note that image has to be initialized).
  \param winx, winy The window is set at position x,y (column index, row index).
  \param title  Window's title.

*/
vpDisplayGDI::vpDisplayGDI(vpImage<vpRGBa> &I,
			   int winx, int winy,
         const char *title)
  : vpDisplayWin32(new vpGDIRenderer())
{
  init(I,winx,winy,title);
}

/*!

  \brief Constructor : Initialize a display to visualize a grayscale image
  (8 bits).

  \param I Image to be displayed (note that image has to be initialized).
  \param winx, winy The window is set at position x,y (column index, row index).
  \param title  Window's title.

*/
vpDisplayGDI::vpDisplayGDI(vpImage<unsigned char> &I,
			   int winx, int winy,
         const char *title)
  : vpDisplayWin32(new vpGDIRenderer())
{
  init(I,winx,winy,title);
}

/*!
  \brief Basic destructor.
*/
vpDisplayGDI::~vpDisplayGDI(){}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDisplayGDI.cpp.o) has no symbols
void dummy_vpDisplayGDI() {};
#endif
