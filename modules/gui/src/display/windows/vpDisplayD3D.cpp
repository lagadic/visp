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
 * windows 32 display using D3D
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

/*! 
\file vpDisplayD3D.cpp
\brief windows 32 display using D3D
*/ 

#include <visp3/core/vpConfig.h>
#if ( defined(_WIN32) & defined(VISP_HAVE_D3D9) )

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpD3DRenderer.h>

/*!
  \brief Basic constructor.
*/
vpDisplayD3D::vpDisplayD3D(): vpDisplayWin32(new vpD3DRenderer()){}

/*!

  \brief Constructor : Initialize a display.

  \param winx, winy The window is set at position x,y (column index, row index).
  \param title  Window's title.

*/
vpDisplayD3D::vpDisplayD3D(int winx, int winy, const char *title)
  : vpDisplayWin32(new vpD3DRenderer())
{
  windowXPosition = winx;
  windowYPosition = winy;

  if (title != NULL)
    title_ = std::string(title);
  else
    title_ = std::string(" ");
}

/*!

\brief Constructor : initialize a display to visualize a RGBa image
(32 bits).

\param I : Image to be displayed (note that image has to be initialized).
\param winx, winy : The window is set at position x,y (column index, row index).
\param title : Window's title.

*/
vpDisplayD3D::vpDisplayD3D(vpImage<vpRGBa> &I,
			   int winx, int winy,
         const char *title)
  : vpDisplayWin32(new vpD3DRenderer())
{
  init(I,winx,winy,title);
}

/*!

\brief Constructor : initialize a display to visualize a grayscale image
(8 bits).

\param I  Image to be displayed (note that image has to be initialized).
\param winx, winy The window is set at position x,y (column index, row index).
\param title  Window's title.

*/
vpDisplayD3D::vpDisplayD3D(vpImage<unsigned char> &I,
			   int winx, int winy,
         const char *title)
  : vpDisplayWin32(new vpD3DRenderer())
{
  init(I,winx,winy,title);
}

/*!
  \brief Basic destructor.
*/
vpDisplayD3D::~vpDisplayD3D(){}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDisplayD3D.cpp.o) has no symbols
void dummy_vpDisplayD3D() {};
#endif

