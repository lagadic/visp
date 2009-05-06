/****************************************************************************
 *
 * $Id$
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
 * This file is part of the ViSP toolkit
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
