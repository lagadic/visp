/****************************************************************************
 *
 * $Id: vpDisplayD3D.cpp,v 1.1 2006-08-24 08:29:52 brenier Exp $
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
 * windows 32 display using D3D
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#if ( defined(WIN32) & defined(VISP_HAVE_D3D9) ) 

#include <visp/vpDisplayD3D.h>
#include <visp/vpD3DRenderer.h>

/*!
  \brief Basic constructor.
*/
vpDisplayD3D::vpDisplayD3D(): vpDisplayWin32(new vpD3DRenderer()){}

/*!

  \brief Constructor : initialize a display to visualize a RGBa image
  (32 bits).

  \param I  Image to be displayed (note that image has to be initialized).
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title Window's title.

*/
vpDisplayD3D::vpDisplayD3D(vpImage<vpRGBa> &I,
		  int winx, int winy,
		  char *_title)
  : vpDisplayWin32(new vpD3DRenderer())
{
  init(I,winx,winy,_title);
}

/*!

  \brief Constructor : initialize a display to visualize a grayscale image
  (8 bits).

  \param I  Image to be displayed (note that image has to be initialized).
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  Window's title.

*/
vpDisplayD3D::vpDisplayD3D(vpImage<unsigned char> &I,
		  int winx, int winy,
		  char *_title)
  : vpDisplayWin32(new vpD3DRenderer())
{
  init(I,winx,winy,_title);
}

/*!
  \brief Basic destructor.
*/
vpDisplayD3D::~vpDisplayD3D(){}

#endif

