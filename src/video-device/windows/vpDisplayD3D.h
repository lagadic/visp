/****************************************************************************
 *
 * $Id$
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
 * Windows 32 display using D3D
 *
 * Authors:
 * Bruno Renier
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_D3D9) ) 

#ifndef VPDISPLAYD3D_HH
#define VPDISPLAYD3D_HH


#include <visp/vpDisplayWin32.h>

/*!
  \class vpDisplayD3D

  \ingroup ImageGUI

  \brief Display for windows using Direct3D.

  Direct3D is part of the DirectX API available under Windows
  operating systems.

  \warning Requires DirectX9 SDK to compile and DirectX9 DLLs to run.

  The example below shows how to display an image with this video device.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayD3D.h>

int main()
{
#if defined(VISP_HAVE_D3D9)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
  vpImageIo::readPGM(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");

  vpDisplayD3D d;

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My Direct 3D display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(10);
  topLeftCorner.set_j(20);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for a click in the display window
  vpDisplay::getClick(I);
#endif
}
  \endcode
*/
class VISP_EXPORT vpDisplayD3D : public vpDisplayWin32
{
public:
  vpDisplayD3D();
  
  vpDisplayD3D(vpImage<vpRGBa> &I,
		    int winx=-1, int winy=-1,
		    const char *_title=NULL);
  
  vpDisplayD3D(vpImage<unsigned char> &I,
		    int winx=-1, int winy=-1,
		    const char *_title=NULL);
  
  virtual ~vpDisplayD3D();
  
};
#endif
#endif
