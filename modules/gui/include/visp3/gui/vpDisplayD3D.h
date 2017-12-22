/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * Windows 32 display using D3D
 *
 * Authors:
 * Bruno Renier
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#if (defined(VISP_HAVE_D3D9))

#ifndef VPDISPLAYD3D_HH
#define VPDISPLAYD3D_HH

#include <visp3/gui/vpDisplayWin32.h>

/*!
  \class vpDisplayD3D

  \ingroup group_gui_display

  \brief Display for windows using Direct3D 3rd party. Thus to enable this
class Direct3D should be installed. Installation instructions are provided
here https://visp.inria.fr/3rd_d3d/

  Direct3D is part of the DirectX API available under Windows
  operating systems.

  \warning Requires DirectX9 SDK to compile and DirectX9 DLLs to run.

  The example below shows how to display an image with this video device.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_D3D9)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
  vpImageIo::read(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");

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
  vpDisplayD3D(int winx, int winy, const std::string &title = "");
  vpDisplayD3D(vpImage<unsigned char> &I, vpScaleType type);
  vpDisplayD3D(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);
  vpDisplayD3D(vpImage<vpRGBa> &I, vpScaleType type);
  vpDisplayD3D(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);

  virtual ~vpDisplayD3D();
};
#endif
#endif
