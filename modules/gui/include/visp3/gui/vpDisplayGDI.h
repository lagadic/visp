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
 * Windows 32 display using GDI
 *
 * Authors:
 * Bruno Renier
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>

#if (defined(VISP_HAVE_GDI))

#ifndef vpDisplayGDI_HH
#define vpDisplayGDI_HH

#include <visp3/gui/vpDisplayWin32.h>

/*!
  \class vpDisplayGDI

  \ingroup group_gui_display

  \brief Display for windows using GDI (available on any windows 32 platform).

  GDI stands for Graphics Device Interface and is a core component of
Microsoft Windows operating systems used for displaying graphics in a window.

  The example below shows how to display an image with this video device.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_GDI)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
#ifdef _WIN32
  vpImageIo::read(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");
#else
  vpImageIo::read(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
#endif

  vpDisplayGDI d;

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My GDI display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(50);
  topLeftCorner.set_j(10);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl;
  char key[10];
  bool ret;
  for (int i=0; i< 200; i++) {
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    if (ret)
      std::cout << "keyboard event: key: " << "\"" << key << "\"" << std::endl;
    vpTime::wait(40);
  }

  // Get a blocking keyboard event
  std::cout << "Wait for a keyboard event..." << std::endl;
  ret = vpDisplay::getKeyboardEvent(I, key, true);
  std::cout << "keyboard event: " << ret << std::endl;
  if (ret)
    std::cout << "key: " << "\"" << key << "\"" << std::endl;

  // Wait for a click in the display window
  std::cout << "Wait for a button click..." << std::endl;
  vpDisplay::getClick(I);
#endif
}
  \endcode
*/
class VISP_EXPORT vpDisplayGDI : public vpDisplayWin32
{
public:
  vpDisplayGDI();
  vpDisplayGDI(int winx, int winy, const std::string &title = "");
  vpDisplayGDI(vpImage<unsigned char> &I, vpScaleType type);
  vpDisplayGDI(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);
  vpDisplayGDI(vpImage<vpRGBa> &I, vpScaleType type);
  vpDisplayGDI(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);

  virtual ~vpDisplayGDI();
};

#endif
#endif
