/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Image display.
 *
 * Authors:
 * Fabien Spindler
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpDisplayX.cpp
  \brief Define the X11 console to display images.
*/

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_X11

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>

// Display stuff
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayX.h>

// debug / exception
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplayException.h>

// math
#include <visp3/core/vpMath.h>

/*!

  Constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is downscaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is downscaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is downscaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is downscaled by 5 along the lines
  and the columns.

*/
vpDisplayX::vpDisplayX(vpImage<unsigned char> &I, vpScaleType scaleType)
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
  setScale(scaleType, I.getWidth(), I.getHeight());

  init(I);
}

/*!

  Constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is downscaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is downscaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is downscaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is downscaled by 5 along the lines
  and the columns.

*/
vpDisplayX::vpDisplayX(vpImage<unsigned char> &I, int x, int y, const std::string &title, vpScaleType scaleType)
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I, x, y, title);
}

/*!
  Constructor : initialize a display to visualize a RGBa image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized).
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is downscaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is downscaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is downscaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is downscaled by 5 along the lines
  and the columns.
*/
vpDisplayX::vpDisplayX(vpImage<vpRGBa> &I, vpScaleType scaleType)
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I);
}

/*!
  Constructor : initialize a display to visualize a RGBa image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized).
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
  \param scaleType : If this parameter is set to:
  - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image
    is fully displayed in the screen;
  - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the
  same than the image size.
  - vpDisplay::SCALE_2, the display size is downscaled by 2 along the lines
  and the columns.
  - vpDisplay::SCALE_3, the display size is downscaled by 3 along the lines
  and the columns.
  - vpDisplay::SCALE_4, the display size is downscaled by 4 along the lines
  and the columns.
  - vpDisplay::SCALE_5, the display size is downscaled by 5 along the lines
  and the columns.
*/
vpDisplayX::vpDisplayX(vpImage<vpRGBa> &I, int x, int y, const std::string &title, vpScaleType scaleType)
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I, x, y, title);
}

/*!

  Constructor that just initialize the display position in the screen
  and the display title.

  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

  To initialize the display size, you need to call init().

  \code
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
  vpDisplayX d(100, 200, "My display");
  vpImage<unsigned char> I(240, 384);
  d.init(I);
}
  \endcode
*/
vpDisplayX::vpDisplayX(int x, int y, const std::string &title)
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
  m_windowXPosition = x;
  m_windowYPosition = y;

  m_title = title;
}

/*!
  Basic constructor.

  To initialize the window position, title and size you may call
  init(vpImage<unsigned char> &, int, int, const std::string &) or
  init(vpImage<vpRGBa> &, int, int, const std::string &).

  \code
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
  vpDisplayX d;
  vpImage<unsigned char> I(240, 384);
  d.init(I, 100, 200, "My display");
}
  \endcode
*/
vpDisplayX::vpDisplayX()
  : display(NULL), window(), Ximage(NULL), lut(), context(), screen(0), event(), pixmap(), x_color(NULL),
    screen_depth(8), xcolor(), values(), ximage_data_init(false), RMask(0), GMask(0), BMask(0), RShift(0), GShift(0),
    BShift(0)
{
}

/*!
  Destructor.
*/
vpDisplayX::~vpDisplayX() { closeDisplay(); }

/*!
  Initialize the display (size, position and title) of a gray level image.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayX::init(vpImage<unsigned char> &I, int x, int y, const std::string &title)
{
  setScale(m_scaleType, I.getWidth(), I.getHeight());

  if (x_color == NULL) {
    // id_unknown = number of predefined colors
    x_color = new unsigned long[vpColor::id_unknown];
  }

  XSizeHints hints;
  if (x != -1)
    m_windowXPosition = x;
  if (y != -1)
    m_windowYPosition = y;

  if (!title.empty())
    m_title = title;

  // Positionnement de la fenetre dans l'ecran.
  if ((m_windowXPosition < 0) || (m_windowYPosition < 0)) {
    hints.flags = 0;
  } else {
    hints.flags = USPosition;
    hints.x = m_windowXPosition;
    hints.y = m_windowYPosition;
  }

  // setup X11 --------------------------------------------------
  m_width = I.getWidth() / m_scale;
  m_height = I.getHeight() / m_scale;
  display = XOpenDisplay(NULL);
  if (display == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError, "Can't connect display on server."));
  }

  screen = DefaultScreen(display);
  lut = DefaultColormap(display, screen);
  screen_depth = (unsigned int)DefaultDepth(display, screen);

  if ((window = XCreateSimpleWindow(display, RootWindow(display, screen), m_windowXPosition, m_windowYPosition, m_width,
                                    m_height, 1, BlackPixel(display, screen), WhitePixel(display, screen))) == 0) {
    vpERROR_TRACE("Can't create window.");
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError, "Can't create window."));
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    lut = XCreateColormap(display, window, DefaultVisual(display, screen), AllocAll);
    xcolor.flags = DoRed | DoGreen | DoBlue;

    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pixel = i;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor(display, lut, &xcolor);
    }

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);
  }

  else if (screen_depth == 16) {
    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if (XAllocColor(display, lut, &xcolor) == 0) {
        vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
        throw(vpDisplayException(vpDisplayException::colorAllocError, "Can't allocate 256 colors."));
      }
      colortable[i] = xcolor.pixel;
    }

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);

    Visual *visual = DefaultVisual(display, screen);
    RMask = visual->red_mask;
    GMask = visual->green_mask;
    BMask = visual->blue_mask;

    RShift = 15 - getMsb(RMask); /* these are right-shifts */
    GShift = 15 - getMsb(GMask);
    BShift = 15 - getMsb(BMask);
  }

  //
  // Create colors for overlay
  //
  switch (screen_depth) {
  case 8:
    // Color BLACK and WHITE are set properly by default.

    // Color LIGHT GRAY.
    x_color[vpColor::id_lightGray] = 254;
    xcolor.pixel = x_color[vpColor::id_lightGray];
    xcolor.red = 256 * 192;
    xcolor.green = 256 * 192;
    xcolor.blue = 256 * 192;
    XStoreColor(display, lut, &xcolor);

    // Color GRAY.
    x_color[vpColor::id_gray] = 253;
    xcolor.pixel = x_color[vpColor::id_gray];
    xcolor.red = 256 * 128;
    xcolor.green = 256 * 128;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GRAY.
    x_color[vpColor::id_darkGray] = 252;
    xcolor.pixel = x_color[vpColor::id_darkGray];
    xcolor.red = 256 * 64;
    xcolor.green = 256 * 64;
    xcolor.blue = 256 * 64;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT RED.
    x_color[vpColor::id_lightRed] = 251;
    xcolor.pixel = x_color[vpColor::id_lightRed];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 140;
    XStoreColor(display, lut, &xcolor);

    // Color RED.
    x_color[vpColor::id_red] = 250;
    xcolor.pixel = x_color[vpColor::id_red];
    xcolor.red = 256 * 255;
    xcolor.green = 0;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color DARK RED.
    x_color[vpColor::id_darkRed] = 249;
    xcolor.pixel = x_color[vpColor::id_darkRed];
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT GREEN.
    x_color[vpColor::id_lightGreen] = 248;
    xcolor.pixel = x_color[vpColor::id_lightGreen];
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 140;
    XStoreColor(display, lut, &xcolor);

    // Color GREEN.
    x_color[vpColor::id_green] = 247;
    xcolor.pixel = x_color[vpColor::id_green];
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GREEN.
    x_color[vpColor::id_darkGreen] = 246;
    xcolor.pixel = x_color[vpColor::id_darkGreen];
    xcolor.red = 0;
    xcolor.green = 256 * 128;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT BLUE.
    x_color[vpColor::id_lightBlue] = 245;
    xcolor.pixel = x_color[vpColor::id_lightBlue];
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color BLUE.
    x_color[vpColor::id_blue] = 244;
    xcolor.pixel = x_color[vpColor::id_blue];
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color DARK BLUE.
    x_color[vpColor::id_darkBlue] = 243;
    xcolor.pixel = x_color[vpColor::id_darkBlue];
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    // Color YELLOW.
    x_color[vpColor::id_yellow] = 242;
    xcolor.pixel = x_color[vpColor::id_yellow];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color ORANGE.
    x_color[vpColor::id_orange] = 241;
    xcolor.pixel = x_color[vpColor::id_orange];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 165;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color CYAN.
    x_color[vpColor::id_cyan] = 240;
    xcolor.pixel = x_color[vpColor::id_cyan];
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color PURPLE.
    x_color[vpColor::id_purple] = 239;
    xcolor.pixel = x_color[vpColor::id_purple];
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    break;

  case 16:
  case 24: {
    xcolor.flags = DoRed | DoGreen | DoBlue;

    // Couleur BLACK.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_black] = xcolor.pixel;

    // Couleur WHITE.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_white] = xcolor.pixel;

    // Couleur LIGHT GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 192;
    xcolor.green = 256 * 192;
    xcolor.blue = 256 * 192;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGray] = xcolor.pixel;

    // Couleur GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 256 * 128;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_gray] = xcolor.pixel;

    // Couleur DARK GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 64;
    xcolor.green = 256 * 64;
    xcolor.blue = 256 * 64;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGray] = xcolor.pixel;

    // Couleur LIGHT RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 140;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightRed] = xcolor.pixel;

    // Couleur RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_red] = xcolor.pixel;

    // Couleur DARK RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkRed] = xcolor.pixel;

    // Couleur LIGHT GREEN.
    xcolor.pad = 0;
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 140;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGreen] = xcolor.pixel;

    // Couleur GREEN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_green] = xcolor.pixel;

    // Couleur DARK GREEN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 128;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGreen] = xcolor.pixel;

    // Couleur LIGHT Blue.
    xcolor.pad = 0;
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightBlue] = xcolor.pixel;

    // Couleur BLUE.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_blue] = xcolor.pixel;

    // Couleur DARK BLUE.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkBlue] = xcolor.pixel;

    // Couleur YELLOW.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_yellow] = xcolor.pixel;

    // Couleur ORANGE.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 165;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_orange] = xcolor.pixel;

    // Couleur CYAN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_cyan] = xcolor.pixel;

    // Couleur PURPLE.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_purple] = xcolor.pixel;
    break;
  }
  }

  XSetStandardProperties(display, window, this->m_title.c_str(), this->m_title.c_str(), None, 0, 0, &hints);
  XMapWindow(display, window);
  // Selection des evenements.
  XSelectInput(display, window,
               ExposureMask | ButtonPressMask | ButtonReleaseMask | KeyPressMask | KeyReleaseMask |
                   StructureNotifyMask | PointerMotionMask);

  // graphic context creation
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window, GCPlaneMask | GCFillStyle | GCForeground | GCBackground, &values);

  if (context == NULL) {
    vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError, "Can't create graphics context"));
  }

  // Pixmap creation.
  pixmap = XCreatePixmap(display, window, m_width, m_height, screen_depth);

  // Hangs when forward X11 is used to send the display to an other computer
  //  do
  //    XNextEvent ( display, &event );
  //  while ( event.xany.type != Expose );

  {
    Ximage = XCreateImage(display, DefaultVisual(display, screen), screen_depth, ZPixmap, 0, NULL, m_width, m_height,
                          XBitmapPad(display), 0);

    Ximage->data = (char *)malloc(m_height * (unsigned int)Ximage->bytes_per_line);
    ximage_data_init = true;
  }
  m_displayHasBeenInitialized = true;

  XStoreName(display, window, m_title.c_str());

  XSync(display, 1);

  I.display = this;
}

/*!
  Initialize the display (size, position and title) of a color
  image in RGBa format.

  \param I : Image to be displayed (note that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayX::init(vpImage<vpRGBa> &I, int x, int y, const std::string &title)
{
  setScale(m_scaleType, I.getWidth(), I.getHeight());

  XSizeHints hints;
  if (x != -1)
    m_windowXPosition = x;
  if (y != -1)
    m_windowYPosition = y;

  if (x_color == NULL) {
    // id_unknown = number of predefined colors
    x_color = new unsigned long[vpColor::id_unknown];
  }

  if (!title.empty())
    m_title = title;

  // Positionnement de la fenetre dans l'ecran.
  if ((m_windowXPosition < 0) || (m_windowYPosition < 0)) {
    hints.flags = 0;
  } else {
    hints.flags = USPosition;
    hints.x = m_windowXPosition;
    hints.y = m_windowYPosition;
  }

  // setup X11 --------------------------------------------------
  m_width = I.getWidth() / m_scale;
  m_height = I.getHeight() / m_scale;

  if ((display = XOpenDisplay(NULL)) == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError, "Can't connect display on server."));
  }

  screen = DefaultScreen(display);
  lut = DefaultColormap(display, screen);
  screen_depth = (unsigned int)DefaultDepth(display, screen);

  vpDEBUG_TRACE(1, "Screen depth: %d\n", screen_depth);

  if ((window = XCreateSimpleWindow(display, RootWindow(display, screen), m_windowXPosition, m_windowYPosition, m_width,
                                    m_height, 1, BlackPixel(display, screen), WhitePixel(display, screen))) == 0) {
    vpERROR_TRACE("Can't create window.");
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError, "Can't create window."));
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    lut = XCreateColormap(display, window, DefaultVisual(display, screen), AllocAll);
    xcolor.flags = DoRed | DoGreen | DoBlue;

    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pixel = i;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor(display, lut, &xcolor);
    }

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);
  }

  else if (screen_depth == 16) {
    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if (XAllocColor(display, lut, &xcolor) == 0) {
        vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
        throw(vpDisplayException(vpDisplayException::colorAllocError, "Can't allocate 256 colors."));
      }
      colortable[i] = xcolor.pixel;
    }

    Visual *visual = DefaultVisual(display, screen);
    RMask = visual->red_mask;
    GMask = visual->green_mask;
    BMask = visual->blue_mask;

    RShift = 15 - getMsb(RMask); /* these are right-shifts */
    GShift = 15 - getMsb(GMask);
    BShift = 15 - getMsb(BMask);

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);
  }

  //
  // Create colors for overlay
  //
  switch (screen_depth) {

  case 8:
    // Color BLACK and WHITE are set properly.

    // Color LIGHT GRAY.
    x_color[vpColor::id_lightGray] = 254;
    xcolor.pixel = x_color[vpColor::id_lightGray];
    xcolor.red = 256 * 192;
    xcolor.green = 256 * 192;
    xcolor.blue = 256 * 192;
    XStoreColor(display, lut, &xcolor);

    // Color GRAY.
    x_color[vpColor::id_gray] = 253;
    xcolor.pixel = x_color[vpColor::id_gray];
    xcolor.red = 256 * 128;
    xcolor.green = 256 * 128;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GRAY.
    x_color[vpColor::id_darkGray] = 252;
    xcolor.pixel = x_color[vpColor::id_darkGray];
    xcolor.red = 256 * 64;
    xcolor.green = 256 * 64;
    xcolor.blue = 256 * 64;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT RED.
    x_color[vpColor::id_lightRed] = 251;
    xcolor.pixel = x_color[vpColor::id_lightRed];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 140;
    XStoreColor(display, lut, &xcolor);

    // Color RED.
    x_color[vpColor::id_red] = 250;
    xcolor.pixel = x_color[vpColor::id_red];
    xcolor.red = 256 * 255;
    xcolor.green = 0;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color DARK RED.
    x_color[vpColor::id_darkRed] = 249;
    xcolor.pixel = x_color[vpColor::id_darkRed];
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT GREEN.
    x_color[vpColor::id_lightGreen] = 248;
    xcolor.pixel = x_color[vpColor::id_lightGreen];
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 140;
    XStoreColor(display, lut, &xcolor);

    // Color GREEN.
    x_color[vpColor::id_green] = 247;
    xcolor.pixel = x_color[vpColor::id_green];
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GREEN.
    x_color[vpColor::id_darkGreen] = 246;
    xcolor.pixel = x_color[vpColor::id_darkGreen];
    xcolor.red = 0;
    xcolor.green = 256 * 128;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT BLUE.
    x_color[vpColor::id_lightBlue] = 245;
    xcolor.pixel = x_color[vpColor::id_lightBlue];
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color BLUE.
    x_color[vpColor::id_blue] = 244;
    xcolor.pixel = x_color[vpColor::id_blue];
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color DARK BLUE.
    x_color[vpColor::id_darkBlue] = 243;
    xcolor.pixel = x_color[vpColor::id_darkBlue];
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    // Color YELLOW.
    x_color[vpColor::id_yellow] = 242;
    xcolor.pixel = x_color[vpColor::id_yellow];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color ORANGE.
    x_color[vpColor::id_orange] = 241;
    xcolor.pixel = x_color[vpColor::id_orange];
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 165;
    xcolor.blue = 0;
    XStoreColor(display, lut, &xcolor);

    // Color CYAN.
    x_color[vpColor::id_cyan] = 240;
    xcolor.pixel = x_color[vpColor::id_cyan];
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XStoreColor(display, lut, &xcolor);

    // Color PURPLE.
    x_color[vpColor::id_purple] = 239;
    xcolor.pixel = x_color[vpColor::id_purple];
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XStoreColor(display, lut, &xcolor);

    break;

  case 16:
  case 24: {
    xcolor.flags = DoRed | DoGreen | DoBlue;

    // Couleur BLACK.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_black] = xcolor.pixel;

    // Couleur WHITE.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_white] = xcolor.pixel;

    // Couleur LIGHT GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 192;
    xcolor.green = 256 * 192;
    xcolor.blue = 256 * 192;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGray] = xcolor.pixel;

    // Couleur GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 256 * 128;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_gray] = xcolor.pixel;

    // Couleur DARK GRAY.
    xcolor.pad = 0;
    xcolor.red = 256 * 64;
    xcolor.green = 256 * 64;
    xcolor.blue = 256 * 64;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGray] = xcolor.pixel;

    // Couleur LIGHT RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 140;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightRed] = xcolor.pixel;

    // Couleur RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_red] = xcolor.pixel;

    // Couleur DARK RED.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkRed] = xcolor.pixel;

    // Couleur LIGHT GREEN.
    xcolor.pad = 0;
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 140;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGreen] = xcolor.pixel;

    // Couleur GREEN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_green] = xcolor.pixel;

    // Couleur DARK GREEN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 128;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGreen] = xcolor.pixel;

    // Couleur LIGHT Blue.
    xcolor.pad = 0;
    xcolor.red = 256 * 140;
    xcolor.green = 256 * 140;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightBlue] = xcolor.pixel;

    // Couleur BLUE.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_blue] = xcolor.pixel;

    // Couleur DARK BLUE.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkBlue] = xcolor.pixel;

    // Couleur YELLOW.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 255;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_yellow] = xcolor.pixel;

    // Couleur ORANGE.
    xcolor.pad = 0;
    xcolor.red = 256 * 255;
    xcolor.green = 256 * 165;
    xcolor.blue = 0;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_orange] = xcolor.pixel;

    // Couleur CYAN.
    xcolor.pad = 0;
    xcolor.red = 0;
    xcolor.green = 256 * 255;
    xcolor.blue = 256 * 255;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_cyan] = xcolor.pixel;

    // Couleur PURPLE.
    xcolor.pad = 0;
    xcolor.red = 256 * 128;
    xcolor.green = 0;
    xcolor.blue = 256 * 128;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_purple] = xcolor.pixel;
    break;
  }
  }

  XSetStandardProperties(display, window, this->m_title.c_str(), this->m_title.c_str(), None, 0, 0, &hints);
  XMapWindow(display, window);
  // Selection des evenements.
  XSelectInput(display, window,
               ExposureMask | ButtonPressMask | ButtonReleaseMask | KeyPressMask | KeyReleaseMask |
                   StructureNotifyMask | PointerMotionMask);

  // Creation du contexte graphique
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window, GCPlaneMask | GCFillStyle | GCForeground | GCBackground, &values);

  if (context == NULL) {
    vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError, "Can't create graphics context"));
  }

  // Pixmap creation.
  pixmap = XCreatePixmap(display, window, m_width, m_height, screen_depth);

  // Hangs when forward X11 is used to send the display to an other computer
  //  do
  //    XNextEvent ( display, &event );
  //  while ( event.xany.type != Expose );

  {
    Ximage = XCreateImage(display, DefaultVisual(display, screen), screen_depth, ZPixmap, 0, NULL, m_width, m_height,
                          XBitmapPad(display), 0);

    Ximage->data = (char *)malloc(m_height * (unsigned int)Ximage->bytes_per_line);
    ximage_data_init = true;
  }
  m_displayHasBeenInitialized = true;

  XSync(display, true);

  XStoreName(display, window, m_title.c_str());

  I.display = this;
}

/*!
  Initialize the display size, position and title.

  \param w, h : Width and height of the window.
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
*/
void vpDisplayX::init(unsigned int w, unsigned int h, int x, int y, const std::string &title)
{
  setScale(m_scaleType, w, h);

  if (x_color == NULL) {
    // id_unknown = number of predefined colors
    x_color = new unsigned long[vpColor::id_unknown];
  }
  /* setup X11 -------------------------------------------------------------
   */
  this->m_width = w / m_scale;
  this->m_height = h / m_scale;

  XSizeHints hints;

  if (x != -1)
    m_windowXPosition = x;
  if (y != -1)
    m_windowYPosition = y;
  // Positionnement de la fenetre dans l'ecran.
  if ((m_windowXPosition < 0) || (m_windowYPosition < 0)) {
    hints.flags = 0;
  } else {
    hints.flags = USPosition;
    hints.x = m_windowXPosition;
    hints.y = m_windowYPosition;
  }

  m_title = title;

  if ((display = XOpenDisplay(NULL)) == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError, "Can't connect display on server."));
  }

  screen = DefaultScreen(display);
  lut = DefaultColormap(display, screen);
  screen_depth = (unsigned int)DefaultDepth(display, screen);

  vpTRACE("Screen depth: %d\n", screen_depth);

  if ((window = XCreateSimpleWindow(display, RootWindow(display, screen), m_windowXPosition, m_windowYPosition, m_width,
                                    m_height, 1, BlackPixel(display, screen), WhitePixel(display, screen))) == 0) {
    vpERROR_TRACE("Can't create window.");
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError, "Can't create window."));
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    lut = XCreateColormap(display, window, DefaultVisual(display, screen), AllocAll);
    xcolor.flags = DoRed | DoGreen | DoBlue;

    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pixel = i;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor(display, lut, &xcolor);
    }

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);
  }

  else if (screen_depth == 16) {
    for (unsigned int i = 0; i < 256; i++) {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if (XAllocColor(display, lut, &xcolor) == 0) {
        vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
        throw(vpDisplayException(vpDisplayException::colorAllocError, "Can't allocate 256 colors."));
      }
      colortable[i] = xcolor.pixel;
    }

    XSetWindowColormap(display, window, lut);
    XInstallColormap(display, lut);

    Visual *visual = DefaultVisual(display, screen);
    RMask = visual->red_mask;
    GMask = visual->green_mask;
    BMask = visual->blue_mask;

    RShift = 15 - getMsb(RMask); /* these are right-shifts */
    GShift = 15 - getMsb(GMask);
    BShift = 15 - getMsb(BMask);
  }

  vpColor pcolor; // predefined colors

  //
  // Create colors for overlay
  //
  switch (screen_depth) {

  case 8:
    // Color BLACK: default set to 0

    // Color WHITE: default set to 255

    // Color LIGHT GRAY.
    pcolor = vpColor::lightGray;
    xcolor.pixel = 254; // affected to 254
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color GRAY.
    pcolor = vpColor::gray;
    xcolor.pixel = 253; // affected to 253
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GRAY.
    pcolor = vpColor::darkGray;
    xcolor.pixel = 252; // affected to 252
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT RED.
    pcolor = vpColor::lightRed;
    xcolor.pixel = 251; // affected to 251
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color RED.
    pcolor = vpColor::red;
    xcolor.pixel = 250; // affected to 250
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color DARK RED.
    pcolor = vpColor::darkRed;
    xcolor.pixel = 249; // affected to 249
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT GREEN.
    pcolor = vpColor::lightGreen;
    xcolor.pixel = 248; // affected to 248
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color GREEN.
    pcolor = vpColor::green;
    xcolor.pixel = 247; // affected to 247
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color DARK GREEN.
    pcolor = vpColor::darkGreen;
    xcolor.pixel = 246; // affected to 246
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color LIGHT BLUE.
    pcolor = vpColor::lightBlue;
    xcolor.pixel = 245; // affected to 245
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color BLUE.
    pcolor = vpColor::blue;
    xcolor.pixel = 244; // affected to 244
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color DARK BLUE.
    pcolor = vpColor::darkBlue;
    xcolor.pixel = 243; // affected to 243
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color YELLOW.
    pcolor = vpColor::yellow;
    xcolor.pixel = 242; // affected to 242
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color ORANGE.
    pcolor = vpColor::orange;
    xcolor.pixel = 241; // affected to 241
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color CYAN.
    pcolor = vpColor::cyan;
    xcolor.pixel = 240; // affected to 240
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    // Color PURPLE.
    pcolor = vpColor::purple;
    xcolor.pixel = 239; // affected to 239
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XStoreColor(display, lut, &xcolor);

    break;

  case 16:
  case 24: {
    xcolor.flags = DoRed | DoGreen | DoBlue;

    // Couleur BLACK.
    pcolor = vpColor::black;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_black] = xcolor.pixel;

    // Color WHITE.
    pcolor = vpColor::white;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_white] = xcolor.pixel;

    // Color LIGHT GRAY.
    pcolor = vpColor::lightGray;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGray] = xcolor.pixel;

    // Color GRAY.
    pcolor = vpColor::gray;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_gray] = xcolor.pixel;

    // Color DARK GRAY.
    pcolor = vpColor::darkGray;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGray] = xcolor.pixel;

    // Color LIGHT RED.
    pcolor = vpColor::lightRed;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightRed] = xcolor.pixel;

    // Color RED.
    pcolor = vpColor::red;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_red] = xcolor.pixel;

    // Color DARK RED.
    pcolor = vpColor::darkRed;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkRed] = xcolor.pixel;

    // Color LIGHT GREEN.
    pcolor = vpColor::lightGreen;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightGreen] = xcolor.pixel;

    // Color GREEN.
    pcolor = vpColor::green;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_green] = xcolor.pixel;

    // Color DARK GREEN.
    pcolor = vpColor::darkGreen;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkGreen] = xcolor.pixel;

    // Color LIGHT BLUE.
    pcolor = vpColor::lightBlue;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_lightBlue] = xcolor.pixel;

    // Color BLUE.
    pcolor = vpColor::blue;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_blue] = xcolor.pixel;

    // Color DARK BLUE.
    pcolor = vpColor::darkBlue;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_darkBlue] = xcolor.pixel;

    // Color YELLOW.
    pcolor = vpColor::yellow;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_yellow] = xcolor.pixel;

    // Color ORANGE.
    pcolor = vpColor::orange;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_orange] = xcolor.pixel;

    // Color CYAN.
    pcolor = vpColor::cyan;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_cyan] = xcolor.pixel;

    // Color PURPLE.
    pcolor = vpColor::purple;
    xcolor.pad = 0;
    xcolor.red = 256 * pcolor.R;
    xcolor.green = 256 * pcolor.G;
    xcolor.blue = 256 * pcolor.B;
    XAllocColor(display, lut, &xcolor);
    x_color[vpColor::id_purple] = xcolor.pixel;
    break;
  }
  }

  XSetStandardProperties(display, window, this->m_title.c_str(), this->m_title.c_str(), None, 0, 0, &hints);
  XMapWindow(display, window);
  // Selection des evenements.
  XSelectInput(display, window,
               ExposureMask | ButtonPressMask | ButtonReleaseMask | KeyPressMask | KeyReleaseMask |
                   StructureNotifyMask | PointerMotionMask);

  /* Creation du contexte graphique */
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window, GCPlaneMask | GCFillStyle | GCForeground | GCBackground, &values);

  if (context == NULL) {
    vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError, "Can't create graphics context"));
  }

  // Pixmap creation.
  pixmap = XCreatePixmap(display, window, m_width, m_height, screen_depth);

  // Hangs when forward X11 is used to send the display to an other computer
  //  do
  //    XNextEvent ( display, &event );
  //  while ( event.xany.type != Expose );

  {
    Ximage = XCreateImage(display, DefaultVisual(display, screen), screen_depth, ZPixmap, 0, NULL, m_width, m_height,
                          XBitmapPad(display), 0);

    Ximage->data = (char *)malloc(m_height * (unsigned int)Ximage->bytes_per_line);
    ximage_data_init = true;
  }
  m_displayHasBeenInitialized = true;

  XSync(display, true);

  XStoreName(display, window, m_title.c_str());
}

/*!

  Set the font used to display a text in overlay. The display is
  performed using displayCharString().

  \param font : The expected font name. The available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \note Under UNIX, to know all the available fonts, use the
  "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

  \sa displayCharString()
*/
void vpDisplayX::setFont(const std::string &font)
{
  if (m_displayHasBeenInitialized) {
    if (!font.empty()) {
      try {
        Font stringfont;
        stringfont = XLoadFont(display, font.c_str()); //"-adobe-times-bold-r-normal--18*");
        XSetFont(display, context, stringfont);
      } catch (...) {
        throw(vpDisplayException(vpDisplayException::notInitializedError, "Bad font"));
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Set the window title.
  \param title : Window title.
*/
void vpDisplayX::setTitle(const std::string &title)
{
  if (m_displayHasBeenInitialized) {
    m_title = title;
    if (!title.empty())
      XStoreName(display, window, m_title.c_str());
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Set the window position in the screen.

  \param winx, winy : Position of the upper-left window's border in the
  screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplayX::setWindowPosition(int winx, int winy)
{
  if (m_displayHasBeenInitialized) {
    XMoveWindow(display, window, winx, winy);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage(const vpImage<unsigned char> &I)
{
  if (m_displayHasBeenInitialized) {
    switch (screen_depth) {
    case 8: {
      // Correction de l'image de facon a liberer les niveaux de gris
      // ROUGE, VERT, BLEU, JAUNE
      unsigned char nivGrisMax = 255 - vpColor::id_unknown;
      if (m_scale == 1) {
        unsigned char *src_8 = (unsigned char *)I.bitmap;
        unsigned char *dst_8 = (unsigned char *)Ximage->data;
        unsigned int i = 0;
        unsigned int size = m_width * m_height;

        while (i < size) {
          unsigned char nivGris = src_8[i];
          if (nivGris > nivGrisMax)
            dst_8[i] = 255;
          else
            dst_8[i] = nivGris;
          i++;
        }
      } else {
        // Correction de l'image de facon a liberer les niveaux de gris
        // ROUGE, VERT, BLEU, JAUNE
        unsigned char *dst_8 = (unsigned char *)Ximage->data;
        unsigned int k = 0;
        for (unsigned int i = 0; i < m_height; i++) {
          for (unsigned int j = 0; j < m_width; j++) {
            unsigned char nivGris = I[i * m_scale][j * m_scale];
            if (nivGris > nivGrisMax)
              dst_8[k++] = 255;
            else
              dst_8[k++] = nivGris;
          }
        }
      }

      // Affichage de l'image dans la Pixmap.
      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    case 16: {
      unsigned int bytes_per_line = (unsigned int)Ximage->bytes_per_line;
      if (m_scale == 1) {
        for (unsigned int i = 0; i < m_height; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = 0; j < m_width; j++) {
            *(dst_16 + j) = (unsigned short)colortable[I[i][j]];
          }
        }
      } else {
        for (unsigned int i = 0; i < m_height; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = 0; j < m_width; j++) {
            *(dst_16 + j) = (unsigned short)colortable[I[i * m_scale][j * m_scale]];
          }
        }
      }

      // Affichage de l'image dans la Pixmap.
      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }

    case 24:
    default: {
      unsigned char *dst_32 = (unsigned char *)Ximage->data;
      if (m_scale == 1) {
        unsigned int size_ = m_width * m_height;
        unsigned char *bitmap = I.bitmap;
        unsigned char *n = I.bitmap + size_;
        // for (unsigned int i = 0; i < size; i++) // suppression de
        // l'iterateur i
        if (XImageByteOrder(display) == 1) {
          // big endian
          while (bitmap < n) {
            unsigned char val = *(bitmap++);
            *(dst_32++) = vpRGBa::alpha_default;
            *(dst_32++) = val; // Red
            *(dst_32++) = val; // Green
            *(dst_32++) = val; // Blue
          }
        } else {
          // little endian
          while (bitmap < n) {
            unsigned char val = *(bitmap++);
            *(dst_32++) = val; // Blue
            *(dst_32++) = val; // Green
            *(dst_32++) = val; // Red
            *(dst_32++) = vpRGBa::alpha_default;
          }
        }
      } else {
        if (XImageByteOrder(display) == 1) {
          // big endian
          for (unsigned int i = 0; i < m_height; i++) {
            for (unsigned int j = 0; j < m_width; j++) {
              unsigned char val = I[i * m_scale][j * m_scale];
              *(dst_32++) = vpRGBa::alpha_default;
              *(dst_32++) = val; // Red
              *(dst_32++) = val; // Green
              *(dst_32++) = val; // Blue
            }
          }
        } else {
          // little endian
          for (unsigned int i = 0; i < m_height; i++) {
            for (unsigned int j = 0; j < m_width; j++) {
              unsigned char val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val; // Blue
              *(dst_32++) = val; // Green
              *(dst_32++) = val; // Red
              *(dst_32++) = vpRGBa::alpha_default;
            }
          }
        }
      }

      // Affichage de l'image dans la Pixmap.
      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}
/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage(const vpImage<vpRGBa> &I)
{
  if (m_displayHasBeenInitialized) {
    switch (screen_depth) {
    case 16: {
      vpRGBa *bitmap = I.bitmap;
      unsigned int r, g, b;
      unsigned int bytes_per_line = (unsigned int)Ximage->bytes_per_line;

      if (m_scale == 1) {
        for (unsigned int i = 0; i < m_height; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = 0; j < m_width; j++) {
            r = bitmap->R;
            g = bitmap->G;
            b = bitmap->B;
            *(dst_16 + j) =
                (((r << 8) >> RShift) & RMask) | (((g << 8) >> GShift) & GMask) | (((b << 8) >> BShift) & BMask);
            bitmap++;
          }
        }
      } else {
        for (unsigned int i = 0; i < m_height; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = 0; j < m_width; j++) {
            vpRGBa val = I[i * m_scale][j * m_scale];
            r = val.R;
            g = val.G;
            b = val.B;
            *(dst_16 + j) =
                (((r << 8) >> RShift) & RMask) | (((g << 8) >> GShift) & GMask) | (((b << 8) >> BShift) & BMask);
            bitmap++;
          }
        }
      }

      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
      XSetWindowBackgroundPixmap(display, window, pixmap);

      break;
    }
    case 24:
    case 32: {
      /*
       * 32-bit source, 24/32-bit destination
       */
      unsigned char *dst_32 = NULL;
      dst_32 = (unsigned char *)Ximage->data;
      if (m_scale == 1) {
        vpRGBa *bitmap = I.bitmap;
        unsigned int sizeI = m_width * m_height;
        if (XImageByteOrder(display) == 1) {
          // big endian
          for (unsigned int i = 0; i < sizeI; i++) {
            *(dst_32++) = bitmap->A;
            *(dst_32++) = bitmap->R;
            *(dst_32++) = bitmap->G;
            *(dst_32++) = bitmap->B;
            bitmap++;
          }
        } else {
          // little endian
          for (unsigned int i = 0; i < sizeI; i++) {
            *(dst_32++) = bitmap->B;
            *(dst_32++) = bitmap->G;
            *(dst_32++) = bitmap->R;
            *(dst_32++) = bitmap->A;
            bitmap++;
          }
        }
      } else {
        if (XImageByteOrder(display) == 1) {
          // big endian
          for (unsigned int i = 0; i < m_height; i++) {
            for (unsigned int j = 0; j < m_width; j++) {
              vpRGBa val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val.A;
              *(dst_32++) = val.R;
              *(dst_32++) = val.G;
              *(dst_32++) = val.B;
            }
          }
        } else {
          // little endian
          for (unsigned int i = 0; i < m_height; i++) {
            for (unsigned int j = 0; j < m_width; j++) {
              vpRGBa val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val.B;
              *(dst_32++) = val.G;
              *(dst_32++) = val.R;
              *(dst_32++) = val.A;
            }
          }
        }
      }

      // Affichage de l'image dans la Pixmap.
      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    default:
      throw(vpDisplayException(vpDisplayException::depthNotSupportedError,
                               "Unsupported depth (%d bpp) for color display", screen_depth));
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display an image with a reference to the bitmap.

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param bitmap : Pointer to the image bitmap.

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage(const unsigned char *bitmap)
{

  if (m_displayHasBeenInitialized) {
    unsigned char *dst_32 = (unsigned char *)Ximage->data;
    for (unsigned int i = 0; i < m_width * m_height; i++) {
      *(dst_32++) = *bitmap; // red component.
      *(dst_32++) = *bitmap; // green component.
      *(dst_32++) = *bitmap; // blue component.
      *(dst_32++) = *bitmap; // luminance component.
      bitmap++;
    }

    // Affichage de l'image dans la Pixmap.
    XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, m_width, m_height);
    XSetWindowBackgroundPixmap(display, window, pixmap);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a selection of the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.

  \param iP : Top left corner of the region of interest

  \param w, h : Width and height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int w,
                                 const unsigned int h)
{
  if (m_displayHasBeenInitialized) {
    switch (screen_depth) {
    case 8: {
      // Correction de l'image de facon a liberer les niveaux de gris
      // ROUGE, VERT, BLEU, JAUNE
      unsigned char nivGrisMax = 255 - vpColor::id_unknown;
      if (m_scale == 1) {
        unsigned char *src_8 = (unsigned char *)I.bitmap;
        unsigned char *dst_8 = (unsigned char *)Ximage->data;
        unsigned int iwidth = I.getWidth();

        src_8 = src_8 + (int)(iP.get_i() * iwidth + iP.get_j());
        dst_8 = dst_8 + (int)(iP.get_i() * m_width + iP.get_j());

        unsigned int i = 0;
        while (i < h) {
          unsigned int j = 0;
          while (j < w) {
            unsigned char nivGris = *(src_8 + j);
            if (nivGris > nivGrisMax)
              *(dst_8 + j) = 255;
            else
              *(dst_8 + j) = nivGris;
            j++;
          }
          src_8 = src_8 + iwidth;
          dst_8 = dst_8 + m_width;
          i++;
        }

        XPutImage(display, pixmap, context, Ximage, (int)iP.get_u(), (int)iP.get_v(), (int)iP.get_u(), (int)iP.get_v(),
                  w, h);
      } else {
        // Correction de l'image de facon a liberer les niveaux de gris
        // ROUGE, VERT, BLEU, JAUNE
        int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
        int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
        int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
        int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);

        unsigned int i_min_ = (unsigned int)i_min;
        unsigned int i_max_ = (unsigned int)i_max;
        unsigned int j_min_ = (unsigned int)j_min;
        unsigned int j_max_ = (unsigned int)j_max;

        for (unsigned int i = i_min_; i < i_max_; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * m_width;
          for (unsigned int j = j_min_; j < j_max_; j++) {
            unsigned char nivGris = I[i * m_scale][j * m_scale];
            if (nivGris > nivGrisMax)
              dst_8[j] = 255;
            else
              dst_8[j] = nivGris;
          }
        }
        XPutImage(display, pixmap, context, Ximage, j_min, i_min, j_min, i_min, j_max_ - j_min_, i_max_ - i_min_);
      }

      // Affichage de l'image dans la Pixmap.
      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    case 16: {
      unsigned int bytes_per_line = (unsigned int)Ximage->bytes_per_line;
      if (m_scale == 1) {
        for (unsigned int i = (unsigned int)iP.get_i(); i < (unsigned int)(iP.get_i() + h); i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = (unsigned int)iP.get_j(); j < (unsigned int)(iP.get_j() + w); j++) {
            *(dst_16 + j) = (unsigned short)colortable[I[i][j]];
          }
        }

        XPutImage(display, pixmap, context, Ximage, (int)iP.get_u(), (int)iP.get_v(), (int)iP.get_u(), (int)iP.get_v(),
                  w, h);
      } else {
        int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
        int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
        int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
        int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);

        unsigned int i_min_ = (unsigned int)i_min;
        unsigned int i_max_ = (unsigned int)i_max;
        unsigned int j_min_ = (unsigned int)j_min;
        unsigned int j_max_ = (unsigned int)j_max;

        for (unsigned int i = i_min_; i < i_max_; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = j_min_; j < j_max_; j++) {
            *(dst_16 + j) = (unsigned short)colortable[I[i * m_scale][j * m_scale]];
          }
        }

        XPutImage(display, pixmap, context, Ximage, j_min, i_min, j_min, i_min, j_max_ - j_min_, i_max_ - i_min_);
      }

      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }

    case 24:
    default: {
      if (m_scale == 1) {
        unsigned int iwidth = I.getWidth();
        unsigned char *src_8 = I.bitmap + (int)(iP.get_i() * iwidth + iP.get_j());
        unsigned char *dst_32 = (unsigned char *)Ximage->data + (int)(iP.get_i() * 4 * m_width + iP.get_j() * 4);

        if (XImageByteOrder(display) == 1) {
          // big endian
          unsigned int i = 0;
          while (i < h) {
            unsigned int j = 0;
            while (j < w) {
              unsigned char val = *(src_8 + j);
              *(dst_32 + 4 * j) = vpRGBa::alpha_default;
              *(dst_32 + 4 * j + 1) = val;
              *(dst_32 + 4 * j + 2) = val;
              *(dst_32 + 4 * j + 3) = val;
              j++;
            }
            src_8 = src_8 + iwidth;
            dst_32 = dst_32 + 4 * m_width;
            i++;
          }
        } else {
          // little endian
          unsigned int i = 0;
          while (i < h) {
            unsigned int j = 0;
            while (j < w) {
              unsigned char val = *(src_8 + j);
              *(dst_32 + 4 * j) = val;
              *(dst_32 + 4 * j + 1) = val;
              *(dst_32 + 4 * j + 2) = val;
              *(dst_32 + 4 * j + 3) = vpRGBa::alpha_default;
              j++;
            }
            src_8 = src_8 + iwidth;
            dst_32 = dst_32 + 4 * m_width;
            i++;
          }
        }

        XPutImage(display, pixmap, context, Ximage, (int)iP.get_u(), (int)iP.get_v(), (int)iP.get_u(), (int)iP.get_v(),
                  w, h);
      } else {
        int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
        int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
        int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
        int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);

        unsigned int i_min_ = (unsigned int)i_min;
        unsigned int i_max_ = (unsigned int)i_max;
        unsigned int j_min_ = (unsigned int)j_min;
        unsigned int j_max_ = (unsigned int)j_max;

        if (XImageByteOrder(display) == 1) {
          // big endian
          for (unsigned int i = i_min_; i < i_max_; i++) {
            unsigned char *dst_32 = (unsigned char *)Ximage->data + (int)(i * 4 * m_width + j_min_ * 4);
            for (unsigned int j = j_min_; j < j_max_; j++) {
              unsigned char val = I[i * m_scale][j * m_scale];
              *(dst_32++) = vpRGBa::alpha_default;
              *(dst_32++) = val;
              *(dst_32++) = val;
              *(dst_32++) = val;
            }
          }
        } else {
          // little endian
          for (unsigned int i = i_min_; i < i_max_; i++) {
            unsigned char *dst_32 = (unsigned char *)Ximage->data + (int)(i * 4 * m_width + j_min_ * 4);
            for (unsigned int j = j_min_; j < j_max_; j++) {
              unsigned char val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val;
              *(dst_32++) = val;
              *(dst_32++) = val;
              *(dst_32++) = vpRGBa::alpha_default;
            }
          }
        }

        XPutImage(display, pixmap, context, Ximage, j_min, i_min, j_min, i_min, j_max_ - j_min_, i_max_ - i_min_);
      }

      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a selection of the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.

  \param iP : Top left corner of the region of interest

  \param w, h : Width and height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int w,
                                 const unsigned int h)
{
  if (m_displayHasBeenInitialized) {
    switch (screen_depth) {
    case 16: {
      if (m_scale == 1) {
        unsigned int bytes_per_line = (unsigned int)Ximage->bytes_per_line;
        for (unsigned int i = (unsigned int)iP.get_i(); i < (unsigned int)(iP.get_i() + h); i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = (unsigned int)iP.get_j(); j < (unsigned int)(iP.get_j() + w); j++) {
            vpRGBa val = I[i][j];
            unsigned int r = val.R;
            unsigned int g = val.G;
            unsigned int b = val.B;
            *(dst_16 + j) =
                (((r << 8) >> RShift) & RMask) | (((g << 8) >> GShift) & GMask) | (((b << 8) >> BShift) & BMask);
          }
        }
        XPutImage(display, pixmap, context, Ximage, (int)iP.get_u(), (int)iP.get_v(), (int)iP.get_u(), (int)iP.get_v(),
                  w, h);
      } else {
        unsigned int bytes_per_line = (unsigned int)Ximage->bytes_per_line;
        int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
        int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
        int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
        int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);

        unsigned int i_min_ = (unsigned int)i_min;
        unsigned int i_max_ = (unsigned int)i_max;
        unsigned int j_min_ = (unsigned int)j_min;
        unsigned int j_max_ = (unsigned int)j_max;

        for (unsigned int i = i_min_; i < i_max_; i++) {
          unsigned char *dst_8 = (unsigned char *)Ximage->data + i * bytes_per_line;
          unsigned short *dst_16 = (unsigned short *)dst_8;
          for (unsigned int j = j_min_; j < j_max_; j++) {
            vpRGBa val = I[i * m_scale][j * m_scale];
            unsigned int r = val.R;
            unsigned int g = val.G;
            unsigned int b = val.B;
            *(dst_16 + j) =
                (((r << 8) >> RShift) & RMask) | (((g << 8) >> GShift) & GMask) | (((b << 8) >> BShift) & BMask);
          }
        }
        XPutImage(display, pixmap, context, Ximage, j_min, i_min, j_min, i_min, j_max_ - j_min_, i_max_ - i_min_);
      }

      XSetWindowBackgroundPixmap(display, window, pixmap);

      break;
    }
    case 24:
    case 32: {
      /*
       * 32-bit source, 24/32-bit destination
       */

      if (m_scale == 1) {
        unsigned char *dst_32 = (unsigned char *)Ximage->data;
        vpRGBa *src_32 = I.bitmap;

        unsigned int iwidth = I.getWidth();

        src_32 = src_32 + (int)(iP.get_i() * iwidth + iP.get_j());
        dst_32 = dst_32 + (int)(iP.get_i() * 4 * m_width + iP.get_j() * 4);

        unsigned int i = 0;

        if (XImageByteOrder(display) == 1) {
          // big endian
          while (i < h) {
            unsigned int j = 0;
            while (j < w) {
              *(dst_32 + 4 * j) = (src_32 + j)->A;
              *(dst_32 + 4 * j + 1) = (src_32 + j)->R;
              *(dst_32 + 4 * j + 2) = (src_32 + j)->G;
              *(dst_32 + 4 * j + 3) = (src_32 + j)->B;

              j++;
            }
            src_32 = src_32 + iwidth;
            dst_32 = dst_32 + 4 * m_width;
            i++;
          }

        } else {
          // little endian
          while (i < h) {
            unsigned int j = 0;
            while (j < w) {
              *(dst_32 + 4 * j) = (src_32 + j)->B;
              *(dst_32 + 4 * j + 1) = (src_32 + j)->G;
              *(dst_32 + 4 * j + 2) = (src_32 + j)->R;
              *(dst_32 + 4 * j + 3) = (src_32 + j)->A;

              j++;
            }
            src_32 = src_32 + iwidth;
            dst_32 = dst_32 + 4 * m_width;
            i++;
          }
        }

        XPutImage(display, pixmap, context, Ximage, (int)iP.get_u(), (int)iP.get_v(), (int)iP.get_u(), (int)iP.get_v(),
                  w, h);
      } else {
        int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
        int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
        int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
        int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);

        unsigned int i_min_ = (unsigned int)i_min;
        unsigned int i_max_ = (unsigned int)i_max;
        unsigned int j_min_ = (unsigned int)j_min;
        unsigned int j_max_ = (unsigned int)j_max;

        if (XImageByteOrder(display) == 1) {
          // big endian
          for (unsigned int i = i_min_; i < i_max_; i++) {
            unsigned char *dst_32 = (unsigned char *)Ximage->data + (int)(i * 4 * m_width + j_min_ * 4);
            for (unsigned int j = j_min_; j < j_max_; j++) {
              vpRGBa val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val.A;
              *(dst_32++) = val.R;
              *(dst_32++) = val.G;
              *(dst_32++) = val.B;
            }
          }
        } else {
          // little endian
          for (unsigned int i = i_min_; i < i_max_; i++) {
            unsigned char *dst_32 = (unsigned char *)Ximage->data + (int)(i * 4 * m_width + j_min_ * 4);
            for (unsigned int j = j_min_; j < j_max_; j++) {
              vpRGBa val = I[i * m_scale][j * m_scale];
              *(dst_32++) = val.B;
              *(dst_32++) = val.G;
              *(dst_32++) = val.R;
              *(dst_32++) = val.A;
            }
          }
        }
        XPutImage(display, pixmap, context, Ximage, j_min, i_min, j_min, i_min, j_max_ - j_min_, i_max_ - i_min_);
      }

      XSetWindowBackgroundPixmap(display, window, pixmap);
      break;
    }
    default:
      throw(vpDisplayException(vpDisplayException::depthNotSupportedError,
                               "Unsupported depth (%d bpp) for color display", screen_depth));
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!

  Close the window.

  \sa init()

*/
void vpDisplayX::closeDisplay()
{
  if (m_displayHasBeenInitialized) {
    if (ximage_data_init == true)
      free(Ximage->data);

    Ximage->data = NULL;
    XDestroyImage(Ximage);

    XFreePixmap(display, pixmap);

    XFreeGC(display, context);
    XDestroyWindow(display, window);
    XCloseDisplay(display);

    m_displayHasBeenInitialized = false;

    if (x_color != NULL) {
      delete[] x_color;
      x_color = NULL;
    }
  }
}

/*!
  Flushes the X buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayX::flushDisplay()
{
  if (m_displayHasBeenInitialized) {
    XClearWindow(display, window);
    XFlush(display);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Flushes a part of the X buffer.
  It's necessary to use this function to see the results of any drawing.

  \param iP : Top left corner of the region of interest
  \param w,h  : Width and height of the region of interest
*/
void vpDisplayX::flushDisplayROI(const vpImagePoint &iP, const unsigned int w, const unsigned int h)
{
  if (m_displayHasBeenInitialized) {
    XClearArea(display, window, (int)(iP.get_u() / m_scale), (int)(iP.get_v() / m_scale), w / m_scale, h / m_scale, 0);
    XFlush(display);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Set the window backgroud to \e color.
  \param color : Background color.
*/
void vpDisplayX::clearDisplay(const vpColor &color)
{
  if (m_displayHasBeenInitialized) {

    if (color.id < vpColor::id_unknown)
      XSetWindowBackground(display, window, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XClearWindow(display, window);

    XFreePixmap(display, pixmap);
    // Pixmap creation.
    pixmap = XCreatePixmap(display, window, m_width, m_height, screen_depth);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayX::displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w,
                              unsigned int h, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double a = ip2.get_i() - ip1.get_i();
    double b = ip2.get_j() - ip1.get_j();
    double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

    // if ( ( a==0 ) && ( b==0 ) )
    if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
        (std::fabs(b) <= std::numeric_limits<double>::epsilon())) {
      // DisplayCrossLarge(i1,j1,3,col) ;
    } else {
      a /= lg;
      b /= lg;

      vpImagePoint ip3;
      ip3.set_i(ip2.get_i() - w * a);
      ip3.set_j(ip2.get_j() - w * b);

      vpImagePoint ip4;
      ip4.set_i(ip3.get_i() - b * h);
      ip4.set_j(ip3.get_j() + a * h);

      if (lg > 2 * vpImagePoint::distance(ip2, ip4))
        displayLine(ip2, ip4, color, thickness);

      ip4.set_i(ip3.get_i() + b * h);
      ip4.set_j(ip3.get_j() - a * h);

      if (lg > 2 * vpImagePoint::distance(ip2, ip4))
        displayLine(ip2, ip4, color, thickness);

      displayLine(ip1, ip2, color, thickness);
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param ip : Upper left image point location of the string in the display.
  \param text : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplayX::displayCharString(const vpImagePoint &ip, const char *text, const vpColor &color)
{
  if (m_displayHasBeenInitialized) {
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }
    XDrawString(display, pixmap, context, (int)(ip.get_u() / m_scale), (int)(ip.get_v() / m_scale), text,
                (int)strlen(text));
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a circle.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void vpDisplayX::displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                               unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XSetLineAttributes(display, context, thickness, LineSolid, CapButt, JoinBevel);

    if (fill == false) {
      XDrawArc(display, pixmap, context, vpMath::round((center.get_u() - radius) / m_scale),
               vpMath::round((center.get_v() - radius) / m_scale), radius * 2 / m_scale, radius * 2 / m_scale, 0,
               23040); /* 23040 = 360*64 */
    } else {
      XFillArc(display, pixmap, context, vpMath::round((center.get_u() - radius) / m_scale),
               vpMath::round((center.get_v() - radius) / m_scale), radius * 2 / m_scale, radius * 2 / m_scale, 0,
               23040); /* 23040 = 360*64 */
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param cross_size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayX::displayCross(const vpImagePoint &ip, unsigned int cross_size, const vpColor &color,
                              unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double i = ip.get_i();
    double j = ip.get_j();
    vpImagePoint ip1, ip2;

    ip1.set_i(i - cross_size / 2);
    ip1.set_j(j);
    ip2.set_i(i + cross_size / 2);
    ip2.set_j(j);
    displayLine(ip1, ip2, color, thickness);

    ip1.set_i(i);
    ip1.set_j(j - cross_size / 2);
    ip2.set_i(i);
    ip2.set_j(j + cross_size / 2);

    displayLine(ip1, ip2, color, thickness);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}
/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayX::displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XSetLineAttributes(display, context, thickness, LineOnOffDash, CapButt, JoinBevel);

    XDrawLine(display, pixmap, context, vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale),
              vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale));
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayX::displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                             unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;

    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XSetLineAttributes(display, context, thickness, LineSolid, CapButt, JoinBevel);

    XDrawLine(display, pixmap, context, vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale),
              vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale));
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Point thickness.
*/
void vpDisplayX::displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    if (thickness == 1) {
      XDrawPoint(display, pixmap, context, vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale));
    } else {
      XFillRectangle(display, pixmap, context, vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale),
                     thickness, thickness);
    }

  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param topLeft : Top-left corner of the rectangle.
  \param w,h : Rectangle size in terms of width and height.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplayX::displayRectangle(const vpImagePoint &topLeft, unsigned int w, unsigned int h, const vpColor &color,
                                  bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }
    XSetLineAttributes(display, context, thickness, LineSolid, CapButt, JoinBevel);
    if (fill == false) {
      XDrawRectangle(display, pixmap, context, vpMath::round(topLeft.get_u() / m_scale),
                     vpMath::round(topLeft.get_v() / m_scale), w / m_scale, h / m_scale);
    } else {
      XFillRectangle(display, pixmap, context, vpMath::round(topLeft.get_u() / m_scale),
                     vpMath::round(topLeft.get_v() / m_scale), w / m_scale, h / m_scale);
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a rectangle.

  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplayX::displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight, const vpColor &color,
                                  bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XSetLineAttributes(display, context, thickness, LineSolid, CapButt, JoinBevel);

    vpImagePoint topLeft_ = topLeft / m_scale;
    vpImagePoint bottomRight_ = bottomRight / m_scale;
    unsigned int w = (unsigned int)vpMath::round(std::fabs(bottomRight_.get_u() - topLeft_.get_u()));
    unsigned int h = (unsigned int)vpMath::round(std::fabs(bottomRight_.get_v() - topLeft_.get_v()));
    if (fill == false) {

      XDrawRectangle(display, pixmap, context,
                     vpMath::round(topLeft_.get_u() < bottomRight_.get_u() ? topLeft_.get_u() : bottomRight_.get_u()),
                     vpMath::round(topLeft_.get_v() < bottomRight_.get_v() ? topLeft_.get_v() : bottomRight_.get_v()),
                     w > 0 ? w : 1, h > 0 ? h : 1);
    } else {
      XFillRectangle(display, pixmap, context,
                     vpMath::round(topLeft_.get_u() < bottomRight_.get_u() ? topLeft_.get_u() : bottomRight_.get_u()),
                     vpMath::round(topLeft_.get_v() < bottomRight_.get_v() ? topLeft_.get_v() : bottomRight_.get_v()),
                     w, h);
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Display a rectangle.

  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void vpDisplayX::displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (thickness == 1)
      thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground(display, context, x_color[color.id]);
    else {
      xcolor.pad = 0;
      xcolor.red = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue = 256 * color.B;
      XAllocColor(display, lut, &xcolor);
      XSetForeground(display, context, xcolor.pixel);
    }

    XSetLineAttributes(display, context, thickness, LineSolid, CapButt, JoinBevel);

    if (fill == false) {
      XDrawRectangle(display, pixmap, context, vpMath::round(rectangle.getLeft() / m_scale),
                     vpMath::round(rectangle.getTop() / m_scale),
                     (unsigned int)vpMath::round(rectangle.getWidth() / m_scale - 1),
                     (unsigned int)vpMath::round(rectangle.getHeight() / m_scale - 1));
    } else {
      XFillRectangle(display, pixmap, context, vpMath::round(rectangle.getLeft() / m_scale),
                     vpMath::round(rectangle.getTop() / m_scale),
                     (unsigned int)vpMath::round(rectangle.getWidth() / m_scale),
                     (unsigned int)vpMath::round(rectangle.getHeight() / m_scale));
    }

  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!

  Wait for a click from one of the mouse button.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayX::getClick(bool blocking)
{

  bool ret = false;

  if (m_displayHasBeenInitialized) {
    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;

    // Event testing
    if (blocking) {
      XCheckMaskEvent(display, ButtonPressMask, &event);
      XCheckMaskEvent(display, ButtonReleaseMask, &event);
      XMaskEvent(display, ButtonPressMask, &event);
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, ButtonPressMask, &event);
    }

    if (ret) {
      /* Recuperation de la coordonnee du pixel clique. */
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*!

  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayX::getClick(vpImagePoint &ip, bool blocking)
{

  bool ret = false;
  if (m_displayHasBeenInitialized) {

    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;
    // Event testing
    if (blocking) {
      XCheckMaskEvent(display, ButtonPressMask, &event);
      XCheckMaskEvent(display, ButtonReleaseMask, &event);
      XMaskEvent(display, ButtonPressMask, &event);
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, ButtonPressMask, &event);
    }

    if (ret) {
      // Get mouse position
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
        ip.set_u((double)event.xbutton.x * m_scale);
        ip.set_v((double)event.xbutton.y * m_scale);
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*!

  Wait for a mouse button click and get the position of the clicked
  pixel. The button used to click is also set.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplayX::getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{

  bool ret = false;
  if (m_displayHasBeenInitialized) {

    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;

    // Event testing
    if (blocking) {
      XCheckMaskEvent(display, ButtonPressMask, &event);
      XCheckMaskEvent(display, ButtonReleaseMask, &event);
      XMaskEvent(display, ButtonPressMask, &event);
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, ButtonPressMask, &event);
    }

    if (ret) {
      // Get mouse position
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
        ip.set_u((double)event.xbutton.x * m_scale);
        ip.set_v((double)event.xbutton.y * m_scale);
        switch (event.xbutton.button) {
        case Button1:
          button = vpMouseButton::button1;
          break;
        case Button2:
          button = vpMouseButton::button2;
          break;
        case Button3:
          button = vpMouseButton::button3;
          break;
        }
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*!

  Wait for a mouse button click release and get the position of the
  image point were the click release occurs.  The button used to click is
  also set. Same method as getClick(unsigned int&, unsigned int&,
  vpMouseButton::vpMouseButtonType &, bool).

  \param ip [out] : Position of the clicked image point.

  \param button [in] : Button used to click.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

  \sa getClick(vpImagePoint &, vpMouseButton::vpMouseButtonType &, bool)

*/
bool vpDisplayX::getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{

  bool ret = false;
  if (m_displayHasBeenInitialized) {
    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;

    // Event testing
    if (blocking) {
      XCheckMaskEvent(display, ButtonPressMask, &event);
      XCheckMaskEvent(display, ButtonReleaseMask, &event);
      XMaskEvent(display, ButtonReleaseMask, &event);
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, ButtonReleaseMask, &event);
    }

    if (ret) {
      /* Recuperation de la coordonnee du pixel clique. */
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
        ip.set_u((double)event.xbutton.x * m_scale);
        ip.set_v((double)event.xbutton.y * m_scale);
        switch (event.xbutton.button) {
        case Button1:
          button = vpMouseButton::button1;
          break;
        case Button2:
          button = vpMouseButton::button2;
          break;
        case Button3:
          button = vpMouseButton::button3;
          break;
        }
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*
  Gets the displayed image (including the overlay plane)
  and returns an RGBa image. If a scale factor is set using setScale(), the
  size of the image is the size of the downscaled image.

  \param I : Image to get.
*/
void vpDisplayX::getImage(vpImage<vpRGBa> &I)
{
  if (m_displayHasBeenInitialized) {
    XImage *xi;

    XCopyArea(display, window, pixmap, context, 0, 0, m_width, m_height, 0, 0);

    xi = XGetImage(display, pixmap, 0, 0, m_width, m_height, AllPlanes, ZPixmap);

    I.resize(m_height, m_width);

    unsigned char *src_32 = NULL;
    src_32 = (unsigned char *)xi->data;

    if (screen_depth == 16) {
      for (unsigned int i = 0; i < I.getHeight(); i++) {
        size_t i_ = i * m_width;
        for (unsigned int j = 0; j < m_height; j++) {
          size_t ij_ = i_ + j;
          unsigned long pixel = XGetPixel(xi, (int)j, (int)i);
          I.bitmap[ij_].R = (((pixel & RMask) << RShift) >> 8);
          I.bitmap[ij_].G = (((pixel & GMask) << GShift) >> 8);
          I.bitmap[ij_].B = (((pixel & BMask) << BShift) >> 8);
          // On OSX the bottom/right corner (arround the resizing icon) has
          // alpha component with different values than 255. That's why we
          // force alpha to vpRGBa::alpha_default
          I.bitmap[ij_].A = vpRGBa::alpha_default;
        }
      }

    } else {
      if (XImageByteOrder(display) == 1) {
        // big endian
        for (unsigned int i = 0; i < m_width * m_height; i++) {
          // On OSX the bottom/right corner (arround the resizing icon) has
          // alpha component with different values than 255. That's why we
          // force alpha to vpRGBa::alpha_default
          I.bitmap[i].A = vpRGBa::alpha_default; // src_32[i*4] ;
          I.bitmap[i].R = src_32[i * 4 + 1];
          I.bitmap[i].G = src_32[i * 4 + 2];
          I.bitmap[i].B = src_32[i * 4 + 3];
        }
      } else {
        // little endian
        for (unsigned int i = 0; i < m_width * m_height; i++) {
          I.bitmap[i].B = src_32[i * 4];
          I.bitmap[i].G = src_32[i * 4 + 1];
          I.bitmap[i].R = src_32[i * 4 + 2];
          // On OSX the bottom/right corner (arround the resizing icon) has
          // alpha component with different values than 255. That's why we
          // force alpha to vpRGBa::alpha_default
          I.bitmap[i].A = vpRGBa::alpha_default; // src_32[i*4 + 3];
        }
      }
    }
    XDestroyImage(xi);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
}

/*!
  Gets the window depth (8, 16, 24, 32).
*/
unsigned int vpDisplayX::getScreenDepth()
{
  Display *display_;
  int screen_;
  unsigned int depth;

  if ((display_ = XOpenDisplay(NULL)) == NULL) {
    throw(vpDisplayException(vpDisplayException::connexionError, "Can't connect display on server %s.",
                             XDisplayName(NULL)));
  }
  screen_ = DefaultScreen(display_);
  depth = (unsigned int)DefaultDepth(display_, screen_);

  XCloseDisplay(display_);

  return (depth);
}

/*!
  Gets screen resolution in pixels.
  \param w, h : Horizontal and vertical screen resolution.
 */
void vpDisplayX::getScreenSize(unsigned int &w, unsigned int &h)
{
  Display *display_;
  int screen_;

  if ((display_ = XOpenDisplay(NULL)) == NULL) {
    throw(vpDisplayException(vpDisplayException::connexionError, "Can't connect display on server %s.",
                             XDisplayName(NULL)));
  }
  screen_ = DefaultScreen(display_);
  w = (unsigned int)DisplayWidth(display_, screen_);
  h = (unsigned int)DisplayHeight(display_, screen_);

  XCloseDisplay(display_);
}

/*!
  Gets the screen horizontal resolution in pixels.
 */
unsigned int vpDisplayX::getScreenWidth()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return width;
}

/*!
  Gets the screen vertical resolution in pixels.
 */
unsigned int vpDisplayX::getScreenHeight()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return height;
}

/*!

  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool vpDisplayX::getKeyboardEvent(bool blocking)
{

  bool ret = false;

  if (m_displayHasBeenInitialized) {
    // Event testing
    if (blocking) {
      XMaskEvent(display, KeyPressMask, &event);
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, KeyPressMask, &event);
    }
  } else {
    vpERROR_TRACE("X not initialized ");
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}
/*!

  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \param key [out]: If possible, an ISO Latin-1 character
  corresponding to the keyboard key.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool vpDisplayX::getKeyboardEvent(std::string &key, bool blocking)
{
  bool ret = false;
  KeySym keysym;
  //   int     count;
  XComposeStatus compose_status;
  char buffer;

  if (m_displayHasBeenInitialized) {
    // Event testing
    if (blocking) {
      XMaskEvent(display, KeyPressMask, &event);
      /* count = */ XLookupString((XKeyEvent *)&event, &buffer, 1, &keysym, &compose_status);
      key = buffer;
      ret = true;
    } else {
      ret = XCheckMaskEvent(display, KeyPressMask, &event);
      if (ret) {
        /* count = */ XLookupString((XKeyEvent *)&event, &buffer, 1, &keysym, &compose_status);
        key = buffer;
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}
/*!

  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool vpDisplayX::getPointerMotionEvent(vpImagePoint &ip)
{

  bool ret = false;
  if (m_displayHasBeenInitialized) {

    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;
    // Event testing
    ret = XCheckMaskEvent(display, PointerMotionMask, &event);

    if (ret) {
      // Get mouse position
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
        ip.set_u((double)event.xbutton.x * m_scale);
        ip.set_v((double)event.xbutton.y * m_scale);
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*!
  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.
*/
bool vpDisplayX::getPointerPosition(vpImagePoint &ip)
{

  bool ret = false;
  if (m_displayHasBeenInitialized) {

    Window rootwin, childwin;
    int root_x, root_y, win_x, win_y;
    unsigned int modifier;
    // Event testing
    ret = true;

    if (ret) {
      // Get mouse position
      if (XQueryPointer(display, window, &rootwin, &childwin, &root_x, &root_y, &win_x, &win_y, &modifier)) {
        ip.set_u((double)win_x * m_scale);
        ip.set_v((double)win_y * m_scale);
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "X not initialized"));
  }
  return ret;
}

/*!
  Get the position of the most significant bit.
*/
int vpDisplayX::getMsb(unsigned int u32val)
{
  int i;

  for (i = 31; i >= 0; --i) {
    if (u32val & 0x80000000L)
      break;
    u32val <<= 1;
  }
  return i;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDisplayX.cpp.o) has no
// symbols
void dummy_vpDisplayX(){};
#endif
