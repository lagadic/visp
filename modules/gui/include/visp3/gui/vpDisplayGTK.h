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
 * Image display.
 *
 * Authors:
 * Christophe Collewet
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpDisplayGTK_h
#define vpDisplayGTK_h

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_GTK))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>

#include <gdk/gdkrgb.h>
#include <gtk/gtk.h>

/*!
  \file vpDisplayGTK.h
  \brief Define the GTK console to display images.
*/

/*!

  \class vpDisplayGTK

  \ingroup group_gui_display

  \brief The vpDisplayGTK allows to display image using the GTK 3rd party
library. Thus to enable this class GTK should be installed. Installation
  instructions are provided here https://visp.inria.fr/3rd_gtk.

  The example below shows how to display an image with this video device.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_GTK)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
#ifdef _WIN32
  vpImageIo::read(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");
#else
  vpImageIo::read(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
#endif

  vpDisplayGTK d;

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My GTK display");

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
class VISP_EXPORT vpDisplayGTK : public vpDisplay
{
private:
  //! true if GTK display is ready to use
  GtkWidget *widget;
  GdkPixmap *m_background;
  GdkGC *m_gc;
  GdkColor blue, red, yellow, green, cyan, orange, white, black, gdkcolor, lightBlue, darkBlue, lightRed, darkRed,
      lightGreen, darkGreen, purple, lightGray, gray, darkGray;
  GdkColormap *colormap;

  GdkFont *font;
  guchar *vectgtk;
  GdkColor **col;
  int ncol, nrow;

  typedef enum {
    id_black = 0,
    id_white,
    id_lightGray,
    id_gray,
    id_darkGray,
    id_lightRed,
    id_red,
    id_darkRed,
    id_lightGreen,
    id_green,
    id_darkGreen,
    id_lightBlue,
    id_blue,
    id_darkBlue,
    id_yellow,
    id_cyan,
    id_orange,
    id_purple,
    id_npredefined // Number of predefined colors
  } vpColorIdentifier;

public:
  vpDisplayGTK();
  vpDisplayGTK(int winx, int winy, const std::string &title = "");
  vpDisplayGTK(vpImage<unsigned char> &I, vpScaleType type);
  vpDisplayGTK(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);
  vpDisplayGTK(vpImage<vpRGBa> &I, vpScaleType type);
  vpDisplayGTK(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "",
               vpScaleType type = SCALE_DEFAULT);

  virtual ~vpDisplayGTK();

  void getImage(vpImage<vpRGBa> &I);
  unsigned int getScreenDepth();
  unsigned int getScreenHeight();
  void getScreenSize(unsigned int &width, unsigned int &height);
  unsigned int getScreenWidth();

  void init(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "");
  void init(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "");
  void init(unsigned int width, unsigned int height, int winx = -1, int winy = -1, const std::string &title = "");

protected:
  void setFont(const std::string &fontname);
  void setTitle(const std::string &title);
  void setWindowPosition(int winx, int winy);

  void clearDisplay(const vpColor &color = vpColor::white);

  void closeDisplay();

  void displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color = vpColor::white,
                    unsigned int w = 4, unsigned int h = 2, unsigned int thickness = 1);
  void displayCharString(const vpImagePoint &ip, const char *text, const vpColor &color = vpColor::green);

  void displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill = false,
                     unsigned int thickness = 1);
  void displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness = 1);
  void displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                      unsigned int thickness = 1);

  void displayImage(const vpImage<vpRGBa> &I);
  void displayImage(const vpImage<unsigned char> &I);
  void displayImage(const unsigned char *I);

  void displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width,
                       const unsigned int height);
  void displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width,
                       const unsigned int height);

  void displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness = 1);

  void displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness = 1);
  void displayRectangle(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                        bool fill = false, unsigned int thickness = 1);
  void displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight, const vpColor &color,
                        bool fill = false, unsigned int thickness = 1);
  void displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill = false, unsigned int thickness = 1);

  void flushDisplay();
  void flushDisplayROI(const vpImagePoint &iP, const unsigned int width, const unsigned int height);

  bool getClick(bool blocking = true);
  bool getClick(vpImagePoint &ip, bool blocking = true);
  bool getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true);
  bool getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true);
  bool getKeyboardEvent(bool blocking = true);
  bool getKeyboardEvent(std::string &key, bool blocking = true);
  bool getPointerMotionEvent(vpImagePoint &ip);
  bool getPointerPosition(vpImagePoint &ip);
};

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
