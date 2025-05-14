/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

#ifndef VP_DISPLAY_OPENCV_H
#define VP_DISPLAY_OPENCV_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>

#if defined(HAVE_OPENCV_HIGHGUI)

#include <functional>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>

BEGIN_VISP_NAMESPACE

/*!
 * \file vpDisplayOpenCV.h
 * \brief Define the OpenCV console to display images.
 */

/*!
 * \class vpDisplayOpenCV
 *
 * \ingroup group_gui_display
 *
 * \brief The vpDisplayOpenCV allows to display image using the OpenCV library.
 * Thus to enable this class OpenCV should be installed. Installation
 * instructions are provided here https://visp.inria.fr/3rd_opencv.
 *
 * \warning Since ViSP 3.3.1 or higher we introduce the alpha channel support for color
 * transparency. This new feature is only supported yet using vpDisplayOpenCV. See vpColor
 * header documentation and displayOpenCV.cpp example for usage displaying filled
 * transparent circles and rectangles.
 *
 * The example below shows how to display an image with this video device.
 * \code
 * #include <visp3/core/vpImagePoint.h>
 * #include <visp3/gui/vpDisplayOpenCV.h>
 * #include <visp3/io/vpImageIo.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_OPENCV)
 *   vpImage<unsigned char> I; // Grey level image
 *
 *   // Read an image in PGM P5 format
 *   vpImageIo::read(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
 *
 *   vpDisplayOpenCV d;
 *
 *   // Initialize the display with the image I. Display and image are
 *   // now link together.
 *   d.init(I);
 *
 *   // Specify the window location
 *   vpDisplay::setWindowPosition(I, 400, 100);
 *
 *   // Set the display window title
 *   vpDisplay::setTitle(I, "My OpenCV display");
 *
 *   // Set the display background with image I content
 *   vpDisplay::display(I);
 *
 *   // Draw a red rectangle in the display overlay (foreground)
 *   vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);
 *
 *   // Draw a red rectangle in the display overlay (foreground)
 *   vpImagePoint topLeftCorner;
 *   topLeftCorner.set_i(10);
 *   topLeftCorner.set_j(50);
 *   vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);
 *
 *   // Flush the foreground and background display
 *   vpDisplay::flush(I);
 *
 *   // Get non blocking keyboard events
 *   std::cout << "Check keyboard events..." << std::endl;
 *   char key[10];
 *   bool ret;
 *   for (int i=0; i< 200; ++i) {
 *     bool ret = vpDisplay::getKeyboardEvent(I, key, false);
 *     if (ret)
 *       std::cout << "keyboard event: key: " << "\"" << key << "\"" << std::endl;
 *   vpTime::wait(40);
 *   }
 *
 *   // Get a blocking keyboard event
 *   std::cout << "Wait for a keyboard event..." << std::endl;
 *   ret = vpDisplay::getKeyboardEvent(I, key, true);
 *   std::cout << "keyboard event: " << ret << std::endl;
 *   if (ret)
 *     std::cout << "key: " << "\"" << key << "\"" << std::endl;
 *
 *   // Wait for a click in the display window
 *   std::cout << "Wait for a button click..." << std::endl;
 *   vpDisplay::getClick(I);
 * #endif
 * }
 * \endcode
*/
class VISP_EXPORT vpDisplayOpenCV : public vpDisplay
{
public:
  vpDisplayOpenCV();
  vpDisplayOpenCV(int winx, int winy, const std::string &title = "");
  vpDisplayOpenCV(vpImage<unsigned char> &I, vpScaleType type);
  vpDisplayOpenCV(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "",
                  vpScaleType type = SCALE_DEFAULT);
  vpDisplayOpenCV(vpImage<vpRGBa> &I, vpScaleType type);
  vpDisplayOpenCV(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "",
                  vpScaleType type = SCALE_DEFAULT);
  vpDisplayOpenCV(const vpDisplayOpenCV &display);
  virtual ~vpDisplayOpenCV() VP_OVERRIDE;
  vpDisplayOpenCV &operator=(const vpDisplayOpenCV &display);

  void getImage(vpImage<vpRGBa> &I) VP_OVERRIDE;
  unsigned int getScreenHeight() VP_OVERRIDE;
  void getScreenSize(unsigned int &width, unsigned int &height) VP_OVERRIDE;
  unsigned int getScreenWidth() VP_OVERRIDE;

  void init(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "") VP_OVERRIDE;
  void init(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "") VP_OVERRIDE;
  void init(unsigned int width, unsigned int height, int winx = -1, int winy = -1, const std::string &title = "") VP_OVERRIDE;

protected:
  void setFont(const std::string &font) VP_OVERRIDE;
  void setTitle(const std::string &title) VP_OVERRIDE;
  void setWindowPosition(int winx, int winy) VP_OVERRIDE;

  void clearDisplay(const vpColor &color = vpColor::white) VP_OVERRIDE;

  void closeDisplay() VP_OVERRIDE;

  void displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color = vpColor::white,
                    unsigned int w = 4, unsigned int h = 2, unsigned int thickness = 1) VP_OVERRIDE;

  void displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill = false,
                     unsigned int thickness = 1) VP_OVERRIDE;
  void displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness = 1) VP_OVERRIDE;
  void displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                      unsigned int thickness = 1) VP_OVERRIDE;

  void displayImage(const vpImage<unsigned char> &I) VP_OVERRIDE;
  void displayImage(const vpImage<vpRGBa> &I) VP_OVERRIDE;
  void displayImage(const unsigned char *I);

  void displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int width,
                       unsigned int height) VP_OVERRIDE;
  void displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, unsigned int width, unsigned int height) VP_OVERRIDE;

  void displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness = 1) VP_OVERRIDE;
  void displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness = 1) VP_OVERRIDE;

  void displayRectangle(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                        bool fill = false, unsigned int thickness = 1) VP_OVERRIDE;
  void displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight, const vpColor &color,
                        bool fill = false, unsigned int thickness = 1) VP_OVERRIDE;
  void displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill = false, unsigned int thickness = 1) VP_OVERRIDE;

  void displayText(const vpImagePoint &ip, const std::string &text, const vpColor &color = vpColor::green) VP_OVERRIDE;

  void flushDisplay() VP_OVERRIDE;
  void flushDisplayROI(const vpImagePoint &iP, unsigned int width, unsigned int height) VP_OVERRIDE;

  bool getClick(bool blocking = true) VP_OVERRIDE;
  bool getClick(vpImagePoint &ip, bool blocking = true) VP_OVERRIDE;
  bool getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true) VP_OVERRIDE;
  bool getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking = true) VP_OVERRIDE;

  bool getKeyboardEvent(bool blocking = true) VP_OVERRIDE;
  bool getKeyboardEvent(std::string &key, bool blocking = true) VP_OVERRIDE;
  bool getPointerMotionEvent(vpImagePoint &ip) VP_OVERRIDE;
  bool getPointerPosition(vpImagePoint &ip) VP_OVERRIDE;

private:
  // Implementation
  class Impl;
  Impl *m_impl;
};

END_VISP_NAMESPACE
#endif
#endif
