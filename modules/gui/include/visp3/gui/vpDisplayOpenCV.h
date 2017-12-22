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

#ifndef vpDisplayOpenCV_h
#define vpDisplayOpenCV_h

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_OPENCV)

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#endif

/*!
  \file vpDisplayOpenCV.h
  \brief Define the OpenCV console to display images.
*/

/*!

  \class vpDisplayOpenCV

  \ingroup group_gui_display

  \brief The vpDisplayOpenCV allows to display image using the OpenCV library.
  Thus to enable this class OpenCV should be installed. Installation
  instructions are provided here https://visp.inria.fr/3rd_opencv.

  The example below shows how to display an image with this video device.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
  vpImageIo::read(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");

  vpDisplayOpenCV d;

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My OpenCV display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(10);
  topLeftCorner.set_j(50);
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

class VISP_EXPORT vpDisplayOpenCV : public vpDisplay
{
private:
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  //! true if OpenCV display is ready to use
  IplImage *m_background;
  CvScalar *col;
  CvScalar cvcolor;
  CvFont *font;
#else
  cv::Mat m_background;
  cv::Scalar *col;
  cv::Scalar cvcolor;
  int font;
  float fontScale;
#endif
  static std::vector<std::string> m_listTitles;
  static unsigned int m_nbWindows;
  int fontHeight;
  int x_move;
  int y_move;
  bool move;
  int x_lbuttondown;
  int y_lbuttondown;
  bool lbuttondown;
  int x_mbuttondown;
  int y_mbuttondown;
  bool mbuttondown;
  int x_rbuttondown;
  int y_rbuttondown;
  bool rbuttondown;
  int x_lbuttonup;
  int y_lbuttonup;
  bool lbuttonup;
  int x_mbuttonup;
  int y_mbuttonup;
  bool mbuttonup;
  int x_rbuttonup;
  int y_rbuttonup;
  bool rbuttonup;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpDisplayOpenCV(const vpDisplayOpenCV &)
  //    : vpDisplay(),
  //    #if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  //      background(NULL), col(NULL), cvcolor(), font(NULL),
  //    #else
  //      background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN),
  //      fontScale(0.8f),
  //    #endif
  //      fontHeight(10), x_move(0), y_move(0) , move(false),
  //      x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
  //      x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false),
  //      x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
  //      x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false),
  //      x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false),
  //      x_rbuttonup(0), y_rbuttonup(0), rbuttonup(false)
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpDisplayOpenCV &operator=(const vpDisplayOpenCV &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  vpDisplayOpenCV();
  vpDisplayOpenCV(int winx, int winy, const std::string &title = "");
  vpDisplayOpenCV(vpImage<unsigned char> &I, vpScaleType type);
  vpDisplayOpenCV(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "",
                  vpScaleType type = SCALE_DEFAULT);
  vpDisplayOpenCV(vpImage<vpRGBa> &I, vpScaleType type);
  vpDisplayOpenCV(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "",
                  vpScaleType type = SCALE_DEFAULT);

  virtual ~vpDisplayOpenCV();

  void getImage(vpImage<vpRGBa> &I);
  unsigned int getScreenHeight();
  void getScreenSize(unsigned int &width, unsigned int &height);
  unsigned int getScreenWidth();

  void init(vpImage<unsigned char> &I, int winx = -1, int winy = -1, const std::string &title = "");
  void init(vpImage<vpRGBa> &I, int winx = -1, int winy = -1, const std::string &title = "");
  void init(unsigned int width, unsigned int height, int winx = -1, int winy = -1, const std::string &title = "");

protected:
  void setFont(const std::string &font);
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

  void displayImage(const vpImage<unsigned char> &I);
  void displayImage(const vpImage<vpRGBa> &I);
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

  static void on_mouse(int event, int x, int y, int flags, void *param);
};

#endif
#endif
