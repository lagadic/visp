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
 * Christophe Collewet
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpDisplayOpenCV.cpp
  \brief Define the OpenCV console to display images.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>

// Display stuff
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayOpenCV.h>

// debug / exception
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplayException.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)

#include <opencv2/core/core_c.h> // for CV_FILLED versus cv::FILLED
#include <opencv2/imgproc/imgproc.hpp>

#ifndef CV_RGB
#define CV_RGB(r, g, b) cv::Scalar((b), (g), (r), 0)
#endif
#endif

#ifdef VISP_HAVE_X11
#include <visp3/gui/vpDisplayX.h> // to get screen resolution
#elif defined(_WIN32)
#include <windows.h>
#endif

std::vector<std::string> vpDisplayOpenCV::m_listTitles = std::vector<std::string>();
unsigned int vpDisplayOpenCV::m_nbWindows = 0;

/*!

  Constructor. Initialize a display to visualize a gray level image
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
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<unsigned char> &I, vpScaleType scaleType)
  : vpDisplay(),
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I);
}

/*!

  Constructor. Initialize a display to visualize a gray level image
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
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<unsigned char> &I, int x, int y, const std::string &title,
                                 vpScaleType scaleType)
  : vpDisplay(),
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I, x, y, title);
}

/*!
  Constructor. Initialize a display to visualize a RGBa image (32 bits).

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
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<vpRGBa> &I, vpScaleType scaleType)
  :
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
{
  setScale(scaleType, I.getWidth(), I.getHeight());
  init(I);
}

/*!
  Constructor. Initialize a display to visualize a RGBa image (32 bits).

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
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<vpRGBa> &I, int x, int y, const std::string &title, vpScaleType scaleType)
  :
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
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
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpDisplayOpenCV d(100, 200, "My display");
  vpImage<unsigned char> I(240, 384);
  d.init(I);
}
  \endcode
*/
vpDisplayOpenCV::vpDisplayOpenCV(int x, int y, const std::string &title)
  :
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
{
  m_windowXPosition = x;
  m_windowYPosition = y;

  if (!title.empty()) {
    m_title = title;
  } else {
    std::ostringstream s;
    s << m_nbWindows++;
    m_title = std::string("Window ") + s.str();
  }

  bool isInList;
  do {
    isInList = false;
    for (size_t i = 0; i < m_listTitles.size(); i++) {
      if (m_listTitles[i] == m_title) {
        std::ostringstream s;
        s << m_nbWindows++;
        m_title = std::string("Window ") + s.str();
        isInList = true;
        break;
      }
    }
  } while (isInList);

  m_listTitles.push_back(m_title);
}

/*!
  Basic constructor.

  To initialize the window position, title and size you may call
  init(vpImage<unsigned char> &, int, int, const std::string &) or
  init(vpImage<vpRGBa> &, int, int, const std::string &).

  \code
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpDisplayOpenCV d;
  vpImage<unsigned char> I(240, 384);
  d.init(I, 100, 200, "My display");
}
  \endcode
*/
vpDisplayOpenCV::vpDisplayOpenCV()
  :
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    m_background(NULL), col(NULL), cvcolor(), font(NULL),
#else
    m_background(), col(NULL), cvcolor(), font(cv::FONT_HERSHEY_PLAIN), fontScale(0.8f),
#endif
    fontHeight(10), x_move(0), y_move(0), move(false), x_lbuttondown(0), y_lbuttondown(0), lbuttondown(false),
    x_mbuttondown(0), y_mbuttondown(0), mbuttondown(false), x_rbuttondown(0), y_rbuttondown(0), rbuttondown(false),
    x_lbuttonup(0), y_lbuttonup(0), lbuttonup(false), x_mbuttonup(0), y_mbuttonup(0), mbuttonup(false), x_rbuttonup(0),
    y_rbuttonup(0), rbuttonup(false)
{
}

/*!
  Destructor.
*/
vpDisplayOpenCV::~vpDisplayOpenCV()
{
  closeDisplay();
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  cvReleaseImage(&m_background);
#endif
}

/*!
  Initialize the display (size, position and title) of a gray level image.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayOpenCV::init(vpImage<unsigned char> &I, int x, int y, const std::string &title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }
  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), x, y, title);
  I.display = this;
  m_displayHasBeenInitialized = true;
}

/*!
  Initialize the display (size, position and title) of a color
  image in RGBa format.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayOpenCV::init(vpImage<vpRGBa> &I, int x, int y, const std::string &title)
{
  if ((I.getHeight() == 0) || (I.getWidth() == 0)) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "Image not initialized"));
  }

  setScale(m_scaleType, I.getWidth(), I.getHeight());
  init(I.getWidth(), I.getHeight(), x, y, title);
  I.display = this;
  m_displayHasBeenInitialized = true;
}

/*!
  Initialize the display size, position and title.

  \param w, h : Width and height of the window.
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

  \exception vpDisplayException::notInitializedError If OpenCV was not build
  with an available display device suach as Gtk, Cocoa, Carbon, Qt.
*/
void vpDisplayOpenCV::init(unsigned int w, unsigned int h, int x, int y, const std::string &title)
{
  setScale(m_scaleType, w, h);

  this->m_width = w / m_scale;
  this->m_height = h / m_scale;

  if (x != -1)
    this->m_windowXPosition = x;
  if (y != -1)
    this->m_windowYPosition = y;
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  int flags = CV_WINDOW_AUTOSIZE;
#else
  int flags = cv::WINDOW_AUTOSIZE;
#endif

  if (m_title.empty()) {
    if (!title.empty()) {
      m_title = std::string(title);
    } else {

      std::ostringstream s;
      s << m_nbWindows++;
      m_title = std::string("Window ") + s.str();
    }

    bool isInList;
    do {
      isInList = false;
      for (size_t i = 0; i < m_listTitles.size(); i++) {
        if (m_listTitles[i] == m_title) {
          std::ostringstream s;
          s << m_nbWindows++;
          m_title = std::string("Window ") + s.str();
          isInList = true;
          break;
        }
      }
    } while (isInList);

    m_listTitles.push_back(m_title);
  }

/* Create the window*/
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if (cvNamedWindow(this->m_title.c_str(), flags) < 0) {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV was not built with a display device"));
  }
#else
  cv::namedWindow(this->m_title, flags);
#endif
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  cvMoveWindow(this->m_title.c_str(), this->m_windowXPosition, this->m_windowYPosition);
#else
  cv::moveWindow(this->m_title.c_str(), this->m_windowXPosition, this->m_windowYPosition);
#endif
  move = false;
  lbuttondown = false;
  mbuttondown = false;
  rbuttondown = false;
  lbuttonup = false;
  mbuttonup = false;
  rbuttonup = false;
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  cvSetMouseCallback(this->m_title.c_str(), on_mouse, this);
  col = new CvScalar[vpColor::id_unknown];
#else
  cv::setMouseCallback(this->m_title, on_mouse, this);
  col = new cv::Scalar[vpColor::id_unknown];
#endif

  /* Create color */
  vpColor pcolor; // Predefined colors
  pcolor = vpColor::lightBlue;
  col[vpColor::id_lightBlue] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::blue;
  col[vpColor::id_blue] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkBlue;
  col[vpColor::id_darkBlue] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightRed;
  col[vpColor::id_lightRed] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::red;
  col[vpColor::id_red] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkRed;
  col[vpColor::id_darkRed] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGreen;
  col[vpColor::id_lightGreen] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::green;
  col[vpColor::id_green] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGreen;
  col[vpColor::id_darkGreen] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::yellow;
  col[vpColor::id_yellow] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::cyan;
  col[vpColor::id_cyan] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::orange;
  col[vpColor::id_orange] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::purple;
  col[vpColor::id_purple] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::white;
  col[vpColor::id_white] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::black;
  col[vpColor::id_black] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGray;
  col[vpColor::id_lightGray] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::gray;
  col[vpColor::id_gray] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGray;
  col[vpColor::id_darkGray] = CV_RGB(pcolor.R, pcolor.G, pcolor.B);

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  font = new CvFont;
  cvInitFont(font, CV_FONT_HERSHEY_PLAIN, 0.70f, 0.70f);
  CvSize fontSize;
  int baseline;
  cvGetTextSize("A", font, &fontSize, &baseline);
#else
  int thickness = 1;
  cv::Size fontSize;
  int baseline;
  fontSize = cv::getTextSize("A", font, fontScale, thickness, &baseline);
#endif

  fontHeight = fontSize.height + baseline;
  m_displayHasBeenInitialized = true;
}

/*!
  \warning This method is not yet implemented.

  Set the font used to display a text in overlay. The display is
  performed using displayCharString().

  \param font : The expected font name. The available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \note Under UNIX, to know all the available fonts, use the
  "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

  \sa displayCharString()
*/
void vpDisplayOpenCV::setFont(const std::string & /* font */) { vpERROR_TRACE("Not yet implemented"); }

/*!
  Set the window title.

  \warning This method is not implemented yet.

  \param title : Window title.
 */
void vpDisplayOpenCV::setTitle(const std::string & /* title */)
{
  //  static bool warn_displayed = false;
  //  if (! warn_displayed) {
  //    vpTRACE("Not implemented");
  //    warn_displayed = true;
  //  }
}

/*!
  Set the window position in the screen.

  \param winx, winy : Position of the upper-left window's border in the
  screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplayOpenCV::setWindowPosition(int winx, int winy)
{
  if (m_displayHasBeenInitialized) {
    this->m_windowXPosition = winx;
    this->m_windowYPosition = winy;
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    cvMoveWindow(this->m_title.c_str(), winx, winy);
#else
    cv::moveWindow(this->m_title.c_str(), winx, winy);
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}
/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const vpImage<unsigned char> &I)
{
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int depth = 8;
    int channels = 3;
    CvSize size = cvSize((int)this->m_width, (int)this->m_height);
    if (m_background != NULL) {
      if (m_background->nChannels != channels || m_background->depth != depth ||
          m_background->height != (int)m_height || m_background->width != (int)m_width) {
        if (m_background->nChannels != 0)
          cvReleaseImage(&m_background);
        m_background = cvCreateImage(size, depth, channels);
      }
    } else {
      m_background = cvCreateImage(size, depth, channels);
    }

    if (m_scale == 1) {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep);
        for (unsigned int j = 0; j < m_width; j++) {
          unsigned char val = I[i][j];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    } else {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep);
        for (unsigned int j = 0; j < m_width; j++) {
          unsigned char val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    }

#else
    int depth = CV_8U;
    int channels = 3;
    cv::Size size((int)m_width, (int)m_height);
    if (m_background.channels() != channels || m_background.depth() != depth || m_background.rows != (int)m_height ||
        m_background.cols != (int)m_width) {
      m_background = cv::Mat(size, CV_MAKETYPE(depth, channels));
    }

    if (m_scale == 1) {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width);
        for (unsigned int j = 0; j < m_width; j++) {
          unsigned char val = I[i][j];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    } else {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width);
        for (unsigned int j = 0; j < m_width; j++) {
          unsigned char val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    }
#endif

  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayImageROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int w,
                                      const unsigned int h)
{
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int depth = 8;
    int channels = 3;
    CvSize size = cvSize((int)this->m_width, (int)this->m_height);
    if (m_background != NULL) {
      if (m_background->nChannels != channels || m_background->depth != depth ||
          m_background->height != (int)m_height || m_background->width != (int)m_width) {
        if (m_background->nChannels != 0)
          cvReleaseImage(&m_background);
        m_background = cvCreateImage(size, depth, channels);
      }
    } else {
      m_background = cvCreateImage(size, depth, channels);
    }

    if (m_scale == 1) {
      unsigned int i_min = (unsigned int)iP.get_i();
      unsigned int j_min = (unsigned int)iP.get_j();
      unsigned int i_max = (std::min)(i_min + h, m_height);
      unsigned int j_max = (std::min)(j_min + w, m_width);
      for (unsigned int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 =
            (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep + j_min * 3);
        for (unsigned int j = j_min; j < j_max; j++) {
          unsigned char val = I[i][j];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    } else {
      int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
      int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
      int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
      int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);
      for (int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 =
            (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep + j_min * 3);
        for (int j = j_min; j < j_max; j++) {
          unsigned char val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    }

#else
    int depth = CV_8U;
    int channels = 3;
    cv::Size size((int)m_width, (int)m_height);
    if (m_background.channels() != channels || m_background.depth() != depth || m_background.rows != (int)m_height ||
        m_background.cols != (int)m_width) {
      m_background = cv::Mat(size, CV_MAKETYPE(depth, channels));
    }

    if (m_scale == 1) {
      unsigned int i_min = (unsigned int)iP.get_i();
      unsigned int j_min = (unsigned int)iP.get_j();
      unsigned int i_max = (std::min)(i_min + h, m_height);
      unsigned int j_max = (std::min)(j_min + w, m_width);
      for (unsigned int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width + j_min * 3);
        for (unsigned int j = j_min; j < j_max; j++) {
          unsigned char val = I[i][j];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    } else {
      int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
      int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
      int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
      int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);
      for (int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width + j_min * 3);
        for (int j = j_min; j < j_max; j++) {
          unsigned char val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val;
          *(dst_24++) = val;
          *(dst_24++) = val;
        }
      }
    }
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const vpImage<vpRGBa> &I)
{

  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int depth = 8;
    int channels = 3;
    CvSize size = cvSize((int)this->m_width, (int)this->m_height);
    if (m_background != NULL) {
      if (m_background->nChannels != channels || m_background->depth != depth ||
          m_background->height != (int)m_height || m_background->width != (int)m_width) {
        if (m_background->nChannels != 0)
          cvReleaseImage(&m_background);
        m_background = cvCreateImage(size, depth, channels);
      }
    } else {
      m_background = cvCreateImage(size, depth, channels);
    }

    if (m_scale == 1) {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep);
        for (unsigned int j = 0; j < m_width; j++) {
          vpRGBa val = I[i][j];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    } else {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep);
        for (unsigned int j = 0; j < m_width; j++) {
          vpRGBa val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    }
#else
    int depth = CV_8U;
    int channels = 3;
    cv::Size size((int)this->m_width, (int)this->m_height);
    if (m_background.channels() != channels || m_background.depth() != depth || m_background.rows != (int)m_height ||
        m_background.cols != (int)m_width) {
      m_background = cv::Mat(size, CV_MAKETYPE(depth, channels));
    }

    if (m_scale == 1) {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width);
        for (unsigned int j = 0; j < m_width; j++) {
          vpRGBa val = I[i][j];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    } else {
      for (unsigned int i = 0; i < m_height; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width);
        for (unsigned int j = 0; j < m_width; j++) {
          vpRGBa val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    }
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayImageROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int w,
                                      const unsigned int h)
{
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int depth = 8;
    int channels = 3;
    CvSize size = cvSize((int)this->m_width, (int)this->m_height);
    if (m_background != NULL) {
      if (m_background->nChannels != channels || m_background->depth != depth ||
          m_background->height != (int)m_height || m_background->width != (int)m_width) {
        if (m_background->nChannels != 0)
          cvReleaseImage(&m_background);
        m_background = cvCreateImage(size, depth, channels);
      }
    } else {
      m_background = cvCreateImage(size, depth, channels);
    }

    if (m_scale == 1) {
      unsigned int i_min = (unsigned int)iP.get_i();
      unsigned int j_min = (unsigned int)iP.get_j();
      unsigned int i_max = (std::min)(i_min + h, m_height);
      unsigned int j_max = (std::min)(j_min + w, m_width);
      for (unsigned int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 =
            (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep + j_min * 3);
        for (unsigned int j = j_min; j < j_max; j++) {
          vpRGBa val = I[i][j];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    } else {
      int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
      int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
      int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
      int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);
      for (int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 =
            (unsigned char *)m_background->imageData + (int)(i * m_background->widthStep + j_min * 3);
        for (int j = j_min; j < j_max; j++) {
          vpRGBa val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    }
#else
    int depth = CV_8U;
    int channels = 3;
    cv::Size size((int)this->m_width, (int)this->m_height);
    if (m_background.channels() != channels || m_background.depth() != depth || m_background.rows != (int)m_height ||
        m_background.cols != (int)m_width) {
      m_background = cv::Mat(size, CV_MAKETYPE(depth, channels));
    }

    if (m_scale == 1) {
      unsigned int i_min = (unsigned int)iP.get_i();
      unsigned int j_min = (unsigned int)iP.get_j();
      unsigned int i_max = (std::min)(i_min + h, m_height);
      unsigned int j_max = (std::min)(j_min + w, m_width);
      for (unsigned int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width + j_min * 3);
        for (unsigned int j = j_min; j < j_max; j++) {
          vpRGBa val = I[i][j];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    } else {
      int i_min = (std::max)((int)ceil(iP.get_i() / m_scale), 0);
      int j_min = (std::max)((int)ceil(iP.get_j() / m_scale), 0);
      int i_max = (std::min)((int)ceil((iP.get_i() + h) / m_scale), (int)m_height);
      int j_max = (std::min)((int)ceil((iP.get_j() + w) / m_scale), (int)m_width);
      for (int i = i_min; i < i_max; i++) {
        unsigned char *dst_24 = (unsigned char *)m_background.data + (int)(i * 3 * m_width + j_min * 3);
        for (int j = j_min; j < j_max; j++) {
          vpRGBa val = I[i * m_scale][j * m_scale];
          *(dst_24++) = val.B;
          *(dst_24++) = val.G;
          *(dst_24++) = val.R;
        }
      }
    }
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  \warning ot implemented yet

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const unsigned char * /* I */) { vpTRACE(" not implemented "); }

/*!

  Close the window.

  \sa init()

*/
void vpDisplayOpenCV::closeDisplay()
{
  if (col != NULL) {
    delete[] col;
    col = NULL;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if (font != NULL) {
    delete font;
    font = NULL;
  }
#endif
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    cvDestroyWindow(this->m_title.c_str());
#else
    cv::destroyWindow(this->m_title);
#endif

    for (size_t i = 0; i < m_listTitles.size(); i++) {
      if (m_title == m_listTitles[i]) {
        m_listTitles.erase(m_listTitles.begin() + (long int)i);
        break;
      }
    }

    m_title.clear();

    m_displayHasBeenInitialized = false;
  }
}

/*!
  Flushes the OpenCV buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayOpenCV::flushDisplay()
{
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    cvShowImage(this->m_title.c_str(), m_background);
    cvWaitKey(5);
#else
    cv::imshow(this->m_title, m_background);
    cv::waitKey(5);
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Flushes the OpenCV buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayOpenCV::flushDisplayROI(const vpImagePoint & /*iP*/, const unsigned int /*width*/,
                                      const unsigned int /*height*/)
{
  if (m_displayHasBeenInitialized) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    cvShowImage(this->m_title.c_str(), m_background);
    cvWaitKey(5);
#else
    cv::imshow(this->m_title.c_str(), m_background);
    cv::waitKey(5);
#endif
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  \warning Not implemented yet.
*/
void vpDisplayOpenCV::clearDisplay(const vpColor & /* color */)
{
  static bool warn_displayed = false;
  if (!warn_displayed) {
    vpTRACE("Not implemented");
    warn_displayed = true;
  }
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayOpenCV::displayArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                   unsigned int w, unsigned int h, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double a = ip2.get_i() - ip1.get_i();
    double b = ip2.get_j() - ip1.get_j();
    double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

    // if ((a==0)&&(b==0))
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
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayCharString(const vpImagePoint &ip, const char *text, const vpColor &color)
{
  if (m_displayHasBeenInitialized) {
    if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
      cvPutText(m_background, text,
                cvPoint(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale + fontHeight)), font,
                col[color.id]);
#else
      cv::putText(m_background, text,
                  cv::Point(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale + fontHeight)),
                  font, fontScale, col[color.id]);
#endif
    } else {
      cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
      cvPutText(m_background, text,
                cvPoint(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale + fontHeight)), font,
                cvcolor);
#else
      cv::putText(m_background, text,
                  cv::Point(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale + fontHeight)),
                  font, fontScale, cvcolor);
#endif
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                                    unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvCircle(m_background,
                 cvPoint(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                 (int)radius / m_scale, col[color.id], (int)thickness);
#else
        cv::circle(m_background,
                   cv::Point(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                   (int)radius / m_scale, col[color.id], (int)thickness);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvCircle(m_background,
                 cvPoint(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                 (int)radius / m_scale, cvcolor, (int)thickness);
#else
        cv::circle(m_background,
                   cv::Point(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                   (int)radius / m_scale, cvcolor, (int)thickness);
#endif
      }
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      int filled = cv::FILLED;
#else
      int filled = CV_FILLED;
#endif
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvCircle(m_background,
                 cvPoint(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                 (int)radius / m_scale, col[color.id], filled);
#else
        cv::circle(m_background,
                   cv::Point(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                   (int)radius / m_scale, col[color.id], filled);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvCircle(m_background,
                 cvPoint(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                 (int)radius / m_scale, cvcolor, filled);
#else
        cv::circle(m_background,
                   cv::Point(vpMath::round(center.get_u() / m_scale), vpMath::round(center.get_v() / m_scale)),
                   (int)radius / m_scale, cvcolor, filled);
#endif
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayOpenCV::displayCross(const vpImagePoint &ip, unsigned int size, const vpColor &color,
                                   unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    vpImagePoint top, bottom, left, right;
    top.set_i(ip.get_i() - size / 2);
    top.set_j(ip.get_j());
    bottom.set_i(ip.get_i() + size / 2);
    bottom.set_j(ip.get_j());
    left.set_i(ip.get_i());
    left.set_j(ip.get_j() - size / 2);
    right.set_i(ip.get_i());
    right.set_j(ip.get_j() + size / 2);
    displayLine(top, bottom, color, thickness);
    displayLine(left, right, color, thickness);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.

  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayOpenCV::displayDotLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                     unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    double size = 10. * m_scale;
    double length = sqrt(vpMath::sqr(ip2.get_i() - ip1.get_i()) + vpMath::sqr(ip2.get_j() - ip1.get_j()));
    double deltaj = size / length * (ip2.get_j() - ip1.get_j());
    double deltai = size / length * (ip2.get_i() - ip1.get_i());
    double slope = (ip2.get_i() - ip1.get_i()) / (ip2.get_j() - ip1.get_j());
    double orig = ip1.get_i() - slope * ip1.get_j();
    for (unsigned int j = (unsigned int)ip1.get_j(); j < ip2.get_j(); j += (unsigned int)(2 * deltaj)) {
      double i = slope * j + orig;
      displayLine(vpImagePoint(i, j), vpImagePoint(i + deltai, j + deltaj), color, thickness);
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayOpenCV::displayLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                                  unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
      cvLine(m_background, cvPoint(vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale)),
             cvPoint(vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale)), col[color.id],
             (int)thickness);
#else
      cv::line(m_background, cv::Point(vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale)),
               cv::Point(vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale)), col[color.id],
               (int)thickness);
#endif
    } else {
      cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
      cvLine(m_background, cvPoint(vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale)),
             cvPoint(vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale)), cvcolor,
             (int)thickness);
#else
      cv::line(m_background, cv::Point(vpMath::round(ip1.get_u() / m_scale), vpMath::round(ip1.get_v() / m_scale)),
               cv::Point(vpMath::round(ip2.get_u() / m_scale), vpMath::round(ip2.get_v() / m_scale)), cvcolor,
               (int)thickness);
#endif
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : point thickness.
*/
void vpDisplayOpenCV::displayPoint(const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    for (unsigned int i = 0; i < thickness; i++) {
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvLine(m_background, cvPoint(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale)),
               cvPoint(vpMath::round(ip.get_u() / m_scale + thickness - 1), vpMath::round(ip.get_v() / m_scale)),
               col[color.id], (int)thickness);
#else
        cv::line(m_background, cv::Point(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale)),
                 cv::Point(vpMath::round(ip.get_u() / m_scale + thickness - 1), vpMath::round(ip.get_v() / m_scale)),
                 col[color.id], (int)thickness);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvLine(m_background, cvPoint(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale)),
               cvPoint(vpMath::round(ip.get_u() / m_scale + thickness - 1), vpMath::round(ip.get_v() / m_scale)),
               cvcolor, (int)thickness);
#else
        cv::line(m_background, cv::Point(vpMath::round(ip.get_u() / m_scale), vpMath::round(ip.get_v() / m_scale)),
                 cv::Point(vpMath::round(ip.get_u() / m_scale + thickness - 1), vpMath::round(ip.get_v() / m_scale)),
                 cvcolor, (int)thickness);
#endif
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayRectangle(const vpImagePoint &topLeft, unsigned int w, unsigned int h,
                                       const vpColor &color, bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background, cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cvPoint(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            col[color.id], (int)thickness);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            col[color.id], (int)thickness);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background, cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cvPoint(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            cvcolor, (int)thickness);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            cvcolor, (int)thickness);
#endif
      }
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      int filled = cv::FILLED;
#else
      int filled = CV_FILLED;
#endif
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background, cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cvPoint(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            col[color.id], filled);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            col[color.id], filled);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background, cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cvPoint(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            cvcolor, filled);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round((topLeft.get_u() + w) / m_scale), vpMath::round((topLeft.get_v() + h) / m_scale)),
            cvcolor, filled);
#endif
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayRectangle(const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                                       const vpColor &color, bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(m_background,
                    cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
                    cvPoint(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
                    col[color.id], (int)thickness);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
            col[color.id], (int)thickness);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(m_background,
                    cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
                    cvPoint(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
                    cvcolor, (int)thickness);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
            cvcolor, (int)thickness);
#endif
      }
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      int filled = cv::FILLED;
#else
      int filled = CV_FILLED;
#endif
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(m_background,
                    cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
                    cvPoint(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
                    col[color.id], filled);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
            col[color.id], filled);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(m_background,
                    cvPoint(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
                    cvPoint(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
                    cvcolor, filled);
#else
        cv::rectangle(
            m_background, cv::Point(vpMath::round(topLeft.get_u() / m_scale), vpMath::round(topLeft.get_v() / m_scale)),
            cv::Point(vpMath::round(bottomRight.get_u() / m_scale), vpMath::round(bottomRight.get_v() / m_scale)),
            cvcolor, filled);
#endif
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
void vpDisplayOpenCV::displayRectangle(const vpRect &rectangle, const vpColor &color, bool fill, unsigned int thickness)
{
  if (m_displayHasBeenInitialized) {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background,
            cvPoint(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cvPoint(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            col[color.id], (int)thickness);
#else
        cv::rectangle(
            m_background,
            cv::Point(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cv::Point(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            col[color.id], (int)thickness);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background,
            cvPoint(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cvPoint(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            cvcolor, (int)thickness);

#else
        cv::rectangle(
            m_background,
            cv::Point(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cv::Point(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            cvcolor, (int)thickness);

#endif
      }
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      int filled = cv::FILLED;
#else
      int filled = CV_FILLED;
#endif
      if (color.id < vpColor::id_unknown) {
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background,
            cvPoint(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cvPoint(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            col[color.id], filled);
#else
        cv::rectangle(
            m_background,
            cv::Point(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cv::Point(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            col[color.id], filled);
#endif
      } else {
        cvcolor = CV_RGB(color.R, color.G, color.B);
#if VISP_HAVE_OPENCV_VERSION < 0x020408
        cvRectangle(
            m_background,
            cvPoint(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cvPoint(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            cvcolor, filled);
#else
        cv::rectangle(
            m_background,
            cv::Point(vpMath::round(rectangle.getLeft() / m_scale), vpMath::round(rectangle.getBottom() / m_scale)),
            cv::Point(vpMath::round(rectangle.getRight() / m_scale), vpMath::round(rectangle.getTop() / m_scale)),
            cvcolor, filled);
#endif
      }
    }
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
bool vpDisplayOpenCV::getClick(bool blocking)
{
  bool ret = false;
  if (m_displayHasBeenInitialized) {
    flushDisplay();
    if (blocking) {
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown) {
        ret = true;
        lbuttondown = false;
      }
      if (mbuttondown) {
        ret = true;
        mbuttondown = false;
      }
      if (rbuttondown) {
        ret = true;
        rbuttondown = false;
      }
      if (blocking)
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
        cvWaitKey(10);
#else
        cv::waitKey(10);
#endif
    } while (ret == false && blocking == true);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
bool vpDisplayOpenCV::getClick(vpImagePoint &ip, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    flushDisplay();

    double u, v;

    if (blocking) {
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown) {
        ret = true;
        u = (unsigned int)x_lbuttondown * m_scale;
        v = (unsigned int)y_lbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        lbuttondown = false;
      }
      if (mbuttondown) {
        ret = true;
        u = (unsigned int)x_mbuttondown * m_scale;
        v = (unsigned int)y_mbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        mbuttondown = false;
      }
      if (rbuttondown) {
        ret = true;
        u = (unsigned int)x_rbuttondown * m_scale;
        v = (unsigned int)y_rbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        rbuttondown = false;
      }
      if (blocking)
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
        cvWaitKey(10);
#else
        cv::waitKey(10);
#endif
    } while (ret == false && blocking == true);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
bool vpDisplayOpenCV::getClick(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    // flushDisplay() ;
    double u, v;
    if (blocking) {
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown) {
        ret = true;
        u = (unsigned int)x_lbuttondown * m_scale;
        v = (unsigned int)y_lbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button1;
        lbuttondown = false;
      }
      if (mbuttondown) {
        ret = true;
        u = (unsigned int)x_mbuttondown * m_scale;
        v = (unsigned int)y_mbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button2;
        mbuttondown = false;
      }
      if (rbuttondown) {
        ret = true;
        u = (unsigned int)x_rbuttondown * m_scale;
        v = (unsigned int)y_rbuttondown * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button3;
        rbuttondown = false;
      }
      if (blocking)
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
        cvWaitKey(10);
#else
        cv::waitKey(10);
#endif
    } while (ret == false && blocking == true);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
bool vpDisplayOpenCV::getClickUp(vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  bool ret = false;
  if (m_displayHasBeenInitialized) {
    // flushDisplay() ;
    double u, v;
    if (blocking) {
      lbuttonup = false;
      mbuttonup = false;
      rbuttonup = false;
    }
    do {
      if (lbuttonup) {
        ret = true;
        u = (unsigned int)x_lbuttonup * m_scale;
        v = (unsigned int)y_lbuttonup * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button1;
        lbuttonup = false;
      }
      if (mbuttonup) {
        ret = true;
        u = (unsigned int)x_mbuttonup * m_scale;
        v = (unsigned int)y_mbuttonup * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button2;
        mbuttonup = false;
      }
      if (rbuttonup) {
        ret = true;
        u = (unsigned int)x_rbuttonup * m_scale;
        v = (unsigned int)y_rbuttonup * m_scale;
        ip.set_u(u);
        ip.set_v(v);
        button = vpMouseButton::button3;
        rbuttonup = false;
      }
      if (blocking)
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
        cvWaitKey(10);
#else
        cv::waitKey(10);
#endif
    } while (ret == false && blocking == true);
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
  return ret;
}

/*
  Gets the displayed image (including the overlay plane)
  and returns an RGBa image.
*/
void vpDisplayOpenCV::getImage(vpImage<vpRGBa> &I)
{
  vpImageConvert::convert(m_background, I);
  // should certainly be optimized.
}

void vpDisplayOpenCV::on_mouse(int event, int x, int y, int /*flags*/, void *display)
{
  vpDisplayOpenCV *disp = static_cast<vpDisplayOpenCV *>(display);
  switch (event) {
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_MOUSEMOVE:
#else
  case cv::EVENT_MOUSEMOVE:
#endif
  {
    disp->move = true;
    disp->x_move = x;
    disp->y_move = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_LBUTTONDOWN:
#else
  case cv::EVENT_LBUTTONDOWN:
#endif
  {
    disp->lbuttondown = true;
    disp->x_lbuttondown = x;
    disp->y_lbuttondown = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_MBUTTONDOWN:
#else
  case cv::EVENT_MBUTTONDOWN:
#endif
  {
    disp->mbuttondown = true;
    disp->x_mbuttondown = x;
    disp->y_mbuttondown = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_RBUTTONDOWN:
#else
  case cv::EVENT_RBUTTONDOWN:
#endif
  {
    disp->rbuttondown = true;
    disp->x_rbuttondown = x;
    disp->y_rbuttondown = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_LBUTTONUP:
#else
  case cv::EVENT_LBUTTONUP:
#endif
  {
    disp->lbuttonup = true;
    disp->x_lbuttonup = x;
    disp->y_lbuttonup = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_MBUTTONUP:
#else
  case cv::EVENT_MBUTTONUP:
#endif
  {
    disp->mbuttonup = true;
    disp->x_mbuttonup = x;
    disp->y_mbuttonup = y;
    break;
  }
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  case CV_EVENT_RBUTTONUP:
#else
  case cv::EVENT_RBUTTONUP:
#endif
  {
    disp->rbuttonup = true;
    disp->x_rbuttonup = x;
    disp->y_rbuttonup = y;
    break;
  }

  default:
    break;
  }
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
*/
bool vpDisplayOpenCV::getKeyboardEvent(bool blocking)
{
  if (m_displayHasBeenInitialized) {
    int delay;
    flushDisplay();
    if (blocking)
      delay = 0;
    else
      delay = 10;

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int key_pressed = cvWaitKey(delay);
#else
    int key_pressed = cv::waitKey(delay);
#endif

    if (key_pressed == -1)
      return false;
    return true;
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
  // return false; // Never reached after throw()
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
*/
bool vpDisplayOpenCV::getKeyboardEvent(std::string &key, bool blocking)
{
  if (m_displayHasBeenInitialized) {
    int delay;
    flushDisplay();
    if (blocking)
      delay = 0;
    else
      delay = 10;

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    int key_pressed = cvWaitKey(delay);
#else
    int key_pressed = cv::waitKey(delay);
#endif
    if (key_pressed == -1)
      return false;
    else {
      // std::cout << "Key pressed: \"" << key_pressed << "\"" << std::endl;
      std::stringstream ss;
      ss << key_pressed;
      key = ss.str();
    }
    return true;
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
  // return false; // Never reached after throw()
}

/*!
  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.
*/
bool vpDisplayOpenCV::getPointerMotionEvent(vpImagePoint &ip)
{
  bool ret = false;

  if (m_displayHasBeenInitialized) {
    // flushDisplay() ;
    if (move) {
      ret = true;
      double u = (unsigned int)x_move / m_scale;
      double v = (unsigned int)y_move / m_scale;
      ip.set_u(u);
      ip.set_v(v);
      move = false;
    }
  }

  else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
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
bool vpDisplayOpenCV::getPointerPosition(vpImagePoint &ip)
{
  if (m_displayHasBeenInitialized) {
    bool moved = getPointerMotionEvent(ip);
    if (!moved) {
      double u, v;
      u = (unsigned int)x_move / m_scale;
      v = (unsigned int)y_move / m_scale;
      ip.set_u(u);
      ip.set_v(v);
    }
    return false;
  } else {
    throw(vpDisplayException(vpDisplayException::notInitializedError, "OpenCV not initialized"));
  }
}

/*!
  Gets screen resolution in pixels.
  \param w, h : Horizontal and vertical screen resolution.
 */
void vpDisplayOpenCV::getScreenSize(unsigned int &w, unsigned int &h)
{
  w = h = 0;

#if defined(VISP_HAVE_X11)
  vpDisplayX d;
  d.getScreenSize(w, h);
#elif defined(VISP_HAVE_XRANDR)
  std::string command = "xrandr | grep '*'";
  FILE *fpipe = (FILE *)popen(command.c_str(), "r");
  char line[256];
  while (fgets(line, sizeof(line), fpipe)) {
    std::string str(line);
    std::size_t found = str.find("Failed");

    if (found == std::string::npos) {
      std::vector<std::string> elm;
      elm = vpIoTools::splitChain(str, " ");
      for (size_t i = 0; i < elm.size(); i++) {
        if (!elm[i].empty()) {
          std::vector<std::string> resolution = vpIoTools::splitChain(elm[i], "x");
          if (resolution.size() == 2) {
            std::istringstream sswidth(resolution[0]), ssheight(resolution[1]);
            sswidth >> w;
            ssheight >> h;
            break;
          }
        }
      }
    }
  }
  pclose(fpipe);
#elif defined(_WIN32)
#if !defined(WINRT)
  w = GetSystemMetrics(SM_CXSCREEN);
  h = GetSystemMetrics(SM_CYSCREEN);
#else
  throw(vpException(vpException::functionNotImplementedError, "The function vpDisplayOpenCV::getScreenSize() is not "
                                                              "implemented on winrt"));
#endif
#endif
}

/*!
  Gets the screen horizontal resolution in pixels.
 */
unsigned int vpDisplayOpenCV::getScreenWidth()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return width;
}

/*!
  Gets the screen vertical resolution in pixels.
 */
unsigned int vpDisplayOpenCV::getScreenHeight()
{
  unsigned int width, height;
  getScreenSize(width, height);
  return height;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDisplayOpenCV.cpp.o) has no
// symbols
void dummy_vpDisplayOpenCV(){};
#endif
