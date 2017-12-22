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
 * Windows 32 renderer base class
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

#ifndef vpWin32Renderer_HH
#define vpWin32Renderer_HH

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <visp3/core/vpDebug.h>
#include <windows.h>

class VISP_EXPORT vpWin32Renderer
{

protected:
  // the size of the display
  unsigned int m_rwidth;
  unsigned int m_rheight;
  unsigned int m_rscale;

public:
  vpWin32Renderer() : m_rwidth(0), m_rheight(0), m_rscale(1){};
  //! Destructor.
  virtual ~vpWin32Renderer(){};

  //! Inits the display .
  virtual bool init(HWND hWnd, unsigned int w, unsigned int h) = 0;

  //! Renders the image.
  virtual bool render() = 0;

  /*!
    Sets the image to display.
    \param im The image to display.
  */
  virtual void setImg(const vpImage<vpRGBa> &im) = 0;
  virtual void setImg(const vpImage<unsigned char> &im) = 0;
  virtual void setImgROI(const vpImage<vpRGBa> &im, const vpImagePoint &iP, const unsigned int width,
                         const unsigned int height) = 0;
  virtual void setImgROI(const vpImage<unsigned char> &im, const vpImagePoint &iP, const unsigned int width,
                         const unsigned int height) = 0;

  /*!
    Sets the pixel at (x,y).
    \param iP The coordinates of the pixel.
    \param color The color of the pixel.
  */
  virtual void setPixel(const vpImagePoint &iP, const vpColor &color) = 0;

  void setScale(unsigned int scale) { m_rscale = scale; }
  void setHeight(unsigned int height) { m_rheight = height; }
  void setWidth(unsigned int width) { m_rwidth = width; }

  /*!
    Draws a line.
    \param ip1 it's starting point coordinates
    \param ip2 it's ending point coordinates
    \param color the line's color
    \param thickness line thickness
    \param style style of the line
  */
  virtual void drawLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness,
                        int style = PS_SOLID) = 0;

  /*!
    Draws a rectangle.
    \param topLeft it's top left point coordinates
    \param width width of the rectangle
    \param height height of the rectangle
    \param color The rectangle's color
    \param fill True if it is a filled rectangle
    \param thickness line thickness
  */
  virtual void drawRect(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                        bool fill = false, unsigned int thickness = 1) = 0;

  /*!
    Clears the image to color c.
    \param color The color used to fill the image.
  */
  virtual void clear(const vpColor &color) = 0;

  /*!
    Draws a circle.
    \param center its center point coordinates
    \param radius The circle's radius
    \param color The circle's color
    \param fill When true fill the circle with the given color
    \param thickness Drawing thickness
  */
  virtual void drawCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                          unsigned int thickness = 1) = 0;

  /*!
    Draws some text.
    \param ip it's top left point coordinates
    \param text The string to display
    \param color The text's color
  */
  virtual void drawText(const vpImagePoint &ip, const char *text, const vpColor &color) = 0;

  /*!
    Draws a cross.
    \param ip it's center point coordinates
    \param size Size of the cross
    \param color The cross' color
    \param thickness Thickness of the drawing
  */
  virtual void drawCross(const vpImagePoint &ip, unsigned int size, const vpColor &color,
                         unsigned int thickness = 1) = 0;

  /*!
    Draws an arrow.
    \param ip1 it's starting point coordinates
    \param ip2 it's ending point coordinates
    \param color The line's color
    \param w,h Width and height of the arrow
    \param thickness Thickness of the drawing
  */
  virtual void drawArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w,
                         unsigned int h, unsigned int thickness) = 0;

  /*!
    Gets the currently displayed image.
    \param I Image returned.
  */
  virtual void getImage(vpImage<vpRGBa> &I) = 0;
};

#endif
#endif
#endif
