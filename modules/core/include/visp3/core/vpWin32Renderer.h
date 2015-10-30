/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#if ( defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) )

#ifndef vpWin32Renderer_HH
#define vpWin32Renderer_HH

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpImage.h>
#include <visp3/core/vpColor.h>
#include <windows.h>
#include <visp3/core/vpDebug.h>

class VISP_EXPORT vpWin32Renderer
{

 protected:
  //the size of the display
  unsigned int nbCols;
  unsigned int nbRows;

 public:
  //! Destructor.
  virtual ~vpWin32Renderer() {};

  //! Returns the image dimensions.
  unsigned int getImageWidth(){ return nbCols; }
  unsigned int getImageHeight(){ return nbRows; }



  //! Inits the display .
  virtual bool init(HWND hWnd, unsigned int w, unsigned int h) =0;

  //! Renders the image.
  virtual bool render() =0;


  /*!
    Sets the image to display.
    \param im The image to display.
  */
  virtual void setImg(const vpImage<vpRGBa>& im) =0;
  virtual void setImg(const vpImage<unsigned char>& im) =0;
  virtual void setImgROI(const vpImage<vpRGBa>& im, const vpImagePoint &iP, const unsigned int width, const unsigned int height ) =0;
  virtual void setImgROI(const vpImage<unsigned char>& im, const vpImagePoint &iP, const unsigned int width, const unsigned int height ) =0;

  /*!
    Sets the pixel at (x,y).
    \param y The y coordinate of the pixel.
    \param x The x coordinate of the pixel.
    \param color The color of the pixel.
  */
  virtual void setPixel(const vpImagePoint &iP, const vpColor &color) =0;

  /*!
    Draws a line.
    \param i1 its starting point's first coordinate
    \param j1 its starting point's second coordinate
    \param i2 its ending point's first coordinate
    \param j2 its ending point's second coordinate
    \param e line thickness
    \param col the line's color
    \param style style of the line
  */
  virtual void drawLine(const vpImagePoint &ip1, 
		const vpImagePoint &ip2,
		const vpColor &color, unsigned int thickness, int style=PS_SOLID) =0;

  /*!
    Draws a rectangle.
    \param i its top left point's first coordinate
    \param j its top left point's second coordinate
    \param width width of the rectangle
    \param height height of the rectangle
    \param col The rectangle's color
    \param fill True if it is a filled rectangle
    \param e line thickness
  */
  virtual void drawRect(const vpImagePoint &topLeft,
		unsigned int width, unsigned int height,
		const vpColor &color, bool fill=false,
		unsigned int thickness=1) =0;

  /*!
    Clears the image to color c.
    \param c The color used to fill the image.
  */
  virtual void clear(const vpColor &color) =0;

  /*!
    Draws a circle.
    \param i its center point's first coordinate
    \param j its center point's second coordinate
    \param r The circle's radius
    \param col The circle's color
  */
  virtual void drawCircle(const vpImagePoint &center, unsigned int radius,
		  const vpColor &color, bool fill, unsigned int thickness=1) =0;

  /*!
    Draws some text.
    \param i its top left point's first coordinate
    \param j its top left point's second coordinate
    \param s The string to display
    \param col The text's color
  */
  virtual void drawText(const vpImagePoint &ip, const char * text,
		const vpColor &color) =0;

  /*!
    Draws a cross.
    \param i its center point's first coordinate
    \param j its center point's second coordinate
    \param size Size of the cross
    \param col The cross' color
    \param e width of the cross
  */
  virtual void drawCross(const vpImagePoint &ip, unsigned int size,
		 const vpColor &color, unsigned int thickness=1) =0;

  /*!
    Draws an arrow.
    \param i1 its starting point's first coordinate
    \param j1 its starting point's second coordinate
    \param i2 its ending point's first coordinate
    \param j2 its ending point's second coordinate
    \param color The line's color
    \param L ...
    \param l ...
  */
  virtual void drawArrow(const vpImagePoint &ip1, 
		 const vpImagePoint &ip2,
		 const vpColor &color, unsigned int w,unsigned int h, unsigned int thickness) =0;

  /*!
    Gets the currently displayed image.
    \param I Image returned.
  */
  virtual void getImage(vpImage<vpRGBa> &I) =0;
};

#endif
#endif
#endif
