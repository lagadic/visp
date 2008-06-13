/****************************************************************************
 *
 * $Id: vpWin32Renderer.h,v 1.8 2008-06-13 13:37:38 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Windows 32 renderer base class
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#if ( defined(WIN32) )

#ifndef vpWin32Renderer_HH
#define vpWin32Renderer_HH

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <windows.h>
#include <visp/vpDebug.h>

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

  /*!
    Sets the pixel at (x,y).
    \param y The y coordinate of the pixel.
    \param x The x coordinate of the pixel.
    \param color The color of the pixel.
  */
  virtual void setPixel(int y,int x,
			vpColor::vpColorType color) =0;

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
  virtual void drawLine(int i1, int j1,
			int i2, int j2,
			vpColor::vpColorType col, unsigned int e,
			int style=PS_SOLID) =0;

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
  virtual void drawRect(int i, int j,
			unsigned int width, unsigned int height,
			vpColor::vpColorType col, bool fill=false,
			unsigned int e=1) =0;

  /*!
    Clears the image to color c.
    \param c The color used to fill the image.
  */
  virtual void clear(vpColor::vpColorType c) =0;

  /*!
    Draws a circle.
    \param i its center point's first coordinate
    \param j its center point's second coordinate
    \param r The circle's radius
    \param col The circle's color
  */
  virtual void drawCircle(int i, int j, unsigned int r,
			  vpColor::vpColorType c) =0;

  /*!
    Draws some text.
    \param i its top left point's first coordinate
    \param j its top left point's second coordinate
    \param s The string to display
    \param col The text's color
  */
  virtual void drawText(int i, int j, const char * s,
			vpColor::vpColorType c) =0;

  /*!
    Draws a cross.
    \param i its center point's first coordinate
    \param j its center point's second coordinate
    \param size Size of the cross
    \param col The cross' color
    \param e width of the cross
  */
  virtual void drawCross(int i, int j, unsigned int size,
			 vpColor::vpColorType col, unsigned int e=1) =0;

  /*!
    Draws an arrow.
    \param i1 its starting point's first coordinate
    \param j1 its starting point's second coordinate
    \param i2 its ending point's first coordinate
    \param j2 its ending point's second coordinate
    \param col The line's color
    \param L ...
    \param l ...
  */
  virtual void drawArrow(int i1, int j1,
			 int i2, int j2,
			 vpColor::vpColorType col,
			 unsigned int L,unsigned int l) =0;

  /*!
    Gets the currently displayed image.
    \param I Image returned.
  */
  virtual void getImage(vpImage<vpRGBa> &I) =0;
};

#endif
#endif
#endif
