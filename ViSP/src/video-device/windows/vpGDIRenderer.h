/****************************************************************************
 *
 * $Id: vpGDIRenderer.h,v 1.4 2007-02-26 17:26:45 fspindle Exp $
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
 * GDI renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#if ( defined(VISP_HAVE_GDI) )
#ifndef vpGDIRenderer_HH
#define vpGDIRenderer_HH


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <windows.h>

#include <visp/vpWin32Renderer.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpDisplayException.h>

#include <visp/vpMath.h>

class VISP_EXPORT vpGDIRenderer : public vpWin32Renderer
{
  //the handle of the associated window
  HWND hWnd;

  //the bitmap object to display
  HBITMAP bmp;

  //colors for overlay
  COLORREF colors[8];

  //font used to draw text
  HFONT hFont;

  //used to ensure that only one thread at a time is accessing bmp
  CRITICAL_SECTION CriticalSection;

 public:
  vpGDIRenderer();
  ~vpGDIRenderer();

  //inits the display.
  bool init(HWND hWnd, unsigned width, unsigned height);

  //renders on the window's DC.
  bool render();

  // gets the image's width.
  unsigned getImageWidth(){ return nbCols; }

  // gets the image's height.
  unsigned getImageHeight(){ return nbRows; }

  // sets the image to display.
  void setImg(const vpImage<vpRGBa>& im);
  void setImg(const vpImage<unsigned char>& im);

  //Draws a pixel of color color at (x,y).
  void setPixel(unsigned y, unsigned x, vpColor::vpColorType color);

  //other drawing methods
  void drawLine(unsigned i1, unsigned j1, unsigned i2, unsigned j2, 
		vpColor::vpColorType col, unsigned e, int style=PS_SOLID);

  void drawRect(unsigned i, unsigned j, unsigned width, unsigned height, 
		vpColor::vpColorType col, bool fill=false);

  void clear(vpColor::vpColorType c);

  void drawCircle(unsigned i, unsigned j, unsigned r, vpColor::vpColorType c);

  void drawText(unsigned i, unsigned j, char * s, vpColor::vpColorType c);

  void drawCross(unsigned i,unsigned j, unsigned size, 
		 vpColor::vpColorType col, unsigned e=1);

  void drawArrow(unsigned i1,unsigned j1, unsigned i2, unsigned j2, 
		 vpColor::vpColorType col, unsigned L,unsigned l);


  // returns the currently displayed image.
  void getImage(vpImage<vpRGBa> &I);

 private:

  //updates the renderer hbitmaps.
  bool updateBitmap(HBITMAP& hBmp, unsigned char * imBuffer, 
		    unsigned w, unsigned h);

  //converts a vpImage<vpRGBa> into a HBITMAP .
  void convert(const vpImage<vpRGBa> &I, HBITMAP& hBmp);

  //converst a vpImage<unsigned char> into a HBITMAP .
  void convert(const vpImage<unsigned char> &I, HBITMAP& hBmp);

};
#endif
#endif
#endif
