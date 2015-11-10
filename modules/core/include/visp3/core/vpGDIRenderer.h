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
 * GDI renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if ( defined(VISP_HAVE_GDI) )
#ifndef vpGDIRenderer_HH
#define vpGDIRenderer_HH


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <windows.h>

#include <visp3/gui/vpWin32Renderer.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpDisplayException.h>

#include <visp3/core/vpMath.h>

class VISP_EXPORT vpGDIRenderer : public vpWin32Renderer
{
  //the handle of the associated window
  HWND hWnd;

  //the bitmap object to display
  HBITMAP bmp;

  //colors for overlay
  COLORREF colors[vpColor::id_unknown];

  //font used to draw text
  HFONT hFont;

  //used to ensure that only one thread at a time is accessing bmp
  CRITICAL_SECTION CriticalSection;

 public:
  double timelost;
  vpGDIRenderer();
  virtual ~vpGDIRenderer();

  bool init(HWND hWnd, unsigned int width, unsigned int height);

  bool render();

  // gets the image's width.
  unsigned int getImageWidth(){ return nbCols; }

  // gets the image's height.
  unsigned int getImageHeight(){ return nbRows; }

  void setImg(const vpImage<vpRGBa>& I);
  void setImg(const vpImage<unsigned char>& I);
  void setImgROI(const vpImage<vpRGBa>& I, const vpImagePoint &iP, const unsigned int width, const unsigned int height );
  void setImgROI(const vpImage<unsigned char>& I, const vpImagePoint &iP, const unsigned int width, const unsigned int height );

  void setPixel(const vpImagePoint &iP, const vpColor &color);

  void drawLine(const vpImagePoint &ip1, 
		const vpImagePoint &ip2,
		const vpColor &color, unsigned int thickness, int style=PS_SOLID);

  void drawRect(const vpImagePoint &topLeft,
		unsigned int width, unsigned int height,
		const vpColor &color, bool fill=false,
		unsigned int thickness=1);

  void clear(const vpColor &color);

  void drawCircle(const vpImagePoint &center, unsigned int radius,
		  const vpColor &color, bool fill=false, unsigned int thickness=1);

  void drawText(const vpImagePoint &ip, const char * text,
		const vpColor &color);

  void drawCross(const vpImagePoint &ip, unsigned int size,
		 const vpColor &color, unsigned int thickness=1);

  void drawArrow(const vpImagePoint &ip1, 
		 const vpImagePoint &ip2,
		 const vpColor &color, unsigned int w,unsigned int h, unsigned int thickness=1);

  void getImage(vpImage<vpRGBa> &I);

 private:

  //updates the renderer hbitmaps.
  bool updateBitmap(HBITMAP& hBmp, unsigned char * imBuffer,
		    unsigned int w, unsigned int h);
  //updates the renderer hbitmaps.
  bool updateBitmapROI(unsigned char * imBuffer, const vpImagePoint &iP,
		    unsigned int w, unsigned int h);


  //converts a vpImage<vpRGBa> into a HBITMAP .
  void convert(const vpImage<vpRGBa> &I, HBITMAP& hBmp);

  //converst a vpImage<unsigned char> into a HBITMAP .
  void convert(const vpImage<unsigned char> &I, HBITMAP& hBmp);

  //converts a vpImage<vpRGBa> into a HBITMAP .
  void convertROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width, const unsigned int height);

  //converst a vpImage<unsigned char> into a HBITMAP .
  void convertROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width, const unsigned int height);

};
#endif
#endif
#endif
