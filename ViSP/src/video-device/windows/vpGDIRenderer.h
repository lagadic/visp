/****************************************************************************
 *
 * $Id$
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
  COLORREF colors[vpColor::none];

  //font used to draw text
  HFONT hFont;

  //used to ensure that only one thread at a time is accessing bmp
  CRITICAL_SECTION CriticalSection;

 public:
  vpGDIRenderer();
  virtual ~vpGDIRenderer();

  bool init(HWND hWnd, unsigned int width, unsigned int height);

  bool render();

  // gets the image's width.
  unsigned int getImageWidth(){ return nbCols; }

  // gets the image's height.
  unsigned int getImageHeight(){ return nbRows; }

  void setImg(const vpImage<vpRGBa>& im);
  void setImg(const vpImage<unsigned char>& im);

  void setPixel(const vpImagePoint iP, vpColor color);

  void drawLine(const vpImagePoint &ip1, 
		const vpImagePoint &ip2,
		vpColor color, unsigned int thickness, int style=PS_SOLID);

  void drawRect(const vpImagePoint &topLeft,
		unsigned int width, unsigned int height,
		vpColor color, bool fill=false,
		unsigned int thickness=1);

  void clear(vpColor color);

  void drawCircle(const vpImagePoint &center, unsigned int radius,
		  vpColor color, bool fill=false, unsigned char thickness=1);

  void drawText(const vpImagePoint &ip, const char * text,
		vpColor color);

  void drawCross(const vpImagePoint &ip, unsigned int size,
		 vpColor color, unsigned int thickness=1);

  void drawArrow(const vpImagePoint &ip1, 
		 const vpImagePoint &ip2,
		 vpColor color, unsigned int w,unsigned int h, unsigned int thickness=1);

  void getImage(vpImage<vpRGBa> &I);

 private:

  //updates the renderer hbitmaps.
  bool updateBitmap(HBITMAP& hBmp, unsigned char * imBuffer,
		    unsigned int w, unsigned int h);

  //converts a vpImage<vpRGBa> into a HBITMAP .
  void convert(const vpImage<vpRGBa> &I, HBITMAP& hBmp);

  //converst a vpImage<unsigned char> into a HBITMAP .
  void convert(const vpImage<unsigned char> &I, HBITMAP& hBmp);

};
#endif
#endif
#endif
