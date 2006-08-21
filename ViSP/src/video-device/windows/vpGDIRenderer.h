/****************************************************************************
 *
 * $Id: vpGDIRenderer.h,v 1.2 2006-08-21 10:02:43 brenier Exp $
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

#if ( defined(WIN32) ) 
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
	bool init(HWND hWnd, int width, int height);

	//renders on the window's DC.
	bool render();

	// gets the image's width.
	int getImageWidth(){ return nbCols; }

	// gets the image's height.
	int getImageHeight(){ return nbRows; }

	// sets the image to display.
	void setImg(vpImage<vpRGBa>& im);
	void setImg(vpImage<unsigned char>& im);

	//Draws a pixel of color color at (x,y).
	void setPixel(int y, int x, int color);

	//other drawing methods
	void drawLine(int i1, int j1, int i2, int j2, int col, int e, int style=PS_SOLID);

	void drawRect(int i, int j, int width, int height, int col, bool fill=false);

	void clear(int c);

	void drawCircle(int i, int j, int r, int c);

	void drawText(int i, int j, char * s, int c);

	void drawCross(int i,int j, int size, int col, int e=1);

	void drawArrow(int i1,int j1, int i2, int j2, int col, int L,int l);


	// returns the currently displayed image.
	void getImage(vpImage<vpRGBa> &I);

private:

	//updates the renderer hbitmaps.
	bool updateBitmap(HBITMAP& hBmp, unsigned char * imBuffer, int w, int h);

	//converts a vpImage<vpRGBa> into a HBITMAP .
	void convert(vpImage<vpRGBa> &I, HBITMAP& hBmp);

	//converst a vpImage<unsigned char> into a HBITMAP .
	void convert(vpImage<unsigned char> &I, HBITMAP& hBmp);

};
#endif
#endif
#endif
