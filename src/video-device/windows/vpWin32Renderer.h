/****************************************************************************
 *
 * $Id: vpWin32Renderer.h,v 1.1 2006-07-18 14:43:30 brenier Exp $
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

class VISP_EXPORT vpWin32Renderer
{

protected:
	//the size of the display
	int nbCols;
	int nbRows;

public:
	virtual~vpWin32Renderer() {};

	int getImageWidth(){ return nbCols; }
	int getImageHeight(){ return nbRows; }



	//init the display 
	virtual bool init(HWND hWnd, int w, int h) =0;

	//render on the window's DC
	virtual bool render() =0;

	

	virtual void setImg(vpImage<vpRGBa>& im) =0;

	virtual void setImg(vpImage<unsigned char>& im) =0;

	virtual void setPixel(int x, int y, int color) =0;

	virtual void drawLine(int i1, int j1, int i2, int j2, int col, int e, int style=PS_SOLID) =0;

	virtual void drawRect(int i, int j, int width, int height, int col, bool fill=false) =0;

	virtual void drawCircle(int i, int j, int r, int c) =0;

	virtual void drawText(int i, int j, char * s, int c) =0;

	virtual void drawCross(int i,int j, int size, int col, int e=1) =0;

	virtual void drawArrow(int i1,int j1, int i2, int j2, int col, int L,int l) =0;

	virtual void getImage(vpImage<vpRGBa> &I) =0;
};

#endif
#endif
#endif
