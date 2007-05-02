/****************************************************************************
 *
 * $Id: vpD3DRenderer.h,v 1.6 2007-05-02 13:29:41 fspindle Exp $
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
 * D3D renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_D3D9) )

#ifndef VPD3DRENDERER_HH
#define VPD3DRENDERER_HH

#include <windows.h>
#include <d3dx9.h>
#include <visp/vpWin32Renderer.h>
#include <visp/vpDisplayException.h>


#include <iostream>

/*!
  \class vpD3DRenderer.h

  \brief Display under windows using Direct3D9.
  Is used by vpD3DDisplay to do the drawing.

*/
class VISP_EXPORT vpD3DRenderer : public vpWin32Renderer
{

  IDirect3D9 * pD3D;

  //The d3d device we will be working with.
  IDirect3DDevice9 * pd3dDevice;

  //Sprite used to render the texture.
  ID3DXSprite * pSprite;

  //The system memory texture :
  //The one we will be drawing on.
  IDirect3DTexture9 * pd3dText;

  //The video memory texture :
  //The one we will use for display.
  IDirect3DTexture9 * pd3dVideoText;

  //The texture's width.
  int textWidth;

  //The window's handle.
  HWND hWnd;

  //Colors  for overlay drawn with d3d directly.
  long colors[8];

  //Colors for overlay drawn with GDI.
  COLORREF colorsGDI[8];

  //Font used for text drawing.
  HFONT hFont;

 public:

  bool init(HWND hwnd, unsigned int width, unsigned int height);
  bool render();

  vpD3DRenderer();
  virtual ~vpD3DRenderer();

  void setImg(const vpImage<vpRGBa>& im);

  void setImg(const vpImage<unsigned char>& im);

  void setPixel(unsigned int x, unsigned int y, vpColor::vpColorType color);

  void drawLine(unsigned int i1, unsigned int j1,
		unsigned int i2, unsigned int j2,
		vpColor::vpColorType col, unsigned int e, int style=PS_SOLID);

  void drawRect(unsigned int i, unsigned int j,
		unsigned int width, unsigned int height,
		vpColor::vpColorType col, bool fill=false);

  void clear(vpColor::vpColorType c);

  void drawCircle(unsigned int i, unsigned int j, unsigned int r,
		  vpColor::vpColorType c);

  void drawText(unsigned int i, unsigned int j, char * s,
		vpColor::vpColorType c);

  void drawCross(unsigned int i,unsigned int j, unsigned int size,
		 vpColor::vpColorType col, unsigned int e=1);

  void drawArrow(unsigned int i1,unsigned int j1,
		 unsigned int i2, unsigned int j2,
		 vpColor::vpColorType col, unsigned int L,unsigned int l);

  void getImage(vpImage<vpRGBa> &I);



 private:

  void initView(float, float);

  /*!
    Sub function for circle drawing.
    Circle drawing is based on Bresenham 's circle algorithm.
  */
  void subDrawCircle(unsigned int i, unsigned int j,
		     unsigned int x, unsigned int y,
		     vpColor::vpColorType col, unsigned char* buf,
		     unsigned int pitch, unsigned int maxX, unsigned int maxY);


  /*!
    Useful inline function to set a pixel in a texture buffer.
    \param buf The texture's buffer.
    \param pitch The image pitch.
    \param x The x-coordinate of the pixel (in the locked rectangle base)
    \param y The y-coordinate of the pixel (in the locked rectangle base)
    \param color The color of the pixel.
    \param maxX The maximum x value (equals to the width of the locked rectangle).
    \param maxY The maximum y value (equals to the height of the locked rectangle).

  */
  inline void setBufferPixel(unsigned char* buf, unsigned int pitch,
			     unsigned int x, unsigned int y,
			     vpColor::vpColorType color,
			     unsigned int maxX, unsigned int maxY)
    {
      if(x>=0 && y>=0 && x<=maxX && y<=maxY)
	*(long*)(buf + (y*pitch) + (x<<2)) = colors[color];
    }

  int supPowerOf2(int n);

};
#endif
#endif
#endif
