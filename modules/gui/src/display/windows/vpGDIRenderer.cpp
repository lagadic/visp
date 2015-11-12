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
#define GDI_ROBUST
#if ( defined(VISP_HAVE_GDI) )


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpGDIRenderer.h>

/*!
  Constructor.
*/
vpGDIRenderer::vpGDIRenderer()
{
  //if the screen depth is not 32bpp, throw an exception
  int bpp = GetDeviceCaps(GetDC(NULL), BITSPIXEL);
  if( bpp != 32 )
    throw vpDisplayException(vpDisplayException::depthNotSupportedError,
           "vpGDIRenderer supports only 32bits depth: screen is %dbits depth!", bpp);

  InitializeCriticalSection(&CriticalSection);

  //initialize GDI the palette
  vpColor pcolor; // Predefined colors
  
  pcolor = vpColor::black;
  colors[vpColor::id_black] =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightBlue;
  colors[vpColor::id_lightBlue]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::blue;
  colors[vpColor::id_blue]  =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkBlue;
  colors[vpColor::id_darkBlue]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::cyan;
  colors[vpColor::id_cyan]  =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGreen;
  colors[vpColor::id_lightGreen]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::green;
  colors[vpColor::id_green] =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGreen;
  colors[vpColor::id_darkGreen]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightRed;
  colors[vpColor::id_lightRed]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::red;
  colors[vpColor::id_red]   =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkRed;
  colors[vpColor::id_darkRed]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::white;
  colors[vpColor::id_white] =  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGray;
  colors[vpColor::id_lightGray]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::gray;
  colors[vpColor::id_gray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGray;
  colors[vpColor::id_darkGray]  = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::yellow;
  colors[vpColor::id_yellow]=  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::orange;
  colors[vpColor::id_orange]=  RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::purple;
  colors[vpColor::id_purple]= RGB(pcolor.R, pcolor.G, pcolor.B);

  nbCols = 0;
  nbRows = 0;
  bmp = NULL;
}

/*!
  Destructor.
*/
vpGDIRenderer::~vpGDIRenderer()
{
  //Deletes the critical section object
  DeleteCriticalSection(&CriticalSection);
  //Deletes the bitmap
  DeleteObject(bmp);
  //Deletes the font object
  DeleteObject(hFont);
}

/*!
  Initialiaze the renderer
  \param hWindow Handle of the window we are working with
  \param width The window's width.
  \param height The window's height.
*/
bool vpGDIRenderer::init(HWND hWindow, unsigned int width, unsigned int height)
{
  timelost = 0.;
  hWnd = hWindow;
  nbCols = width;
  nbRows = height;

  //creates the font
  hFont = CreateFont(18, 0, 0, 0, FW_NORMAL, false, false, false,
		     DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
		     CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
		     DEFAULT_PITCH | FF_DONTCARE, NULL);
  return true;
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
*/
void vpGDIRenderer::setImg(const vpImage<vpRGBa>& I)
{
  //converts the image into a HBITMAP
  convert(I, bmp);
  //updates the size of the image
  nbCols=I.getWidth();
  nbRows=I.getHeight();
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::setImgROI(const vpImage<vpRGBa>& I, const vpImagePoint &iP, const unsigned int width, const unsigned int height )
{
  //converts the image into a HBITMAP
  convertROI(I, iP, width, height);
  //updates the size of the image
  nbCols=I.getWidth();
  nbRows=I.getHeight();
}

/*!
  Sets the image to display.
  \param I : The grayscale image to display.
*/
void vpGDIRenderer::setImg(const vpImage<unsigned char>& I)
{
  //converts the image into a HBITMAP
  convert(I, bmp);
  //updates the size of the image
  nbCols=I.getWidth();
  nbRows=I.getHeight();
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::setImgROI(const vpImage<unsigned char>& I, const vpImagePoint &iP, const unsigned int width, const unsigned int height )
{
  //converts the image into a HBITMAP
  convertROI(I, iP, width, height);
  //updates the size of the image
  nbCols=I.getWidth();
  nbRows=I.getHeight();
}

/*!
  Render the current image buffer.
*/
bool vpGDIRenderer::render()
{
  //gets the window's DC
  PAINTSTRUCT ps;
  HDC hDCScreen = BeginPaint(hWnd, &ps);

  //create a memory DC
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //selects this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);
  

  //blits it on the window's DC
  BitBlt(hDCScreen, 0, 0,
    static_cast<int>( nbCols ),
    static_cast<int>( nbRows ),
    hDCMem, 0, 0, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);
  //DeleteDC(hDCMem);
  DeleteObject(hDCMem);

  EndPaint(hWnd, &ps);

  return true;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param hBmp The destination image.
*/
void vpGDIRenderer::convert(const vpImage<vpRGBa> &I, HBITMAP& hBmp)
{
  //get the image's width and height
  unsigned int w = I.getWidth();
  unsigned int h = I.getHeight();

  //each line of a HBITMAP needs to be word aligned
  //we need padding if the width is an odd number
  bool needPad = ((w%2) == 0) ? false : true;
  unsigned int newW = w;

  //in case of padding, the new width is width+1
  newW = (needPad) ? (w+1) : w;

  //allocate the buffer
  unsigned char * imBuffer = new unsigned char[newW * h * 4];

  //if we need padding (width needs to be a multiple of 2)
  if(needPad)
    {
      unsigned int j = 0;
      for(unsigned int i=0, k=0 ; i<newW * h * 4; i+=4, k++)
	{
	  //end of a line = padding = inserts 0s
	  if(j==w && needPad)
	    {
	      imBuffer[i+0] = 0;
	      imBuffer[i+1] = 0;
	      imBuffer[i+2] = 0;
	      imBuffer[i+3] = 0;
	      j = 0;
	      k --;
	    }
	  else
	    {
	      //RGBA -> BGRA
	      imBuffer[i+0] = I.bitmap[k].B;
	      imBuffer[i+1] = I.bitmap[k].G;
	      imBuffer[i+2] = I.bitmap[k].R;
	      imBuffer[i+3] = I.bitmap[k].A;
	      j++;
	    }
	}
    }
  else
    //Simple conversion (no padding)
    {
      for(unsigned int i=0, k=0 ; i<w * h * 4 ; i+=4, k++)
	{
	  imBuffer[i+0] = I.bitmap[k].B;
	  imBuffer[i+1] = I.bitmap[k].G;
	  imBuffer[i+2] = I.bitmap[k].R;
	  imBuffer[i+3] = I.bitmap[k].A;
	}
    }

  //updates the bitmap's pixel data
  updateBitmap(hBmp,imBuffer, newW, h);

  //we don't need this buffer anymore
  delete [] imBuffer;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I : The image to convert.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::convertROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width, const unsigned int height)
{
  //get the image's width and height
  unsigned int w = width;
  unsigned int h = height;

  //each line of a HBITMAP needs to be word aligned
  //we need padding if the width is an odd number
  bool needPad = ((w%2) == 0) ? false : true;
  unsigned int newW = w;

  //in case of padding, the new width is width+1
  newW = (needPad) ? (w+1) : w;

  //allocate the buffer
  unsigned char * imBuffer = new unsigned char[newW * h * 4];

  vpRGBa* bitmap = I.bitmap;
  unsigned int iwidth = I.getWidth();
  bitmap = bitmap + (int)(iP.get_i()*iwidth+ iP.get_j());

  //if we need padding (width needs to be a multiple of 2)
  if(needPad)
  {
    unsigned int j = 0;
	unsigned int k = 0;
    for(unsigned int i=0 ; i<newW * h * 4; i+=4)
	{
	  //end of a line = padding = inserts 0s
	  if(j==w && needPad)
	  {
	    imBuffer[i+0] = 0;
	    imBuffer[i+1] = 0;
	    imBuffer[i+2] = 0;
	    imBuffer[i+3] = 0;
	    j = 0;	    
	  }
	  else
	  {
	    //RGBA -> BGRA
	    imBuffer[i+0] = (bitmap+k)->B;
	    imBuffer[i+1] = (bitmap+k)->G;
	    imBuffer[i+2] = (bitmap+k)->R;
	    imBuffer[i+3] = (bitmap+k)->A;
		//bitmap++;
	    j++;
		k++;
	  }
	  if (k == newW)
	  {
	    bitmap = bitmap+iwidth;
		k = 0;
	  }
	}
  }
  else
  //Simple conversion (no padding)
  {
    unsigned int k = 0;
    for (unsigned int i=0 ; i < w * h * 4 ; i+=4)
	{
	  imBuffer[i+0] = (bitmap+k)->B;
	  imBuffer[i+1] = (bitmap+k)->G;
	  imBuffer[i+2] = (bitmap+k)->R;
	  imBuffer[i+3] = (bitmap+k)->A;
	  //bitmap++;
	  k++;
	  if (k == newW)
	  {
	    bitmap = bitmap+iwidth;
	    k = 0;
	  }
	}
  }

  //updates the bitmap's pixel data
  updateBitmapROI(imBuffer,iP, newW, h);

  //we don't need this buffer anymore
  delete [] imBuffer;
}


/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param hBmp The destination image.
*/
void vpGDIRenderer::convert(const vpImage<unsigned char> &I, HBITMAP& hBmp)
{
  //get the image's width and height
  unsigned int w = I.getWidth();
  unsigned int h = I.getHeight();

  //each line of a HBITMAP needs to be word aligned
  //we need padding if the width is an odd number
  bool needPad = ((w%2) == 0) ? false : true;
  unsigned int newW = w;

  //in case of padding, the new width is width+1
  newW = (needPad) ? (w+1) : w;

  //allocate the buffer
  unsigned char * imBuffer = new unsigned char[newW * h * 4];

  //if we need padding
  if(needPad)
    {
      unsigned int j = 0;
      for(unsigned int i=0, k=0 ; i<newW * h * 4; i+=4, k++)
	{
	  //end of a line = padding = inserts 0s
	  if(j==w && needPad)
	    {
	      imBuffer[i+0] = 0;
	      imBuffer[i+1] = 0;
	      imBuffer[i+2] = 0;
	      imBuffer[i+3] = 0;
	      j = 0;
	      k --;
	    }
	  else
	    {
	      imBuffer[i+0] = I.bitmap[k];
	      imBuffer[i+1] = I.bitmap[k];
	      imBuffer[i+2] = I.bitmap[k];
	      imBuffer[i+3] = I.bitmap[k];
	      j++;
	    }
	}
    }
  else
    //Simple conversion
    {
      for(unsigned int i=0, k=0 ; i<w * h * 4 ; i+=4, k++)
	{
	  imBuffer[i+0] = I.bitmap[k];
	  imBuffer[i+1] = I.bitmap[k];
	  imBuffer[i+2] = I.bitmap[k];
	  imBuffer[i+3] = I.bitmap[k];
	}
    }

  //updates the bitmap's pixel data
  updateBitmap(hBmp,imBuffer, newW, h);

  //we don't need this buffer anymore
  delete [] imBuffer;
}


/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::convertROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width, const unsigned int height)
{
  //get the image's width and height
  unsigned int w = width;
  unsigned int h = height;

  //each line of a HBITMAP needs to be word aligned
  //we need padding if the width is an odd number
  bool needPad = ((w%2) == 0) ? false : true;
  unsigned int newW = w;

  //in case of padding, the new width is width+1
  newW = (needPad) ? (w+1) : w;

  //allocate the buffer
  unsigned char * imBuffer = new unsigned char[newW * h * 4];

  unsigned char* bitmap = I.bitmap;
  unsigned int iwidth = I.getWidth();
  bitmap = bitmap + (int)(iP.get_i()*iwidth+ iP.get_j());

  //if we need padding (width needs to be a multiple of 2)
  if(needPad)
  {
    unsigned int j = 0;
	unsigned int k = 0;
    for(unsigned int i=0 ; i<newW * h * 4; i+=4)
	{
	  //end of a line = padding = inserts 0s
	  if(j==w && needPad)
	  {
	    imBuffer[i+0] = 0;
	    imBuffer[i+1] = 0;
	    imBuffer[i+2] = 0;
	    imBuffer[i+3] = 0;
	    j = 0;	    
	  }
	  else
	  {
	    //RGBA -> BGRA
	    imBuffer[i+0] = *(bitmap+k);
	    imBuffer[i+1] = *(bitmap+k);
	    imBuffer[i+2] = *(bitmap+k);
	    imBuffer[i+3] = *(bitmap+k);
		//bitmap++;
	    j++;
		k++;
	  }
	  if (k == newW)
	  {
	    bitmap = bitmap+iwidth;
		k = 0;
	  }
	}
  }
  else
  //Simple conversion (no padding)
  {
    unsigned int k = 0;
    for (unsigned int i=0 ; i < w * h * 4 ; i+=4)
	{
	  imBuffer[i+0] = *(bitmap+k);
	  imBuffer[i+1] = *(bitmap+k);
	  imBuffer[i+2] = *(bitmap+k);
	  imBuffer[i+3] = *(bitmap+k);
	  //bitmap++;
	  k++;
	  if (k == newW)
	  {
	    bitmap = bitmap+iwidth;
	    k = 0;
	  }
	}
  }

  //updates the bitmap's pixel data
  updateBitmapROI(imBuffer,iP, newW, h);

  //we don't need this buffer anymore
  delete [] imBuffer;
}

/*!
  Updates the bitmap to display.
  Contains a critical section.
  \param hBmp The bitmap to update
  \param imBuffer The new pixel data
  \param w The image's width
  \param h The image's height

  \return the operation succefulness
*/
bool vpGDIRenderer::updateBitmap(HBITMAP& hBmp, unsigned char * imBuffer,
				 unsigned int w, unsigned int h)
{
  //the bitmap may only be accessed by one thread at the same time
  //that's why we enter critical section
  EnterCriticalSection(&CriticalSection);

  //if the existing bitmap object is of the right size
  if( (nbCols==w) && (nbRows==h) )
    {
      //just replace the content
      SetBitmapBits(hBmp, w * h * 4, imBuffer);
    }
  else
    {
      if(hBmp != NULL)
	{
	  //delete the old BITMAP
	  DeleteObject(hBmp);
	}
      //create a new BITMAP from this buffer
      if( (hBmp = CreateBitmap(static_cast<int>(w),static_cast<int>(h),
                               1,32,(void*)imBuffer)) == NULL)
	return false;
    }

  LeaveCriticalSection(&CriticalSection);
  return true;
}


/*!
  Updates the bitmap to display.
  Contains a critical section.
  \param imBuffer The new pixel data
  \param iP The topleft corner of the roi 
  \param w The roi's width
  \param h The roi's height

  \return the operation succefulness
*/
bool vpGDIRenderer::updateBitmapROI(unsigned char * imBuffer, const vpImagePoint &iP,
				 unsigned int w, unsigned int h)
{
  int w_ = static_cast<int>(w);
  int h_ = static_cast<int>(h);
  HBITMAP htmp = CreateBitmap(w_,h_,1,32,(void*)imBuffer);

  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);
  HDC hDCMem2 = CreateCompatibleDC(hDCScreen);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);
  SelectObject(hDCMem2, htmp);

  BitBlt(hDCMem,(int)iP.get_u(),(int)iP.get_v(),w_,h_, hDCMem2, 0, 0,SRCCOPY);
  LeaveCriticalSection(&CriticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
  DeleteObject(htmp);

  return true;
}

/*!
  Sets a pixel to color at position (j,i).

  \param ip : The pixel coordinates.
  \param color : the color of the point.
*/
void vpGDIRenderer::setPixel(const vpImagePoint &iP,
			     const vpColor &color)
{
  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  if (color.id < vpColor::id_unknown)
    SetPixel(hDCMem, vpMath::round(iP.get_u()), vpMath::round(iP.get_v()),
	     colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    SetPixel(hDCMem, vpMath::round(iP.get_u()), vpMath::round(iP.get_v()),
	     gdicolor);
  }
  //display the result (flush)
  // BitBlt(hDCScreen, x, y, 1, 1, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}

/*!
  Draws a line.
  \param ip1,ip2 : Initial and final image point.
  \param color the line's color
  \param thickness : Thickness of the line.
  \param style style of the line
*/
void vpGDIRenderer::drawLine(const vpImagePoint &ip1, 
			     const vpImagePoint &ip2,
			     const vpColor &color,
			     unsigned int thickness, int style)
{
  HDC hDCScreen= NULL, hDCMem = NULL;
  HPEN hPen = NULL;
#ifdef GDI_ROBUST
  double start = vpTime::measureTimeMs();  
  while(vpTime::measureTimeMs()-start<1000){
    hDCScreen = GetDC(hWnd);
    if(!hDCScreen) continue;
    hDCMem = CreateCompatibleDC(hDCScreen);
    if(!hDCMem){
      ReleaseDC(hWnd, hDCScreen);
      continue;
    }

    //create the pen    
    if (color.id < vpColor::id_unknown)
      hPen = CreatePen(style, static_cast<int>(thickness), colors[color.id]);
    else {
      COLORREF gdicolor = RGB(color.R, color.G, color.B);
      hPen = CreatePen(style, static_cast<int>(thickness), gdicolor);
    }
    if(!hPen){
      DeleteDC(hDCMem);
      ReleaseDC(hWnd, hDCScreen);
      continue;
    }
    if(!SetBkMode(hDCMem, TRANSPARENT)){
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(hWnd, hDCScreen);
      continue;
    }

    //select this bmp in memory
    EnterCriticalSection(&CriticalSection);

    if(!SelectObject(hDCMem, bmp)){
      LeaveCriticalSection(&CriticalSection);
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(hWnd, hDCScreen);
      continue;
    }

    //select the pen
    if(!SelectObject(hDCMem, hPen)){
      LeaveCriticalSection(&CriticalSection);
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(hWnd, hDCScreen);
      continue;
    }
    break;
  }
  timelost+=(vpTime::measureTimeMs()-start);
#else
  //get the window's DC
  hDCScreen = GetDC(hWnd);
  hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(style, static_cast<int>(thickness), colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(style, static_cast<int>(thickness), gdicolor);
  }
  SetBkMode(hDCMem, TRANSPARENT);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the pen
  SelectObject(hDCMem, hPen);
#endif
  //move to the starting point
  MoveToEx(hDCMem, vpMath::round(ip1.get_u()), vpMath::round(ip1.get_v()), NULL);
  //Draw the line
  LineTo(hDCMem, vpMath::round(ip2.get_u()), vpMath::round(ip2.get_v()));

  //computes the coordinates of the rectangle to blit
//   int x = (j2 >= j1) ? j1 : j2;
//   int y = (i2 >= i1) ? i1 : i2;
//   int w = (j2 >= j1) ? j2-j1 : j1-j2;
//   int h = (i2 >= i1) ? i2-i1 : i1-i2;

  //display the result (flush)
 // BitBlt(hDCScreen, x, y, w, h, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);

}

/*!
  Draws a rectangle.
  \param topLeft its top left point's coordinates
  \param width width of the rectangle
  \param height height of the rectangle
  \param color The rectangle's color
  \param fill  When set to true fill the rectangle.
  \param thickness : Line thickness
*/
void vpGDIRenderer::drawRect(const vpImagePoint &topLeft,
			     unsigned int width, unsigned int height,
			     const vpColor &color, bool fill,
			     unsigned int thickness)
{
  if (thickness == 0) thickness = 1;
  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen;
  COLORREF gdicolor = RGB(0,0,0);

  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), colors[color.id]);
  else {
    gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  //create an hollow or solid brush (depends on boolean fill)
  LOGBRUSH lBrush;
  if(fill) {
    lBrush.lbStyle = BS_SOLID;
    if (color.id < vpColor::id_unknown)
      lBrush.lbColor = colors[color.id];
    else {
      lBrush.lbColor = gdicolor;
    }
  }
  else lBrush.lbStyle = BS_HOLLOW;
  HBRUSH hbrush = CreateBrushIndirect(&lBrush);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the brush
  SelectObject(hDCMem, hbrush);
  //select the pen
  SelectObject(hDCMem, hPen);

  //draw the rectangle
  Rectangle(hDCMem, vpMath::round(topLeft.get_u()), vpMath::round(topLeft.get_v()),
            vpMath::round(topLeft.get_u())+static_cast<int>(width),
            vpMath::round(topLeft.get_v())+static_cast<int>(height));

  //display the result (flush)
//  BitBlt(hDCScreen, j, i, width, height, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteObject(hbrush);
  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);

}

/*!
  Clears the image to a specific color.
  \param color The color used to fill the image.
*/
void vpGDIRenderer::clear(const vpColor &color)
{
	vpImagePoint ip;
	ip.set_i(0);
	ip.set_j(0);
  drawRect(ip, nbCols, nbRows, color, true, 0);
}


/*!
  Draws a circle.
  \param center its center point's coordinates
  \param radius The circle's radius
  \param color The circle's color
  \param fill  When set to true fill the circle.
  \param thickness : Line thickness
*/
void vpGDIRenderer::drawCircle(const vpImagePoint &center, unsigned int radius,
			       const vpColor &color, bool fill, unsigned int thickness)
{

  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen;
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  //create an hollow brush
  LOGBRUSH lBrush;
  lBrush.lbStyle = BS_HOLLOW;
  HBRUSH hbrush = CreateBrushIndirect(&lBrush);

  //computes bounding rectangle
  int radius_ = static_cast<int>(radius);
  int x1 = vpMath::round(center.get_u())-radius_;
  int y1 = vpMath::round(center.get_v())-radius_;
  int x2 = vpMath::round(center.get_u())+radius_;
  int y2 = vpMath::round(center.get_v())+radius_;

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the brush
  SelectObject(hDCMem, hbrush);
  //select the pen
  SelectObject(hDCMem, hPen);

  //draw the circle
  if (fill==false)
    Ellipse(hDCMem, x1, y1, x2, y2);

  else
  {
    while (x2-x1 > 0)
	{
		x1++;
		x2--;
		y1++;
		y2--;
		Ellipse(hDCMem, x1, y1, x2, y2);
	}
  }

    //display the result (flush)
   // BitBlt(hDCScreen, x1, y1, x2-x1, y2-y1, hDCMem, x1, y1, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteObject(hbrush);
  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}


/*!
  Draws some text.
  \param ip its top left point's coordinates
  \param text The string to display
  \param color The text's color
*/
void vpGDIRenderer::drawText(const vpImagePoint &ip, const char * text,
			     const vpColor &color)
{
  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //Select the font
  SelectObject(hDCMem, hFont);

  //set the text color
  if (color.id < vpColor::id_unknown)
    SetTextColor(hDCMem, colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    SetTextColor(hDCMem, gdicolor);
  }

  //we don't use the bkColor
  SetBkMode(hDCMem, TRANSPARENT);

  SIZE size;
  int length = (int) strlen(text);

  //get the displayed string dimensions
  GetTextExtentPoint32(hDCMem, text, length, &size);

  //displays the string
  TextOut(hDCMem, vpMath::round(ip.get_u()), vpMath::round(ip.get_v()), text, length);

  //display the result (flush)
 // BitBlt(hDCScreen, j, i, size.cx, size.cy, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}



/*!
  Draws a cross.
  \param ip its center point's coordinates
  \param size Size of the cross
  \param color The cross' color
  \param thickness width of the cross
*/
void vpGDIRenderer::drawCross(const vpImagePoint &ip, unsigned int size,
			      const vpColor &color, unsigned int thickness)
{
  /* unsigned */ int half_size = static_cast<int>( size/2 );

  // if half_size is equal to zero, nothing is displayed with the code
  // just below. So, if half_size is equal to zero we just draw the
  // pixel.
  if (half_size) {
    //get the window's DC
    HDC hDCScreen = GetDC(hWnd);
    HDC hDCMem = CreateCompatibleDC(hDCScreen);

    //create the pen
    HPEN hPen;
    if (color.id < vpColor::id_unknown)
      hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), colors[color.id]);
    else {
      COLORREF gdicolor = RGB(color.R, color.G, color.B);
      hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
    }

    //select this bmp in memory
    EnterCriticalSection(&CriticalSection);
    SelectObject(hDCMem, bmp);

    //select the pen
    SelectObject(hDCMem, hPen);

    //move to the starting point
    MoveToEx(hDCMem, vpMath::round(ip.get_u())-half_size, vpMath::round(ip.get_v()), NULL);
    //Draw the first line (horizontal)
    LineTo(hDCMem, vpMath::round(ip.get_u())+half_size, vpMath::round(ip.get_v()));

    //move to the starting point
    MoveToEx(hDCMem, vpMath::round(ip.get_u()), vpMath::round(ip.get_v())-half_size, NULL);
    //Draw the second line (vertical)
    LineTo(hDCMem, vpMath::round(ip.get_u()), vpMath::round(ip.get_v())+half_size);

    //display the result (flush)
  //  BitBlt(hDCScreen, j-(size/2), i-(size/2), size, size,
//	   hDCMem, j-(size/2), i-(size/2), SRCCOPY);

    LeaveCriticalSection(&CriticalSection);

    DeleteObject(hPen);
    DeleteDC(hDCMem);
    ReleaseDC(hWnd, hDCScreen);
  }
  else {
    setPixel(ip, color);
  }


}

/*!
  Draws an arrow.
  \param ip1,ip2 : Initial and final image point.
  \param color The arrow's color
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpGDIRenderer::drawArrow(const vpImagePoint &ip1, 
			      const vpImagePoint &ip2,
			      const vpColor &color,
			      unsigned int w,unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i() ;
  double b = ip2.get_j() - ip1.get_j() ;
  double lg = sqrt(vpMath::sqr(a)+vpMath::sqr(b)) ;

  //computes the coordinates of the rectangle to blit later
//   unsigned int x = (j2 >= j1) ? j1 : j2;
//   unsigned int y = (i2 >= i1) ? i1 : i2;
//   unsigned int w = (j2 >= j1) ? j2-j1 : j1-j2;
//   unsigned int h = (i2 >= i1) ? i2-i1 : i1-i2;

  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen;
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the pen
  SelectObject(hDCMem, hPen);


  if ((a==0)&&(b==0))
    {
      // DisplayCrossLarge(i1,j1,3,col) ;
    }
  else
    {
      a /= lg ;
      b /= lg ;

      vpImagePoint ip3;
      ip3.set_i( ip2.get_i() - w*a );
      ip3.set_j( ip2.get_j() - w*b );


      vpImagePoint ip4;

      //double t = 0 ;
      //while (t<=_l)
      {
        ip4.set_i( ip3.get_i() - b*h );
        ip4.set_j( ip3.get_j() + a*h );

        if (lg > 2*vpImagePoint::distance(ip2, ip4) ) {
          MoveToEx(hDCMem, vpMath::round(ip2.get_u()), vpMath::round(ip2.get_v()), NULL);
          LineTo(hDCMem, vpMath::round(ip4.get_u()), vpMath::round(ip4.get_v()));
        }
        // t+=0.1 ;
      }

      //t = 0 ;
      //while (t>= -_l)
      {
        ip4.set_i( ip3.get_i() + b*h );
        ip4.set_j( ip3.get_j() - a*h );

        if (lg > 2*vpImagePoint::distance(ip2, ip4) ) {
          MoveToEx(hDCMem, vpMath::round(ip2.get_u()), vpMath::round(ip2.get_v()), NULL);
          LineTo(hDCMem, vpMath::round(ip4.get_u()), vpMath::round(ip4.get_v()));
        }

        // t-=0.1 ;
      }
      MoveToEx(hDCMem, vpMath::round(ip1.get_u()), vpMath::round(ip1.get_v()), NULL);
      LineTo(hDCMem, vpMath::round(ip2.get_u()), vpMath::round(ip2.get_v()));

    }


  //display the result (flush)
//  BitBlt(hDCScreen, x, y, w, h, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}

/*!
  Gets the currently displayed image.
  \param I Image returned.
*/
void vpGDIRenderer::getImage(vpImage<vpRGBa> &I)
{
  //size of image buffer : nbCols*nbRows*4
  unsigned int size = nbCols*nbRows*4;
  unsigned char * imBuffer = new unsigned char[size];

  //gets the hbitmap's bitmap
  GetBitmapBits(bmp, static_cast<LONG>(size), (void *)imBuffer);

  //resize the destination image as needed
  I.resize(nbRows, nbCols);

  //copy the content
  for(unsigned int i=0 ; i<size ; i+=4)
    {
      I.bitmap[i>>2].R = imBuffer[i+2];
      I.bitmap[i>>2].G = imBuffer[i+1];
      I.bitmap[i>>2].B = imBuffer[i+0];
      I.bitmap[i>>2].A =  0xFF; //255 = maximum opacity
    }

  delete [] imBuffer;
}
#endif
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpGDIRenderer.cpp.o) has no symbols
void dummy_vpGDIRenderer() {};
#endif
