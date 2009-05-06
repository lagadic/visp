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


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpGDIRenderer.h>

/*!
  Constructor.
*/
vpGDIRenderer::vpGDIRenderer()
{
  //if the screen depth is not 32bpp, throw an exception
  if( GetDeviceCaps(GetDC(NULL),BITSPIXEL) != 32 )
    throw vpDisplayException(vpDisplayException::depthNotSupportedError,
			     "Only works in 32bits mode!");

  InitializeCriticalSection(&CriticalSection);

  //initialize the palette
  colors[vpColor::black] =  RGB(0,0,0);
  colors[vpColor::blue]  =  RGB(0,0,0xFF);
  colors[vpColor::cyan]  =  RGB(0,0xFF,0xFF);
  colors[vpColor::green] =  RGB(0,0xFF,0);
  colors[vpColor::red]   =  RGB(0xFF,0,0);
  colors[vpColor::white] =  RGB(0xFF,0xFF,0xFF);
  colors[vpColor::yellow]=  RGB(0xFF,0xFF,0);
  colors[vpColor::orange]=  RGB(0xFF,0xA5,0);

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
  \param width largeur de la fenêtre
  \param height hauteur de la fenêtre
*/
bool vpGDIRenderer::init(HWND hWindow, unsigned int width, unsigned int height)
{
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
  \param im The rgba image to display.
*/
void vpGDIRenderer::setImg(const vpImage<vpRGBa>& im)
{
  //converts the image into a HBITMAP
  convert(im, bmp);
  //updates the size of the image
  nbCols=im.getWidth();
  nbRows=im.getHeight();
}

/*!
  Sets the image to display.
  \param im The grayscale image to display.
*/
void vpGDIRenderer::setImg(const vpImage<unsigned char>& im)
{
  //converts the image into a HBITMAP
  convert(im, bmp);
  //updates the size of the image
  nbCols=im.getWidth();
  nbRows=im.getHeight();
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
  BitBlt(hDCScreen, 0, 0, nbCols, nbRows, hDCMem, 0, 0, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);
  DeleteDC(hDCMem);

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
      if( (hBmp = CreateBitmap(w,h,1,32,(void*)imBuffer)) == NULL)
	return false;
    }

  LeaveCriticalSection(&CriticalSection);
  return true;
}

/*!
  Sets the pixel at (x,y).
  \param y The y coordinate of the pixel.
  \param x The x coordinate of the pixel.
  \param color The color of the pixel.
*/
void vpGDIRenderer::setPixel(int y, int x,
			     vpColor::vpColorType color)
{
  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);


  SetPixel(hDCMem,x,y,colors[color]);
  //display the result (flush)
 // BitBlt(hDCScreen, x, y, 1, 1, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}

/*!
  Draws a line.
  \param i1 its starting point's first coordinate
  \param j1 its starting point's second coordinate
  \param i2 its ending point's first coordinate
  \param j2 its ending point's second coordinate
  \param e width of the line
  \param col the line's color
  \param style style of the line
*/
void vpGDIRenderer::drawLine(int i1, int j1,
			     int i2, int j2,
			     vpColor::vpColorType col,
			     unsigned int e, int style)
{

  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen = CreatePen(style, e, colors[col]);

  SetBkMode(hDCMem, TRANSPARENT);

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the pen
  SelectObject(hDCMem, hPen);

  //move to the starting point
  MoveToEx(hDCMem, j1, i1, NULL);
  //Draw the line
  LineTo(hDCMem, j2, i2);

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
  \param i its top left point's first coordinate
  \param j its top left point's second coordinate
  \param width width of the rectangle
  \param height height of the rectangle
  \param col The rectangle's color
  \param fill True if it is a filled rectangle
  \param e : Line thickness
*/
void vpGDIRenderer::drawRect(int i, int j,
			     unsigned int width, unsigned int height,
			     vpColor::vpColorType col, bool fill,
			     unsigned int e)
{
  if (e == 0) e = 1;
  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen = CreatePen(PS_SOLID, e, colors[col]);

  //create an hollow or solid brush (depends on boolean fill)
  LOGBRUSH lBrush;
  if(fill)
    {
      lBrush.lbStyle = BS_SOLID;
      lBrush.lbColor = colors[col];
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
  Rectangle(hDCMem, j, i, j+width, i+height);

  //display the result (flush)
//  BitBlt(hDCScreen, j, i, width, height, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteObject(hbrush);
  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);

}

/*!
  Clears the image to color c.
  \param c The color used to fill the image.
*/
void vpGDIRenderer::clear(vpColor::vpColorType c)
{
  drawRect(0, 0, nbCols, nbRows, c, true, 0);
}


/*!
  Draws a circle.
  \param i its center point's first coordinate
  \param j its center point's second coordinate
  \param r The circle's radius
  \param col The circle's color
*/
void vpGDIRenderer::drawCircle(int i, int j, unsigned int r,
			       vpColor::vpColorType c)
{

  //get the window's DC
  HDC hDCScreen = GetDC(hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  //create the pen
  HPEN hPen = CreatePen(PS_SOLID, 1, colors[c]);

  //create an hollow brush
  LOGBRUSH lBrush;
  lBrush.lbStyle = BS_HOLLOW;
  HBRUSH hbrush = CreateBrushIndirect(&lBrush);

  //computes bounding rectangle
  int x1 = j-r;
  int y1 = i-r;
  int x2 = j+r;
  int y2 = i+r;

  //select this bmp in memory
  EnterCriticalSection(&CriticalSection);
  SelectObject(hDCMem, bmp);

  //select the brush
  SelectObject(hDCMem, hbrush);
  //select the pen
  SelectObject(hDCMem, hPen);

  //draw the circle
  Ellipse(hDCMem, x1, y1, x2, y2),

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
  \param i its top left point's first coordinate
  \param j its top left point's second coordinate
  \param s The string to display
  \param col The text's color
*/
void vpGDIRenderer::drawText(int i, int j, const char * s,
			     vpColor::vpColorType c)
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
  SetTextColor(hDCMem, colors[c]);

  //we don't use the bkColor
  SetBkMode(hDCMem, TRANSPARENT);

  SIZE size;
  int length = (int) strlen(s);

  //get the displayed string dimensions
  GetTextExtentPoint32(hDCMem, s, length, &size);

  //displays the string
  TextOut(hDCMem, j, i, s, length);

  //display the result (flush)
 // BitBlt(hDCScreen, j, i, size.cx, size.cy, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&CriticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(hWnd, hDCScreen);
}



/*!
  Draws a cross.
  \param i its center point's first coordinate
  \param j its center point's second coordinate
  \param size Size of the cross
  \param col The cross' color
  \param e width of the cross
*/
void vpGDIRenderer::drawCross(int i, int j, unsigned int size,
			      vpColor::vpColorType col, unsigned int e)
{
  unsigned int half_size = size / 2;

  // if half_size is equal to zero, nothing is displayed with the code
  // just below. So, if half_size is equal to zero we just draw the
  // pixel.
  if (half_size) {
    //get the window's DC
    HDC hDCScreen = GetDC(hWnd);
    HDC hDCMem = CreateCompatibleDC(hDCScreen);

    //create the pen
    HPEN hPen = CreatePen(PS_SOLID, e, colors[col]);

    //select this bmp in memory
    EnterCriticalSection(&CriticalSection);
    SelectObject(hDCMem, bmp);

    //select the pen
    SelectObject(hDCMem, hPen);

    //move to the starting point
    MoveToEx(hDCMem, j-half_size, i, NULL);
    //Draw the first line (horizontal)
    LineTo(hDCMem, j+half_size, i);

    //move to the starting point
    MoveToEx(hDCMem, j, i-half_size, NULL);
    //Draw the second line (vertical)
    LineTo(hDCMem, j, i+half_size);

    //display the result (flush)
  //  BitBlt(hDCScreen, j-(size/2), i-(size/2), size, size,
//	   hDCMem, j-(size/2), i-(size/2), SRCCOPY);

    LeaveCriticalSection(&CriticalSection);

    DeleteObject(hPen);
    DeleteDC(hDCMem);
    ReleaseDC(hWnd, hDCScreen);
  }
  else {
    setPixel(i, j, col);
  }


}

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
void vpGDIRenderer::drawArrow(int i1, int j1,
			      int i2, int j2,
			      vpColor::vpColorType col,
			      unsigned int L,unsigned int l)
{
  int _l = l;
  double a = j2 - j1 ;
  double b = i2 - i1 ;
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
  HPEN hPen = CreatePen(PS_SOLID, 1, colors[col]);

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

      double i3,j3  ;
      i3 = i2 - L*a ;
      j3 = j2 - L*b ;


      double i4,j4 ;

      //double t = 0 ;
      //while (t<=_l)
	{
	  i4 = i3 - b*_l ;
	  j4 = j3 + a*_l ;

	  MoveToEx(hDCMem, j2, i2, NULL);
	  LineTo(hDCMem, (int)j4, (int)i4);

	  // t+=0.1 ;
	}

	//t = 0 ;
	//while (t>= -_l)
	{
	  i4 = i3 + b*_l ;
	  j4 = j3 - a*_l ;

	  MoveToEx(hDCMem, j2, i2, NULL);
	  LineTo(hDCMem, (int)j4, (int)i4);

	  // t-=0.1 ;
	}
      MoveToEx(hDCMem, j1, i1, NULL);
      LineTo(hDCMem, j2, i2);

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
  GetBitmapBits(bmp, size, (void *)imBuffer);

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
#endif
