/****************************************************************************
 *
 * $Id: vpD3DRenderer.cpp,v 1.3 2007-02-26 17:26:44 fspindle Exp $
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
#if ( defined(WIN32) & defined(VISP_HAVE_D3D9) ) 

#include <visp/vpD3DRenderer.h>
#include <visp/vpColor.h>
#include <visp/vpMath.h>

/*
  Be careful, when using :

  pd3dText->LockRect(0, &d3dLRect, &r, 0)
  ...
  pd3dText->UnlockRect(0, &d3dLRect, &r, 0)

  to write directly to the texture's surface,
  the pointer returned in d3dLRect points to
  the beginning of the locked suface and not
  to the beginning of the texture's surface.
  That's why setBufferPixel and other accesses
  to this buffer are done in the locked surface
  coordinates system.

  Moreover, when directly writing to a texture's surface,
  you musn't forget to take the pitch of this texture 
  into account (see Direct3D documentation).

*/

/*!
  Constructor.
  Initializes the color palettes and the font.
*/
vpD3DRenderer::vpD3DRenderer()
{
  pD3D=NULL;
  pd3dDevice=NULL;
  pd3dText=NULL;
  pd3dVideoText=NULL;

  //D3D palette
  colors[vpColor::black] = D3DCOLOR_ARGB(0xFF,0,0,0);
  colors[vpColor::blue]  = D3DCOLOR_ARGB(0xFF,0,0,0xFF);
  colors[vpColor::cyan]  = D3DCOLOR_ARGB(0xFF,0,0xFF,0xFF);
  colors[vpColor::green] = D3DCOLOR_ARGB(0xFF,0,0xFF,0);
  colors[vpColor::red]   = D3DCOLOR_ARGB(0xFF,0xFF,0,0);
  colors[vpColor::white] = D3DCOLOR_ARGB(0xFF,0xFF,0xFF,0xFF);
  colors[vpColor::yellow]= D3DCOLOR_ARGB(0xFF,0xFF,0xFF,0);

	
  //initialize the GDI palette
  colorsGDI[vpColor::black] =  RGB(0,0,0);
  colorsGDI[vpColor::blue]  =  RGB(0,0,0xFF);
  colorsGDI[vpColor::cyan]  =  RGB(0,0xFF,0xFF);
  colorsGDI[vpColor::green] =  RGB(0,0xFF,0);
  colorsGDI[vpColor::red]   =  RGB(0xFF,0,0);
  colorsGDI[vpColor::white] =  RGB(0xFF,0xFF,0xFF);
  colorsGDI[vpColor::yellow]=  RGB(0xFF,0xFF,0);

  //Creates a logical font
  hFont = CreateFont(18, 0, 0, 0, FW_NORMAL, false, false, false,
		     DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, 
		     CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
		     DEFAULT_PITCH | FF_DONTCARE, NULL);
}

/*!
  Destructor.
  Releases all the remaining interfaces.
*/
vpD3DRenderer::~vpD3DRenderer()
{
  DeleteObject(hFont);

  if(pd3dDevice != NULL)
    pd3dDevice->Release();
  if(pD3D != NULL)
    pD3D->Release();
  if(pd3dText != NULL)
    pd3dText->Release();
  if(pd3dVideoText != NULL)
    pd3dVideoText->Release();
}

/*!
  Computes the nearest power of 2 superior to n.
  \param n Number whose nearest superior power of 2 we want.
  \return Nearest power of 2 superior to n.
*/
int vpD3DRenderer::supPowerOf2(int n)
{
  int i=0;
  while(n>1)
    {
      n>>=1;
      i++;
    }
  return 1<<(i+1);
}

/*!
  Initialize the Direct3D renderer.
  \param hwnd The window's handle.
  \param width The window's width.
  \param height The window's height.

*/
bool vpD3DRenderer::init(HWND hwnd, unsigned width, unsigned height)
{
  //simple stuff
  nbCols = width;
  nbRows = height;
  hWnd = hwnd;

  //D3D initialize
  if(NULL == (pD3D = Direct3DCreate9(D3D_SDK_VERSION)))
    throw vpDisplayException(vpDisplayException::notInitializedError, 
			     "Can't initialize D3D!");

  D3DDISPLAYMODE d3ddm;
  if(FAILED(pD3D->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &d3ddm)))
    throw vpDisplayException(vpDisplayException::notInitializedError, 
			     "Can't get the adapter's display mode!");


  D3DPRESENT_PARAMETERS d3dpp; 
  ZeroMemory(&d3dpp, sizeof(d3dpp));
  d3dpp.BackBufferCount=1;
  d3dpp.Windowed   = TRUE;
  d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
  d3dpp.BackBufferFormat = d3ddm.Format;

  //creates a d3d device
  if( FAILED(pD3D->CreateDevice(D3DADAPTER_DEFAULT , D3DDEVTYPE_HAL, hWnd,
				D3DCREATE_SOFTWARE_VERTEXPROCESSING |
				D3DCREATE_MULTITHREADED,
				&d3dpp, &pd3dDevice )))
    throw vpDisplayException(vpDisplayException::notInitializedError, 
			     "Can't create the Direct3D device!");


  //disables scene lightning
  pd3dDevice->SetRenderState(D3DRS_LIGHTING, FALSE);


  //inits the direct3D view (for 2D rendering)
  initView((float)nbCols,(float)nbRows);
	

  //computes texture size (needs to be a power-of-2 large square)
  textWidth = supPowerOf2( (nbCols>nbRows) ? nbCols : nbRows );

  //creates the system memory texture (the one we will directly modify)
  //unfortunately, needs to be X8R8G8B8 in order to be able to use GDI drawing functions
  if( D3DXCreateTexture(pd3dDevice, textWidth, textWidth, D3DX_DEFAULT, 0, 
			D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM , &pd3dText)
      != D3D_OK)
    {
      throw vpDisplayException(vpDisplayException::notInitializedError, 
			       "Can't create memory texture!");
    }

  //creates the video memory texture (the one we will display) - 
  if( D3DXCreateTexture(pd3dDevice, textWidth, textWidth, D3DX_DEFAULT, 
			D3DUSAGE_DYNAMIC , 
			D3DFMT_X8R8G8B8, D3DPOOL_DEFAULT, &pd3dVideoText)
      != D3D_OK)
    {
      throw vpDisplayException(vpDisplayException::notInitializedError, 
			       "Can't create video texture!");
    }

  //creates the sprite used to render the texture
  if(D3DXCreateSprite(pd3dDevice, &pSprite) != S_OK)
    throw vpDisplayException(vpDisplayException::notInitializedError, 
			     "Can't create the texture's sprite!");



  return true;
}




/*!
  Initialize the view (orthogonal ...).
  \param WindowWidth The Width of the window.
  \param WindowHeight The height of the window.
*/
void vpD3DRenderer::initView(float WindowWidth, float WindowHeight)
{
  D3DXMATRIX Ortho2D;	
  D3DXMATRIX Identity;

  D3DXMatrixOrthoLH(&Ortho2D, WindowWidth, WindowHeight, 0.0f, 1.0f);
  D3DXMatrixIdentity(&Identity);

  if( pd3dDevice->SetTransform(D3DTS_PROJECTION, &Ortho2D) != D3D_OK
      || pd3dDevice->SetTransform(D3DTS_WORLD, &Identity) != D3D_OK
      || pd3dDevice->SetTransform(D3DTS_VIEW, &Identity) != D3D_OK)
    throw vpDisplayException(vpDisplayException::notInitializedError, 
			     "Can't set the view!");
}


/*!
  Converts a ViSP RGBA image to the Direct3D texture format (BGRA).
  \param I Image to convert.
  \param imBuffer Destination buffer.
  \param pitch Pitch of the destination texture.
*/
void RGBaToTexture(vpImage<vpRGBa>& I, unsigned char * imBuffer, 
		   unsigned pitch)
{
  unsigned j = I.getWidth();

  unsigned k=0;
  for(unsigned i=0; i<I.getHeight()* I.getWidth(); i++)
    {
      if(j==0){
	k += pitch - (I.getWidth() * 4);
	j = I.getWidth();
      }

      imBuffer[k+0] = I.bitmap[i].B;
      imBuffer[k+1] = I.bitmap[i].G;
      imBuffer[k+2] = I.bitmap[i].R;
      imBuffer[k+3] = I.bitmap[i].A;// pb in vpimconvert? , 0xFF?
      k+=4;
      j--;
    }
}

/*!
  Converts a ViSP gray image to the Direct3D texture format (BGRA).
  \param I Image to convert.
  \param imBuffer Destination buffer.
  \param pitch Pitch of the destination texture.
*/
void GreyToTexture(vpImage<unsigned char>& I, unsigned char * imBuffer, 
		   unsigned pitch)
{
  unsigned j = I.getWidth();

  unsigned k=0;
  for(unsigned i=0; i<I.getHeight()* I.getWidth(); i++)
    {
      if(j==0){
	k += pitch - (I.getWidth() * 4);
	j = I.getWidth();
      }

      imBuffer[k+0] = imBuffer[k+1] =	imBuffer[k+2] = I.bitmap[i];
      imBuffer[k+3] = 0xFF; //full opacity

      k+=4;
      j--;
    }
}

/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImg(const vpImage<vpRGBa>& im)
{
  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;

      RECT r;
      r.top=0; r.left=0;
      r.bottom=nbRows; r.right=nbCols;
		
      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      //fills this texture with the image data (converted to bgra)
      RGBaToTexture(im, buf, pitch);		

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    
    }
}


/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImg(const vpImage<unsigned char>& im)
{
  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;

      RECT r;
      r.top=0; r.left=0;
      r.bottom=nbRows; r.right=nbCols;

      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      //fills this texture with the image data (converted to bgra)
      GreyToTexture(im, buf, pitch);		

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }

}

/*!
  Renders the memory texture to the screen.
  \return True.

*/
bool vpD3DRenderer::render()
{
  // Clears the back buffer to a blue color
  //pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0,0,255), 1.0f, 0 );
	
  //Begins the scene.
  pd3dDevice->BeginScene();

  //Texture rectangle to display
  RECT r;
  r.top=0; r.left=0;
  r.bottom=nbRows; r.right=nbCols;
			
  //Updates the video memory texture with the content of the system
  //memory texture
  pd3dDevice->UpdateTexture(pd3dText,pd3dVideoText);

  //Displays this texture as a sprite
  
#if (D3DX_SDK_VERSION <= 9)
  pSprite->Begin(); //
  pSprite->Draw(pd3dVideoText, &r, NULL, NULL, NULL, NULL, 0xFFFFFFFF );
#else
  pSprite->Begin(0);
  pSprite->Draw(pd3dVideoText, &r, NULL, NULL, 0xFFFFFFFF );
#endif
  pSprite->End();

  //Ends the scene.
  pd3dDevice->EndScene();
  //Presents the backbuffer
  pd3dDevice->Present( NULL, NULL, NULL, NULL );

  return true;
}


/*!
  Sets a pixel to color at position (j,i).
*/
void vpD3DRenderer::setPixel(unsigned i, unsigned j, 
			     vpColor::vpColorType color)
{
  if(i<0 || j<0 || i>=nbRows || j>=nbCols)
    {
      vpCERROR<<"Invalid parameters!"<<endl;
      return;
    }

  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;

      RECT r;

      r.top=i; 
      r.left=j;
      r.bottom=i+1; 
      r.right=j+1;

      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      //the coordinates are in the locked area base
      setBufferPixel(buf, pitch, 0, 0,color, 1, 1);

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }

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
void vpD3DRenderer::drawLine(unsigned i1, unsigned j1, 
			     unsigned i2, unsigned j2, 
			     vpColor::vpColorType col, unsigned e, int style)
{
  if(i1<0 || j1<0 || i2<0 || j2<0 || e<0)
    {
      vpCERROR<<"Invalid parameters!"<<endl;
      return;
    }
  
  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      
      //Will contain the texture's surface drawing context
      HDC hDCMem;
      
      //The texture's surface
      IDirect3DSurface9 * pd3dSurf;
      pd3dText->GetSurfaceLevel(0, &pd3dSurf);
      
      //We get its DC
      pd3dSurf->GetDC(&hDCMem);
      
      //create the pen
      HPEN hPen = CreatePen(style, e, colorsGDI[col]);
      
      //we don't use the bkColor
      SetBkMode(hDCMem, TRANSPARENT);
      
      //select the pen
      SelectObject(hDCMem, hPen);
      
      //move to the starting point
      MoveToEx(hDCMem, j1, i1, NULL);
      //Draw the line
      LineTo(hDCMem, j2, i2);
      
      
      //Releases the DC
      pd3dSurf->ReleaseDC(hDCMem);
      //Releases the surface's interface
      pd3dSurf->Release();
      //Deletes additional objects
      DeleteObject(hPen);
    }
}


/*!
  Draws a rectangle.
  \param i its top left point's first coordinate
  \param j its top left point's second coordinate
  \param width width of the rectangle
  \param height height of the rectangle
  \param col The rectangle's color
  \param fill Ignored
*/
void vpD3DRenderer::drawRect(unsigned i, unsigned j,
			     unsigned width, unsigned height, 
			     vpColor::vpColorType col, bool fill)
{
  if(i<0 || j<0 || i>nbRows || j>nbCols || width<0 || height<0)
    {
      vpCERROR<<"Invalid parameters!"<<endl;
      return;
    }

  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;
		
      RECT r;
      r.top=i;
      r.left=j;
      r.bottom=(i+height < nbRows) ? i+height : nbRows-1;
      r.right=(j+width < nbCols) ? j+width : nbCols-1;

      unsigned rectW = r.right - j;
      unsigned rectH = r.bottom - i;

      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      long x=0;
      long y=0;

      //draws the top horizontal line
      for(x; x<width ; x++)
	setBufferPixel(buf, pitch, x, y, col, rectW, rectH);

      //draws the right vertical line
      for(y; y<height ; y++)
	setBufferPixel(buf, pitch, x, y, col, rectW, rectH);

      //draws the bottom horizontal line
      for(x; x>0 ; x--)
	setBufferPixel(buf, pitch, x, y, col, rectW, rectH);

      //draws the left vertical line
      for(y; y>0 ; y--)
	setBufferPixel(buf, pitch, x, y, col, rectW, rectH);

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }
}

/*!
  Clears the image to color c.
  \param c The color used to fill the image.
*/
void vpD3DRenderer::clear(vpColor::vpColorType c)
{
  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;
      
      RECT r;
      r.top = 0;
      r.left = 0;
      r.bottom = nbRows;
      r.right = nbCols;

      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      long * buf = (long *) d3dLRect.pBits;

      vpColor::vpColorType color = colors[c];
      long * end = (long*)((long)buf + (pitch * nbRows));
      
      //fills the whole image
      while (buf < end)
	*buf++ = color;

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }
}



//writes current circle pixels using symetry to reduce the algorithm's complexity
void vpD3DRenderer::subDrawCircle(unsigned i, unsigned j, 
				  unsigned x, unsigned y, 
				  vpColor::vpColorType col, 
				  unsigned char* buf, unsigned pitch, 
				  unsigned maxX, unsigned maxY)
{      
  if (x == 0) {
    setBufferPixel(buf, pitch, i  , j+y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i  , j-y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i+y, j, col, maxX, maxY);
    setBufferPixel(buf, pitch, i-y, j, col, maxX, maxY);
  } else 
    if (x == y) {
      setBufferPixel(buf, pitch, i+x,j+y,col, maxX, maxY);
      setBufferPixel(buf, pitch, i-x,j+y,col, maxX, maxY);
      setBufferPixel(buf, pitch, i+x,j-y,col, maxX, maxY);
      setBufferPixel(buf, pitch, i-x,j-y,col, maxX, maxY);
    } else 
      if (x < y) {
	setBufferPixel(buf, pitch, i+x,j+y,col, maxX, maxY);
	setBufferPixel(buf, pitch, i-x,j+y,col, maxX, maxY);
	setBufferPixel(buf, pitch, i+x,j-y,col, maxX, maxY);
	setBufferPixel(buf, pitch, i-x,j-y,col, maxX, maxY);
	setBufferPixel(buf, pitch, i+y,j+x,col, maxX, maxY);
	setBufferPixel(buf, pitch, i-y,j+x,col, maxX, maxY);
	setBufferPixel(buf, pitch, i+y,j-x,col, maxX, maxY);
	setBufferPixel(buf, pitch, i-y,j-x,col, maxX, maxY);
      }

}

/*!
  Draws a circle.
  \param i its center point's first coordinate
  \param j its center point's second coordinate
  \param r The circle's radius
  \param col The circle's color
*/
void vpD3DRenderer::drawCircle(unsigned i, unsigned j, unsigned r, 
			       vpColor::vpColorType c)
{
  if(r<1)
    return;

  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;
		
      RECT rec;

      int rleft = (j-r >=0) ? j-r : 0;
      int rtop = (i-r >=0) ? i-r : 0;

      rec.top= rtop;
      rec.left= rleft;
      rec.bottom=(i+r < nbRows) ? i+r : nbRows-1;
      rec.right=(j+r < nbCols) ? j+r : nbCols-1;

      //used as maxX and maxY for setBufferPixel
      int rectW = rec.right - rleft;
      int rectH = rec.bottom - rtop;

      //locks the texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &rec, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}
      
      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      // Bresenham 's circle algorithm

      int x = 0;
      int y = r;
      int p = (5 - (r<<2))>>2;

      subDrawCircle(j-rleft, i-rtop, x, y, c, buf, pitch, rectW, rectH);
      while(x < y){
	x++;
	if (p < 0) 
	  {
	    p += (x+1)<<1;
	  }
	else 
	  {
	    y--;
	    p += ((x-y)<<1)+1;
	  }
	subDrawCircle(j-rleft, i-rtop, x, y, c, buf, pitch, rectW, rectH);
      }



      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }
}



/*!
  Draws some text.
  \param i its top left point's first coordinate
  \param j its top left point's second coordinate
  \param s The string to display
  \param col The text's color
*/
void vpD3DRenderer::drawText(unsigned i, unsigned j, char * s, 
			     vpColor::vpColorType c)
{
  //Will contain the texture's surface drawing context
  HDC hDCMem;
  
  //The texture's surface
  IDirect3DSurface9 * pd3dSurf;
  pd3dText->GetSurfaceLevel(0, &pd3dSurf);

  //We get its DC
  pd3dSurf->GetDC(&hDCMem);

  //Select the font
  SelectObject(hDCMem, hFont);
  
  //set the text color
  SetTextColor(hDCMem, colorsGDI[c]);
  
  //we don't use the bkColor
  SetBkMode(hDCMem, TRANSPARENT);
  
  SIZE size;
  int length = (int) strlen(s);
  
  //get the displayed string dimensions
  GetTextExtentPoint32(hDCMem, s, length, &size);
    
  //displays the string
  TextOut(hDCMem, j, i, s, length); 

  //Releases the DC
  pd3dSurf->ReleaseDC(hDCMem);
  //Releases the surface's interface
  pd3dSurf->Release();
  //Deletes additional objects
  DeleteObject(hFont);
}


/*!
  Draws a cross.
  \param i its center point's first coordinate
  \param j its center point's second coordinate
  \param size Size of the cross
  \param col The cross' color
  \param e width of the cross
*/
void vpD3DRenderer::drawCross(unsigned i,unsigned j, unsigned size, 
			      vpColor::vpColorType col, unsigned e)
{
  if(i<0 || j<0 || e<=0)
    return;

  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      D3DLOCKED_RECT d3dLRect;
      
      RECT rec;
      
      //if j-size/2 is inferior to 0, use 0
      int rleft = ( (j - (size/2)) < 0 ) ? 
	0 
	: j - (size/2);
      //if j-size/2 is inferior to 0, use 0
      int rtop  = ( (i - (size/2)) < 0 ) ? 
	0 
	: i - (size/2);

      rec.top   = rtop;
      rec.left  = rleft;
      rec.bottom= i + (size/2);
      rec.right = j + (size/2);

      //locks the texture to directly access it
      if( pd3dText->LockRect(0, &d3dLRect, &rec, 0) != D3D_OK)
      	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      int x;         //xpos

      //y-coordinate of the line in the locked rectangle base
      int y =( i < (size/2) ) ?
	i
	: (size/2);

      int cpt = 0;   //number of lines
      int re = e;    //remaining "width"

      //horizontal lines
      //stops when there is enough line for e
      while(re!=0)
	{
	  //draws a line
	  for(x=0; x<size; x++)
	    setBufferPixel(buf, pitch, x, y, col, size, size);

	  re--;
	  cpt++;
	  
	  //write alternatively a line at the top and a line at the bottom
	  //eg : y=4 -> y=5 -> y=3 -> y=6
	  y += ( (re&1) != 0) ? cpt : -cpt;
	}		       
      
      cpt = 0;
      re = e;

      //x-coordinate of the line in the locked rectangle base
      x =( j < (size/2) ) ?
	j
	: size/2;

      //vertical lines
      while(re!=0)
	{
	  //draws a vertical line
	  for(y=0; y<size; y++)
	    setBufferPixel(buf, pitch, x, y, col, size, size);
	  
	  re--;
	  cpt++;

	  //write alternatively a line on the left and a line on the right
	  x += ( (re&1) != 0) ? cpt : -cpt;
	}

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
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
void vpD3DRenderer::drawArrow(unsigned i1,unsigned j1, 
			      unsigned i2, unsigned j2, 
			      vpColor::vpColorType col, unsigned L,unsigned l)
{
  double a = j2 - j1 ;
  double b = i2 - i1 ;
  double lg = sqrt(vpMath::sqr(a)+vpMath::sqr(b)) ;

  //Will contain the texture's surface drawing context
  HDC hDCMem;
  
  //The texture's surface
  IDirect3DSurface9 * pd3dSurf;
  pd3dText->GetSurfaceLevel(0, &pd3dSurf);

  //We get its DC
  pd3dSurf->GetDC(&hDCMem);

  //create the pen
  HPEN hPen = CreatePen(PS_SOLID, 1, colorsGDI[col]);

  //select the pen
  SelectObject(hDCMem, hPen);

  //based on code from other displays
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

      double t = 0 ;
      while (t<=l)
	{
	  i4 = i3 - b*t ;
	  j4 = j3 + a*t ;

	  MoveToEx(hDCMem, (int)j2, (int)i2, NULL);
	  LineTo(hDCMem, (int)j4, (int)i4);
			
	  t+=0.1 ;
	}

      t = 0 ;
      while (t>= -l)
	{
	  i4 = i3 - b*t ;
	  j4 = j3 + a*t ;

	  MoveToEx(hDCMem, (int)j2, (int)i2, NULL);
	  LineTo(hDCMem, (int)j4, (int)i4);

	  t-=0.1 ;
	}
      MoveToEx(hDCMem, j1, i1, NULL);
      LineTo(hDCMem, j2, i2);

    }

  //Deletes the pen
  DeleteObject(hPen); 
  //Releases the DC
  pd3dSurf->ReleaseDC(hDCMem);
  //Releases the surface's interface
  pd3dSurf->Release();


}

/*!
  Converts the D3D textures to vpImage<vpRGBa>.
  \param I The destination image.
  \param imBuffer The texture's data.
  \param pitch The texture's pitch.
*/
void TextureToRGBa(vpImage<vpRGBa>& I, unsigned char * imBuffer, 
		   unsigned pitch)
{
  unsigned j = I.getWidth();

  unsigned k=0;
  for(unsigned i=0; i<I.getHeight()* I.getWidth(); i++)
    {
      //go to the next line
      if(j==0){
	k += pitch - (I.getWidth() * 4);
	j = I.getWidth();
      }
      
      //simple conversion from bgra to rgba
      I.bitmap[i].B = imBuffer[k+0];
      I.bitmap[i].G = imBuffer[k+1];
      I.bitmap[i].R = imBuffer[k+2];
      I.bitmap[i].A = imBuffer[k+3];

      k+=4;
      j--;
    }
}

/*!
  Gets the currently displayed image with its overlay.
  \param I The image to fill.
*/
void vpD3DRenderer::getImage(vpImage<vpRGBa> &I)
{
  //if the device has been initialized
  if(pd3dDevice != NULL)
    {
      
      //resize the destination image as needed
      I.resize(nbRows, nbCols);

      D3DLOCKED_RECT d3dLRect;

      RECT r;
      r.top=0; r.left=0;
      r.bottom=nbRows; r.right=nbCols;

      //locks the whole texture to directly access it
      if(pd3dText->LockRect(0, &d3dLRect, &r, 0)!= D3D_OK)
	{
	  vpCERROR<<"D3D : Couldn't lock the texture!"<<endl;
	  return;
	}

      //gets the buffer and pitch of the texture
      unsigned pitch = d3dLRect.Pitch;
      unsigned char * buf = (unsigned char *) d3dLRect.pBits;

      //fills this image with the texture's data
      TextureToRGBa(I,buf, pitch);		

      //unlocks the texture
      if( pd3dText->UnlockRect(0) != D3D_OK)
	vpCERROR<<"D3D : Couldn't unlock the texture!"<<endl;
    }
}

#endif
#endif
