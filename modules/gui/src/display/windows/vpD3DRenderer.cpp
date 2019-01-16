/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * D3D renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <algorithm>

#include <visp3/core/vpConfig.h>
#if (defined(_WIN32) & defined(VISP_HAVE_D3D9))

#include <visp3/core/vpColor.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpD3DRenderer.h>

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
  pD3D = NULL;
  pd3dDevice = NULL;
  pSprite = NULL;
  pd3dText = NULL;
  pd3dVideoText = NULL;
  textWidth = 0;

  // D3D palette
  vpColor pcolor; // Predefined colors
  pcolor = vpColor::black;
  colors[vpColor::id_black] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightBlue;
  colors[vpColor::id_lightBlue] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::blue;
  colors[vpColor::id_blue] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkBlue;
  colors[vpColor::id_darkBlue] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::cyan;
  colors[vpColor::id_cyan] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGreen;
  colors[vpColor::id_lightGreen] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::green;
  colors[vpColor::id_green] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGreen;
  colors[vpColor::id_darkGreen] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightRed;
  colors[vpColor::id_lightRed] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::red;
  colors[vpColor::id_red] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkRed;
  colors[vpColor::id_darkRed] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::white;
  colors[vpColor::id_white] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGray;
  colors[vpColor::id_lightGray] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::gray;
  colors[vpColor::id_gray] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGray;
  colors[vpColor::id_darkGray] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::yellow;
  colors[vpColor::id_yellow] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::orange;
  colors[vpColor::id_orange] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::purple;
  colors[vpColor::id_purple] = D3DCOLOR_ARGB(0xFF, pcolor.R, pcolor.G, pcolor.B);

  // initialize the GDI palette
  pcolor = vpColor::black;
  colorsGDI[vpColor::id_black] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightBlue;
  colorsGDI[vpColor::id_lightBlue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::blue;
  colorsGDI[vpColor::id_blue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkBlue;
  colorsGDI[vpColor::id_darkBlue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::cyan;
  colorsGDI[vpColor::id_cyan] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGreen;
  colorsGDI[vpColor::id_lightGreen] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::green;
  colorsGDI[vpColor::id_green] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGreen;
  colorsGDI[vpColor::id_darkGreen] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightRed;
  colorsGDI[vpColor::id_lightRed] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::red;
  colorsGDI[vpColor::id_red] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkRed;
  colorsGDI[vpColor::id_darkRed] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::white;
  colorsGDI[vpColor::id_white] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGray;
  colorsGDI[vpColor::id_lightGray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::gray;
  colorsGDI[vpColor::id_gray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGray;
  colorsGDI[vpColor::id_darkGray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::yellow;
  colorsGDI[vpColor::id_yellow] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::orange;
  colorsGDI[vpColor::id_orange] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::purple;
  colorsGDI[vpColor::id_purple] = RGB(pcolor.R, pcolor.G, pcolor.B);

  // Creates a logical font
  hFont = CreateFont(18, 0, 0, 0, FW_NORMAL, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                     CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_DONTCARE, NULL);
}

/*!
  Destructor.
  Releases all the remaining interfaces.
*/
vpD3DRenderer::~vpD3DRenderer()
{
  DeleteObject(hFont);

  if (pd3dDevice != NULL)
    pd3dDevice->Release();
  if (pD3D != NULL)
    pD3D->Release();
  if (pd3dText != NULL)
    pd3dText->Release();
  if (pd3dVideoText != NULL)
    pd3dVideoText->Release();
}

/*!
  Computes the nearest power of 2 superior to n.
  \param n Number whose nearest superior power of 2 we want.
  \return Nearest power of 2 superior to n.
*/
unsigned int vpD3DRenderer::supPowerOf2(unsigned int n)
{
  unsigned int i = 0;
  while (n > 1) {
    n >>= 1;
    i++;
  }
  return static_cast<unsigned int>(1 << (i + 1));
}

/*!
  Initialize the Direct3D renderer.
  \param hwnd The window's handle.
  \param width The window's width.
  \param height The window's height.

*/
bool vpD3DRenderer::init(HWND hwnd, unsigned int width, unsigned int height)
{
  // simple stuff
  m_rwidth = width;
  m_rheight = height;
  hWnd = hwnd;

  // D3D initialize
  if (NULL == (pD3D = Direct3DCreate9(D3D_SDK_VERSION)))
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't initialize D3D!");

  D3DDISPLAYMODE d3ddm;
  if (FAILED(pD3D->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &d3ddm)))
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't get the adapter's display mode!");

  D3DPRESENT_PARAMETERS d3dpp;
  ZeroMemory(&d3dpp, sizeof(d3dpp));
  d3dpp.BackBufferCount = 1;
  d3dpp.Windowed = TRUE;
  d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
  d3dpp.BackBufferFormat = d3ddm.Format;

  // creates a d3d device
  if (FAILED(pD3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd,
                                D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_MULTITHREADED, &d3dpp, &pd3dDevice)))
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't create the Direct3D device!");

  // disables scene lightning
  pd3dDevice->SetRenderState(D3DRS_LIGHTING, FALSE);

  // inits the direct3D view (for 2D rendering)
  initView((float)m_rwidth, (float)m_rheight);

  // computes texture size (needs to be a power-of-2 large square)
  textWidth = supPowerOf2((m_rwidth > m_rheight) ? m_rwidth : m_rheight);

  // creates the system memory texture (the one we will directly modify)
  // unfortunately, needs to be X8R8G8B8 in order to be able to use GDI
  // drawing  functions
  if (D3DXCreateTexture(pd3dDevice, textWidth, textWidth, D3DX_DEFAULT, 0, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM,
                        &pd3dText) != D3D_OK) {
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't create memory texture!");
  }

  // creates the video memory texture (the one we will display) -
  if (D3DXCreateTexture(pd3dDevice, textWidth, textWidth, D3DX_DEFAULT, D3DUSAGE_DYNAMIC, D3DFMT_X8R8G8B8,
                        D3DPOOL_DEFAULT, &pd3dVideoText) != D3D_OK) {
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't create video texture!");
  }

  // creates the sprite used to render the texture
  if (D3DXCreateSprite(pd3dDevice, &pSprite) != S_OK)
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't create the texture's sprite!");

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

  if (pd3dDevice->SetTransform(D3DTS_PROJECTION, &Ortho2D) != D3D_OK ||
      pd3dDevice->SetTransform(D3DTS_WORLD, &Identity) != D3D_OK ||
      pd3dDevice->SetTransform(D3DTS_VIEW, &Identity) != D3D_OK)
    throw vpDisplayException(vpDisplayException::notInitializedError, "Can't set the view!");
}

/*!
  Converts a ViSP RGBA image to the Direct3D texture format (BGRA).
  \param I Image to convert.
  \param imBuffer Destination buffer.
  \param pitch Pitch of the destination texture.
*/
void vpD3DRenderer::convert(const vpImage<vpRGBa> &I, unsigned char *imBuffer, unsigned int pitch)
{
  if (m_rscale == 1) {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int ii_ = i * pitch;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        vpRGBa val = I[i][j];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = val.A;
      }
    }
  } else {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int i_ = i * m_rscale;
      unsigned int ii_ = i * pitch;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        vpRGBa val = I[i_][j * m_rscale];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = val.A;
      }
    }
  }
}

/*!
  Converts a ViSP gray image to the Direct3D texture format (BGRA).
  \param I Image to convert.
  \param imBuffer Destination buffer.
  \param pitch Pitch of the destination texture.
*/
void vpD3DRenderer::convert(const vpImage<unsigned char> &I, unsigned char *imBuffer, unsigned int pitch)
{
  if (m_rscale == 1) {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int ii_ = i * pitch;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        unsigned char val = I[i][j];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  } else {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int i_ = i * m_rscale;
      unsigned int ii_ = i * pitch;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        unsigned char val = I[i_][j * m_rscale];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  }
}

/*!
Converts a ViSP gray image to the Direct3D texture format (BGRA).
\param I Image to convert.
\param imBuffer Destination buffer.
\param pitch Pitch of the destination texture.
\param i_min, i_min Upper/left coordinate of the roi.
\param i_max, j_max Bottom/right coordinate of the roi.
*/
void vpD3DRenderer::convertROI(const vpImage<unsigned char> &I, unsigned char *imBuffer, unsigned int pitch, int i_min,
                               int j_min, int i_max, int j_max)
{
  int h = i_max - i_min;
  int w = j_max - j_min;

  if (m_rscale == 1) {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = i_min + i;
      unsigned int ii_ = i * pitch;
      for (int j = 0; j < w; j++) {
        unsigned char val = I[i_][j_min + j];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  } else {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = (i_min + i) * m_rscale;
      unsigned int ii_ = i * pitch;
      for (int j = 0; j < w; j++) {
        unsigned char val = I[i_][(j_min + j) * m_rscale];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  }
}

/*!
Converts a ViSP gray image to the Direct3D texture format (BGRA).
\param I Image to convert.
\param imBuffer Destination buffer.
\param pitch Pitch of the destination texture.
\param i_min, i_min Upper/left coordinate of the roi.
\param i_max, j_max Bottom/right coordinate of the roi.
*/
void vpD3DRenderer::convertROI(const vpImage<vpRGBa> &I, unsigned char *imBuffer, unsigned int pitch, int i_min,
                               int j_min, int i_max, int j_max)
{
  int h = i_max - i_min;
  int w = j_max - j_min;

  if (m_rscale == 1) {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = i_min + i;
      unsigned int ii_ = i * pitch;
      for (int j = 0; j < w; j++) {
        vpRGBa val = I[i_][j_min + j];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  } else {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = (i_min + i) * m_rscale;
      unsigned int ii_ = i * pitch;
      for (int j = 0; j < w; j++) {
        vpRGBa val = I[i_][(j_min + j) * m_rscale];
        unsigned int index_ = ii_ + j * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  }
}

/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImg(const vpImage<vpRGBa> &im)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    RECT r;
    r.top = 0;
    r.left = 0;
    r.bottom = static_cast<signed long>(m_rheight);
    r.right = static_cast<signed long>(m_rwidth);

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // fills this texture with the image data (converted to bgra)
    convert(im, buf, pitch);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImgROI(const vpImage<vpRGBa> &im, const vpImagePoint &iP, const unsigned int width,
                              const unsigned int height)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    int i_min = (std::max)((int)ceil(iP.get_i() / m_rscale), 0);
    int j_min = (std::max)((int)ceil(iP.get_j() / m_rscale), 0);
    int i_max = (std::min)((int)ceil((iP.get_i() + height) / m_rscale), (int)m_rheight);
    int j_max = (std::min)((int)ceil((iP.get_j() + width) / m_rscale), (int)m_rwidth);

    RECT r;
    r.top = (LONG)i_min;
    r.left = (LONG)j_min;
    r.bottom = (LONG)i_max;
    r.right = (LONG)j_max;

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // fills this texture with the image data (converted to bgra)
    convertROI(im, buf, pitch, i_min, j_min, i_max, j_max);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImg(const vpImage<unsigned char> &im)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    RECT r;
    r.top = 0;
    r.left = 0;
    r.bottom = static_cast<LONG>(m_rheight);
    r.right = static_cast<LONG>(m_rwidth);

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // fills this texture with the image data (converted to bgra)
    convert(im, buf, pitch);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Sets the image to display.
  \param im The image to display.
*/
void vpD3DRenderer::setImgROI(const vpImage<unsigned char> &im, const vpImagePoint &iP, const unsigned int width,
                              const unsigned int height)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    int i_min = (std::max)((int)ceil(iP.get_i() / m_rscale), 0);
    int j_min = (std::max)((int)ceil(iP.get_j() / m_rscale), 0);
    int i_max = (std::min)((int)ceil((iP.get_i() + height) / m_rscale), (int)m_rheight);
    int j_max = (std::min)((int)ceil((iP.get_j() + width) / m_rscale), (int)m_rwidth);

    RECT r;
    r.top = (LONG)i_min;
    r.left = (LONG)j_min;
    r.bottom = (LONG)i_max;
    r.right = (LONG)j_max;

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // fills this texture with the image data (converted to bgra)
    convertROI(im, buf, pitch, i_min, j_min, i_max, j_max);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Renders the memory texture to the screen.
  \return True.

*/
bool vpD3DRenderer::render()
{
  // Clears the back buffer to a blue color
  // pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET,
  // D3DCOLOR_XRGB(0,0,255), 1.0f, 0 );

  // Begins the scene.
  pd3dDevice->BeginScene();

  // Texture rectangle to display
  RECT r;
  r.top = 0;
  r.left = 0;
  r.bottom = static_cast<LONG>(m_rheight);
  r.right = static_cast<LONG>(m_rwidth);

  // Updates the video memory texture with the content of the system
  // memory texture
  pd3dDevice->UpdateTexture(pd3dText, pd3dVideoText);

// Displays this texture as a sprite

#if (D3DX_SDK_VERSION <= 9)
  pSprite->Begin(); //
  pSprite->Draw(pd3dVideoText, &r, NULL, NULL, NULL, NULL, 0xFFFFFFFF);
#else
  pSprite->Begin(0);
  pSprite->Draw(pd3dVideoText, &r, NULL, NULL, 0xFFFFFFFF);
#endif
  pSprite->End();

  // Ends the scene.
  pd3dDevice->EndScene();
  // Presents the backbuffer
  pd3dDevice->Present(NULL, NULL, NULL, NULL);

  return true;
}

/*!
  Sets a pixel to color at position (j,i).

  \param ip : The pixel coordinates.
  \param color : the color of the point.
*/
void vpD3DRenderer::setPixel(const vpImagePoint &iP, const vpColor &color)
{
  vpImagePoint iPscaled = iP / m_rscale;
  if (iPscaled.get_i() < 0 || iPscaled.get_j() < 0 || iPscaled.get_i() >= (int)m_rheight ||
      iPscaled.get_j() >= (int)m_rwidth) {
    return;
  }

  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    RECT r;

    r.top = (LONG)iPscaled.get_i();
    r.left = (LONG)iPscaled.get_j();
    r.bottom = (LONG)iPscaled.get_i() + 1;
    r.right = (LONG)iPscaled.get_j() + 1;

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // the coordinates are in the locked area base
    setBufferPixel(buf, pitch, 0, 0, color);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Draws a line.
  \param ip1,ip2 : Initial and final image point.
  \param color the line's color
  \param thickness : Thickness of the line.
  \param style style of the line
*/
void vpD3DRenderer::drawLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                             unsigned int thickness, int style)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    // Will contain the texture's surface drawing context
    HDC hDCMem;

    // The texture's surface
    IDirect3DSurface9 *pd3dSurf;
    pd3dText->GetSurfaceLevel(0, &pd3dSurf);

    // We get its DC
    pd3dSurf->GetDC(&hDCMem);

    // create the pen
    HPEN hPen;
    if (color.id < vpColor::id_unknown)
      hPen = CreatePen(style, static_cast<int>(thickness), colorsGDI[color.id]);
    else {
      COLORREF gdicolor = RGB(color.R, color.G, color.B);
      hPen = CreatePen(style, static_cast<int>(thickness), gdicolor);
    }

    // we don't use the bkColor
    SetBkMode(hDCMem, TRANSPARENT);

    // select the pen
    SelectObject(hDCMem, hPen);

    // Warning: When thickness > 1 and pen style is PS_DASHDOT, the drawing
    // displays a solid line That's why in that case we implement the dashdot
    // line manually drawing multiple small lines
    if (thickness != 1 && style != PS_SOLID) {
      double size = 10. * m_rscale;
      double length = sqrt(vpMath::sqr(ip2.get_i() - ip1.get_i()) + vpMath::sqr(ip2.get_j() - ip1.get_j()));
      double deltaj = size / length * (ip2.get_j() - ip1.get_j());
      double deltai = size / length * (ip2.get_i() - ip1.get_i());
      double slope = (ip2.get_i() - ip1.get_i()) / (ip2.get_j() - ip1.get_j());
      double orig = ip1.get_i() - slope * ip1.get_j();
      for (unsigned int j = (unsigned int)ip1.get_j(); j < ip2.get_j(); j += (unsigned int)(2 * deltaj)) {
        double i = slope * j + orig;
        // move to the starting point
        MoveToEx(hDCMem, vpMath::round(j / m_rscale), vpMath::round(i / m_rscale), NULL);
        // Draw the line
        LineTo(hDCMem, vpMath::round((j + deltaj) / m_rscale), vpMath::round((i + deltai) / m_rscale));
      }
    } else {
      // move to the starting point
      MoveToEx(hDCMem, vpMath::round(ip1.get_u() / m_rscale), vpMath::round(ip1.get_v() / m_rscale), NULL);
      // Draw the line
      LineTo(hDCMem, vpMath::round(ip2.get_u() / m_rscale), vpMath::round(ip2.get_v() / m_rscale));
    }

    // Releases the DC
    pd3dSurf->ReleaseDC(hDCMem);
    // Releases the surface's interface
    pd3dSurf->Release();
    // Deletes additional objects
    DeleteObject(hPen);
  }
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
void vpD3DRenderer::drawRect(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                             bool fill, unsigned int thickness)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    if (fill == false) {
      drawLine(topLeft, topLeft + vpImagePoint(0, width), color, thickness);
      drawLine(topLeft + vpImagePoint(0, width), topLeft + vpImagePoint(height, width), color, thickness);
      drawLine(topLeft + vpImagePoint(height, width), topLeft + vpImagePoint(height, 0), color, thickness);
      drawLine(topLeft + vpImagePoint(height, 0), topLeft, color, thickness);
    } else {
      vpImagePoint topLeftScaled = topLeft / m_rscale;
      unsigned int widthScaled = width / m_rscale;
      unsigned int heightScaled = height / m_rscale;

      if (topLeftScaled.get_i() > (int)m_rheight - 1 || topLeftScaled.get_j() > (int)m_rwidth - 1 ||
          topLeftScaled.get_i() + height < 0 || topLeftScaled.get_j() + width < 0) {
        //       vpCERROR<<"Invalid parameters!"<<std::endl;
        return;
      }

      D3DLOCKED_RECT d3dLRect;

      RECT r;
      r.top = (LONG)((topLeftScaled.get_i() > 0) ? topLeftScaled.get_i() : 0);
      r.left = (LONG)((topLeftScaled.get_j() > 0) ? topLeftScaled.get_j() : 0);
      r.bottom = (LONG)((topLeftScaled.get_i() + heightScaled < (int)m_rheight) ? topLeftScaled.get_i() + heightScaled
                                                                                : m_rheight - 1);
      r.right = (LONG)((topLeftScaled.get_j() + widthScaled < (int)m_rwidth) ? topLeftScaled.get_j() + widthScaled
                                                                             : m_rwidth - 1);

      /* unsigned */ int rectW = r.right - r.left;
      /* unsigned */ int rectH = r.bottom - r.top;

      // locks the texture to directly access it
      if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
        vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
        return;
      }

      // gets the buffer and pitch of the texture
      unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
      unsigned char *buf = (unsigned char *)d3dLRect.pBits;

      if (topLeftScaled.get_i() >= 0 && topLeftScaled.get_j() + widthScaled < m_rwidth &&
          topLeftScaled.get_i() + heightScaled < m_rheight && topLeftScaled.get_j() >= 0) {
        for (int x = 0; x < rectW; x++) {
          for (int y = 0; y < rectH; y++)
            setBufferPixel(buf, pitch, x, y, color);
        }
      }

      // unlocks the texture
      if (pd3dText->UnlockRect(0) != D3D_OK)
        vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
    }
  }
}

/*!
  Clears the image to a specific color.
  \param color The color used to fill the image.
*/
void vpD3DRenderer::clear(const vpColor &color)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    RECT r;
    r.top = 0;
    r.left = 0;
    r.bottom = static_cast<LONG>(m_rheight);
    r.right = static_cast<LONG>(m_rwidth);

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    long *buf = (long *)(d3dLRect.pBits);

    unsigned long c;
    if (color.id < vpColor::id_unknown)
      c = colors[color.id];
    else {
      c = D3DCOLOR_ARGB(0xFF, color.R, color.G, color.B);
    }
    long *end = (long *)(buf + (pitch * m_rheight));

    // fills the whole image
    while (buf < end)
      *buf++ = static_cast<long>(c);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

// writes current circle pixels using symetry to reduce the algorithm's
// complexity
void vpD3DRenderer::subDrawCircle(int i, int j, int x, int y, vpColor col, unsigned char *buf, unsigned int pitch,
                                  unsigned int maxX, unsigned int maxY)
{
  if (x == 0) {
    setBufferPixel(buf, pitch, i, j + y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i, j - y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i + y, j, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - y, j, col, maxX, maxY);
  } else if (x == y) {
    setBufferPixel(buf, pitch, i + x, j + y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - x, j + y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i + x, j - y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - x, j - y, col, maxX, maxY);
  } else if (x < y) {
    setBufferPixel(buf, pitch, i + x, j + y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - x, j + y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i + x, j - y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - x, j - y, col, maxX, maxY);
    setBufferPixel(buf, pitch, i + y, j + x, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - y, j + x, col, maxX, maxY);
    setBufferPixel(buf, pitch, i + y, j - x, col, maxX, maxY);
    setBufferPixel(buf, pitch, i - y, j - x, col, maxX, maxY);
  }
}

/*!
  Draws a circle.
  \param center its center point's coordinates
  \param radius The circle's radius
  \param color The circle's color
*/
void vpD3DRenderer::drawCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool /*fill*/,
                               unsigned int /*thickness*/)
{
  unsigned int radiusScaled = radius / m_rscale;
  vpImagePoint centerScaled = center / m_rscale;
  if (radiusScaled < 1 || vpMath::round(centerScaled.get_i() + radiusScaled) < 0 ||
      vpMath::round(centerScaled.get_i() - radiusScaled) > (int)m_rheight ||
      vpMath::round(centerScaled.get_j() + radiusScaled) < 0 ||
      vpMath::round(centerScaled.get_j() - radiusScaled) > (int)m_rwidth)
    return;

  // if the device has been initialized
  if (pd3dDevice != NULL) {
    D3DLOCKED_RECT d3dLRect;

    RECT rec;
    int radiusScaled_ = static_cast<int>(radiusScaled);
    int rleft = (vpMath::round(centerScaled.get_j() - radiusScaled_) > 0)
                    ? vpMath::round(centerScaled.get_j()) - radiusScaled_
                    : 0;
    int rtop = (vpMath::round(centerScaled.get_i() - radiusScaled_) > 0)
                   ? vpMath::round(centerScaled.get_i()) - radiusScaled_
                   : 0;

    rec.top = rtop;
    rec.left = rleft;
    rec.bottom = (LONG)((vpMath::round(centerScaled.get_i() + radiusScaled_) < (int)m_rheight)
                            ? centerScaled.get_i() + radiusScaled_
                            : m_rheight - 1);
    rec.right = (LONG)((vpMath::round(centerScaled.get_j() + radiusScaled_) < (int)m_rwidth)
                           ? centerScaled.get_j() + radiusScaled_
                           : m_rwidth - 1);

    // used as maxX and maxY for setBufferPixel
    unsigned int rectW = static_cast<unsigned int>(rec.right - rleft);
    unsigned int rectH = static_cast<unsigned int>(rec.bottom - rtop);

    // locks the texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &rec, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // Bresenham 's circle algorithm

    int x = 0;
    int y = static_cast<int>(radiusScaled);
    int p = (3 - (y << 1));

    vpImagePoint ip;
    ip.set_i(centerScaled.get_i() - rtop);
    ip.set_j(centerScaled.get_j() - rleft);

    subDrawCircle(vpMath::round(ip.get_i()), vpMath::round(ip.get_j()), x, y, color, buf, pitch, rectW, rectH);
    while (x < y) {
      x++;
      if (p < 0) {
        p += ((x << 1) + 1) << 1;
      } else {
        y--;
        p += (((x - y) << 1) + 1) << 1;
      }
      subDrawCircle(vpMath::round(ip.get_i()), vpMath::round(ip.get_j()), x, y, color, buf, pitch, rectW, rectH);
    }

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

/*!
  Draws some text.
  \param ip its top left point's coordinates
  \param text The string to display
  \param color The text's color
*/
void vpD3DRenderer::drawText(const vpImagePoint &ip, const char *text, const vpColor &color)
{
  // Will contain the texture's surface drawing context
  HDC hDCMem;

  // The texture's surface
  IDirect3DSurface9 *pd3dSurf;
  pd3dText->GetSurfaceLevel(0, &pd3dSurf);

  // We get its DC
  pd3dSurf->GetDC(&hDCMem);

  // Select the font
  SelectObject(hDCMem, hFont);

  // set the text color
  if (color.id < vpColor::id_unknown)
    SetTextColor(hDCMem, colorsGDI[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    SetTextColor(hDCMem, gdicolor);
  }

  // we don't use the bkColor
  SetBkMode(hDCMem, TRANSPARENT);

  SIZE size;
  int length = (int)strlen(text);

  // get the displayed string dimensions
  GetTextExtentPoint32(hDCMem, text, length, &size);

  // displays the string
  TextOut(hDCMem, vpMath::round(ip.get_u() / m_rscale), vpMath::round(ip.get_v() / m_rscale), text, length);

  // Releases the DC
  pd3dSurf->ReleaseDC(hDCMem);
  // Releases the surface's interface
  pd3dSurf->Release();
  // Deletes additional objects
  DeleteObject(hFont);
}

/*!
  Draws a cross.
  \param ip its center point's coordinates
  \param size Size of the cross
  \param color The cross' color
  \param thickness width of the cross
*/
void vpD3DRenderer::drawCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness)
{
  double i = ip.get_i();
  double j = ip.get_j();
  vpImagePoint ip1, ip2;

  ip1.set_i(i - size / 2);
  ip1.set_j(j);
  ip2.set_i(i + size / 2);
  ip2.set_j(j);
  drawLine(ip1, ip2, color, thickness);

  ip1.set_i(i);
  ip1.set_j(j - size / 2);
  ip2.set_i(i);
  ip2.set_j(j + size / 2);

  drawLine(ip1, ip2, color, thickness);
}

/*!
  Draws an arrow.
  \param ip1,ip2 : Initial and final image point.
  \param color The arrow's color
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpD3DRenderer::drawArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w,
                              unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i();
  double b = ip2.get_j() - ip1.get_j();
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

  // if ( ( a==0 ) && ( b==0 ) )
  if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(b) <= std::numeric_limits<double>::epsilon())) {
    // DisplayCrossLarge(i1,j1,3,col) ;
  } else {
    a /= lg;
    b /= lg;

    vpImagePoint ip3;
    ip3.set_i(ip2.get_i() - w * a);
    ip3.set_j(ip2.get_j() - w * b);

    vpImagePoint ip4;
    ip4.set_i(ip3.get_i() - b * h);
    ip4.set_j(ip3.get_j() + a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4))
      drawLine(ip2, ip4, color, thickness);

    ip4.set_i(ip3.get_i() + b * h);
    ip4.set_j(ip3.get_j() - a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4))
      drawLine(ip2, ip4, color, thickness);

    drawLine(ip1, ip2, color, thickness);
  }
}

/*!
  Converts the D3D textures to vpImage<vpRGBa>.
  \param I The destination image.
  \param imBuffer The texture's data.
  \param pitch The texture's pitch.
*/
void TextureToRGBa(vpImage<vpRGBa> &I, unsigned char *imBuffer, unsigned int pitch)
{
  unsigned int j = I.getWidth();

  unsigned int k = 0;
  for (unsigned int i = 0; i < I.getHeight() * I.getWidth(); i++) {
    // go to the next line
    if (j == 0) {
      k += pitch - (I.getWidth() * 4);
      j = I.getWidth();
    }

    // simple conversion from bgra to rgba
    I.bitmap[i].B = imBuffer[k + 0];
    I.bitmap[i].G = imBuffer[k + 1];
    I.bitmap[i].R = imBuffer[k + 2];
    I.bitmap[i].A = imBuffer[k + 3];

    k += 4;
    j--;
  }
}

/*!
  Gets the currently displayed image with its overlay.
  \param I The image to fill.
*/
void vpD3DRenderer::getImage(vpImage<vpRGBa> &I)
{
  // if the device has been initialized
  if (pd3dDevice != NULL) {

    // resize the destination image as needed
    I.resize(m_rheight, m_rwidth);

    D3DLOCKED_RECT d3dLRect;

    RECT r;
    r.top = 0;
    r.left = 0;
    r.bottom = static_cast<LONG>(m_rheight);
    r.right = static_cast<LONG>(m_rwidth);

    // locks the whole texture to directly access it
    if (pd3dText->LockRect(0, &d3dLRect, &r, 0) != D3D_OK) {
      vpCERROR << "D3D : Couldn't lock the texture!" << std::endl;
      return;
    }

    // gets the buffer and pitch of the texture
    unsigned int pitch = static_cast<unsigned int>(d3dLRect.Pitch);
    unsigned char *buf = (unsigned char *)d3dLRect.pBits;

    // fills this image with the texture's data
    TextureToRGBa(I, buf, pitch);

    // unlocks the texture
    if (pd3dText->UnlockRect(0) != D3D_OK)
      vpCERROR << "D3D : Couldn't unlock the texture!" << std::endl;
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpD3DRenderer.cpp.o) has no
// symbols
void dummy_vpD3DRenderer(){};
#endif
#endif
