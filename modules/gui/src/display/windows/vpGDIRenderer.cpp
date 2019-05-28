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
 * GDI renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#define GDI_ROBUST
#if (defined(VISP_HAVE_GDI))

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/gui/vpGDIRenderer.h>

/*!
  Constructor.
*/
vpGDIRenderer::vpGDIRenderer() : m_bmp(NULL), m_bmp_width(0), m_bmp_height(0), timelost(0)
{
  // if the screen depth is not 32bpp, throw an exception
  int bpp = GetDeviceCaps(GetDC(NULL), BITSPIXEL);
  if (bpp != 32)
    throw vpDisplayException(vpDisplayException::depthNotSupportedError,
                             "vpGDIRenderer supports only 32bits depth: screen is %dbits depth!", bpp);

  InitializeCriticalSection(&m_criticalSection);

  // initialize GDI the palette
  vpColor pcolor; // Predefined colors

  pcolor = vpColor::black;
  m_colors[vpColor::id_black] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightBlue;
  m_colors[vpColor::id_lightBlue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::blue;
  m_colors[vpColor::id_blue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkBlue;
  m_colors[vpColor::id_darkBlue] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::cyan;
  m_colors[vpColor::id_cyan] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGreen;
  m_colors[vpColor::id_lightGreen] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::green;
  m_colors[vpColor::id_green] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGreen;
  m_colors[vpColor::id_darkGreen] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightRed;
  m_colors[vpColor::id_lightRed] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::red;
  m_colors[vpColor::id_red] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkRed;
  m_colors[vpColor::id_darkRed] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::white;
  m_colors[vpColor::id_white] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::lightGray;
  m_colors[vpColor::id_lightGray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::gray;
  m_colors[vpColor::id_gray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::darkGray;
  m_colors[vpColor::id_darkGray] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::yellow;
  m_colors[vpColor::id_yellow] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::orange;
  m_colors[vpColor::id_orange] = RGB(pcolor.R, pcolor.G, pcolor.B);
  pcolor = vpColor::purple;
  m_colors[vpColor::id_purple] = RGB(pcolor.R, pcolor.G, pcolor.B);

  m_rwidth = 0;
  m_rheight = 0;
}

/*!
  Destructor.
*/
vpGDIRenderer::~vpGDIRenderer()
{
  // Deletes the critical section object
  DeleteCriticalSection(&m_criticalSection);
  // Deletes the bitmap
  DeleteObject(m_bmp);
  // Deletes the font object
  DeleteObject(m_hFont);
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
  m_hWnd = hWindow;

  m_rwidth = width;
  m_rheight = height;

  // creates the font
  m_hFont = CreateFont(18, 0, 0, 0, FW_NORMAL, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                       CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_DONTCARE, NULL);
  return true;
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
*/
void vpGDIRenderer::setImg(const vpImage<vpRGBa> &I)
{
  // converts the image into a HBITMAP
  convert(I, m_bmp);
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::setImgROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width,
                              const unsigned int height)
{
  // converts the image into a HBITMAP
  convertROI(I, iP, width, height);
}

/*!
  Sets the image to display.
  \param I : The grayscale image to display.
*/
void vpGDIRenderer::setImg(const vpImage<unsigned char> &I)
{
  // converts the image into a HBITMAP
  convert(I, m_bmp);
}

/*!
  Sets the image to display.
  \param I : The rgba image to display.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::setImgROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width,
                              const unsigned int height)
{
  // converts the image into a HBITMAP
  convertROI(I, iP, width, height);
}

/*!
  Render the current image buffer.
*/
bool vpGDIRenderer::render()
{
  // gets the window's DC
  PAINTSTRUCT ps;
  HDC hDCScreen = BeginPaint(m_hWnd, &ps);

  // create a memory DC
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // selects this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // blits it on the window's DC
  BitBlt(hDCScreen, 0, 0, static_cast<int>(m_rwidth), static_cast<int>(m_rheight), hDCMem, 0, 0, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);
  // DeleteDC(hDCMem);
  DeleteObject(hDCMem);

  EndPaint(m_hWnd, &ps);

  return true;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param hBmp The destination image.
*/
void vpGDIRenderer::convert(const vpImage<vpRGBa> &I, HBITMAP &hBmp)
{
  // allocate the buffer
  unsigned char *imBuffer = new unsigned char[m_rwidth * m_rheight * 4];

  if (m_rscale == 1) {
    for (unsigned int i = 0, k = 0; i < m_rwidth * m_rheight * 4; i += 4, k++) {
      imBuffer[i + 0] = I.bitmap[k].B;
      imBuffer[i + 1] = I.bitmap[k].G;
      imBuffer[i + 2] = I.bitmap[k].R;
      imBuffer[i + 3] = I.bitmap[k].A;
    }
  } else {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int i_ = i * m_rscale;
      unsigned int ii_ = i * m_rwidth;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        vpRGBa val = I[i_][j * m_rscale];
        unsigned int index_ = (ii_ + j) * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = val.A;
      }
    }
  }

  // updates the bitmap's pixel data
  updateBitmap(hBmp, imBuffer, m_rwidth, m_rheight);

  // we don't need this buffer anymore
  delete[] imBuffer;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I : The image to convert.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::convertROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, const unsigned int width,
                               const unsigned int height)
{
  int i_min = (std::max)((int)ceil(iP.get_i() / m_rscale), 0);
  int j_min = (std::max)((int)ceil(iP.get_j() / m_rscale), 0);
  int i_max = (std::min)((int)ceil((iP.get_i() + height) / m_rscale), (int)m_rheight);
  int j_max = (std::min)((int)ceil((iP.get_j() + width) / m_rscale), (int)m_rwidth);

  int h = i_max - i_min;
  int w = j_max - j_min;

  // allocate the buffer
  unsigned char *imBuffer = new unsigned char[w * h * 4];

  if (m_rscale == 1) {
    vpRGBa *bitmap = I.bitmap;
    unsigned int iwidth = I.getWidth();
    bitmap = bitmap + (int)(i_min * iwidth + j_min);

    int k = 0;
    for (int i = 0; i < w * h * 4; i += 4) {
      imBuffer[i + 0] = (bitmap + k)->B;
      imBuffer[i + 1] = (bitmap + k)->G;
      imBuffer[i + 2] = (bitmap + k)->R;
      imBuffer[i + 3] = (bitmap + k)->A;
      // bitmap++;
      k++;
      if (k == w) {
        bitmap = bitmap + iwidth;
        k = 0;
      }
    }
  } else {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = (i_min + i) * m_rscale;
      unsigned int ii_ = i * w;
      for (int j = 0; j < w; j++) {
        vpRGBa val = I[i_][(j_min + j) * m_rscale];
        unsigned int index_ = (ii_ + j) * 4;
        imBuffer[index_] = val.B;
        imBuffer[++index_] = val.G;
        imBuffer[++index_] = val.R;
        imBuffer[++index_] = val.A;
      }
    }
  }

  // updates the bitmap's pixel data
  updateBitmapROI(imBuffer, i_min, j_min, w, h);

  // we don't need this buffer anymore
  delete[] imBuffer;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param hBmp The destination image.
*/
void vpGDIRenderer::convert(const vpImage<unsigned char> &I, HBITMAP &hBmp)
{
  // allocate the buffer
  unsigned char *imBuffer = new unsigned char[m_rwidth * m_rheight * 4];

  if (m_rscale == 1) {
    for (unsigned int i = 0, k = 0; i < m_rwidth * m_rheight * 4; i += 4, k++) {
      imBuffer[i + 0] = I.bitmap[k];
      imBuffer[i + 1] = I.bitmap[k];
      imBuffer[i + 2] = I.bitmap[k];
      imBuffer[i + 3] = vpRGBa::alpha_default;
    }
  } else {
    for (unsigned int i = 0; i < m_rheight; i++) {
      unsigned int i_ = i * m_rscale;
      unsigned int ii_ = i * m_rwidth;
      for (unsigned int j = 0; j < m_rwidth; j++) {
        unsigned char val = I[i_][j * m_rscale];
        unsigned int index_ = (ii_ + j) * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  }

  // updates the bitmap's pixel data
  updateBitmap(hBmp, imBuffer, m_rwidth, m_rheight);

  // we don't need this buffer anymore
  delete[] imBuffer;
}

/*!
  Converts the image form ViSP in GDI's image format (bgra with padding).
  \param I The image to convert.
  \param iP : Top left coordinates of the ROI.
  \param width, height : ROI width and height.
*/
void vpGDIRenderer::convertROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int width,
                               const unsigned int height)
{
  int i_min = (std::max)((int)ceil(iP.get_i() / m_rscale), 0);
  int j_min = (std::max)((int)ceil(iP.get_j() / m_rscale), 0);
  int i_max = (std::min)((int)ceil((iP.get_i() + height) / m_rscale), (int)m_rheight);
  int j_max = (std::min)((int)ceil((iP.get_j() + width) / m_rscale), (int)m_rwidth);

  int h = i_max - i_min;
  int w = j_max - j_min;

  // allocate the buffer
  unsigned char *imBuffer = new unsigned char[w * h * 4];

  if (m_rscale == 1) {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = i_min + i;
      unsigned int ii_ = i * w;
      for (int j = 0; j < w; j++) {
        unsigned char val = I[i_][j_min + j];
        unsigned int index_ = (ii_ + j) * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  } else {
    for (int i = 0; i < h; i++) {
      unsigned int i_ = (i_min + i) * m_rscale;
      unsigned int ii_ = i * w;
      for (int j = 0; j < w; j++) {
        unsigned char val = I[i_][(j_min + j) * m_rscale];
        unsigned int index_ = (ii_ + j) * 4;
        imBuffer[index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = val;
        imBuffer[++index_] = vpRGBa::alpha_default;
      }
    }
  }

  // updates the bitmap's pixel data
  updateBitmapROI(imBuffer, i_min, j_min, w, h);

  // we don't need this buffer anymore
  delete[] imBuffer;
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
bool vpGDIRenderer::updateBitmap(HBITMAP &hBmp, unsigned char *imBuffer, unsigned int w, unsigned int h)
{
  // the bitmap may only be accessed by one thread at the same time
  // that's why we enter critical section
  EnterCriticalSection(&m_criticalSection);

  // if the existing bitmap object is of the right size
  if ((m_bmp_width == w) && (m_bmp_height == h) && w != 0 && h != 0) {
    // just replace the content
    SetBitmapBits(hBmp, w * h * 4, imBuffer);
  } else {
    if (hBmp != NULL) {
      // delete the old BITMAP
      DeleteObject(hBmp);
    }
    // create a new BITMAP from this buffer
    if ((hBmp = CreateBitmap(static_cast<int>(w), static_cast<int>(h), 1, 32, (void *)imBuffer)) == NULL)
      return false;

    m_bmp_width = w;
    m_bmp_height = h;
  }

  LeaveCriticalSection(&m_criticalSection);
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
bool vpGDIRenderer::updateBitmapROI(unsigned char *imBuffer, int i_min, int j_min, int w, int h)
{
  HBITMAP htmp = CreateBitmap(w, h, 1, 32, (void *)imBuffer);

  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);
  HDC hDCMem2 = CreateCompatibleDC(hDCScreen);

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);
  SelectObject(hDCMem2, htmp);

  BitBlt(hDCMem, j_min, i_min, w, h, hDCMem2, 0, 0, SRCCOPY);
  LeaveCriticalSection(&m_criticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
  DeleteObject(htmp);

  return true;
}

/*!
  Sets a pixel to color at position (j,i).

  \param iP : The pixel coordinates.
  \param color : the color of the point.
*/
void vpGDIRenderer::setPixel(const vpImagePoint &iP, const vpColor &color)
{
  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  if (color.id < vpColor::id_unknown)
    SetPixel(hDCMem, vpMath::round(iP.get_u() / m_rscale), vpMath::round(iP.get_v() / m_rscale), m_colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    SetPixel(hDCMem, vpMath::round(iP.get_u() / m_rscale), vpMath::round(iP.get_v() / m_rscale), gdicolor);
  }
  // display the result (flush)
  // BitBlt(hDCScreen, x, y, 1, 1, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
}

/*!
  Draws a line.
  \param ip1,ip2 : Initial and final image point.
  \param color the line's color
  \param thickness : Thickness of the line.
  \param style style of the line
*/
void vpGDIRenderer::drawLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                             unsigned int thickness, int style)
{
  HDC hDCScreen = NULL, hDCMem = NULL;
  HPEN hPen = NULL;
#ifdef GDI_ROBUST
  double start = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - start < 1000) {
    hDCScreen = GetDC(m_hWnd);
    if (!hDCScreen)
      continue;
    hDCMem = CreateCompatibleDC(hDCScreen);
    if (!hDCMem) {
      ReleaseDC(m_hWnd, hDCScreen);
      continue;
    }

    // create the pen
    if (color.id < vpColor::id_unknown)
      hPen = CreatePen(style, static_cast<int>(thickness), m_colors[color.id]);
    else {
      COLORREF gdicolor = RGB(color.R, color.G, color.B);
      hPen = CreatePen(style, static_cast<int>(thickness), gdicolor);
    }
    if (!hPen) {
      DeleteDC(hDCMem);
      ReleaseDC(m_hWnd, hDCScreen);
      continue;
    }
    if (!SetBkMode(hDCMem, TRANSPARENT)) {
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(m_hWnd, hDCScreen);
      continue;
    }

    // select this bmp in memory
    EnterCriticalSection(&m_criticalSection);

    if (!SelectObject(hDCMem, m_bmp)) {
      LeaveCriticalSection(&m_criticalSection);
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(m_hWnd, hDCScreen);
      continue;
    }

    // select the pen
    if (!SelectObject(hDCMem, hPen)) {
      LeaveCriticalSection(&m_criticalSection);
      DeleteObject(hPen);
      DeleteDC(hDCMem);
      ReleaseDC(m_hWnd, hDCScreen);
      continue;
    }
    break;
  }
  timelost += (vpTime::measureTimeMs() - start);
#else
  // get the window's DC
  hDCScreen = GetDC(m_hWnd);
  hDCMem = CreateCompatibleDC(hDCScreen);
  // create the pen
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(style, static_cast<int>(thickness), m_colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(style, static_cast<int>(thickness), gdicolor);
  }
  SetBkMode(hDCMem, TRANSPARENT);

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // select the pen
  SelectObject(hDCMem, hPen);
#endif
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

  LeaveCriticalSection(&m_criticalSection);

  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
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
void vpGDIRenderer::drawRect(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                             bool fill, unsigned int thickness)
{
  if (thickness == 0)
    thickness = 1;
  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // create the pen
  HPEN hPen;
  COLORREF gdicolor = RGB(0, 0, 0);

  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), m_colors[color.id]);
  else {
    gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  // create an hollow or solid brush (depends on boolean fill)
  LOGBRUSH lBrush;
  if (fill) {
    lBrush.lbStyle = BS_SOLID;
    if (color.id < vpColor::id_unknown)
      lBrush.lbColor = m_colors[color.id];
    else {
      lBrush.lbColor = gdicolor;
    }
  } else
    lBrush.lbStyle = BS_HOLLOW;
  HBRUSH hbrush = CreateBrushIndirect(&lBrush);

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // select the brush
  SelectObject(hDCMem, hbrush);
  // select the pen
  SelectObject(hDCMem, hPen);

  // draw the rectangle
  Rectangle(hDCMem, vpMath::round(topLeft.get_u() / m_rscale), vpMath::round(topLeft.get_v() / m_rscale),
            vpMath::round((topLeft.get_u() + width) / m_rscale), vpMath::round((topLeft.get_v() + height) / m_rscale));

  // display the result (flush)
  //  BitBlt(hDCScreen, j, i, width, height, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);

  DeleteObject(hbrush);
  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
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
  drawRect(ip, m_rwidth, m_rheight, color, true, 0);
}

/*!
  Draws a circle.
  \param center its center point's coordinates
  \param radius The circle's radius
  \param color The circle's color
  \param fill  When set to true fill the circle.
  \param thickness : Line thickness
*/
void vpGDIRenderer::drawCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill,
                               unsigned int thickness)
{

  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // create the pen
  HPEN hPen;
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), m_colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  // create an hollow brush
  LOGBRUSH lBrush;
  lBrush.lbStyle = BS_HOLLOW;
  HBRUSH hbrush = CreateBrushIndirect(&lBrush);

  // computes bounding rectangle
  int radius_ = static_cast<int>(radius);
  int x1 = vpMath::round(center.get_u() / m_rscale) - radius_ / m_rscale;
  int y1 = vpMath::round(center.get_v() / m_rscale) - radius_ / m_rscale;
  int x2 = vpMath::round(center.get_u() / m_rscale) + radius_ / m_rscale;
  int y2 = vpMath::round(center.get_v() / m_rscale) + radius_ / m_rscale;

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // select the brush
  SelectObject(hDCMem, hbrush);
  // select the pen
  SelectObject(hDCMem, hPen);

  // draw the circle
  if (fill == false)
    Ellipse(hDCMem, x1, y1, x2, y2);

  else {
    while (x2 - x1 > 0) {
      x1++;
      x2--;
      y1++;
      y2--;
      Ellipse(hDCMem, x1, y1, x2, y2);
    }
  }

  // display the result (flush)
  // BitBlt(hDCScreen, x1, y1, x2-x1, y2-y1, hDCMem, x1, y1, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);

  DeleteObject(hbrush);
  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
}

/*!
  Draws some text.
  \param ip its top left point's coordinates
  \param text The string to display
  \param color The text's color
*/
void vpGDIRenderer::drawText(const vpImagePoint &ip, const char *text, const vpColor &color)
{
  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // Select the font
  SelectObject(hDCMem, m_hFont);

  // set the text color
  if (color.id < vpColor::id_unknown)
    SetTextColor(hDCMem, m_colors[color.id]);
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

  // display the result (flush)
  // BitBlt(hDCScreen, j, i, size.cx, size.cy, hDCMem, j, i, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);

  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
}

/*!
  Draws a cross.
  \param ip its center point's coordinates
  \param size Size of the cross
  \param color The cross' color
  \param thickness width of the cross
*/
void vpGDIRenderer::drawCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness)
{
  /* unsigned */ int half_size = static_cast<int>(size / 2 / m_rscale);

  // if half_size is equal to zero, nothing is displayed with the code
  // just below. So, if half_size is equal to zero we just draw the
  // pixel.
  if (half_size) {
    // get the window's DC
    HDC hDCScreen = GetDC(m_hWnd);
    HDC hDCMem = CreateCompatibleDC(hDCScreen);

    // create the pen
    HPEN hPen;
    if (color.id < vpColor::id_unknown)
      hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), m_colors[color.id]);
    else {
      COLORREF gdicolor = RGB(color.R, color.G, color.B);
      hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
    }

    // select this bmp in memory
    EnterCriticalSection(&m_criticalSection);
    SelectObject(hDCMem, m_bmp);

    // select the pen
    SelectObject(hDCMem, hPen);

    // move to the starting point
    MoveToEx(hDCMem, vpMath::round(ip.get_u() / m_rscale) - half_size, vpMath::round(ip.get_v() / m_rscale), NULL);
    // Draw the first line (horizontal)
    LineTo(hDCMem, vpMath::round(ip.get_u() / m_rscale) + half_size, vpMath::round(ip.get_v() / m_rscale));

    // move to the starting point
    MoveToEx(hDCMem, vpMath::round(ip.get_u() / m_rscale), vpMath::round(ip.get_v() / m_rscale) - half_size, NULL);
    // Draw the second line (vertical)
    LineTo(hDCMem, vpMath::round(ip.get_u() / m_rscale), vpMath::round(ip.get_v() / m_rscale) + half_size);

    // display the result (flush)
    //  BitBlt(hDCScreen, j-(size/2), i-(size/2), size, size,
    //	   hDCMem, j-(size/2), i-(size/2), SRCCOPY);

    LeaveCriticalSection(&m_criticalSection);

    DeleteObject(hPen);
    DeleteDC(hDCMem);
    ReleaseDC(m_hWnd, hDCScreen);
  } else {
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
void vpGDIRenderer::drawArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w,
                              unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() / m_rscale - ip1.get_i() / m_rscale;
  double b = ip2.get_j() / m_rscale - ip1.get_j() / m_rscale;
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

  // computes the coordinates of the rectangle to blit later
  //   unsigned int x = (j2 >= j1) ? j1 : j2;
  //   unsigned int y = (i2 >= i1) ? i1 : i2;
  //   unsigned int w = (j2 >= j1) ? j2-j1 : j1-j2;
  //   unsigned int h = (i2 >= i1) ? i2-i1 : i1-i2;

  // get the window's DC
  HDC hDCScreen = GetDC(m_hWnd);
  HDC hDCMem = CreateCompatibleDC(hDCScreen);

  // create the pen
  HPEN hPen;
  if (color.id < vpColor::id_unknown)
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), m_colors[color.id]);
  else {
    COLORREF gdicolor = RGB(color.R, color.G, color.B);
    hPen = CreatePen(PS_SOLID, static_cast<int>(thickness), gdicolor);
  }

  // select this bmp in memory
  EnterCriticalSection(&m_criticalSection);
  SelectObject(hDCMem, m_bmp);

  // select the pen
  SelectObject(hDCMem, hPen);

  if ((a == 0) && (b == 0)) {
    // DisplayCrossLarge(i1,j1,3,col) ;
  } else {
    a /= lg;
    b /= lg;

    vpImagePoint ip3;
    ip3.set_i(ip2.get_i() / m_rscale - w * a);
    ip3.set_j(ip2.get_j() / m_rscale - w * b);

    vpImagePoint ip4;

    // double t = 0 ;
    // while (t<=_l)
    {
      ip4.set_i(ip3.get_i() - b * h);
      ip4.set_j(ip3.get_j() + a * h);

      if (lg > 2 * vpImagePoint::distance(ip2 / m_rscale, ip4)) {
        MoveToEx(hDCMem, vpMath::round(ip2.get_u() / m_rscale), vpMath::round(ip2.get_v() / m_rscale), NULL);
        LineTo(hDCMem, vpMath::round(ip4.get_u()), vpMath::round(ip4.get_v()));
      }
      // t+=0.1 ;
    }

    // t = 0 ;
    // while (t>= -_l)
    {
      ip4.set_i(ip3.get_i() + b * h);
      ip4.set_j(ip3.get_j() - a * h);

      if (lg > 2 * vpImagePoint::distance(ip2 / m_rscale, ip4)) {
        MoveToEx(hDCMem, vpMath::round(ip2.get_u() / m_rscale), vpMath::round(ip2.get_v() / m_rscale), NULL);
        LineTo(hDCMem, vpMath::round(ip4.get_u()), vpMath::round(ip4.get_v()));
      }

      // t-=0.1 ;
    }
    MoveToEx(hDCMem, vpMath::round(ip1.get_u() / m_rscale), vpMath::round(ip1.get_v() / m_rscale), NULL);
    LineTo(hDCMem, vpMath::round(ip2.get_u() / m_rscale), vpMath::round(ip2.get_v() / m_rscale));
  }

  // display the result (flush)
  //  BitBlt(hDCScreen, x, y, w, h, hDCMem, x, y, SRCCOPY);

  LeaveCriticalSection(&m_criticalSection);

  DeleteObject(hPen);
  DeleteDC(hDCMem);
  ReleaseDC(m_hWnd, hDCScreen);
}

/*!
  Gets the currently displayed image.
  \param I Image returned.
*/
void vpGDIRenderer::getImage(vpImage<vpRGBa> &I)
{
  // size of image buffer : m_rwidth*m_rheight*4
  unsigned int size = m_rwidth * m_rheight * 4;
  unsigned char *imBuffer = new unsigned char[size];

  // gets the hbitmap's bitmap
  GetBitmapBits(m_bmp, static_cast<LONG>(size), (void *)imBuffer);

  // resize the destination image as needed
  I.resize(m_rheight, m_rwidth);

  // copy the content
  for (unsigned int i = 0; i < size; i += 4) {
    I.bitmap[i >> 2].R = imBuffer[i + 2];
    I.bitmap[i >> 2].G = imBuffer[i + 1];
    I.bitmap[i >> 2].B = imBuffer[i + 0];
    I.bitmap[i >> 2].A = vpRGBa::alpha_default; // default opacity
  }

  delete[] imBuffer;
}
#endif
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpGDIRenderer.cpp.o) has no
// symbols
void dummy_vpGDIRenderer(){};
#endif
