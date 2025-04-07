/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

#ifndef VP_GDI_RENDERER_H
#define VP_GDI_RENDERER_H

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_GDI))

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// Mute warning with clang-cl
// warning : non-portable path to file '<WinSock2.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
// warning : non-portable path to file '<Windows.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wnonportable-system-include-path"
#endif

// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif

#include <visp3/core/vpDisplayException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/gui/vpWin32Renderer.h>

#include <visp3/core/vpMath.h>

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpGDIRenderer : public vpWin32Renderer
{
  // the handle of the associated window
  HWND m_hWnd;

  // the bitmap object to display
  HBITMAP m_bmp;

  // colors for overlay
  COLORREF m_colors[vpColor::id_unknown];

  // font used to draw text
  HFONT m_hFont;

  // used to ensure that only one thread at a time is accessing bmp
  CRITICAL_SECTION m_criticalSection;

  unsigned int m_bmp_width;
  unsigned int m_bmp_height;

public:
  double timelost;
  vpGDIRenderer();
  virtual ~vpGDIRenderer() VP_OVERRIDE;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  vpGDIRenderer(const vpGDIRenderer &) = default;
  vpGDIRenderer &operator=(const vpGDIRenderer &) = default;
#endif

  bool init(HWND hWnd, unsigned int width, unsigned int height) VP_OVERRIDE;

  bool render() VP_OVERRIDE;

  void setImg(const vpImage<vpRGBa> &I) VP_OVERRIDE;
  void setImg(const vpImage<unsigned char> &I) VP_OVERRIDE;
  void setImgROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, unsigned int width, unsigned int height) VP_OVERRIDE;
  void setImgROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int width, unsigned int height);

  void setPixel(const vpImagePoint &iP, const vpColor &color) VP_OVERRIDE;

  void drawLine(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness,
                int style = PS_SOLID) VP_OVERRIDE;

  void drawRect(const vpImagePoint &topLeft, unsigned int width, unsigned int height, const vpColor &color,
                bool fill = false, unsigned int thickness = 1) VP_OVERRIDE;

  void clear(const vpColor &color) VP_OVERRIDE;

  void drawCircle(const vpImagePoint &center, unsigned int radius, const vpColor &color, bool fill = false,
                  unsigned int thickness = 1) VP_OVERRIDE;

  void drawText(const vpImagePoint &ip, const char *text, const vpColor &color) VP_OVERRIDE;

  void drawCross(const vpImagePoint &ip, unsigned int size, const vpColor &color, unsigned int thickness = 1) VP_OVERRIDE;

  void drawArrow(const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int w, unsigned int h,
                 unsigned int thickness = 1) VP_OVERRIDE;

  void getImage(vpImage<vpRGBa> &I) VP_OVERRIDE;

private:
  // updates the renderer hbitmaps.
  bool updateBitmap(HBITMAP &hBmp, unsigned char *imBuffer, unsigned int w, unsigned int h);
  // updates the renderer hbitmaps.
  bool updateBitmapROI(unsigned char *imBuffer, int i_min, int j_min, int w, int h);

  // converts a vpImage<vpRGBa> into a HBITMAP .
  void convert(const vpImage<vpRGBa> &I, HBITMAP &hBmp);

  // converst a vpImage<unsigned char> into a HBITMAP .
  void convert(const vpImage<unsigned char> &I, HBITMAP &hBmp);

  // converts a vpImage<vpRGBa> into a HBITMAP .
  void convertROI(const vpImage<vpRGBa> &I, const vpImagePoint &iP, unsigned int width, unsigned int height);

  // converst a vpImage<unsigned char> into a HBITMAP .
  void convertROI(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int width, unsigned int height);
};

END_VISP_NAMESPACE
#endif
#endif
#endif
