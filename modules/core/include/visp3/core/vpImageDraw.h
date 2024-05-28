/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Drawing functions.
 */

/*!
  \file vpImageDraw.h

  \brief Drawing functions for image.
*/

#ifndef _vpImageDraw_h_
#define _vpImageDraw_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRect.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpImageDraw

  \ingroup group_core_image

  \brief Drawing functions for image.
*/
class VISP_EXPORT vpImageDraw
{

public:
  static void drawArrow(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                        unsigned char color, unsigned int w = 4, unsigned int h = 2, unsigned int thickness = 1);
  static void drawArrow(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                        unsigned int w = 4, unsigned int h = 2, unsigned int thickness = 1);

  static void drawCircle(vpImage<unsigned char> &I, const vpImageCircle &circle,
                         unsigned char color, unsigned int thickness = 1);
  static void drawCircle(vpImage<unsigned char> &I, const vpImagePoint &center, unsigned int radius,
                         unsigned char color, unsigned int thickness = 1);
  static void drawCircle(vpImage<vpRGBa> &I, const vpImageCircle &circle,
                         const vpColor &color, unsigned int thickness = 1);
  static void drawCircle(vpImage<vpRGBa> &I, const vpImagePoint &center, unsigned int radius, const vpColor &color,
                         unsigned int thickness = 1);

  static void drawCross(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size, unsigned char color,
                        unsigned int thickness = 1);
  static void drawCross(vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size, const vpColor &color,
                        unsigned int thickness = 1);

  static void drawDottedLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                             unsigned char color, unsigned int thickness = 1);
  static void drawDottedLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                             unsigned int thickness = 1);

  static void drawEllipse(vpImage<unsigned char> &I, const vpImagePoint &center, double coef1, double coef2,
                          double coef3, bool use_normalized_centered_moments, unsigned char color,
                          double smallalpha = 0, double highalpha = 2 * M_PI, unsigned int thickness = 1);
  static void drawEllipse(vpImage<vpRGBa> &I, const vpImagePoint &center, double coef1, double coef2, double coef3,
                          bool use_normalized_centered_moments, const vpColor &color, double smallalpha = 0,
                          double highalpha = 2 * M_PI, unsigned int thickness = 1);

  static void drawFrame(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                        double size, unsigned char color, unsigned int thickness = 1,
                        const vpImagePoint &offset = vpImagePoint(0, 0));
  static void drawFrame(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, double size,
                        const vpColor &color = vpColor::none, unsigned int thickness = 1,
                        const vpImagePoint &offset = vpImagePoint(0, 0));

  static void drawLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, unsigned char color,
                       unsigned int thickness = 1);
  static void drawLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                       unsigned int thickness = 1);

  static void drawPoint(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned char color,
                        unsigned int thickness = 1);
  static void drawPoint(vpImage<vpRGBa> &I, const vpImagePoint &ip, const vpColor &color, unsigned int thickness = 1);

  static void drawPolygon(vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip, unsigned char color,
                          unsigned int thickness = 1, bool closed = true);
  static void drawPolygon(vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip, const vpColor &color,
                          unsigned int thickness = 1, bool closed = true);

  static void drawRectangle(vpImage<unsigned char> &I, const vpRect &rectangle, unsigned char color, bool fill = false,
                            unsigned int thickness = 1);
  static void drawRectangle(vpImage<vpRGBa> &I, const vpRect &rectangle, const vpColor &color, bool fill = false,
                            unsigned int thickness = 1);
};
END_VISP_NAMESPACE
#endif
