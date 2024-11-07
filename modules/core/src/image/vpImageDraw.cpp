/****************************************************************************
 *
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
 *
*****************************************************************************/
// Contains code from:
/*
 * Simd Library (http://ermig1979.github.io/Simd).
 *
 * Copyright (c) 2011-2017 Yermalayeu Ihar.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPoint.h>

namespace
{
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
template <class Type>
void DrawLine(vpImage<Type> &canvas, int x1, int y1, int x2, int y2, const Type &color, unsigned int width = 1)
{
  const int w = static_cast<int>(canvas.getWidth()) - 1;
  const int h = static_cast<int>(canvas.getHeight()) - 1;

  if (x1 < 0 || y1 < 0 || (x1 - w) > 0 || (y1 - h) > 0 || x2 < 0 || y2 < 0 || (x2 - w) > 0 || (y2 - h) > 0) {
    if ((x1 < 0 && x2 < 0) || (y1 < 0 && y2 < 0) || ((x1 - w) > 0 && (x2 - w) > 0) || ((y1 - h) > 0 && (y2 - h) > 0)) {
      return;
    }

    if (y1 == y2) {
      x1 = std::min<int>(std::max<int>(x1, 0), w);
      x2 = std::min<int>(std::max<int>(x2, 0), w);
    }
    else if (x1 == x2) {
      y1 = std::min<int>(std::max<int>(y1, 0), h);
      y2 = std::min<int>(std::max<int>(y2, 0), h);
    }
    else {
      int x0 = (x1 * y2 - y1 * x2) / (y2 - y1);
      int y0 = (y1 * x2 - x1 * y2) / (x2 - x1);
      int xh = (x1 * y2 - y1 * x2 + h * (x2 - x1)) / (y2 - y1);
      int yw = (y1 * x2 - x1 * y2 + w * (y2 - y1)) / (x2 - x1);

      if (x1 < 0) {
        x1 = 0;
        y1 = y0;
      }
      if (x2 < 0) {
        x2 = 0;
        y2 = y0;
      }
      if (x1 > w) {
        x1 = w;
        y1 = yw;
      }
      if (x2 > w) {
        x2 = w;
        y2 = yw;
      }
      if ((y1 < 0 && y2 < 0) || (y1 > h && y2 > h)) {
        return;
      }

      if (y1 < 0) {
        x1 = x0;
        y1 = 0;
      }
      if (y2 < 0) {
        x2 = x0;
        y2 = 0;
      }

      if (y1 > h) {
        x1 = xh;
        y1 = h;
      }
      if (y2 > h) {
        x2 = xh;
        y2 = h;
      }
    }
  }

  const bool inverse = ((std::abs(y2 - y1) - std::abs(x2 - x1)) > 0);
  if (inverse) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const double dx = x2 - x1;
  const double dy = static_cast<double>(std::abs(y2 - y1));

  double error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y0 = y1 - static_cast<int>(width) / 2;

  for (int x = x1; x <= x2; x++) {
    for (int i = 0; i < static_cast<int>(width); i++) {
      int y = y0 + i;
      if (y >= 0) {
        if (inverse) {
          if (y < w) {
            canvas[x][y] = color;
          }
        }
        else {
          if (y < h) {
            canvas[y][x] = color;
          }
        }
      }
    }

    error -= dy;
    if (error < 0) {
      y0 += ystep;
      error += dx;
    }
  }
}

template <class Type>
void DrawCircle(vpImage<Type> &canvas, const vpImagePoint &center, int radius, const Type &color,
                unsigned int width = 1)
{
  const size_t n = 8 * std::max(static_cast<size_t>(1), static_cast<size_t>(::pow(radius, 0.5)));
  double px = 0, py = 0, da = 2 * M_PI / n;
  for (size_t i = 0; i <= n; i++) {
    double a = i * da;
    double cx = radius * ::cos(a) + center.get_u();
    double cy = radius * ::sin(a) + center.get_v();
    if (i > 0) {
      DrawLine(canvas, static_cast<int>(cx), static_cast<int>(cy), static_cast<int>(px), static_cast<int>(py), color,
               width);
    }
    px = cx;
    py = cy;
  }
}

template <class Type> void DrawFilledRectangle(vpImage<Type> &canvas, vpRect rect, const Type &color)
{
  rect &= vpRect(0, 0, canvas.getWidth(), canvas.getHeight());
  for (int row = static_cast<int>(rect.getTop()); row < static_cast<int>(rect.getBottom()); row++) {
    Type *dst = canvas[row];
    for (int col = static_cast<int>(rect.getLeft()); col < static_cast<int>(rect.getRight()); col++) {
      dst[col] = color;
    }
  }
}

template <class Type>
void DrawPolygon(vpImage<Type> &canvas, const std::vector<vpImagePoint> &polygon, const Type &color,
                 unsigned int width = 1, bool closed = true)
{
  if (closed) {
    for (size_t i = 0; i < polygon.size(); i++) {
      const vpImagePoint &p1 = (i ? polygon[i - 1] : polygon.back()), p2 = polygon[i];
      DrawLine(canvas, static_cast<int>(p1.get_u()), static_cast<int>(p1.get_v()), static_cast<int>(p2.get_u()),
               static_cast<int>(p2.get_v()), color, width);
    }
  }
  else {
    for (size_t i = 1; i < polygon.size(); i++) {
      DrawLine(canvas, static_cast<int>(polygon[i - 1].get_u()), static_cast<int>(polygon[i - 1].get_v()),
               static_cast<int>(polygon[i].get_u()), static_cast<int>(polygon[i].get_v()), color, width);
    }
  }
}

template <class Type>
void DrawRectangle(vpImage<Type> &canvas, const vpRect &rect, const Type &color, unsigned int width = 1)
{
  DrawLine(canvas, static_cast<int>(rect.getLeft()), static_cast<int>(rect.getTop()), static_cast<int>(rect.getRight()),
           static_cast<int>(rect.getTop()), color, width);
  DrawLine(canvas, static_cast<int>(rect.getRight()), static_cast<int>(rect.getTop()),
           static_cast<int>(rect.getRight()), static_cast<int>(rect.getBottom()), color, width);
  DrawLine(canvas, static_cast<int>(rect.getRight()), static_cast<int>(rect.getBottom()),
           static_cast<int>(rect.getLeft()), static_cast<int>(rect.getBottom()), color, width);
  DrawLine(canvas, static_cast<int>(rect.getLeft()), static_cast<int>(rect.getBottom()),
           static_cast<int>(rect.getLeft()), static_cast<int>(rect.getTop()), color, width);
}
} // namespace

BEGIN_VISP_NAMESPACE
/*!
  Draw an arrow from image point \e ip1 to image point \e ip2.
  \param[in,out] I : Image where to draw the arrow.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Arrow color.
  \param[in] w,h : Width and height of the arrow.
  \param[in] thickness : Thickness of the lines used to display the arrow.
*/
void vpImageDraw::drawArrow(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                            unsigned char color, unsigned int w, unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i();
  double b = ip2.get_j() - ip1.get_j();
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

  if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(b) <= std::numeric_limits<double>::epsilon())) {
    // DisplayCrossLarge(i1,j1,3,col) ;
  }
  else {
    a /= lg;
    b /= lg;

    vpImagePoint ip3;
    ip3.set_i(ip2.get_i() - w * a);
    ip3.set_j(ip2.get_j() - w * b);

    vpImagePoint ip4;
    ip4.set_i(ip3.get_i() - b * h);
    ip4.set_j(ip3.get_j() + a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4)) {
      drawLine(I, ip2, ip4, color, thickness);
    }

    ip4.set_i(ip3.get_i() + b * h);
    ip4.set_j(ip3.get_j() - a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4)) {
      drawLine(I, ip2, ip4, color, thickness);
    }

    drawLine(I, ip1, ip2, color, thickness);
  }
}

/*!
  Draw an arrow from image point \e ip1 to image point \e ip2.
  \param[in,out] I : Image where to draw the arrow.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Arrow color.
  \param[in] w,h : Width and height of the arrow.
  \param[in] thickness : Thickness of the lines used to display the arrow.
*/
void vpImageDraw::drawArrow(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                            unsigned int w, unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i();
  double b = ip2.get_j() - ip1.get_j();
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

  if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(b) <= std::numeric_limits<double>::epsilon())) {
    // DisplayCrossLarge(i1,j1,3,col) ;
  }
  else {
    a /= lg;
    b /= lg;

    vpImagePoint ip3;
    ip3.set_i(ip2.get_i() - w * a);
    ip3.set_j(ip2.get_j() - w * b);

    vpImagePoint ip4;
    ip4.set_i(ip3.get_i() - b * h);
    ip4.set_j(ip3.get_j() + a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4)) {
      drawLine(I, ip2, ip4, color, thickness);
    }

    ip4.set_i(ip3.get_i() + b * h);
    ip4.set_j(ip3.get_j() - a * h);

    if (lg > 2 * vpImagePoint::distance(ip2, ip4)) {
      drawLine(I, ip2, ip4, color, thickness);
    }

    drawLine(I, ip1, ip2, color, thickness);
  }
}

/*!
  Draw a circle in an image.
  \param[in,out] I : Image where to draw the circle.
  \param[in] circle : Circle to draw.
  \param[in] color : Circle color.
  \param[in] thickness : Thickness of the circle.
*/
void vpImageDraw::drawCircle(vpImage<unsigned char> &I, const vpImageCircle &circle,
                             unsigned char color, unsigned int thickness)
{
  DrawCircle(I, circle.getCenter(), static_cast<int>(circle.getRadius()), color, thickness);
}

/*!
  Draw a circle in an image.
  \param[in,out] I : Image where to draw the circle.
  \param[in] center : Circle center position.
  \param[in] radius : Circle radius.
  \param[in] color : Circle color.
  \param[in] thickness : Thickness of the circle.
*/
void vpImageDraw::drawCircle(vpImage<unsigned char> &I, const vpImagePoint &center, unsigned int radius,
                             unsigned char color, unsigned int thickness)
{
  DrawCircle(I, center, static_cast<int>(radius), color, thickness);
}

/*!
  Draw a circle in an image.
  \param[in,out] I : Image where to draw the circle.
  \param[in] circle : Circle to draw.
  \param[in] color : Circle color.
  \param[in] thickness : Thickness of the circle.
*/
void vpImageDraw::drawCircle(vpImage<vpRGBa> &I, const vpImageCircle &circle, const vpColor &color,
                             unsigned int thickness)
{
  DrawCircle(I, circle.getCenter(), static_cast<int>(circle.getRadius()), vpRGBa(color.R, color.G, color.B), thickness);
}

/*!
  Draw a circle in an image.
  \param[in,out] I : Image where to draw the circle.
  \param[in] center : Circle center position.
  \param[in] radius : Circle radius.
  \param[in] color : Circle color.
  \param[in] thickness : Thickness of the circle.
*/
void vpImageDraw::drawCircle(vpImage<vpRGBa> &I, const vpImagePoint &center, unsigned int radius, const vpColor &color,
                             unsigned int thickness)
{
  DrawCircle(I, center, static_cast<int>(radius), vpRGBa(color.R, color.G, color.B), thickness);
}

/*!
  Draw a cross in an image at position given by \e ip location.
  \param[in,out] I : Image where to draw the cross.
  \param[in] ip : Cross location.
  \param[in] size : Size (width and height) of the cross.
  \param[in] color : Cross color.
  \param[in] thickness : Thickness of the lines used to display the cross.
*/
void vpImageDraw::drawCross(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size, unsigned char color,
                            unsigned int thickness)
{
  vpImagePoint top, bottom, left, right;
  top.set_i(ip.get_i() - size / 2);
  top.set_j(ip.get_j());
  bottom.set_i(ip.get_i() + size / 2);
  bottom.set_j(ip.get_j());
  left.set_i(ip.get_i());
  left.set_j(ip.get_j() - size / 2);
  right.set_i(ip.get_i());
  right.set_j(ip.get_j() + size / 2);
  drawLine(I, top, bottom, color, thickness);
  drawLine(I, left, right, color, thickness);
}

/*!
  Draw a cross in an image at position given by \e ip location.
  \param[in,out] I : Image where to draw the cross.
  \param[in] ip : Cross location.
  \param[in] size : Size (width and height) of the cross.
  \param[in] color : Cross color.
  \param[in] thickness : Thickness of the lines used to display the cross.
*/
void vpImageDraw::drawCross(vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size, const vpColor &color,
                            unsigned int thickness)
{
  vpImagePoint top, bottom, left, right;
  top.set_i(ip.get_i() - size / 2);
  top.set_j(ip.get_j());
  bottom.set_i(ip.get_i() + size / 2);
  bottom.set_j(ip.get_j());
  left.set_i(ip.get_i());
  left.set_j(ip.get_j() - size / 2);
  right.set_i(ip.get_i());
  right.set_j(ip.get_j() + size / 2);
  drawLine(I, top, bottom, color, thickness);
  drawLine(I, left, right, color, thickness);
}

/*!
  Draw a dashed line in an image between two image points.
  \param[in,out] I : Image where to draw the dashed line.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Line color.
  \param[in] thickness : Dashed line thickness.
*/
void vpImageDraw::drawDottedLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                                 unsigned char color, unsigned int thickness)
{
  vpImagePoint ip1_ = ip1;
  vpImagePoint ip2_ = ip2;

  double size = 10;
  double length = sqrt(vpMath::sqr(ip2_.get_i() - ip1_.get_i()) + vpMath::sqr(ip2_.get_j() - ip1_.get_j()));
  bool vertical_line = static_cast<int>(ip2_.get_j()) == static_cast<int>(ip1_.get_j());
  if (vertical_line) {
    if (ip2_.get_i() < ip1_.get_i()) {
      std::swap(ip1_, ip2_);
    }
  }
  else if (ip2_.get_j() < ip1_.get_j()) {
    std::swap(ip1_, ip2_);
  }

  double diff_j = vertical_line ? 1 : ip2_.get_j() - ip1_.get_j();
  double deltaj = size / length * diff_j;
  double deltai = size / length * (ip2_.get_i() - ip1_.get_i());
  double slope = (ip2_.get_i() - ip1_.get_i()) / diff_j;
  double orig = ip1_.get_i() - slope * ip1_.get_j();

  if (vertical_line) {
    for (unsigned int i = static_cast<unsigned int>(ip1_.get_i()); i < ip2_.get_i();
         i += static_cast<unsigned int>(2 * deltai)) {
      double j = ip1_.get_j();
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j), color, thickness);
    }
  }
  else {
    for (unsigned int j = static_cast<unsigned int>(ip1_.get_j()); j < ip2_.get_j();
         j += static_cast<unsigned int>(2 * deltaj)) {
      double i = slope * j + orig;
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j + deltaj), color, thickness);
    }
  }
}

/*!
  Draw a dashed line in an image between two image points.
  \param[in,out] I : Image where to draw the dashed line.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Line color.
  \param[in] thickness : Dashed line thickness.
*/
void vpImageDraw::drawDottedLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                                 const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip1_ = ip1;
  vpImagePoint ip2_ = ip2;

  double size = 10;
  double length = sqrt(vpMath::sqr(ip2_.get_i() - ip1_.get_i()) + vpMath::sqr(ip2_.get_j() - ip1_.get_j()));
  bool vertical_line = static_cast<int>(ip2_.get_j()) == static_cast<int>(ip1_.get_j());
  if (vertical_line) {
    if (ip2_.get_i() < ip1_.get_i()) {
      std::swap(ip1_, ip2_);
    }
  }
  else if (ip2_.get_j() < ip1_.get_j()) {
    std::swap(ip1_, ip2_);
  }

  double diff_j = vertical_line ? 1 : ip2_.get_j() - ip1_.get_j();
  double deltaj = size / length * diff_j;
  double deltai = size / length * (ip2_.get_i() - ip1_.get_i());
  double slope = (ip2_.get_i() - ip1_.get_i()) / diff_j;
  double orig = ip1_.get_i() - slope * ip1_.get_j();

  if (vertical_line) {
    for (unsigned int i = static_cast<unsigned int>(ip1_.get_i()); i < ip2_.get_i();
         i += static_cast<unsigned int>(2 * deltai)) {
      double j = ip1_.get_j();
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j), color, thickness);
    }
  }
  else {
    for (unsigned int j = static_cast<unsigned int>(ip1_.get_j()); j < ip2_.get_j();
         j += static_cast<unsigned int>(2 * deltaj)) {
      double i = slope * j + orig;
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j + deltaj), color, thickness);
    }
  }
}

/*!
  Draw an ellipse in an image from its parameters expressed in pixels.
  \param[in,out] I : Image where to draw the ellipse.
  \param[in] center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e
  use_normalized_centered_moments these parameters are:
  - second order centered moments of the ellipse normalized by its area
    (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the
    centered moments and a the area) expressed in pixels.
  - the major and minor axis length in pixels and the eccentricity of the
  ellipse in radians: \f$a, b, e\f$.
  \param smallalpha : Smallest \f$ alpha \f$ angle in rad (0 for a complete ellipse).
  \param highalpha : Highest \f$ alpha \f$ angle in rad (2 \f$ \Pi \f$ for a complete ellipse).
  \param use_normalized_centered_moments : When false, the parameters coef1,
  coef2, coef3 are the parameters \f$a, b, e\f$. When true, the parameters
  coef1, coef2, coef3 are rather the normalized centered moments \f$n_{20}, n_{11},
  n_{02}\f$ expressed in pixels. In that case, we compute the parameters \e
  a, \e b and \e e from the centered moments.
  \param[in] color : Ellipse color.
  \param[in] thickness : Ellipse thickness.

  The following example shows how to use for example this function to draw
  the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I);

    vpImageDraw::drawEllipse(I, ellipse.getCenter(),
                             ellipse.get_nij()[0], ellipse.get_nij()[1], ellipse.get_nij()[2],
                             true,
                             ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                             vpColor::orange, 1);
  \endcode
*/
void vpImageDraw::drawEllipse(vpImage<unsigned char> &I, const vpImagePoint &center, double coef1, double coef2,
                              double coef3, bool use_normalized_centered_moments, unsigned char color,
                              double smallalpha, double highalpha, unsigned int thickness)
{
  double a = 0., b = 0., e = 0.;

  if (use_normalized_centered_moments) {
    // Chaumette, Image Moments: A General and Useful Set of Features for Visual Servoing, TRO 2004, eq 24
    // Similar code as in function vpMeEllipse::computeAbeFromNij() in vpMeEllipse.cpp
    double n20_p = coef1;
    double n11_p = coef2;
    double n02_p = coef3;
    double num = n20_p - n02_p;
    double d = num * num + 4.0 * n11_p * n11_p; // always >= 0

    if (d <= std::numeric_limits<double>::epsilon()) { // circle
      e = 0.0;                                         // case n20 = n02 and n11 = 0 : circle, e undefined
      a = b = 2.0 * sqrt(n20_p);
    }
    else {                             // real ellipse
      e = atan2(2.0 * n11_p, num) / 2.0; // e in [-Pi/2 ; Pi/2]
      d = sqrt(d);                       // d in sqrt always >= 0
      num = n20_p + n02_p;
      a = sqrt(2.0 * (num + d)); // term in sqrt always > 0
      b = sqrt(2.0 * (num - d)); // term in sqrt always > 0
    }
  }
  else {
    a = coef1;
    b = coef2;
    e = coef3;
  }

  // For all what follows similar code as in function vpMeEllipse::display() in vpMeEllipse.cpp

  // Approximation of the circumference of an ellipse:
  // [Ramanujan, S., "Modular Equations and Approximations to ,"
  // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
  double angle = highalpha - smallalpha;

  double t = (a - b) / (a + b);
  t *= t; // t^2
  double circumference = (angle / 2.0) * (a + b) * (1.0 + 3.0 * t / (10.0 + sqrt(4.0 - 3.0 * t)));
  unsigned int nbpoints = (unsigned int)(floor(circumference / 20));
  if (nbpoints < 10) {
    nbpoints = 10;
  }
  double incr = angle / nbpoints; // angle increment

  double u0 = center.get_u();
  double v0 = center.get_v();
  double cose = cos(e);
  double sine = sin(e);

  double u = a * cos(smallalpha); // equation of an ellipse
  double v = b * sin(smallalpha); // equation of an ellipse
  angle = smallalpha;
  // (i1,j1) are the coordinates on the origin centered ellipse ;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  vpImagePoint iP11;
  iP11.set_uv(u0 + cose * u - sine * v, v0 + sine * u + cose * v);

  // display the arc of the ellipse by successive small segments
  for (unsigned int i = 0; i < nbpoints; i++) {
    angle += incr;
    // Two concentric circles method used
    u = a * cos(angle);
    v = b * sin(angle);
    // to get the coordinates of the point on the shifted ellipse
    vpImagePoint iP22;
    iP22.set_uv(u0 + cose * u - sine * v, v0 + sine * u + cose * v);

    drawLine(I, iP11, iP22, color, thickness);

    iP11 = iP22;
  }
}

/*!
  Draw an ellipse in an image from its parameters expressed in pixels.
  \param[in,out] I : Image where to draw the ellipse.
  \param[in] center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e
  use_normalized_centered_moments these parameters are:
  - second order centered moments of the ellipse normalized by its area
    (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the
    centered moments and a the area) expressed in pixels.
  - the major and minor axis length in pixels and the eccentricity of the
  ellipse in radians: \f$a, b, e\f$.
  \param smallalpha : Smallest \f$ alpha \f$ angle in rad (0 for a complete ellipse).
  \param highalpha : Highest \f$ alpha \f$ angle in rad (2 \f$ \Pi \f$ for a complete ellipse).
  \param use_normalized_centered_moments : When false, the parameters coef1,
  coef2, coef3 are the parameters \f$a, b, e\f$. When true, the parameters
  coef1, coef2, coef3 are rather the normalized centered moments \f$n_{20}, n_{11},
  n_{02}\f$ expressed in pixels. In that case, we compute the parameters \e
  a, \e b and \e e from the centered moments.
  \param[in] color : Ellipse color.
  \param[in] thickness : Ellipse thickness.

  The following example shows how to use for example this function to draw
  the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I);

    vpImageDraw::drawEllipse(I, ellipse.getCenter(),
                             ellipse.get_nij()[0], ellipse.get_nij()[1], ellipse.get_nij()[2],
                             true,
                             ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                             vpColor::orange, 1);
  \endcode
*/
void vpImageDraw::drawEllipse(vpImage<vpRGBa> &I, const vpImagePoint &center, double coef1, double coef2, double coef3,
                              bool use_normalized_centered_moments, const vpColor &color, double smallalpha,
                              double highalpha, unsigned int thickness)
{
  double a = 0., b = 0., e = 0.;

  if (use_normalized_centered_moments) {
    // Chaumette, Image Moments: A General and Useful Set of Features for Visual Servoing, TRO 2004, eq 24
    // Similar code as in function vpMeEllipse::computeAbeFromNij() in vpMeEllipse.cpp
    double n20_p = coef1;
    double n11_p = coef2;
    double n02_p = coef3;
    double num = n20_p - n02_p;
    double d = num * num + 4.0 * n11_p * n11_p; // always >= 0

    if (d <= std::numeric_limits<double>::epsilon()) { // circle
      e = 0.0;                                         // case n20 = n02 and n11 = 0 : circle, e undefined
      a = b = 2.0 * sqrt(n20_p);
    }
    else {                             // real ellipse
      e = atan2(2.0 * n11_p, num) / 2.0; // e in [-Pi/2 ; Pi/2]
      d = sqrt(d);                       // d in sqrt always >= 0
      num = n20_p + n02_p;
      a = sqrt(2.0 * (num + d)); // term in sqrt always > 0
      b = sqrt(2.0 * (num - d)); // term in sqrt always > 0
    }
  }
  else {
    a = coef1;
    b = coef2;
    e = coef3;
  }

  // For all what follows similar code as in function vpMeEllipse::display() in vpMeEllipse.cpp

  // Approximation of the circumference of an ellipse:
  // [Ramanujan, S., "Modular Equations and Approximations to ,"
  // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
  double angle = highalpha - smallalpha;

  double t = (a - b) / (a + b);
  t *= t; // t^2
  double circumference = (angle / 2.0) * (a + b) * (1.0 + 3.0 * t / (10.0 + sqrt(4.0 - 3.0 * t)));
  unsigned int nbpoints = (unsigned int)(floor(circumference / 20));
  if (nbpoints < 10) {
    nbpoints = 10;
  }
  double incr = angle / nbpoints; // angle increment

  double u0 = center.get_u();
  double v0 = center.get_v();
  double cose = cos(e);
  double sine = sin(e);

  double u = a * cos(smallalpha); // equation of an ellipse
  double v = b * sin(smallalpha); // equation of an ellipse
  angle = smallalpha;
  // (i1,j1) are the coordinates on the origin centered ellipse ;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  vpImagePoint iP11;
  iP11.set_uv(u0 + cose * u - sine * v, v0 + sine * u + cose * v);

  // display the arc of the ellipse by successive small segments
  for (unsigned int i = 0; i < nbpoints; i++) {
    angle += incr;
    // Two concentric circles method used
    u = a * cos(angle);
    v = b * sin(angle);
    // to get the coordinates of the point on the shifted ellipse
    vpImagePoint iP22;
    iP22.set_uv(u0 + cose * u - sine * v, v0 + sine * u + cose * v);

    drawLine(I, iP11, iP22, color, thickness);

    iP11 = iP22;
  }
}

/*!
  Draw the projection in an image of an object frame represented by 3 arrows in
  the image. Red, green and blue arrows correspond to frame X, Y and Z axis respectively.

  \param[in,out] I : Image where to draw the ellipse.
  \param[in] cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param[in] cam : Camera intrinsic parameters.
  \param[in] size : Size of the object frame.
  \param[in] color : Color used to display the frame in the image.
  \param[in] thickness : the thickness of the line.
  \param[in] offset : Offset in pixels applied to the frame origin location in the
  image.
*/
void vpImageDraw::drawFrame(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                            double size, unsigned char color, unsigned int thickness, const vpImagePoint &offset)
{
  vpPoint o(0.0, 0.0, 0.0);
  vpPoint x(size, 0.0, 0.0);
  vpPoint y(0.0, size, 0.0);
  vpPoint z(0.0, 0.0, size);

  o.track(cMo);
  x.track(cMo);
  y.track(cMo);
  z.track(cMo);

  vpImagePoint ipo, ip1;

  vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo);

  vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ip1);
  drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

  vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ip1);
  drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

  vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ip1);
  drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);
}

/*!
  Draw the projection in an image of an object frame represented by 3 arrows in
  the image. Red, green and blue arrows correspond to frame X, Y and Z axis respectively.

  \param[in,out] I : Image where to draw the ellipse.
  \param[in] cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param[in] cam : Camera intrinsic parameters.
  \param[in] size : Size of the object frame.
  \param[in] color : Color used to display the frame in the image.
  \param[in] thickness : the thickness of the line.
  \param[in] offset : Offset in pixels applied to the frame origin location in the
  image.
*/
void vpImageDraw::drawFrame(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                            double size, const vpColor &color, unsigned int thickness, const vpImagePoint &offset)
{
  vpPoint o(0.0, 0.0, 0.0);
  vpPoint x(size, 0.0, 0.0);
  vpPoint y(0.0, size, 0.0);
  vpPoint z(0.0, 0.0, size);

  o.track(cMo);
  x.track(cMo);
  y.track(cMo);
  z.track(cMo);

  vpImagePoint ipo, ip1;

  if (color == vpColor::none) {
    vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo);

    vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, vpColor::red, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, vpColor::green, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, vpColor::blue, 4 * thickness, 2 * thickness, thickness);
  }
  else {
    vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo);

    vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);
  }
}

/*!
  Draw a line in an image between two image points.
  \param[in,out] I : Image where to draw the line.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Line color.
  \param[in] thickness : Dashed line thickness.
*/
void vpImageDraw::drawLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                           unsigned char color, unsigned int thickness)
{
  DrawLine(I, static_cast<int>(ip1.get_u()), static_cast<int>(ip1.get_v()), static_cast<int>(ip2.get_u()),
           static_cast<int>(ip2.get_v()), color, thickness);
}

/*!
  Draw a line in an image between two image points.
  \param[in,out] I : Image where to draw the line.
  \param[in] ip1,ip2 : Initial and final image points.
  \param[in] color : Line color.
  \param[in] thickness : Dashed line thickness.
*/
void vpImageDraw::drawLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color,
                           unsigned int thickness)
{
  DrawLine(I, static_cast<int>(ip1.get_u()), static_cast<int>(ip1.get_v()), static_cast<int>(ip2.get_u()),
           static_cast<int>(ip2.get_v()), vpRGBa(color.R, color.G, color.B), thickness);
}

/*!
  Draw in an image a point at the image point \e ip location.
  \param[in,out] I : Image where to draw the point.
  \param[in] ip : Point location.
  \param[in] color : Point color.
  \param[in] thickness : Thickness of the point
*/
void vpImageDraw::drawPoint(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned char color,
                            unsigned int thickness)
{
  drawRectangle(I, vpRect(ip, thickness, thickness), color, true);
}

/*!
  Draw in an image a point at the image point \e ip location.
  \param[in,out] I : Image where to draw the point.
  \param[in] ip : Point location.
  \param[in] color : Point color.
  \param[in] thickness : Thickness of the point
*/
void vpImageDraw::drawPoint(vpImage<vpRGBa> &I, const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  drawRectangle(I, vpRect(ip, thickness, thickness), color, true);
}

/*!
  Draw in an image a polygon defined by a vector of image points.
  \param[in,out] I : Image where to draw the polygon.
  \param[in] vip : Vector of image point that define the vertexes of the polygon.
  \param[in] color : Polygon color.
  \param[in] thickness : Polygon thickness.
  \param[in] closed : When true display a closed polygon with a segment between first and last image point.
*/
void vpImageDraw::drawPolygon(vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip, unsigned char color,
                              unsigned int thickness, bool closed)
{
  DrawPolygon(I, vip, color, thickness, closed);
}

/*!
  Draw in an image a polygon defined by a vector of image points.
  \param[in,out] I : Image where to draw the polygon.
  \param[in] vip : Vector of image point that define the vertexes of the polygon.
  \param[in] color : Polygon color.
  \param[in] thickness : Polygon thickness.
  \param[in] closed : When true display a closed polygon with a segment between first and last image point.
*/
void vpImageDraw::drawPolygon(vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip, const vpColor &color,
                              unsigned int thickness, bool closed)
{
  DrawPolygon(I, vip, vpRGBa(color.R, color.G, color.B), thickness, closed);
}

/*!
  Draw in an image a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param[in,out] I : Image where to draw the rectangle.
  \param[in] rectangle : Rectangle characteristics.
  \param[in] color : Rectangle color.
  \param[in] fill : When set to true fill the rectangle.
  \param[in] thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpImageDraw::drawRectangle(vpImage<unsigned char> &I, const vpRect &rectangle, unsigned char color, bool fill,
                                unsigned int thickness)
{
  if (fill) {
    DrawFilledRectangle(I, rectangle, color);
  }
  else {
    DrawRectangle(I, rectangle, color, thickness);
  }
}

/*!
  Draw in an image a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param[in,out] I : Image where to draw the rectangle.
  \param[in] rectangle : Rectangle characteristics.
  \param[in] color : Rectangle color.
  \param[in] fill : When set to true fill the rectangle.
  \param[in] thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpImageDraw::drawRectangle(vpImage<vpRGBa> &I, const vpRect &rectangle, const vpColor &color, bool fill,
                                unsigned int thickness)
{
  if (fill) {
    DrawFilledRectangle(I, rectangle, vpRGBa(color.R, color.G, color.B));
  }
  else {
    DrawRectangle(I, rectangle, vpRGBa(color.R, color.G, color.B), thickness);
  }
}
END_VISP_NAMESPACE
