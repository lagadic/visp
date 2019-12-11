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
 * Drawing functions.
 *
 *****************************************************************************/

#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>

#include <Simd/SimdLib.hpp>
#include <Simd/SimdDrawing.hpp>

typedef Simd::View<Simd::Allocator> View;

void vpImageDraw::drawArrow(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                            unsigned char color, unsigned int w, unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i();
  double b = ip2.get_j() - ip1.get_j();
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

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

void vpImageDraw::drawArrow(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                            const vpColor &color, unsigned int w, unsigned int h, unsigned int thickness)
{
  double a = ip2.get_i() - ip1.get_i();
  double b = ip2.get_j() - ip1.get_j();
  double lg = sqrt(vpMath::sqr(a) + vpMath::sqr(b));

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

void vpImageDraw::drawCircle(vpImage<unsigned char> &I, const vpImagePoint &center,
                             unsigned int radius, unsigned char color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth(), View::Gray8, I.bitmap);
  Simd::DrawCircle(canvas, Simd::Point<ptrdiff_t>(static_cast<ptrdiff_t>(center.get_u()), static_cast<ptrdiff_t>(center.get_u())),
                   static_cast<ptrdiff_t>(radius), color, thickness);
}

void vpImageDraw::drawCircle(vpImage<vpRGBa> &I, const vpImagePoint &center,
                             unsigned int radius, const vpColor &color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth()*sizeof(vpRGBa), View::Bgra32, I.bitmap);
  Simd::DrawCircle(canvas, Simd::Point<ptrdiff_t>(static_cast<ptrdiff_t>(center.get_u()), static_cast<ptrdiff_t>(center.get_u())),
                   static_cast<ptrdiff_t>(radius), Simd::Pixel::Bgra32(color.R, color.G, color.B, color.A), thickness);
}

void vpImageDraw::drawCross(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned int size,
                            unsigned char color, unsigned int thickness)
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

void vpImageDraw::drawCross(vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size,
                            const vpColor &color, unsigned int thickness)
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

void vpImageDraw::drawDottedLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                                 unsigned char color, unsigned int thickness)
{
  const int size = 10;
  const double length = sqrt(vpMath::sqr(ip2.get_i() - ip1.get_i()) + vpMath::sqr(ip2.get_j() - ip1.get_j()));

  unsigned int j1 = static_cast<unsigned int>(ip1.get_j());
  unsigned int j2 = static_cast<unsigned int>(ip2.get_j());

  if (j1 == j2) {
    unsigned int i1 = static_cast<unsigned int>(ip1.get_i());
    unsigned int i2 = static_cast<unsigned int>(ip2.get_i());

    for (unsigned int i = i1; i < i2; i += 2*size) {
      drawLine(I, vpImagePoint(i, j1), vpImagePoint(i + size, j1), color, thickness);
    }
  } else {
    double deltaj = size / length * (ip2.get_j() - ip1.get_j());
    double deltai = size / length * (ip2.get_i() - ip1.get_i());
    double slope = (ip2.get_i() - ip1.get_i()) / (ip2.get_j() - ip1.get_j());
    double orig = ip1.get_i() - slope * ip1.get_j();
    unsigned int increment = static_cast<unsigned int>(2*deltaj);

    for (unsigned int j = j1; j < j2; j += increment) {
      double i = slope * j + orig;
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j + deltaj), color, thickness);
    }
  }
}

void vpImageDraw::drawDottedLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness)
{
  const int size = 10;
  const double length = sqrt(vpMath::sqr(ip2.get_i() - ip1.get_i()) + vpMath::sqr(ip2.get_j() - ip1.get_j()));

  unsigned int j1 = static_cast<unsigned int>(ip1.get_j());
  unsigned int j2 = static_cast<unsigned int>(ip2.get_j());

  if (j1 == j2) {
    unsigned int i1 = static_cast<unsigned int>(ip1.get_i());
    unsigned int i2 = static_cast<unsigned int>(ip2.get_i());

    for (unsigned int i = i1; i < i2; i += 2*size) {
      drawLine(I, vpImagePoint(i, j1), vpImagePoint(i + size, j1), color, thickness);
    }
  } else {
    double deltaj = size / length * (ip2.get_j() - ip1.get_j());
    double deltai = size / length * (ip2.get_i() - ip1.get_i());
    double slope = (ip2.get_i() - ip1.get_i()) / (ip2.get_j() - ip1.get_j());
    double orig = ip1.get_i() - slope * ip1.get_j();
    unsigned int increment = static_cast<unsigned int>(2*deltaj);

    for (unsigned int j = j1; j < j2; j += increment) {
      double i = slope * j + orig;
      drawLine(I, vpImagePoint(i, j), vpImagePoint(i + deltai, j + deltaj), color, thickness);
    }
  }
}

void vpImageDraw::drawEllipse(vpImage<unsigned char> &I, const vpImagePoint &center, double coef1,
                              double coef2, double coef3, bool use_centered_moments, unsigned char color,
                              double theta1, double theta2, unsigned int thickness)
{
  double a = 0., b = 0., e = 0.;

  double mu20_p = coef1;
  double mu11_p = coef2;
  double mu02_p = coef3;

  if (use_centered_moments) {
    if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

      double val_p = sqrt(vpMath::sqr(mu20_p - mu02_p) + 4 * vpMath::sqr(mu11_p));
      a = sqrt((mu20_p + mu02_p + val_p) / 2);
      b = sqrt((mu20_p + mu02_p - val_p) / 2);

      e = (mu02_p - mu20_p + val_p) / (2 * mu11_p);
      e = atan(e);
    } else {
      a = sqrt(mu20_p);
      b = sqrt(mu02_p);
      e = 0.;
    }
  } else {
    a = coef1;
    b = coef2;
    e = coef3;
  }

  // Approximation of the circumference of an ellipse:
  // [Ramanujan, S., "Modular Equations and Approximations to ,"
  // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
  double t = (a - b) / (a + b);
  double circumference = M_PI * (a + b) * (1 + 3 * vpMath::sqr(t) / (10 + sqrt(4 - 3 * vpMath::sqr(t))));

  int nbpoints = (int)(floor(circumference / 5));
  if (nbpoints < 10) {
    nbpoints = 10;
  }
  double incr = 2 * M_PI / nbpoints; // angle increment

  double smallalpha = vpMath::rad(theta1);
  double highalpha = vpMath::rad(theta2);
  double ce = cos(e);
  double se = sin(e);

  double k = smallalpha;
  double j1 = a * cos(k); // equation of an ellipse
  double i1 = b * sin(k); // equation of an ellipse

  // (i1,j1) are the coordinates on the origin centered ellipse ;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  vpImagePoint iP11, iP22;
  iP11.set_j(center.get_j() + ce * j1 - se * i1);
  iP11.set_i(center.get_i() + se * j1 + ce * i1);

  while (k + incr < highalpha + incr) {
    double j2 = a * cos(k + incr); // equation of an ellipse
    double i2 = b * sin(k + incr); // equation of an ellipse

    // to get the coordinates of the point on the shifted ellipse
    iP22.set_j(center.get_j() + ce * j2 - se * i2);
    iP22.set_i(center.get_i() + se * j2 + ce * i2);

    drawLine(I, iP11, iP22, color, thickness);

    iP11 = iP22;

    k += incr;
  }
}

void vpImageDraw::drawEllipse(vpImage<vpRGBa> &I, const vpImagePoint &center, double coef1,
                              double coef2, double coef3, bool use_centered_moments, const vpColor &color,
                              double theta1, double theta2, unsigned int thickness)
{
  double a = 0., b = 0., e = 0.;

  double mu20_p = coef1;
  double mu11_p = coef2;
  double mu02_p = coef3;

  if (use_centered_moments) {
    if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

      double val_p = sqrt(vpMath::sqr(mu20_p - mu02_p) + 4 * vpMath::sqr(mu11_p));
      a = sqrt((mu20_p + mu02_p + val_p) / 2);
      b = sqrt((mu20_p + mu02_p - val_p) / 2);

      e = (mu02_p - mu20_p + val_p) / (2 * mu11_p);
      e = atan(e);
    } else {
      a = sqrt(mu20_p);
      b = sqrt(mu02_p);
      e = 0.;
    }
  } else {
    a = coef1;
    b = coef2;
    e = coef3;
  }

  // Approximation of the circumference of an ellipse:
  // [Ramanujan, S., "Modular Equations and Approximations to ,"
  // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
  double t = (a - b) / (a + b);
  double circumference = M_PI * (a + b) * (1 + 3 * vpMath::sqr(t) / (10 + sqrt(4 - 3 * vpMath::sqr(t))));

  int nbpoints = (int)(floor(circumference / 5));
  if (nbpoints < 10) {
    nbpoints = 10;
  }
  double incr = 2 * M_PI / nbpoints; // angle increment

  double smallalpha = vpMath::rad(theta1);
  double highalpha = vpMath::rad(theta2);
  double ce = cos(e);
  double se = sin(e);

  double k = smallalpha;
  double j1 = a * cos(k); // equation of an ellipse
  double i1 = b * sin(k); // equation of an ellipse

  // (i1,j1) are the coordinates on the origin centered ellipse ;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  vpImagePoint iP11, iP22;
  iP11.set_j(center.get_j() + ce * j1 - se * i1);
  iP11.set_i(center.get_i() + se * j1 + ce * i1);

  while (k + incr < highalpha + incr) {
    double j2 = a * cos(k + incr); // equation of an ellipse
    double i2 = b * sin(k + incr); // equation of an ellipse

    // to get the coordinates of the point on the shifted ellipse
    iP22.set_j(center.get_j() + ce * j2 - se * i2);
    iP22.set_i(center.get_i() + se * j2 + ce * i2);

    drawLine(I, iP11, iP22, color, thickness);

    iP11 = iP22;

    k += incr;
  }
}

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
  } else {
    vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo);

    vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);

    vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ip1);
    drawArrow(I, ipo + offset, ip1 + offset, color, 4 * thickness, 2 * thickness, thickness);
  }
}

void vpImageDraw::drawLine(vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                           unsigned char color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth(), View::Gray8, I.bitmap);
  Simd::DrawLine(canvas, static_cast<ptrdiff_t>(ip1.get_u()), static_cast<ptrdiff_t>(ip1.get_v()),
                 static_cast<ptrdiff_t>(ip2.get_u()), static_cast<ptrdiff_t>(ip2.get_v()),
                 color, thickness);
}

void vpImageDraw::drawLine(vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, const vpColor &color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth()*sizeof(vpRGBa), View::Bgra32, I.bitmap);
  Simd::DrawLine(canvas, static_cast<ptrdiff_t>(ip1.get_u()), static_cast<ptrdiff_t>(ip1.get_v()),
                 static_cast<ptrdiff_t>(ip2.get_u()), static_cast<ptrdiff_t>(ip2.get_v()),
                 Simd::Pixel::Bgra32(color.R, color.G, color.B, color.A), thickness);
}

void vpImageDraw::drawPoint(vpImage<unsigned char> &I, const vpImagePoint &ip, unsigned char color, unsigned int thickness)
{
  drawRectangle(I, vpRect(ip, thickness, thickness), color, true);
}

void vpImageDraw::drawPoint(vpImage<vpRGBa> &I, const vpImagePoint &ip, const vpColor &color, unsigned int thickness)
{
  drawRectangle(I, vpRect(ip, thickness, thickness), color, true);
}

namespace
{
std::vector<Simd::Point<ptrdiff_t> > convertTo(const std::vector<vpImagePoint> &vip)
{
  std::vector<Simd::Point<ptrdiff_t> > vec(vip.size());

  for (size_t i = 0; i < vip.size(); i++) {
    vec[i].x = static_cast<ptrdiff_t>(vip[i].get_u());
    vec[i].y = static_cast<ptrdiff_t>(vip[i].get_v());
  }

  return  vec;
}
}

void vpImageDraw::drawPolygon(vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              unsigned char color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth(), View::Gray8, I.bitmap);
  Simd::DrawPolygon(canvas, convertTo(vip), color, thickness);
}

void vpImageDraw::drawPolygon(vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip,
                              const vpColor &color, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth()*sizeof(vpRGBa), View::Bgra32, I.bitmap);
  Simd::DrawPolygon(canvas, convertTo(vip), Simd::Pixel::Bgra32(color.R, color.G, color.B, color.A), thickness);
}

void vpImageDraw::drawRectangle(vpImage<unsigned char> &I, const vpRect &rectangle, unsigned char color, bool fill, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth(), View::Gray8, I.bitmap);
  if (fill) {
    Simd::DrawFilledRectangle(canvas,
                              Simd::Rectangle<ptrdiff_t>(static_cast<ptrdiff_t>(rectangle.getLeft()), static_cast<ptrdiff_t>(rectangle.getTop()),
                                                         static_cast<ptrdiff_t>(rectangle.getRight()), static_cast<ptrdiff_t>(rectangle.getBottom())),
                              color);
  } else {
    Simd::DrawRectangle(canvas, static_cast<ptrdiff_t>(rectangle.getLeft()), static_cast<ptrdiff_t>(rectangle.getTop()),
                        static_cast<ptrdiff_t>(rectangle.getRight()), static_cast<ptrdiff_t>(rectangle.getBottom()),
                        color, thickness);
  }
}

void vpImageDraw::drawRectangle(vpImage<vpRGBa> &I, const vpRect &rectangle, const vpColor &color, bool fill, unsigned int thickness)
{
  View canvas(I.getWidth(), I.getHeight(), I.getWidth()*sizeof(vpRGBa), View::Bgra32, I.bitmap);
  if (fill) {
    Simd::DrawFilledRectangle(canvas,
                              Simd::Rectangle<ptrdiff_t>(static_cast<ptrdiff_t>(rectangle.getLeft()), static_cast<ptrdiff_t>(rectangle.getTop()),
                                                         static_cast<ptrdiff_t>(rectangle.getRight()), static_cast<ptrdiff_t>(rectangle.getBottom())),
                              Simd::Pixel::Bgra32(color.R, color.G, color.B, color.A));
  } else {
    Simd::DrawRectangle(canvas, static_cast<ptrdiff_t>(rectangle.getLeft()), static_cast<ptrdiff_t>(rectangle.getTop()),
                        static_cast<ptrdiff_t>(rectangle.getRight()), static_cast<ptrdiff_t>(rectangle.getBottom()),
                        Simd::Pixel::Bgra32(color.R, color.G, color.B, color.A), thickness);
  }
}
