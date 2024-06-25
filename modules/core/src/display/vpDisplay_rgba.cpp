/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Display implementation.
 *
*****************************************************************************/

#include <visp3/core/vpDisplay.h>

#include "vpDisplay_impl.h"

//************************************************************************
// Modifications done in this file should be reported in all vpDisplay_*.cpp
// files that implement other types (unsigned char, vpRGB, vpRGBa)
//************************************************************************
BEGIN_VISP_NAMESPACE
/*!
  Close the display attached to I.
*/
void vpDisplay::close(vpImage<vpRGBa> &I) { vp_display_close(I); }

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplay::displayArrow(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                             const vpColor &color, unsigned int w, unsigned int h, unsigned int thickness)
{
  vp_display_display_arrow(I, ip1, ip2, color, w, h, thickness);
}

/*!
  Display an arrow from image point (i1,j1) to  image point (i2,j2).

  \param I : The image associated to the display.
  \param i1,j1 : Initial image point.
  \param i2,j2 : Final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplay::displayArrow(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                             unsigned int w, unsigned int h, unsigned int thickness)
{
  vp_display_display_arrow(I, i1, j1, i2, j2, color, w, h, thickness);
}

/*!
  Display the projection of an object camera represented by a cone in
  the image.

  \param I : The image associated to the display.
  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param cam : Camera intrinsic parameters.
  \param size : Size of the object camera.
  \param color : Color used to display the camera in the image.
  \param thickness : Thickness of the graphics drawing.
*/
void vpDisplay::displayCamera(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                              double size, const vpColor &color, unsigned int thickness)
{
  vp_display_display_camera(I, cMo, cam, size, color, thickness);
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated Display a string at the image point \e ip location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.

  \sa setFont(), displayText()
*/
void vpDisplay::displayCharString(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const char *string,
                                  const vpColor &color)
{
  vp_display_display_text(I, ip, string, color);
}

/*!
  \deprecated Display a string at the image point (i,j) location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.

  \sa setFont(), displayText()
*/
void vpDisplay::displayCharString(const vpImage<vpRGBa> &I, int i, int j, const char *string, const vpColor &color)
{
  vp_display_display_text(I, i, j, string, color);
}
#endif

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param circle: Circle to display.
  \param color : Circle color.
  \param fill : When set to true fill the circle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void vpDisplay::displayCircle(const vpImage<vpRGBa> &I, const vpImageCircle &circle,
                              const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_circle(I, circle.getCenter(), static_cast<unsigned int>(circle.getRadius()), color, fill, thickness);
}

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void vpDisplay::displayCircle(const vpImage<vpRGBa> &I, const vpImagePoint &center, unsigned int radius,
                              const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_circle(I, center, radius, color, fill, thickness);
}

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param i,j : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.
  \param thickness : Thickness of the circle. This parameter is only useful
  when \e fill is set to false.
*/
void vpDisplay::displayCircle(const vpImage<vpRGBa> &I, int i, int j, unsigned int radius, const vpColor &color,
                              bool fill, unsigned int thickness)
{
  vp_display_display_circle(I, i, j, radius, color, fill, thickness);
}

/*!
  Display a cross at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross(const vpImage<vpRGBa> &I, const vpImagePoint &ip, unsigned int size, const vpColor &color,
                             unsigned int thickness)
{
  vp_display_display_cross(I, ip, size, color, thickness);
}

/*!
  Display a cross at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross(const vpImage<vpRGBa> &I, int i, int j, unsigned int size, const vpColor &color,
                             unsigned int thickness)
{
  vp_display_display_cross(I, i, j, size, color, thickness);
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                               const vpColor &color, unsigned int thickness)
{
  vp_display_display_dot_line(I, ip1, ip2, color, thickness);
}

/*!
  Display a dashed line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                               unsigned int thickness)
{
  vp_display_display_dot_line(I, i1, j1, i2, j2, color, thickness);
}

/*!
  Display the dashed lines formed by the list of image points
  \param I : The image associated to the display.
  \param ips : Vector of image points.
  \param closeTheShape : If true, display a dashed line from the first and
  last image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips, bool closeTheShape,
                               const vpColor &color, unsigned int thickness)
{
  if (ips.size() <= 1) {
    return;
  }
  size_t ips_size = ips.size();
  for (size_t i = 0; i < (ips_size - 1); ++i) {
    vp_display_display_dot_line(I, ips[i], ips[i + 1], color, thickness);
  }

  if (closeTheShape) {
    vp_display_display_dot_line(I, ips.front(), ips.back(), color, thickness);
  }
}

/*!
  Display the dashed lines formed by the list of image points
  \param I : The image associated to the display.
  \param ips : List of image points.
  \param closeTheShape : If true, display a dashed line from the first and
  last image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine(const vpImage<vpRGBa> &I, const std::list<vpImagePoint> &ips, bool closeTheShape,
                               const vpColor &color, unsigned int thickness)
{
  if (ips.size() <= 1) {
    return;
  }

  std::list<vpImagePoint>::const_iterator it = ips.begin();

  vpImagePoint ip_prev = *(++it);
  std::list<vpImagePoint>::const_iterator ips_end = ips.end();
  for (; it != ips_end; ++it) {
    if (vpImagePoint::distance(ip_prev, *it) > 1) {
      vp_display_display_dot_line(I, ip_prev, *it, color, thickness);
      ip_prev = *it;
    }
  }

  if (closeTheShape) {
    vp_display_display_dot_line(I, ips.front(), ips.back(), color, thickness);
  }
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e
  use_normalized_centered_moments these parameters are:
  - second order centered moments of the ellipse normalized by its area
    (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the
    centered moments and a the area) expressed in pixels.
  - the major and minor axis length in pixels and the eccentricity of the
  ellipse in radians: \f$a, b, e\f$.
  \param use_normalized_centered_moments : When false,
  the parameters coef1, coef2, coef3 are the parameters \f$a, b, e\f$. When
  true, the parameters coef1, coef2, coef3 are rather the normalized centered moments
  \f$n_{20}, n_{11}, n_{02}\f$ expressed in pixels. In that case, we
  compute the parameters \e a, \e b and \e e from the centered moments.
  \param color : Ellipse color.
  \param thickness : Ellipse thickness.
  \param display_center : Display a cross at the center of the ellipse.
  \param display_arc : Display a line between the center and the first arc extremity
  and a line between the center and the second arc extremity.

  The following example shows how to use for example this function to display
  the result of a tracking.
  \code
  vpMeEllipse ellipse;
  ...
  vpDisplay::display(I);
  ellipse.track(I);

  vpDisplay::displayEllipse(I, ellipse.getCenter(),
                            ellipse.get_nij()[0], ellipse.get_nij()[1], ellipse.get_nij()[2],
                            true, vpColor::orange, 1);
  vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &coef1,
                               const double &coef2, const double &coef3, bool use_normalized_centered_moments,
                               const vpColor &color, unsigned int thickness, bool display_center, bool display_arc)
{
  vpDisplay::displayEllipse(I, center, coef1, coef2, coef3, 0., 2 * M_PI, use_normalized_centered_moments, color,
                            thickness, display_center, display_arc);
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
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
  \param color : Ellipse color.
  \param thickness : Ellipse thickness.
  \param display_center : Display a cross at the center of the ellipse.
  \param display_arc : Display a line between the center and the first arc extremity
  and a line between the center and the second arc extremity.

  The following example shows how to use for example this function to display
  the result of a tracking.
  \code
  vpMeEllipse ellipse;
  ...
  vpDisplay::display(I);
  ellipse.track(I);

  vpDisplay::displayEllipse(I, ellipse.getCenter(),
                            ellipse.get_nij()[0], ellipse.get_nij()[1], ellipse.get_nij()[2],
                            ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                            true, vpColor::orange, 1);
  vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &coef1,
                               const double &coef2, const double &coef3, const double &smallalpha,
                               const double &highalpha, bool use_normalized_centered_moments, const vpColor &color,
                               unsigned int thickness, bool display_center, bool display_arc)
{
  vp_display_display_ellipse(I, center, coef1, coef2, coef3, smallalpha, highalpha, use_normalized_centered_moments,
                             color, thickness, display_center, display_arc);
}

/*!
  Display the projection of an object frame represented by 3 arrows in
  the image. Red, green and blue arrows correspond to frame X, Y and Z axis respectively.

  \param I : The image associated to the display.
  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.
  \param cam : Camera intrinsic parameters.
  \param size : Size of the object frame.
  \param color : Color used to display the frame in the image.
  \param thickness : the thickness of the line.
  \param offset : Offset in pixels applied to the frame origin location in the image.
  \param frameName : Text to display along side the origin of the frame.
  \param textColor : Color of the text associated to `frameName`.
  \param textOffset : Offset used to shift the text from the origin of the frame.
*/
void vpDisplay::displayFrame(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                             double size, const vpColor &color, unsigned int thickness, const vpImagePoint &offset,
                             const std::string &frameName, const vpColor &textColor, const vpImagePoint &textOffset)
{
  vp_display_display_frame(I, cMo, cam, size, color, thickness, offset, frameName, textColor, textOffset);
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
  \param segment: If true (default) display the segment between the two image points.
  If false, display the line passing through the two image points.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                            const vpColor &color, unsigned int thickness, bool segment)
{
  displayLine(I, static_cast<int>(ip1.get_i()), static_cast<int>(ip1.get_j()), static_cast<int>(ip2.get_i()),
              static_cast<int>(ip2.get_j()), color, thickness, segment);
}

/*!
  Display a line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Line thickness.
  \param segment: If true (default) display the segment between the two image points.
  If false, display the line passing through the two image points.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, int i1, int j1, int i2, int j2, const vpColor &color,
                            unsigned int thickness, bool segment)
{
  if (segment) {
    vp_display_display_line(I, i1, j1, i2, j2, color, thickness);
  }
  else {
    // line equation in image: i = a * j + b
    double delta_j = static_cast<double>(j2) - static_cast<double>(j1);
    double delta_i = static_cast<double>(i2) - static_cast<double>(i1);
    // Test if horizontal line
    if (std::fabs(delta_i) <= std::numeric_limits<double>::epsilon()) {
      vp_display_display_line(I, i1, 0, i1, (I.getWidth() - 1), color, thickness);
    }
    // Test if vertical line
    else if (std::fabs(delta_j) <= std::numeric_limits<double>::epsilon()) {
      vp_display_display_line(I, 0, j1, (I.getHeight() - 1), j1, color, thickness);
    }
    else {
      double a = delta_i / delta_j;
      double b = static_cast<double>(i1) - (a * static_cast<double>(j1));
      std::vector<vpImagePoint> vip; // Image points that intersect image borders
      // Test intersection with vertical line j=0
      vpImagePoint ip_left(b, 0);
      if ((ip_left.get_i() >= 0.) && (ip_left.get_i() <= (I.getHeight() - 1.))) {
        vip.push_back(ip_left);
      }
      // Test intersection with vertical line j=width-1
      vpImagePoint ip_right((a * (I.getWidth() - 1)) + b, I.getWidth() - 1.);
      if ((ip_right.get_i() >= 0.) && (ip_right.get_i() <= (I.getHeight() - 1.))) {
        vip.push_back(ip_right);
      }
      if (vip.size() == 2) {
        vp_display_display_line(I, vip[0], vip[1], color, thickness);
        return;
      }
      // Test intersection with horizontal line i=0
      vpImagePoint ip_top(0, -b / a);
      if ((ip_top.get_j() >= 0.) && (ip_top.get_j() <= (I.getWidth() - 1.))) {
        vip.push_back(ip_top);
      }
      if (vip.size() == 2) {
        vp_display_display_line(I, vip[0], vip[1], color, thickness);
        return;
      }
      // Test intersection with horizontal line i=height-1
      vpImagePoint ip_bottom(I.getHeight() - 1., (I.getHeight() - 1. - b) / a);
      if ((ip_bottom.get_j() >= 0.) && (ip_bottom.get_j() <= (I.getWidth() - 1.))) {
        vip.push_back(ip_bottom);
      }
      if (vip.size() == 2) {
        vp_display_display_line(I, vip[0], vip[1], color, thickness);
        return;
      }
    }
  }
}

/*!
  Display the lines formed by the list of image points.
  \param I : The image associated to the display.
  \param ips : Vector of image points.
  \param closeTheShape : If true, draw a line from the first and last image
  points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &ips, bool closeTheShape,
                            const vpColor &color, unsigned int thickness)
{
  if (ips.size() <= 1) {
    return;
  }

  size_t ips_size = ips.size();
  for (size_t i = 0; i < (ips_size - 1); ++i) {
    vp_display_display_line(I, ips[i], ips[i + 1], color, thickness);
  }

  if (closeTheShape) {
    vp_display_display_line(I, ips.front(), ips.back(), color, thickness);
  }
}

/*!
  Display the lines formed by the list of image points.
  \param I : The image associated to the display.
  \param ips : List of image points.
  \param closeTheShape : If true, draw a line from the first and last image
  points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine(const vpImage<vpRGBa> &I, const std::list<vpImagePoint> &ips, bool closeTheShape,
                            const vpColor &color, unsigned int thickness)
{
  if (ips.size() <= 1) {
    return;
  }

  std::list<vpImagePoint>::const_iterator it = ips.begin();

  vpImagePoint ip_prev = *(++it);
  std::list<vpImagePoint>::const_iterator ips_end = ips.end();
  for (; it != ips_end; ++it) {
    if (vpImagePoint::distance(ip_prev, *it) > 1) {
      vp_display_display_line(I, ip_prev, *it, color, thickness);
      ip_prev = *it;
    }
  }

  if (closeTheShape) {
    vp_display_display_line(I, ips.front(), ips.back(), color, thickness);
  }
}

/*!
  Display a point at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const vpColor &color,
                             unsigned int thickness)
{
  vp_display_display_point(I, ip, color, thickness);
}

/*!
  Display a point at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint(const vpImage<vpRGBa> &I, int i, int j, const vpColor &color, unsigned int thickness)
{
  vp_display_display_point(I, i, j, color, thickness);
}

/*!
  Display a polygon defined by a vector of image points.
  \param I : The image associated to the display.
  \param vip : Vector of image point that define the vertexes of the polygon.
  \param color : Line color.
  \param thickness : Line thickness.
  \param closed : When true display a closed polygon with a segment between first and last image point.
*/
void vpDisplay::displayPolygon(const vpImage<vpRGBa> &I, const std::vector<vpImagePoint> &vip, const vpColor &color,
                               unsigned int thickness, bool closed)
{
  vp_display_display_polygon(I, vip, color, thickness, closed);
}

/*!
  Display a polygon defined by a set of image points.
  \param I : The image associated to the display.
  \param polygon : Polygon to display.
  \param color : Line color.
  \param thickness : Line thickness.
  \param closed : When true display a closed polygon with a segment between first and last image point.
*/
void vpDisplay::displayPolygon(const vpImage<vpRGBa> &I, const vpPolygon &polygon, const vpColor &color,
                               unsigned int thickness, bool closed)
{
  vp_display_display_polygon(I, polygon, color, thickness, closed);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &topLeft, unsigned int width,
                                 unsigned int height, const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, topLeft, width, height, color, fill, thickness);
}

/*!
  Display a rectangle with (i,j) as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param i,j : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, int i, int j, unsigned int width, unsigned int height,
                                 const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, i, j, width, height, color, fill, thickness);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpRect &rectangle, const vpColor &color, bool fill,
                                 unsigned int thickness)
{
  vp_display_display_rectangle(I, rectangle, color, fill, thickness);
}

/*!
  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param center : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &center, float angle, unsigned int width,
                                 unsigned int height, const vpColor &color, unsigned int thickness)
{
  vp_display_display_rectangle(I, center, angle, width, height, color, thickness);
}

/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle. When vpDisplayOpenCV is used,
  and color alpha channel is set, filling feature can handle transparency. See vpColor
  header class documentation.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, const vpImagePoint &topLeft, const vpImagePoint &bottomRight,
                                 const vpColor &color, bool fill, unsigned int thickness)
{
  vp_display_display_rectangle(I, topLeft, bottomRight, color, fill, thickness);
}

/*!
  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param i,j : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle.
*/
void vpDisplay::displayRectangle(const vpImage<vpRGBa> &I, unsigned int i, unsigned int j, float angle,
                                 unsigned int width, unsigned int height, const vpColor &color, unsigned int thickness)
{
  vp_display_display_rectangle(I, i, j, angle, width, height, color, thickness);
}

/*!
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplay::displayText(const vpImage<vpRGBa> &I, const vpImagePoint &ip, const std::string &s,
                            const vpColor &color)
{
  vp_display_display_text(I, ip, s, color);
}

/*!
  Display a string at the image point (i,j) location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplay::displayText(const vpImage<vpRGBa> &I, int i, int j, const std::string &s, const vpColor &color)
{
  vp_display_display_text(I, i, j, s, color);
}

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \code
  #include <visp3/core/vpColor.h>
  #include <visp3/core/vpDisplay.h>
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImagePoint.h>
  #include <visp3/gui/vpDisplayGDI.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main() {
    vpImage<vpRGBa> I(240, 380);
    vpDisplayGDI d;
    d.init(I);
    vpDisplay::display(I); // display the image
    vpImagePoint center;
    unsigned int radius = 100;
    vpDisplay::displayCircle(I, center, radius, vpColor::red);

    vpDisplay::flush(I); // Mandatory to display the requested features.
  }
  \endcode

  \sa flushROI()
*/
void vpDisplay::flush(const vpImage<vpRGBa> &I) { vp_display_flush(I); }

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parsimoniously.

  \sa flush()
*/
void vpDisplay::flushROI(const vpImage<vpRGBa> &I, const vpRect &roi) { vp_display_flush_roi(I, roi); }

/*!
  Display image \e I.

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), close()
*/
void vpDisplay::display(const vpImage<vpRGBa> &I) { vp_display_display(I); }

/*!
  Update the display with the content of the image that is in the region of
  interest. \param I : Image. \param roi : Region of interest.
 */
void vpDisplay::displayROI(const vpImage<vpRGBa> &I, const vpRect &roi) { vp_display_display_roi(I, roi); }

/*!
  Wait for a click from one of the mouse button.

  \param I [in] : The displayed image.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, bool blocking) { return vp_display_get_click(I, blocking); }

/*!
  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip, bool blocking)
{
  return vp_display_get_click(I, ip, blocking);
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                         bool blocking)
{
  return vp_display_get_click(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise.
*/
bool vpDisplay::getClick(const vpImage<vpRGBa> &I, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClick(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise. If a
  button is released, the location of the mouse pointer is updated in
  \e ip.
*/
bool vpDisplay::getClickUp(const vpImage<vpRGBa> &I, vpImagePoint &ip, vpMouseButton::vpMouseButtonType &button,
                           bool blocking)
{
  return vp_display_get_click_up(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise.
*/
bool vpDisplay::getClickUp(const vpImage<vpRGBa> &I, vpMouseButton::vpMouseButtonType &button, bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClickUp(I, ip, button, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  Below you will find an example showing how to use this method.
  \code
  #include <visp3/core/vpConfig.h>
  #include <visp3/gui/vpDisplayD3D.h>
  #include <visp3/gui/vpDisplayGDI.h>
  #include <visp3/gui/vpDisplayGTK.h>
  #include <visp3/gui/vpDisplayOpenCV.h>
  #include <visp3/gui/vpDisplayX.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<vpRGBa> I(240, 320); // Create a black image

    vpDisplay *d;

  #if defined(VISP_HAVE_X11)
    d = new vpDisplayX;
  #elif defined(VISP_HAVE_GTK)
    d = new vpDisplayGTK;
  #elif defined(VISP_HAVE_GDI)
    d = new vpDisplayGDI;
  #elif defined(VISP_HAVE_D3D9)
    d = new vpDisplayD3D;
  #elif defined(HAVE_OPENCV_HIGHGUI)
    d = new vpDisplayOpenCV;
  #else
    std::cout << "Sorry, no video device is available" << std::endl;
    return -1;
  #endif

    // Initialize the display with the image I. Display and image are
    // now link together.
  #ifdef VISP_HAVE_DISPLAY
    d->init(I);
  #endif

    // Set the display background with image I content
    vpDisplay::display(I);

    // Flush the foreground and background display
    vpDisplay::flush(I);

    // Wait for keyboard event
    std::cout << "Waiting a keyboard event..." << std::endl;
    vpDisplay::getKeyboardEvent(I, true);
    std::cout << "A keyboard event was detected" << std::endl;

    // Non blocking keyboard event loop
    int cpt_event = 0;
    std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
    do {
      bool event = vpDisplay::getKeyboardEvent(I, false);
      if (event) {
        std::cout << "A keyboard event was detected" << std::endl; cpt_event ++;
      }

      vpTime::wait(5); // wait 5 ms
    } while(cpt_event < 5);

  #ifdef VISP_HAVE_DISPLAY
    delete d;
  #endif
  }
  \endcode
*/
bool vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, bool blocking)
{
  return vp_display_get_keyboard_event(I, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \param key [out]: If possible, an ISO Latin-1 character
  corresponding to the keyboard key.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  Below you will find an example showing how to use this method.
  \code
  #include <visp3/gui/vpDisplayD3D.h>
  #include <visp3/gui/vpDisplayGDI.h>
  #include <visp3/gui/vpDisplayGTK.h>
  #include <visp3/gui/vpDisplayOpenCV.h>
  #include <visp3/gui/vpDisplayX.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<vpRGBa> I(240, 320); // Create a black image

    vpDisplay *d;

  #if defined(VISP_HAVE_X11)
    d = new vpDisplayX;
  #elif defined(VISP_HAVE_GTK)
    d = new vpDisplayGTK;
  #elif defined(VISP_HAVE_GDI)
    d = new vpDisplayGDI;
  #elif defined(VISP_HAVE_D3D9)
    d = new vpDisplayD3D;
  #elif defined(HAVE_OPENCV_HIGHGUI)
    d = new vpDisplayOpenCV;
  #else
    std::cout << "Sorry, no video device is available" << std::endl;
    return -1;
  #endif

    // Initialize the display with the image I. Display and image are
    // now link together.
  #ifdef VISP_HAVE_DISPLAY
    d->init(I);
  #endif

    // Set the display background with image I content
    vpDisplay::display(I);

    // Flush the foreground and background display
    vpDisplay::flush(I);

    // Wait for keyboard event
    std::cout << "Waiting a keyboard event..." << std::endl;
    vpDisplay::getKeyboardEvent(I, true);
    std::cout << "A keyboard event was detected" << std::endl;

    // Non blocking keyboard event loop
    int cpt_event = 0;
    std::string key;
    std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
    do {
      bool event = vpDisplay::getKeyboardEvent(I, key, false);
      if (event) {
        std::cout << "Key detected: " << key << std::endl;
        cpt_event ++;
      }

      vpTime::wait(5); // wait 5 ms
    } while(cpt_event < 5);

  #ifdef VISP_HAVE_DISPLAY
    delete d;
  #endif
  }
  \endcode
*/
bool vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, std::string &key, bool blocking)
{
  return vp_display_get_keyboard_event(I, key, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \param key [out]: If possible, an ISO Latin-1 character
  corresponding to the keyboard key.

  \return
  - true if a key was pressed. This is always the case if blocking is set
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  Below you will find an example showing how to use this method.
  \code
  #include <visp3/gui/vpDisplayD3D.h>
  #include <visp3/gui/vpDisplayGDI.h>
  #include <visp3/gui/vpDisplayGTK.h>
  #include <visp3/gui/vpDisplayOpenCV.h>
  #include <visp3/gui/vpDisplayX.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<vpRGBa> I(240, 320); // Create a black image

    vpDisplay *d;

  #if defined(VISP_HAVE_X11)
    d = new vpDisplayX;
  #elif defined(VISP_HAVE_GTK)
    d = new vpDisplayGTK;
  #elif defined(VISP_HAVE_GDI)
    d = new vpDisplayGDI;
  #elif defined(VISP_HAVE_D3D9)
    d = new vpDisplayD3D;
  #elif defined(HAVE_OPENCV_HIGHGUI)
    d = new vpDisplayOpenCV;
  #else
    std::cout << "Sorry, no video device is available" << std::endl;
    return -1;
  #endif

    // Initialize the display with the image I. Display and image are
    // now link together.
  #ifdef VISP_HAVE_DISPLAY
    d->init(I);
  #endif

    // Set the display background with image I content
    vpDisplay::display(I);

    // Flush the foreground and background display
    vpDisplay::flush(I);

    // Wait for keyboard event
    std::cout << "Waiting a keyboard event..." << std::endl;
    vpDisplay::getKeyboardEvent(I, true);
    std::cout << "A keyboard event was detected" << std::endl;

    // Non blocking keyboard event loop
    int cpt_event = 0;
    char key[10];
    std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
    do {
      bool event = vpDisplay::getKeyboardEvent(I, &key[Ø], false);
      if (event) {
        std::cout << "Key detected: " << key << std::endl;
        cpt_event ++;
      }

      vpTime::wait(5); // wait 5 ms
    } while(cpt_event < 5);

  #ifdef VISP_HAVE_DISPLAY
    delete d;
  #endif
  }
  \endcode
*/
bool vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, char *key, bool blocking)
{
  return vp_display_get_keyboard_event(I, key, blocking);
}

/*!
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.
*/
bool vpDisplay::getPointerMotionEvent(const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  return vp_display_get_pointer_motion_event(I, ip);
}

/*!
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true.
*/
bool vpDisplay::getPointerPosition(const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  return vp_display_get_pointer_position(I, ip);
}

/*!
  Set the window background.

  \param I : Image associated to the display window.
  \param color: Background color.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplay::setBackground(const vpImage<vpRGBa> &I, const vpColor &color) { vp_display_set_background(I, color); }

/*!
  Set the font of a text printed in the display overlay. To print a
  text you may use displayText().

  \param I : Image associated to the display window.
  \param fontname : The expected font name.

  \note Under UNIX, the available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \sa displayText()
*/
void vpDisplay::setFont(const vpImage<vpRGBa> &I, const std::string &fontname) { vp_display_set_font(I, fontname); }

/*!
  Set the windows title.
  \note This functionality is not implemented when vpDisplayOpenCV is used.

  \param I : Image associated to the display window.
  \param windowtitle : Window title.
*/
void vpDisplay::setTitle(const vpImage<vpRGBa> &I, const std::string &windowtitle)
{
  vp_display_set_title(I, windowtitle);
}

/*!
  Set the window position in the screen.

  \param I : Image associated to the display window.
  \param winx, winy : Position of the upper-left window's border in the
  screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplay::setWindowPosition(const vpImage<vpRGBa> &I, int winx, int winy)
{
  vp_display_set_window_position(I, winx, winy);
}

/*!
  Return the value of the down scale factor applied to the image in order to
  reduce the size of the window used to display the image.
  When display is not initialized, returns 1.

  \param I : Image associated to the display window.
*/
unsigned int vpDisplay::getDownScalingFactor(const vpImage<vpRGBa> &I) { return vp_display_get_down_scaling_factor(I); }
END_VISP_NAMESPACE
