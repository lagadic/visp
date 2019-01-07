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
 * Defines a rectangle in the plane.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpRect.cpp
  \brief Defines a rectangle in the plane.
  \ingroup libtools
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRect.h>

/*!
  Constructs a default rectangle with the \e top, \e left corner set to (0,0)
  and \e width and \e height set to 1.
*/

vpRect::vpRect() : left(0), top(0), width(0), height(0) {}

/*!
  Constructs a rectangle with the \e top, \e left corner and \e width
  and \e height.

  \param l : coordinate of the top/left corner along the horizontal axis.
  \param t : coordinate of the top/left corner along the vertical axis.
  \param w : rectangle width.
  \param h : rectangle height.
*/
vpRect::vpRect(double l, double t, double w, double h) : left(l), top(t), width(w), height(h) {}

/*!
  Constructs a rectangle with \e topLeft the top-left corner location
  and \e width and \e height the rectangle size.

  \param topLeft : coordinates of the top/left corner.
  \param w : rectangle width.
  \param h : rectangle height.
*/
vpRect::vpRect(const vpImagePoint &topLeft, double w, double h)
  : left(topLeft.get_u()), top(topLeft.get_v()), width(w), height(h)
{
}

/*!
  Constructs a rectangle with \e topLeft the top-left corner location
  and \e bottomRight the bottom-right corner.
*/
vpRect::vpRect(const vpImagePoint &topLeft, const vpImagePoint &bottomRight)
  : left(topLeft.get_u()), top(topLeft.get_v()), width(0), height(0)
{
  this->left = topLeft.get_u();
  this->top = topLeft.get_v();

  setBottom(bottomRight.get_v());
  setRight(bottomRight.get_u());
}

/*!
  Constructs a rectangle that is a copy of \e r.
*/
vpRect::vpRect(const vpRect &r) : left(0), top(0), width(0), height(0) { *this = r; }

/*!

  Assign \e r to this rectangle and return a reference to this rectangle.
*/
vpRect &vpRect::operator=(const vpRect &r)
{
  this->left = r.left;
  this->top = r.top;
  this->width = r.width;
  this->height = r.height;
  return *this;
}

/*!
  Create a rectangle as the bounding box of a vector of image points.
  \param ip : Vector of image points. At least 1 points is mandatory,
  otherwise an exception is thrown.
*/
vpRect::vpRect(const std::vector<vpImagePoint> &ip) : left(0), top(0), width(0), height(0) { set(ip); }

/*!
  Check if an image point belongs to the rectangle.

  \param ip : the image point.

  \return Returns true if the point belongs to the rectangle.
*/
bool vpRect::isInside(const vpImagePoint &ip) const
{
  return (ip.get_i() <= this->getBottom() && ip.get_i() >= this->getTop() && ip.get_j() <= this->getRight() &&
          ip.get_j() >= this->getLeft());
}

/*!
  Set the rectangle with the \e top, \e left corner and \e width
  and \e height.
  \param l : coordinate of the top/left corner along the horizontal axis.
  \param t : coordinate of the top/left corner along the vertical axis.
  \param w : rectangle width.
  \param h : rectangle height.
*/
void vpRect::set(double l, double t, double w, double h)
{
  left = l;
  top = t;
  width = w;
  height = h;
}

/*!
  Set the rectangle with \e topLeft the top-left corner location
  and \e width and \e height the rectangle size.

  \param topLeft : coordinates of the top-left corner.
  \param w : rectangle width.
  \param h : rectangle height.
*/
void vpRect::set(const vpImagePoint &topLeft, double w, double h)
{
  left = topLeft.get_u();
  top = topLeft.get_v();
  width = w;
  height = h;
}

/*!
  Set the rectangle as the bounding box of a vector of image points.
  \param ip : Vector of image points. At least 1 points is mandatory,
  otherwise an exception is thrown.
*/
void vpRect::set(const std::vector<vpImagePoint> &ip)
{
  if (ip.size() < 1)
    throw(vpException(vpException::dimensionError, "At least 1 point is requested to build a rectangle"));
  double minu, maxu;
  double minv, maxv;
  minu = maxu = ip[0].get_u();
  minv = maxv = ip[0].get_v();

  for (size_t i = 1; i < ip.size(); i++) {
    double u = ip[i].get_u();
    double v = ip[i].get_v();
    if (u < minu)
      minu = u;
    else if (u > maxu)
      maxu = u;
    if (v < minv)
      minv = v;
    else if (v > maxv)
      maxv = v;
  }

  setLeft(minu);
  setTop(minv);
  setRight(maxu);
  setBottom(maxv);
}

/*!
  Set the rectangle with \e topLeft the top-left corner location
  and \e bottomRight the bottom-right corner.

  \param topLeft : coordinates of the top-left corner.
  \param bottomRight : coordinates of the bottom-right corner.
*/
void vpRect::set(const vpImagePoint &topLeft, const vpImagePoint &bottomRight)
{
  this->left = topLeft.get_u();
  this->top = topLeft.get_v();

  setBottom(bottomRight.get_v());
  setRight(bottomRight.get_u());
}

/*!
  Set the rectangle from \e r.
*/
void vpRect::set(const vpRect &r) { *this = r; }

/*!
 Operator to compare two rectangles.
 \return true if both objects are similar, false otherwise.
 */
bool vpRect::operator==(const vpRect &r) const
{
  // return (top == r.top && left == r.left && width == r.width && height ==
  // r.height);
  return (std::fabs(top - r.top) <= std::fabs(top) * std::numeric_limits<double>::epsilon() &&
          std::fabs(left - r.left) <= std::fabs(left) * std::numeric_limits<double>::epsilon() &&
          std::fabs(width - r.width) <= std::fabs(width) * std::numeric_limits<double>::epsilon() &&
          std::fabs(height - r.height) <= std::fabs(height) * std::numeric_limits<double>::epsilon());
}

/*!
 Operator to compare two rectangles.
 \return true if both objects differ, false otherwise.
 */
bool vpRect::operator!=(const vpRect &r) const
{
  // return (top != r.top || left != r.left || width != r.width || height !=
  // r.height);
  //  return (std::fabs(top-r.top) >
  //  std::fabs(top)*std::numeric_limits<double>::epsilon()
  //          || std::fabs(left-r.left) >
  //          std::fabs(left)*std::numeric_limits<double>::epsilon()
  //          || std::fabs(width-r.width) >
  //          std::fabs(width)*std::numeric_limits<double>::epsilon()
  //          || std::fabs(height-r.height) >
  //          std::fabs(height)*std::numeric_limits<double>::epsilon());
  return !(*this == r);
}

/*!
 Intersection operator.
 \return Intersection rectangle or null rectangle if the two rectangles do not
 intersect.
 */
vpRect &vpRect::operator&=(const vpRect &r)
{
  double x1 = (std::max)(left, r.left);
  double y1 = (std::max)(top, r.top);
  width = (std::min)(left + width, r.left + r.width) - x1;
  height = (std::min)(top + height, r.top + r.height) - y1;
  left = x1;
  top = y1;

  if (width <= 0 || height <= 0) {
    *this = vpRect();
  }

  return *this;
}

/*!
 Intersection operator.
 \return Intersection rectangle or null rectangle if the two rectangles do not
 intersect.
 */
vpRect vpRect::operator&(const vpRect &r) const
{
  vpRect a = *this;
  return a &= r;
}

/*!
  Check if an image point belongs to a rectangle.

  \param ip : the image point.
  \param rect : the rectangle.

  \return Returns true if the point belongs to the rectangle.
*/
VISP_EXPORT bool inRectangle(const vpImagePoint &ip, const vpRect &rect)
{
  return (ip.get_i() <= rect.getBottom() && ip.get_i() >= rect.getTop() && ip.get_j() <= rect.getRight() &&
          ip.get_j() >= rect.getLeft());
}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRect &r)
{
  os << r.getLeft() << ", " << r.getTop() << ", " << r.getWidth() << ", " << r.getHeight();
  return os;
}
