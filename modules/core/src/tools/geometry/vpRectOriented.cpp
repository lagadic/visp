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
 * Defines a (possibly oriented) rectangle in the plane.
 *
 * Author:
 * Pierre Chatelain
 * Marc Pouliquen
 *
 *****************************************************************************/
#include <visp3/core/vpRectOriented.h>

#include <cmath>

/// Default constructor.
vpRectOriented::vpRectOriented()
  : m_center(), m_width(), m_height(), m_theta(), m_topLeft(), m_topRight(), m_bottomLeft(), m_bottomRight()
{
}

/** Constructor.
 * @param center The rectangle center.
 * @param width The rectangle width.
 * @param height The rectangle height.
 * @param theta The rectangle orientation (rad).
 */
vpRectOriented::vpRectOriented(const vpImagePoint &center, const double width, const double height, const double theta)
{
  m_center = center;
  m_width = width;
  m_height = height;
  m_theta = theta;
  m_topLeft.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2.0);
  m_topLeft.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2.0);
  m_bottomLeft.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2.0);
  m_bottomLeft.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2.0);
  m_bottomRight.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2.0);
  m_bottomRight.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2.0);
  m_topRight.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2.0);
  m_topRight.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2.0);
}

/** Copy constructor.
 * @param rect Rectangle to copy.
 */
vpRectOriented::vpRectOriented(const vpRect &rect)
{
  m_center = rect.getCenter();
  m_width = rect.getWidth();
  m_height = rect.getHeight();
  m_theta = .0;
  m_topLeft.set_i(m_center.get_i() - m_height / 2.0);
  m_topLeft.set_j(m_center.get_j() - m_width / 2.0);
  m_bottomLeft.set_i(m_center.get_i() + m_height / 2.0);
  m_bottomLeft.set_j(m_center.get_j() - m_width / 2.0);
  m_bottomRight.set_i(m_center.get_i() + m_height / 2.0);
  m_bottomRight.set_j(m_center.get_j() + m_width / 2.0);
  m_topRight.set_i(m_center.get_i() - m_height / 2.0);
  m_topRight.set_j(m_center.get_j() + m_width / 2.0);
}

/** Assignement operator.
 * @param rectOriented Oriented rectangle to copy.
 */
vpRectOriented &vpRectOriented::operator=(const vpRectOriented &rectOriented)
{
  m_center = rectOriented.getCenter();
  m_width = rectOriented.getWidth();
  m_height = rectOriented.getHeight();
  m_theta = rectOriented.getOrientation();
  m_topLeft = rectOriented.getTopLeft();
  m_bottomLeft = rectOriented.getBottomLeft();
  m_bottomRight = rectOriented.getBottomRight();
  m_topRight = rectOriented.getTopRight();
  return *this;
}

/** Assignement operator from vpRect.
 * @param rect Rectangle to copy.
 */
vpRectOriented &vpRectOriented::operator=(const vpRect &rect)
{
  m_center = rect.getCenter();
  m_width = rect.getWidth();
  m_height = rect.getHeight();
  m_theta = .0;
  m_topLeft.set_i(m_center.get_i() - m_height / 2.0);
  m_topLeft.set_j(m_center.get_j() - m_width / 2.0);
  m_bottomLeft.set_i(m_center.get_i() + m_height / 2.0);
  m_bottomLeft.set_j(m_center.get_j() - m_width / 2.0);
  m_bottomRight.set_i(m_center.get_i() + m_height / 2.0);
  m_bottomRight.set_j(m_center.get_j() + m_width / 2.0);
  m_topRight.set_i(m_center.get_i() - m_height / 2.0);
  m_topRight.set_j(m_center.get_j() + m_width / 2.0);
  return *this;
}

/** Conversion to vpRect operator.
 */
vpRectOriented::operator vpRect()
{
  if (std::fabs(m_theta) > std::numeric_limits<double>::epsilon())
    throw(vpException(vpException::badValue, "Cannot convert a vpRectOriented with non-zero orientation to a vpRect"));

  return vpRect(m_topLeft, m_bottomRight);
}

/** Set the corners of the rectangle.
 *  @warning This method doesn't check whether the 4 points actually form a rectangle!
 *  The behaviour is undefined if it is not the case.
 */
void vpRectOriented::setPoints(const vpImagePoint &topLeft, const vpImagePoint &topRight,
                               const vpImagePoint &bottomLeft, const vpImagePoint &bottomRight)
{
  m_topLeft = topLeft;
  m_bottomLeft = bottomLeft;
  m_bottomRight = bottomRight;
  m_topRight = topRight;
  m_center.set_i((m_topLeft.get_i() + m_bottomLeft.get_i() + m_bottomRight.get_i() + m_topRight.get_i()) / 4.0);
  m_center.set_j((m_topLeft.get_j() + m_bottomLeft.get_j() + m_bottomRight.get_j() + m_topRight.get_j()) / 4.0);
  m_width = sqrt((m_topRight.get_i() - m_topLeft.get_i()) * (m_topRight.get_i() - m_topLeft.get_i()) +
                 (m_topRight.get_j() - m_topLeft.get_j()) * (m_topRight.get_j() - m_topLeft.get_j()));
  m_height = sqrt((m_bottomLeft.get_i() - m_topLeft.get_i()) * (m_bottomLeft.get_i() - m_topLeft.get_i()) +
                  (m_bottomLeft.get_j() - m_topLeft.get_j()) * (m_bottomLeft.get_j() - m_topLeft.get_j()));
  m_theta = atan2(m_topRight.get_i() - m_topLeft.get_i(), m_topRight.get_j() - m_topLeft.get_j());
}

/// Set the center of the rectangle.
void vpRectOriented::setCenter(const vpImagePoint &center)
{
  m_topLeft += center - m_center;
  m_bottomLeft += center - m_center;
  m_bottomRight += center - m_center;
  m_topRight += center - m_center;
  m_center = center;
}

/// Get the rectangle center point.
vpImagePoint vpRectOriented::getCenter() const { return m_center; }

/// Get the top-left corner.
vpImagePoint vpRectOriented::getTopLeft() const { return m_topLeft; }

/// Get the top-right corner.
vpImagePoint vpRectOriented::getTopRight() const { return m_topRight; }

/// Get the bottom-left corner.
vpImagePoint vpRectOriented::getBottomLeft() const { return m_bottomLeft; }

/// Get the bottom-right corner.
vpImagePoint vpRectOriented::getBottomRight() const { return m_bottomRight; }

/// Set the size of the rectangle : performs a homothety relatively to the rectangle center.
void vpRectOriented::setSize(double width, double height)
{
  m_width = width;
  m_height = height;
  m_topLeft.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2);
  m_topLeft.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2);
  m_bottomLeft.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2);
  m_bottomLeft.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2);
  m_bottomRight.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2);
  m_bottomRight.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2);
  m_topRight.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2);
  m_topRight.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2);
}

/// Get the rectangle width.
double vpRectOriented::getWidth() const { return m_width; }

/// Get the rectangle height.
double vpRectOriented::getHeight() const { return m_height; }

/// Set the rectangle orientation (rad).
void vpRectOriented::setOrientation(double theta)
{
  m_theta = theta;
  m_topLeft.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2);
  m_topLeft.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2);
  m_bottomLeft.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2);
  m_bottomLeft.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2);
  m_bottomRight.set_i(m_center.get_i() + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2);
  m_bottomRight.set_j(m_center.get_j() - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2);
  m_topRight.set_i(m_center.get_i() - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2);
  m_topRight.set_j(m_center.get_j() + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2);
}

/// Get the rectangle orientation (rad).
double vpRectOriented::getOrientation() const { return m_theta; }

/// Check whether the point is inside the rectangle.
bool vpRectOriented::isInside(const vpImagePoint &point) const
{
  if (!isLeft(point, m_topLeft, m_bottomLeft))
    return false;
  if (!isLeft(point, m_bottomLeft, m_bottomRight))
    return false;
  if (!isLeft(point, m_bottomRight, m_topRight))
    return false;
  if (!isLeft(point, m_topRight, m_topLeft))
    return false;
  return true;
}

bool vpRectOriented::isLeft(const vpImagePoint &pointToTest, const vpImagePoint &point1,
                            const vpImagePoint &point2) const
{
  double a = point1.get_j() - point2.get_j();
  double b = point2.get_i() - point1.get_i();
  double c = -(a * point1.get_i() + b * point1.get_j());
  double d = a * pointToTest.get_i() + b * pointToTest.get_j() + c;
  return (d > 0);
}
