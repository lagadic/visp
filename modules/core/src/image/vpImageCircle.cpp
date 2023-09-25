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
 * Image circle, i.e. circle in the image space.
 *
*****************************************************************************/

#include <visp3/core/vpImageCircle.h>

vpImageCircle::vpImageCircle()
  : m_center()
  , m_radius(0.)
{

}

vpImageCircle::vpImageCircle(const vpImagePoint &center, const float &radius)
  : m_center(center)
  , m_radius(radius)
{

}

#ifdef HAVE_OPENCV_CORE
vpImageCircle::vpImageCircle(const cv::Vec3f &vec)
  : m_center(vec[1], vec[0])
  , m_radius(vec[2])
{

}
#endif

vpImageCircle::~vpImageCircle()
{

}

void computeIntersectionsLeftBorderOnly(const float &u_c, const float &radius,
                                        float &delta_theta)
{
  float theta1 = std::acos(-1.f * u_c / radius);
  if (theta1 > M_PI) {
    theta1 -= 2.0 * M_PI;
  }
  else if (theta1 > M_PI) {
    theta1 += 2.0 * M_PI;
  }
  float theta2 = -1.f * theta1;
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  delta_theta = theta_max - theta_min;
}

void computeIntersectionsRightBorderOnly(const float &u_c, const float &width, const float &radius,
                                         float &delta_theta)
{
  float theta1 = std::acos((width - u_c) / radius);
  if (theta1 > M_PI) {
    theta1 -= 2.0 * M_PI;
  }
  else if (theta1 > M_PI) {
    theta1 += 2.0 * M_PI;
  }
  float theta2 = -1.f * theta1;
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  delta_theta = 2.f * M_PI - (theta_max - theta_min);
}

void computeIntersectionsTopBorderOnly(const float &v_c, const float &radius,
                                       float &delta_theta)
{
  float theta1 = std::asin(-1.f * v_c / radius);
  if (theta1 > M_PI) {
    theta1 -= 2.0 * M_PI;
  }
  else if (theta1 > M_PI) {
    theta1 += 2.0 * M_PI;
  }

  float theta2 = 0.f;
  if (theta1 >= 0.f) {
    theta2 = M_PI - theta1;
  }
  else {
    theta2 = -theta1 - M_PI;
  }
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  if (theta1 > 0.f) {
    delta_theta = 2.f * M_PI - (theta_max - theta_min);
  }
  else {
    delta_theta = theta_max - theta_min;
  }
}

void computeIntersectionsBottomBorderOnly(const float &v_c, const float &height, const float &radius,
                                          float &delta_theta)
{
  float theta1 = std::asin((height - v_c) / radius);
  if (theta1 > M_PI) {
    theta1 -= 2.0 * M_PI;
  }
  else if (theta1 > M_PI) {
    theta1 += 2.0 * M_PI;
  }

  float theta2 = 0.f;
  if (theta1 >= 0.f) {
    theta2 = M_PI - theta1;
  }
  else {
    theta2 = -theta1 - M_PI;
  }
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  if (theta1 > 0.f) {
    delta_theta = theta_max - theta_min;
  }
  else {
    delta_theta = 2.f * M_PI - (theta_max - theta_min);
  }
}

float vpImageCircle::computeArcLengthInRoI(const vpRect &roi) const
{
  float deltaTheta = 0.f;
  vpImagePoint center = m_center;
  float center_u = center.get_u();
  float center_v = center.get_v();
  float radius = m_radius;
  float img_w = roi.getWidth();
  float img_h = roi.getHeight();
  bool touchLeftBorder = (center_u - radius) < 0.;
  bool touchRightBorder = (center_u + radius) >= img_w;
  bool touchTopBorder = (center_v - radius) < 0.;
  bool touchBottomBorder = (center_v + radius) >= img_h;
  bool isHorizontallyOK = (!touchLeftBorder && !touchRightBorder);
  bool isVerticallyOK = (!touchTopBorder && !touchBottomBorder);
  if (isHorizontallyOK && isVerticallyOK) {
    // Easy case
    // The circle has its center in the image and its radius is not too great
    // to make it partially occluded
    deltaTheta = 2.f * M_PI;
  }
  else if (touchBottomBorder && !touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touch only the bottom border of the RoI
    computeIntersectionsBottomBorderOnly(center_v, img_h, radius, deltaTheta);
  }
  else if (!touchBottomBorder && touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touch only the left border of the RoI
    computeIntersectionsLeftBorderOnly(center_u, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && touchRightBorder && !touchTopBorder) {
    // Touch only the right border of the RoI
    computeIntersectionsRightBorderOnly(center_u, img_w, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && !touchRightBorder && touchTopBorder) {
    // Touch only the top border of the RoI
    computeIntersectionsTopBorderOnly(center_v, radius, deltaTheta);
  }
  float arcLength = deltaTheta * radius;
  return arcLength;
}

vpImagePoint vpImageCircle::getCenter() const
{
  return m_center;
}

float vpImageCircle::getRadius() const
{
  return m_radius;
}

vpRect vpImageCircle::getBBox() const
{
  vpRect bbox(m_center - vpImagePoint(m_radius, m_radius), 2 * m_radius, 2 * m_radius);
  return bbox;
}

float vpImageCircle::get_n20() const
{
  return m_radius * m_radius / 4;
}

float vpImageCircle::get_n02() const
{
  return m_radius * m_radius / 4;
}

float vpImageCircle::get_n11() const
{
  return 0.;
};
