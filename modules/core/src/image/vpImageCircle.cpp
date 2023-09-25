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

/*!
 * \brief Express \b theta between - Pi and Pi .
 *
 * \param[in] theta The input angle we want to ensure it is in the interval [-Pi ; Pi]
 * \return float The input angle in the interval [-Pi ; Pi]
 */
float getAngleBetweenMinPiAndPi(const float &theta)
{
  float theta1 = theta;
  if (theta1 > M_PI) {
    theta1 -= 2.0 * M_PI;
  }
  else if (theta1 > M_PI) {
    theta1 += 2.0 * M_PI;
  }
  return theta1;
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the left border of the Region of Interest (RoI).
 *
 * \param[in] u_c The u-coordinate of the center of the circle.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsLeftBorderOnly(const float &u_c, const float &radius,
                                        float &delta_theta)
{
  float theta1 = std::acos(-1.f * u_c / radius);
  theta1 = getAngleBetweenMinPiAndPi(theta1);
  float theta2 = -1.f * theta1;
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  delta_theta = theta_max - theta_min;
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the right border of the Region of Interest (RoI).
 *
 * \param[in] u_c The u-coordinate of the center of the circle.
 * \param[in] width The width of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsRightBorderOnly(const float &u_c, const float &width, const float &radius,
                                         float &delta_theta)
{
  float theta1 = std::acos((width - u_c) / radius);
  theta1 = getAngleBetweenMinPiAndPi(theta1);
  float theta2 = -1.f * theta1;
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  delta_theta = 2.f * M_PI - (theta_max - theta_min);
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the top border of the Region of Interest (RoI).
 *
 * \param[in] v_c The v-coordinate of the center of the circle.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsTopBorderOnly(const float &v_c, const float &radius,
                                       float &delta_theta)
{
  float theta1 = std::asin(-1.f * v_c / radius);
  theta1 = getAngleBetweenMinPiAndPi(theta1);

  float theta2 = 0.f;
  if (theta1 >= 0.f) {
    theta2 = M_PI - theta1;
  }
  else {
    theta2 = -theta1 - M_PI;
  }
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  if (std::abs(theta_max - theta_min) * radius < 1.f) {
    // Between the maximum and minimum theta there is less than 1 pixel of difference
    // It meens that the full circle is visible
    delta_theta = 2.f * M_PI;
  }
  else if (theta1 > 0.f) {
    delta_theta = 2.f * M_PI - (theta_max - theta_min);
  }
  else {
    delta_theta = theta_max - theta_min;
  }
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the left border of the Region of Interest (RoI).
 *
 * \param[in] v_c The v-coordinate of the center of the circle.
 * \param[in] height The height of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsBottomBorderOnly(const float &v_c, const float &height, const float &radius,
                                          float &delta_theta)
{
  float theta1 = std::asin((height - v_c) / radius);
  theta1 = getAngleBetweenMinPiAndPi(theta1);

  float theta2 = 0.f;
  if (theta1 >= 0.f) {
    theta2 = M_PI - theta1;
  }
  else {
    theta2 = -theta1 - M_PI;
  }
  float theta_min = std::min(theta1, theta2);
  float theta_max = std::max(theta1, theta2);
  if (std::abs(theta_max - theta_min) * radius < 1.f) {
    // Between the maximum and minimum theta there is less than 1 pixel of difference
    // It meens that the full circle is visible
    delta_theta = 2.f * M_PI;
  }
  else if (theta1 > 0.f) {
    delta_theta = theta_max - theta_min;
  }
  else {
    delta_theta = 2.f * M_PI - (theta_max - theta_min);
  }
}

/*!
 * \brief Compute the angles for which the circle crosses the horizontal u-axis and the
 * vertical v-axis.
 *
 * \param[in] u_c The horizontal u-axis coordinate of the center.
 * \param[in] v_c The vertical v-axis coordinate of the center.
 * \param[in] radius The radius of the circle.
 * \param[in] crossing_u The horizontal u-axis coordinate of the crossing point with the v-axis.
 * \param[in] crossing_v The vertical v-axis coordinate of the crossing point with the u-axis.
 * \param[out] theta_u_cross_min The smallest angle for which the circle intersects the u-axis, i.e. \b theta_u_cross_min < \b theta_u_cross_max .
 * \param[out] theta_u_cross_max The highest angle for which the circle intersects the u-axis.
 * \param[out] theta_v_cross_min The smallest angle for which the circle intersects the v-axis; i.e. \b theta_v_cross_min < \b theta_v_cross_max .
 * \param[out] theta_v_cross_max The highest angle for which the circle intersects the v-axis.
 */
void computePerpendicularAxesIntersections(const float &u_c, const float &v_c, const float &radius,
                                 const float &crossing_u, const float &crossing_v,
                                 float &theta_u_cross_min, float &theta_u_cross_max,
                                 float &theta_v_cross_min, float &theta_v_cross_max)
{
  float theta_u_cross = std::asin((crossing_u - v_c)/radius);
  theta_u_cross = getAngleBetweenMinPiAndPi(theta_u_cross);
  float theta_u_cross_2 = 0.f;
  if (theta_u_cross > 0) {
    theta_u_cross_2 = M_PI - theta_u_cross;
  }
  else {
    theta_u_cross_2 = -M_PI - theta_u_cross;
  }
  theta_u_cross_min = std::min(theta_u_cross, theta_u_cross_2);
  theta_u_cross_max = std::max(theta_u_cross, theta_u_cross_2);

  float theta_v_cross = std::acos((crossing_v - u_c)/radius);
  theta_v_cross = getAngleBetweenMinPiAndPi(theta_v_cross);
  float theta_v_cross_2 = -theta_v_cross;
  theta_v_cross_min = std::min(theta_v_cross, theta_v_cross_2);
  theta_v_cross_max = std::max(theta_v_cross, theta_v_cross_2);
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
  bool touchLeftBorder = (center_u - radius) <= 0.;
  bool touchRightBorder = (center_u + radius) >= img_w;
  bool touchTopBorder = (center_v - radius) <= 0.;
  bool touchBottomBorder = (center_v + radius) >= img_h;
  bool isHorizontallyOK = (!touchLeftBorder && !touchRightBorder);
  bool isVerticallyOK = (!touchTopBorder && !touchBottomBorder);
  if (isHorizontallyOK && isVerticallyOK && roi.isInside(m_center)) {
    // Easy case
    // The circle has its center in the image and its radius is not too great
    // to make it fully contained in the RoI
    deltaTheta = 2.f * M_PI;
  }
  else if (touchBottomBorder && !touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touches/intersects only the bottom border of the RoI
    std::cout << "Case bottom only" << std::endl;
    computeIntersectionsBottomBorderOnly(center_v, img_h, radius, deltaTheta);
  }
  else if (!touchBottomBorder && touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touches/intersects only the left border of the RoI
    std::cout << "Case left only" << std::endl;
    computeIntersectionsLeftBorderOnly(center_u, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && touchRightBorder && !touchTopBorder) {
    // Touches/intersects only the right border of the RoI
    std::cout << "Case right only" << std::endl;
    computeIntersectionsRightBorderOnly(center_u, img_w, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && !touchRightBorder && touchTopBorder) {
    // Touches/intersects only the top border of the RoI
    std::cout << "Case top only" << std::endl;
    computeIntersectionsTopBorderOnly(center_v, radius, deltaTheta);
  }
  else if (touchBottomBorder && touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touches/intersects the bottom and left borders of the RoI
    std::cout << "Case bottom / left" << std::endl;
    float crossing_theta_u_min = 0.f, crossing_theta_v_min = 0.f;
    float crossing_theta_u_max = 0.f, crossing_theta_v_max = 0.f;
    computePerpendicularAxesIntersections(center_u, center_v, radius, 0, img_h, crossing_theta_u_min, crossing_theta_u_max, crossing_theta_v_min, crossing_theta_v_max);
  }
  else if (touchBottomBorder && !touchLeftBorder && touchRightBorder && !touchTopBorder) {
    // Touches/intersects the bottom and right borders of the RoI
    std::cout << "Case bottom / right" << std::endl;
    float crossing_theta_u_min = 0.f, crossing_theta_v_min = 0.f;
    float crossing_theta_u_max = 0.f, crossing_theta_v_max = 0.f;
    computePerpendicularAxesIntersections(center_u, center_v, radius, img_w, img_h, crossing_theta_u_min, crossing_theta_u_max, crossing_theta_v_min, crossing_theta_v_max);
  }
  else if (!touchBottomBorder && touchLeftBorder && !touchRightBorder && touchTopBorder) {
    // Touches/intersects the top and left borders of the RoI
    std::cout << "Case top / left" << std::endl;
    float crossing_theta_u_min = 0.f, crossing_theta_v_min = 0.f;
    float crossing_theta_u_max = 0.f, crossing_theta_v_max = 0.f;
    computePerpendicularAxesIntersections(center_u, center_v, radius, 0, 0, crossing_theta_u_min, crossing_theta_u_max, crossing_theta_v_min, crossing_theta_v_max);
  }
  else if (!touchBottomBorder && !touchLeftBorder && touchRightBorder && touchTopBorder) {
    // Touches/intersects the top and right borders of the RoI
    std::cout << "Case top / right" << std::endl;
    float crossing_theta_u_min = 0.f, crossing_theta_v_min = 0.f;
    float crossing_theta_u_max = 0.f, crossing_theta_v_max = 0.f;
    computePerpendicularAxesIntersections(center_u, center_v, radius, img_w, 0, crossing_theta_u_min, crossing_theta_u_max, crossing_theta_v_min, crossing_theta_v_max);
  }
  else if (touchBottomBorder  && touchTopBorder && (touchLeftBorder ^ touchRightBorder)) {
    // Touches/intersects the top and bottom borders of the RoI
    std::cout << "Case bottom / top" << std::endl;
    // computeOppositeAxesIntersections ?
  }
  else if (touchLeftBorder && touchRightBorder && (touchTopBorder ^ touchBottomBorder)) {
    // Touches/intersects the left and right borders of the RoI
    std::cout << "Case right / left" << std::endl;
    // computeOppositeAxesIntersections ?
  }
  else if (touchBottomBorder && touchLeftBorder && touchRightBorder && touchTopBorder) {
    // Touches/intersects each axis
    std::cout << "Case all" << std::endl;
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
