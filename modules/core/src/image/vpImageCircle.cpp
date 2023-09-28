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
  else if (theta1 < -M_PI) {
    theta1 += 2.0 * M_PI;
  }
  return theta1;
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the left border of the Region of Interest (RoI).
 *
 * \param[in] u_c The u-coordinate of the center of the circle.
 * \param[in] umin_roi The minimum u-coordinate, i.e. the left border, of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsLeftBorderOnly(const float &u_c, const float &umin_roi, const float &radius,
                                        float &delta_theta)
{
  // umin_roi = u_c + r cos(theta)
  // theta = acos((umin_roi - u_c) / r)
  float theta1 = std::acos((umin_roi - u_c)/ radius);
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
 * \param[in] umax_roi The maximum u-coordinate, i.e. the right border, of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsRightBorderOnly(const float &u_c, const float &umax_roi, const float &radius,
                                         float &delta_theta)
{
  // u = u_c + r cos(theta)
  // theta = acos((u - u_c) / r)
  float theta1 = std::acos((umax_roi - u_c) / radius);
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
 * \param[in] vmin_roi The minimum v-coordinate, i.e. the left border, of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsTopBorderOnly(const float &v_c, const float &vmin_roi, const float &radius,
                                       float &delta_theta)
{
  // v = vc - r sin(theta) because the v-axis goes down
  // theta = asin((vc - v)/r)
  float theta1 = std::asin((v_c - vmin_roi) / radius);
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
 * \param[in] vmax_roi The maximum v-coordinate, i.e. the bottom border, of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeIntersectionsBottomBorderOnly(const float &v_c, const float &vmax_roi, const float &radius,
                                          float &delta_theta)
{
  // v = vc - r sin(theta) because the v-axis goes down
  // theta = asin((vc - v)/r)
  float theta1 = std::asin((v_c - vmax_roi) / radius);
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
 * \param[out] theta_u_cross_min The pair angle /u-coordinate for which the circle intersects the u-axis with the lowest u-coordinate, i.e. \b theta_u_cross_min.second < \b theta_u_cross_max.second .
 * \param[out] theta_u_cross_max The pair angle /u-coordinate for which the circle intersects the u-axis with the highest u-coordinate.
 * \param[out] theta_v_cross_min The pair angle /v-coordinate for which the circle intersects the v-axis with the lowest v-coordinate; i.e. \b theta_v_cross_min.second < \b theta_v_cross_max.second .
 * \param[out] theta_v_cross_max The pair angle /v-coordinate for which the circle intersects the v-axis with the highest v-coordinate.
 */
void computePerpendicularAxesIntersections(const float &u_c, const float &v_c, const float &radius,
                                 const float &crossing_u, const float &crossing_v,
                                 std::pair<float, float> &theta_u_cross_min, std::pair<float, float> &theta_u_cross_max,
                                 std::pair<float, float> &theta_v_cross_min, std::pair<float, float> &theta_v_cross_max)
{
  // Computing the two angles for which the u-axis is crossed
  // v = vc - r sin(theta) because the v-axis goes down
  // theta = asin((vc - v)/r)
  float theta_u_cross = std::asin((v_c - crossing_u)/radius);
  theta_u_cross = getAngleBetweenMinPiAndPi(theta_u_cross);
  float theta_u_cross_2 = 0.f;
  if (theta_u_cross > 0) {
    theta_u_cross_2 = M_PI - theta_u_cross;
  }
  else {
    theta_u_cross_2 = -M_PI - theta_u_cross;
  }
  // Computing the corresponding u-coordinates at which the u-axis is crossed
  float u_ucross = u_c + radius * std::cos(theta_u_cross);
  float u_ucross2 = u_c + radius * std::cos(theta_u_cross_2);
  // Sorting the outputs such as theta_X_cross_min.second < theta_X_cross_max.second
  if (u_ucross < u_ucross2) {
    theta_u_cross_min.first = theta_u_cross;
    theta_u_cross_min.second = u_ucross;
    theta_u_cross_max.first = theta_u_cross_2;
    theta_u_cross_max.second = u_ucross2;
  }
  else {
    theta_u_cross_min.first = theta_u_cross_2;
    theta_u_cross_min.second = u_ucross2;
    theta_u_cross_max.first = theta_u_cross;
    theta_u_cross_max.second = u_ucross;
  }

  // Computing the two angles for which the v-axis is crossed
  // u = u_c + r cos(theta)
  // theta = acos((u - u_c) / r)
  float theta_v_cross = std::acos((crossing_v - u_c)/radius);
  theta_v_cross = getAngleBetweenMinPiAndPi(theta_v_cross);
  float theta_v_cross_2 = -theta_v_cross;
  // Computing the corresponding v-coordinates at which the v-axis is crossed
  // v = v_c - radius sin(theta) because the v-axis is oriented towards the bottom
  float v_vcross = v_c - radius * std::sin(theta_v_cross);
  float v_vcross2 = v_c - radius * std::sin(theta_v_cross_2);
  // Sorting the outputs such as theta_X_cross_min.second < theta_X_cross_max.second
  if (v_vcross < v_vcross2) {
    theta_v_cross_min.first = theta_v_cross;
    theta_v_cross_min.second = v_vcross;
    theta_v_cross_max.first = theta_v_cross_2;
    theta_v_cross_max.second = v_vcross2;
  }
  else {
    theta_v_cross_min.first = theta_v_cross_2;
    theta_v_cross_min.second = v_vcross2;
    theta_v_cross_max.first = theta_v_cross;
    theta_v_cross_max.second = v_vcross;
  }
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the left and top borders of the Region of Interest (RoI).
 *
 * \param[in] u_c The horizontal u-axis coordinate of the center.
 * \param[in] v_c The vertical v-axis coordinate of the center.
 * \param[in] topLeft The top left corner of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeTopLeftIntersections(const float &u_c, const float &v_c, const vpImagePoint &topLeft, const float &radius,
                                 float &delta_theta)
{
  std::pair<float, float> crossing_theta_u_min, crossing_theta_u_max;
  std::pair<float, float> crossing_theta_v_min, crossing_theta_v_max;
  float crossing_u = topLeft.get_v(); // We cross the u-axis of the RoI at which v-coordinate
  float vmin_roi = crossing_u; // The minimum v-coordinate of the RoI
  float crossing_v = topLeft.get_u(); // We cross the v-axis of the RoI at which u-coordinate
  float umin_roi = crossing_v; // The minimum u-coordinate of the RoI
  computePerpendicularAxesIntersections(u_c, v_c, radius, crossing_u, crossing_v,
                                        crossing_theta_u_min, crossing_theta_u_max,
                                        crossing_theta_v_min, crossing_theta_v_max);
  float theta_u_min = crossing_theta_u_min.first, theta_v_min = crossing_theta_v_min.first;
  float theta_u_max = crossing_theta_u_max.first, theta_v_max = crossing_theta_v_max.first;
  float u_umin = crossing_theta_u_min.second;
  float u_umax = crossing_theta_u_max.second;
  float v_vmin = crossing_theta_v_min.second;
  float v_vmax = crossing_theta_v_max.second;
  std::cout << "umin_roi = " << umin_roi << "\tvmin_roi = " << vmin_roi << std::endl;
  std::cout << "u_umin = " << u_umin << " (" << theta_u_min << ")\tu_umax = " << u_umax  << " (" << theta_u_max << ")" << std::endl;
  std::cout << "v_vmin = " << v_vmin << " (" << theta_v_min << ")\tv_vmax = " << v_vmax  << " (" << theta_v_max << ")" << std::endl;
  if (u_umin < umin_roi && u_umax >= umin_roi && v_vmin < vmin_roi && v_vmax >= vmin_roi) {
    // The circle crosses only once each axis
    std::cout << "\t|->Case crossing once" << std::endl;
    delta_theta = theta_u_max - theta_v_max;
  }
  else if (u_umin >= umin_roi && u_umax >= umin_roi && v_vmin >= vmin_roi && v_vmax >= vmin_roi) {
    // The circle crosses twice each axis
    std::cout << "\t|->Case crossing twice" << std::endl;
    delta_theta = (theta_v_min - theta_u_min) + (theta_u_max - theta_v_max);
  }
  else if (u_umin < umin_roi && u_umax < umin_roi && v_vmin >= vmin_roi && v_vmax >= vmin_roi) {
    // The circle crosses the u-axis outside the roi
    // so it is equivalent to the case of crossing only the left border
    std::cout << "\t|->Case left only" << std::endl;
    computeIntersectionsLeftBorderOnly(u_c, umin_roi, radius, delta_theta);
  }
  else if (u_umin >= umin_roi && u_umax >= umin_roi && v_vmin <= vmin_roi && v_vmax <= vmin_roi) {
    // The circle crosses the v-axis outside the roi
    // so it is equivalent to the case of crossing only the top border
    std::cout << "\t|->Case top only" << std::endl;
    computeIntersectionsTopBorderOnly(v_c, vmin_roi, radius, delta_theta);
    // delta_theta = theta_u_max - theta_u_min;
  }
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the right and top borders of the Region of Interest (RoI).
 *
 * \param[in] u_c The horizontal u-axis coordinate of the center.
 * \param[in] v_c The vertical v-axis coordinate of the center.
 * \param[in] vmin_roi The top v-coordinate of the RoI.
 * \param[in] umax_roi The right u-coordinate of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeTopRightIntersections(const float &u_c, const float &v_c, const float &vmin_roi, const float &umax_roi, const float &radius,
                                 float &delta_theta)
{
  std::pair<float, float> crossing_theta_u_min, crossing_theta_u_max;
  std::pair<float, float> crossing_theta_v_min, crossing_theta_v_max;
  computePerpendicularAxesIntersections(u_c, v_c, radius, vmin_roi, umax_roi,
                                        crossing_theta_u_min, crossing_theta_u_max,
                                        crossing_theta_v_min, crossing_theta_v_max);
  float theta_u_min = crossing_theta_u_min.first, theta_v_min = crossing_theta_v_min.first;
  float theta_u_max = crossing_theta_u_max.first, theta_v_max = crossing_theta_v_max.first;
  float u_umin = crossing_theta_u_min.second;
  float u_umax = crossing_theta_u_max.second;
  float v_vmin = crossing_theta_v_min.second;
  float v_vmax = crossing_theta_v_max.second;
  if (u_umin <= umax_roi && v_vmin < vmin_roi && u_umax >= umax_roi && v_vmax >= vmin_roi) {
    // The circle crosses only once each axis and the center is below the top border
    std::cout << "\t|->Case crossing once" << std::endl;
    delta_theta = theta_v_max - theta_u_min;
    if (delta_theta < 0) {
      // The arc cannot be negative
      delta_theta += 2.f * M_PI;
    }
  }
  else if (u_umin <= umax_roi && v_vmin >= vmin_roi && u_umax <= umax_roi && v_vmax >= vmin_roi) {
    // The circle crosses twice each axis
    std::cout << "\t|->Case crossing twice" << std::endl;
    delta_theta = 2 * M_PI - ((theta_u_min - theta_u_max)+(theta_v_min - theta_v_max));
  }
  else if (u_umin >= umax_roi && v_vmin >= vmin_roi && u_umax >= umax_roi && v_vmax >= vmin_roi) {
    // The circle crosses the u-axis outside the roi
    // so it is equivalent to the case of crossing only the right border
    std::cout << "\t|->Case crossing right only" << std::endl;
    computeIntersectionsRightBorderOnly(u_c, umax_roi, radius, delta_theta);
  }
  else if (u_umin <= umax_roi && v_vmin <= vmin_roi && u_umax <= umax_roi && v_vmax <= vmin_roi) {
    // The circle crosses the v-axis outside the roi
    // so it is equivalent to the case of crossing only the top border
    std::cout << "\t|->Case crossing top only" << std::endl;
    computeIntersectionsTopBorderOnly(v_c, vmin_roi, radius, delta_theta);
  }
}

/*!
 * \brief Compute the length of the angular interval of the circle when it intersects
 * only with the left and bottom borders of the Region of Interest (RoI).
 *
 * \param[in] u_c The horizontal u-axis coordinate of the center.
 * \param[in] v_c The vertical v-axis coordinate of the center.
 * \param[in] topLeft The top left corner of the RoI.
 * \param[in] height The height of the RoI.
 * \param[in] radius The radius of the circle.
 * \param[out] delta_theta The length of the angular interval that is in the RoI.
 */
void computeBottomLeftIntersections(const float &u_c, const float &v_c, const vpImagePoint &topLeft, const float &height, const float &radius,
                                 float &delta_theta)
{
  std::pair<float, float> crossing_theta_u_min, crossing_theta_u_max;
  std::pair<float, float> crossing_theta_v_min, crossing_theta_v_max;
  float crossing_u = topLeft.get_v() + height; // We cross the u-axis of the RoI at which v-coordinate
  float vmax_roi = crossing_u; // The maximum v-coordinate of the RoI
  float crossing_v = topLeft.get_u(); // We cross the v-axis of the RoI at which u-coordinate
  float umin_roi = crossing_v; // The minimum u-coordinate of the RoI
  computePerpendicularAxesIntersections(u_c, v_c, radius, crossing_u, crossing_v,
                                        crossing_theta_u_min, crossing_theta_u_max,
                                        crossing_theta_v_min, crossing_theta_v_max);
  float theta_u_min = crossing_theta_u_min.first, theta_v_min = crossing_theta_v_min.first;
  float theta_u_max = crossing_theta_u_max.first, theta_v_max = crossing_theta_v_max.first;
  float u_umin = crossing_theta_u_min.second;
  float u_umax = crossing_theta_u_max.second;
  float v_vmin = crossing_theta_v_min.second;
  float v_vmax = crossing_theta_v_max.second;
  std::cout << "umin_roi = " << umin_roi << "\tvmax_roi = " << vmax_roi << std::endl;
  std::cout << "u_umin = " << u_umin << " (" << theta_u_min << ")\tu_umax = " << u_umax  << " (" << theta_u_max << ")" << std::endl;
  std::cout << "v_vmin = " << v_vmin << " (" << theta_v_min << ")\tv_vmax = " << v_vmax  << " (" << theta_v_max << ")" << std::endl;
  if (u_umin < umin_roi && u_umax >= umin_roi && v_vmin <= vmax_roi && v_vmax > vmax_roi) {
    // The circle crosses only once each axis
    std::cout << "\t|->Case crossing once" << std::endl;
    delta_theta = theta_v_min - theta_u_max;
  }
  else if (u_umin >= umin_roi && u_umax >= umin_roi && v_vmin <= vmax_roi && v_vmax <= vmax_roi) {
    // The circle crosses twice each axis
    std::cout << "\t|->Case crossing twice" << std::endl;
    delta_theta = (theta_v_min - theta_u_max) + (theta_u_min - theta_v_max);
  }
  else if (u_umin < umin_roi && u_umax < umin_roi && v_vmin <= vmax_roi && v_vmax <= vmax_roi) {
    // The circle crosses the u-axis outside the roi
    // so it is equivalent to the case of crossing only the left border
    std::cout << "\t|->Case left only" << std::endl;
    computeIntersectionsLeftBorderOnly(u_c, umin_roi, radius, delta_theta);
  }
  else if (u_umin >= umin_roi && u_umax >= umin_roi && v_vmin >= vmax_roi && v_vmax >= vmax_roi) {
    // The circle crosses the v-axis outside the roi
    // so it is equivalent to the case of crossing only the top border
    std::cout << "\t|->Case bottom only" << std::endl;
    computeIntersectionsBottomBorderOnly(v_c, vmax_roi, radius, delta_theta);
  }
}

float vpImageCircle::computeArcLengthInRoI(const vpRect &roi) const
{
  float deltaTheta = 0.f;
  vpImagePoint center = m_center;
  float center_u = center.get_u();
  float center_v = center.get_v();
  float radius = m_radius;
  float roi_w = roi.getWidth();
  float roi_h = roi.getHeight();
  vpImagePoint topLeft = roi.getTopLeft();
  float umin_roi = topLeft.get_u();
  float vmin_roi = topLeft.get_v();
  float umax_roi = topLeft.get_u() + roi_w;
  float vmax_roi = topLeft.get_v() + roi_h;
  bool touchLeftBorder = (center_u - radius) <= umin_roi;
  bool touchRightBorder = (center_u + radius) >= umax_roi;
  bool touchTopBorder = (center_v - radius) <= vmin_roi;
  bool touchBottomBorder = (center_v + radius) >= vmax_roi;
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
    computeIntersectionsBottomBorderOnly(center_v, vmax_roi, radius, deltaTheta);
  }
  else if (!touchBottomBorder && touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touches/intersects only the left border of the RoI
    std::cout << "Case left only" << std::endl;
    computeIntersectionsLeftBorderOnly(center_u, umin_roi, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && touchRightBorder && !touchTopBorder) {
    // Touches/intersects only the right border of the RoI
    std::cout << "Case right only" << std::endl;
    computeIntersectionsRightBorderOnly(center_u, umax_roi, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && !touchRightBorder && touchTopBorder) {
    // Touches/intersects only the top border of the RoI
    std::cout << "Case top only" << std::endl;
    computeIntersectionsTopBorderOnly(center_v, vmin_roi, radius, deltaTheta);
  }
  else if (touchBottomBorder && touchLeftBorder && !touchRightBorder && !touchTopBorder) {
    // Touches/intersects the bottom and left borders of the RoI
    std::cout << "Case bottom / left" << std::endl;
    computeBottomLeftIntersections(center_u, center_v, topLeft, roi_h, radius, deltaTheta);
  }
  else if (touchBottomBorder && !touchLeftBorder && touchRightBorder && !touchTopBorder) {
    // Touches/intersects the bottom and right borders of the RoI
    std::cout << "Case bottom / right" << std::endl;
  }
  else if (!touchBottomBorder && touchLeftBorder && !touchRightBorder && touchTopBorder) {
    // Touches/intersects the top and left borders of the RoI
    std::cout << "Case top / left" << std::endl;
    computeTopLeftIntersections(center_u, center_v, topLeft, radius, deltaTheta);
  }
  else if (!touchBottomBorder && !touchLeftBorder && touchRightBorder && touchTopBorder) {
    // Touches/intersects the top and right borders of the RoI
    std::cout << "Case top / right" << std::endl;
    computeTopRightIntersections(center_u, center_v, vmin_roi, umax_roi, radius, deltaTheta);
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
