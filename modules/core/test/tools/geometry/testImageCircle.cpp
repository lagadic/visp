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
 * Test vpRect.
 *
*****************************************************************************/

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpRect.h>

bool compareAngles(const float &actualVal, const float &theoreticalVal)
{
  // Allow up to 1 pixel of difference, due to rounding effects
  return (std::abs(theoreticalVal - actualVal) < 1.f);
}

float ensureIsBetweenMinPiAndPi(const float &theta)
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

int main()
{
  const float OFFSET = 5.f;
  const float WIDTH = 640.f;
  const float HEIGHT = 480.f;
  const float RADIUS = std::min(WIDTH, HEIGHT) / 10.f;
  vpRect roi(OFFSET, OFFSET, WIDTH, HEIGHT);
  bool hasSucceeded = true;

  // Test with no intersections
  {
    vpImageCircle circle(vpImagePoint(HEIGHT / 2.f, WIDTH / 2.f), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test no intersection." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test circle touching borders of the RoI
  {
    vpRect roiSquare(OFFSET, OFFSET, HEIGHT, HEIGHT);
    vpImageCircle circle(vpImagePoint(OFFSET + HEIGHT / 2.f, OFFSET + HEIGHT / 2.f), HEIGHT / 2.f);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * HEIGHT / 2.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test circle touching borders of the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the left border, more than half a circle visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    // theta := 2 * PI / 3
    float uc = OFFSET + 24.f;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the left border, less than half a circle visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    // theta := PI / 3
    float uc = OFFSET - 24.f;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the left border, all the circle is visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    // theta := PI
    float uc = OFFSET + RADIUS;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the left border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the right border, more than half a circle visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    // theta := PI / 3
    float uc = OFFSET + 616.f;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the right border, less than half a circle visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    // theta := 2 * PI / 3
    float uc = OFFSET + 664.f;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the right border, all the circle is visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    // theta := 0
    float uc = OFFSET + WIDTH - RADIUS;
    float vc = OFFSET + 100.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the right border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top border, more than half a circle visible
  {
    // v = vc - r sin(theta)
    // Formula: vc = OFFSET + RADIUS * sin(theta)
    float theta = M_PI / 3.f;
    float uc = OFFSET + 100.f;
    float vc = OFFSET + RADIUS * sin(theta);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top border, less than half a circle visible
  {
    // v = vc - r sin(theta)
    // Formula: vc = OFFSET + RADIUS * sin(theta)
    float theta = -2.f * M_PI/3.f;
    float uc = OFFSET + 100.f;
    float vc = OFFSET + RADIUS * std::sin(theta);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the top border, all the circle is visible
  {
    // v = vc - r sin(theta)
    // Formula: vc = OFFSET + RADIUS * sin(theta)
    float theta = M_PI_2;
    float uc = OFFSET + 100.f;
    float vc = OFFSET + RADIUS * sin(theta);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the top border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom border, more than half a circle visible
  {
    // v = vc - r sin(theta)
    // Formula: vc = OFFSET + HEIGHT + RADIUS * sin(theta)
    float theta = -M_PI / 3.f;
    float uc = OFFSET + 100.f;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom border, less than half a circle visible
  {
    // v = vc - r sin(theta)
    // Formula: vc = OFFSET + HEIGHT + RADIUS * sin(theta)
    float theta = M_PI / 3.f;
    float uc = OFFSET + 100.f;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the bottom border, all the circle is visible
  {
    // Formula: vc = OFFSET + HEIGHT - RADIUS * sin(theta)
    float uc = OFFSET + 100.f;
    float vc = OFFSET + HEIGHT - RADIUS;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the bottom border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border, crossing each axis once in the RoI
  {
    // Formula: u_cross_top_max   = uc + r cos (theta_u_top_max) >= umin ; vmin = vc - r sin(theta_u_top_max)
    // Formula: umin = uc + r cos(theta_v_max) ; v    = vc - r sin(theta_v_max) >= vmin
    // Choice: theta_u_top_max - theta_v_max = pi / 2 => for theta_u_top_max = 0 theta_v_max = -pi/2
    //      => uc = umin - r cos(theta_v_max) vc = vmin + r sin(theta_u_top_max)
    float uc = OFFSET;
    float vc = OFFSET;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border, crossing each axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // but crossing only the left axis in the RoI
  {
    // (1): u_cross_top_min = uc + rcos(theta_u_top_min) <= umin ; vmin = vc - r sin(theta_u_top_min)
    // (2): u_cross_top_max = uc + rcos(theta_u_top_max) <= umin ; vmin = vc - r sin(theta_u_top_max)
    // (3): umin = uc + r cos(theta_v_min)        ; v_cross_min = vc - r sin(theta_v_min) >= vmin && <= vmin + height
    // (4): umin = uc + r cos(theta_v_max)        ; v_cross_max = vc - r sin(theta_v_max) >= vmin && <= vmin + height
    // (3) & (4) => uc = umin - r cos(theta_v_min) = umin - r cos(theta_v_max) <=> theta_v_min = - theta_v_max
    // (3) & (4) => vc >= vmin + r sin(theta_v_min) && vc >= vmin + r sin (theta_v_max)
    float theta_v_min = M_PI / 4.f;
    float uc = OFFSET - RADIUS * std::cos(theta_v_min);
    float vc = OFFSET + RADIUS * std::sin(theta_v_min) + 1.f;
    vc = std::max(vc, OFFSET + RADIUS * std::sin(-theta_v_min) + 1.f);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border but crossing only the left axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // but crossing only the top axis in the RoI
  {
    // (1): u_cross_top_min   = uc + rcos(theta_u_top_min) >= umin ; vmin = vc - r sin(theta_u_top_min)
    // (2): u_cross_top_max   = uc + rcos(theta_u_top_max) >= umin ; vmin = vc - r sin(theta_u_top_max)
    // (3): umin = uc + r cos(theta_v_min)        ; v_cross_min   = vc - r sin(theta_v_min) <= vmin
    // (4): umin = uc + r cos(theta_v_max)        ; v_cross_max   = vc - r sin(theta_v_max) <= vmin
    // (1) & (2) => vmin = vc - r sin (theta_u_top_min) = vc - r sin(theta_u_top_max) <=> theta_u_top_min = PI - theta_u_top_max
    // (1)       => uc + r cos(theta_u_top_min) >= umin <=> uc >= umin - r cos(theta_u_top_min)
    // (2)       => uc + r cos(theta_u_top_max) >= umin <=> uc >= umin - r cos(theta_u_top_max)

    float theta_u_top_min = -1.1f * M_PI_2;
    float uc = OFFSET - RADIUS * std::cos(theta_u_top_min) + 1.f;
    uc = std::max(uc, OFFSET - RADIUS * std::cos((float)M_PI - theta_u_top_min) + 1.f);
    float vc = OFFSET + RADIUS * std::sin(theta_u_top_min);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = 0.2f * M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border but crossing only the top axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // crossing twice each axis
  {
    // (1): u_cross_top_min   = uc + r cos(theta_u_top_min) >= umin ; vmin = vc - r sin(theta_u_top_min)
    // (2): u_cross_top_max   = uc + r cos(theta_u_top_max) >= umin ; vmin = vc - r sin(theta_u_top_max)
    // (3): umin = uc + r cos(theta_v_min)        ; v_cross_min   = vc - r sin(theta_v_min) >= vmin
    // (4): umin = uc + r cos(theta_v_max)        ; v_cross_max   = vc - r sin(theta_v_max) >= vmin
    // (1) & (2) => vmin = vc - r sin(theta_u_top_min) = vc - r sin(theta_u_top_max) <=> theta_u_top_min = PI - theta_u_top_max
    // (1) & (2) =>{ uc >= umin - r cos(theta_u_top_min) & { uc >= umin - r cos(PI - theta_u_top_min)
    // (1) & (2)   { vc  = vmin + r sin(theta_u_top_min) & { vc  = vmin + r sin(PI - theta_u_top_min)
    // (3) & (4) =>{ uc  = umin - r cos(theta_v_min) & { uc  = umin - r cos(- theta_v_min)
    // (3) & (4)   { vc >= vmin - r sin(theta_v_min) & { vc >= vmin - r cos(- theta_v_min)

    float theta_u_top_min = 5.f * M_PI / 8.f;
    float theta_u_top_max = M_PI - theta_u_top_min;
    float uc = OFFSET - RADIUS * std::cos(theta_u_top_min) + 1.f;
    uc = std::max(uc, OFFSET - RADIUS * std::cos((float)M_PI - theta_u_top_min) + 1.f);
    float vc = OFFSET + RADIUS * std::sin(theta_u_top_min);
    float theta_v_min = std::acos((OFFSET - uc)/RADIUS);
    theta_v_min = ensureIsBetweenMinPiAndPi(theta_v_min);
    float theta_v_max = -theta_v_min;
    if (theta_v_max < 0) {
      float temp = theta_v_max;
      theta_v_max = theta_v_min;
      theta_v_min = temp;
    }
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = ((theta_v_max - theta_u_top_min) + (theta_u_top_max - theta_v_min)) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border crossing twice each axis ." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length = " << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the right border, crossing each axis once in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin + width ; vmin = vc - r sin(theta_u_top_min)
    // (2): umin + width = uc + r cos(theta_v_min) ; v_cross_min = vc - r sin(theta_v_min) <= vmin
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >=  umin + width ; vmin = vc - r sin(theta_u_top_max)
    // (4): umin + width = uc + r cos(theta_v_max) ; v_cross_max = vc - r sin(theta_v_max) >= vmin
    // Choice: for theta_u_top_min = 2PI/3 theta_v_max = -pi/2
    // (4) => uc = umin + width - r cos(theta_v_max)
    // (1) => vc = vmin + r sin(theta_u_top_min)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_top_min = 2.f * M_PI / 3.f;
    float theta_v_max = -M_PI_2;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_v_max);
    float vc = OFFSET + RADIUS * std::sin(theta_u_top_min);;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (M_PI_2 + M_PI / 3.f) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the right border, crossing each axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the right border,
  // but crossing only the right border in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin + width ; vmin = vc - r sin(theta_u_top_min)
    // (2): umin + width = uc + r cos(theta_v_min) ; v_cross_min = vc - r sin(theta_v_min) >= vmin
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >=  umin + width ; vmin = vc - r sin(theta_u_top_max)
    // (4): umin + width = uc + r cos(theta_v_max) ; v_cross_max = vc - r sin(theta_v_max) >= vmin
    // (4) => uc = umin + width - r cos(theta_v_max)
    // (1) => theta_u_top_min = asin((vc - vmin)/r) & uc + r cos(theta_u_top_min) >= umin + width <=> uc + r cos[asin((vc - vmin)/r)] >= umin + width
    // (1) <=> asin((vc - vmin)/r) >= acos[(umin + width - uc)/r] <=> vc >= r sin(acos[(umin + width - uc)/r]) + vmin
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_v_max = -7.f * M_PI / 8.f;
    float theta_v_min = -theta_v_max;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_v_max);
    float vc = RADIUS * std::sin(std::acos((OFFSET + WIDTH - uc)/RADIUS)) + OFFSET + 1.f;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - (theta_v_min - theta_v_max)) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the right border, but crossing only the right border in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the right border,
  // but crossing only the top border in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin + width ; vmin = vc - r sin(theta_u_top_min)
    // (2): umin + width = uc + r cos(theta_v_min) ; v_cross_min = vc - r sin(theta_v_min) <= vmin
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <=  umin + width ; vmin = vc - r sin(theta_u_top_max)
    // (4): umin + width = uc + r cos(theta_v_max) ; v_cross_max = vc - r sin(theta_v_max) <= vmin
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // Choice: theta_u_top_min = -0.9 * PI / 2
    // (1) => vc = vmin + r sin(theta_u_top_min)
    // (2) vc - r sin(theta_v_min) <= vmin => asin((vc - vmin)/r) <= theta_v_min
    float theta_u_top_min = -0.9f * M_PI_2;
    float theta_u_top_max = M_PI - theta_u_top_min;
    theta_u_top_max = ensureIsBetweenMinPiAndPi(theta_u_top_max);
    float vc = OFFSET + RADIUS * std::sin(theta_u_top_min);
    float theta_v_min = std::asin((vc - OFFSET)/RADIUS) + 1.f;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = std::abs(theta_u_top_min - theta_u_top_max) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the right border, but crossing only the top border in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // crossing twice each axis
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) < umin + width ; vmin = vc - r sin(theta_u_top_min)
    // (2): umin + width = uc + r cos(theta_v_min) ; v_cross_min   = vc - r sin(theta_v_min) >= vmin
    // (3): u_cross_top_miax = uc + r cos(theta_u_top_max) <= umin + width; vmin = vc - r sin(theta_u_top_max)
    // (4): umin + width = uc + r cos(theta_v_max) ; v_cross_max   = vc - r sin(theta_v_max) >  vmin
    // (1) & (3) => vmin = vc - r sin(theta_u_top_min) = vc - r sin(theta_u_top_max) <=> theta_u_top_min = PI - theta_u_top_max
    // (1) & (3) =>{ uc < umin + width - r cos(theta_u_top_min) & { uc <= umin + width - r cos(PI - theta_u_top_min)
    // (1) & (3)   { vc  = vmin + r sin(theta_u_top_min) & { vc  = vmin + r sin(PI - theta_u_top_min)
    // (2) & (4) =>{ uc  = umin - r cos(theta_v_min) & { uc  = umin - r cos(- theta_v_min)
    // (2) & (4)   { vc >= vmin - r sin(theta_v_min) & { vc >= vmin - r cos(- theta_v_min)

    float theta_u_top_min = 5.f * M_PI / 8.f;
    float theta_u_top_max = M_PI - theta_u_top_min;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_u_top_min) - 1.f;
    uc = std::min(uc, OFFSET + WIDTH - RADIUS * std::cos((float)M_PI - theta_u_top_min) - 1.f);
    float vc = OFFSET + RADIUS * std::sin(theta_u_top_min);
    float theta_v_min = std::acos((OFFSET + WIDTH - uc)/RADIUS);
    theta_v_min = ensureIsBetweenMinPiAndPi(theta_v_min);
    float theta_v_max = -theta_v_min;
    if (theta_v_min < 0) {
      float temp = theta_v_min;
      theta_v_min = theta_v_max;
      theta_v_max = temp;
    }
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_u_top_min - theta_u_top_max) + (theta_v_min - theta_v_max))) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border crossing twice each axis ." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length = " << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the left border, crossing each axis once in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) <= umin  ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin = uc + r cos(theta_v_min) ; v_cross_min = vc - r sin(theta_v_min) <= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) >=  umin ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin = uc + r cos(theta_v_max) ; v_cross_max = vc - r sin(theta_v_max) >= vmin + height
    // Choice: for theta_v_min = PI/2 theta_u_bot_max = -PI/3
    // (2) => uc = umin - r cos(theta_v_min)
    // (3) => vc = vmin + height + r sin(theta_u_bot_max)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_v_min = M_PI_2;
    float theta_u_bot_max = -M_PI / 3.f;
    float uc = OFFSET - RADIUS * std::cos(theta_v_min);
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_max);;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (M_PI_2 + M_PI / 3.f) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the left border, crossing each axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the left border
  // but crossing only the left border in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) <= umin ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin = uc + r cos(theta_v_min) ; v_cross_min = vc - r sin(theta_v_min) <= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) <= umin ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin = uc + r cos(theta_v_max) ; v_cross_max = vc - r sin(theta_v_max) <= vmin + height
    // Choice: for theta_v_min = PI/8
    // (2) => uc = umin - r cos(theta_v_min)
    // (2) => vc <= vmin + height + r sin(theta_v_min)
    // (4) => vc <= vmin + height + r sin(theta_v_max)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_v_min = M_PI_4 / 2.f;
    float theta_v_max = -theta_v_min;
    float uc = OFFSET - RADIUS * std::cos(theta_v_min);
    float vc = std::min(OFFSET + HEIGHT + RADIUS * std::sin(theta_v_min) - 1.f, OFFSET + HEIGHT + RADIUS * std::sin(theta_v_max) - 1.f);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * theta_v_min) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the left border, but crossing only the left border in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the left border
  // but crossing only the bottom border in the RoI
  {
    // (1): u_cross_bot_min = uc + r cos(theta_u_bot_min) >= umin ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) >= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) >= umin ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) >= vmin + height
    // Choice: for theta_u_bot_min = 5 PI/8
    // (1) => vc = vmin + height + r sin(theta_u_bot_min)
    // (1) => uc >= umin - r cos(theta_u_bot_min)
    // (1) => uc >= umin - r cos(theta_u_bot_max)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_bot_min = 5.f * M_PI_4 / 2.f;
    float theta_u_bot_max = M_PI - theta_u_bot_min;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_min);
    float uc = std::max(OFFSET - RADIUS * std::cos(theta_u_bot_min) + 1.f, OFFSET - RADIUS * std::cos(theta_u_bot_max) + 1.f);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_u_bot_min - theta_u_bot_max) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the left border, but crossing only the bottom border in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the left border
  // crossing each axis twice in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) >= umin ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) < vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) > umin  ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) <= vmin + height
    // (1) & (3) => uc >= umin - r cos(theta_u_bot_min) & uc > umin - r cos(theta_u_bot_max)
    // (1) & (3) => vc = vmin + height + r sin(theta_u_bot_min) & vc = vmin + height + r sin(PI - theta_u_bot_min)
    // (2) & (4) => uc = umin - r cos(theta_v_min) & uc = umin - r cos(-theta_v_min)
    // (2) & (4) => vc < vmin + height + r sin(theta_v_min) & vc < vmin + height + r sin(-theta_v_min)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_bot_min = -5.f * M_PI / 8.f;
    float theta_u_bot_max = M_PI - theta_u_bot_min;
    theta_u_bot_max = ensureIsBetweenMinPiAndPi(theta_u_bot_max);
    float theta_v_min = 7.f * M_PI / 8.f;
    float theta_v_max = -theta_v_min;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_min);
    float uc = OFFSET - RADIUS * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = ((theta_v_min - theta_u_bot_max) + (theta_u_bot_min - theta_v_max)) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the left border, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the right border, crossing each axis once in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) <= umin + width ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin + width = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) <= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) >= umin + width ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin + width = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) >= vmin + height
    // Choice: for theta_u_bot_min = -2PI/3 theta_v_min = PI/2
    // (2) => uc = umin + width - r cos(theta_v_min)
    // (1) => vc = vmin + height + r sin(theta_u_bot_min)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_bot_min = -2.f * M_PI / 3.f;
    float theta_v_min = M_PI_2;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_v_min);
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_min);;
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (M_PI_2 + M_PI / 3.f) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the right border, crossing each axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the right border,
  // crossing only the right axis in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) <= umin + width ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin + width = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) <= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) >= umin + width ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin + width = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) >= vmin + height
    // Choice: for theta_v_min = 5*PI/6
    // (2) => uc = umin + width - r cos(theta_v_min)
    // (2) & (4) => vc <= vmin + height + r sin(theta_v_min) & vc <= vmin + height + r sin(-theta_v_min)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_v_min = 5.f * M_PI / 6.f;
    float uc = OFFSET + WIDTH - RADIUS * std::cos(theta_v_min);
    float vc = std::min(OFFSET + HEIGHT + RADIUS * std::sin(theta_v_min) - 1.f, OFFSET + HEIGHT + RADIUS * std::sin(-theta_v_min) - 1.f);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (M_PI / 3.f) * RADIUS; // <=> 2.f * M_PI / 6.f
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the right border, crossing only the right axis in the RoI in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the right border,
  // crossing only the bottom axis in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) < umin + width ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin + width = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) >= vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) <= umin + width ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin + width = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) >= vmin + height
    // Choice: for theta_u_bot_min = 4*PI/6
    // (1) => vc = vmin + height + r cos(theta_u_bot_min)
    // (1) & (3) => uc < umin + width - r cos(theta_u_bot_min) & uc <= umin + width - r cos(PI - theta_u_bot_min)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_bot_min = 4.f * M_PI / 6.f;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_min);
    float uc = std::min(OFFSET + WIDTH - RADIUS * std::cos(theta_u_bot_min) - 1.f, OFFSET + WIDTH - RADIUS * std::cos((float)M_PI -theta_u_bot_min) - 1.f);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (M_PI / 3.f) * RADIUS; // <=> 2.f * M_PI / 6.f
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the right border, crossing only the bottom axis in the RoI in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the bottom and the right border
  // crossing each axis twice in the RoI
  {
    // (1): u_cross_bot_min   = uc + r cos(theta_u_bot_min) <  umin + width ; vmin + height = vc - r sin(theta_u_bot_min)
    // (2): umin + width = uc + r cos(theta_v_min)         ; v_cross_min = vc - r sin(theta_v_min) < vmin + height
    // (3): u_cross_bot_max   = uc + r cos(theta_u_bot_max) <= umin + width ; vmin + height = vc - r sin(theta_u_bot_max)
    // (4): umin + width = uc + r cos(theta_v_max)         ; v_cross_max = vc - r sin(theta_v_max) <= vmin + height
    // (1) & (3) => uc < umin + width - r cos(theta_u_bot_min) & uc <= umin + width - r cos(PI - theta_u_bot_min)
    // (1) & (3) => vc = vmin + height + r sin(theta_u_bot_min) & vc = vmin + height + r sin(PI - theta_u_bot_min)
    // (2) & (4) => uc = umin + width - r cos(theta_v_min) & uc = umin + width - r cos(-theta_v_min)
    // (2) & (4) => vc < vmin + height + r sin(theta_v_min) & vc < vmin + height + r sin(-theta_v_min)
    // (1) & (3) theta_u_bot_min = PI - theta_u_bot_max
    // (2) & (4) theta_v_min = - theta_v_max
    float theta_u_bot_min = -7.f * M_PI / 8.f;
    float theta_u_bot_max = M_PI - theta_u_bot_min;
    theta_u_bot_max = ensureIsBetweenMinPiAndPi(theta_u_bot_max);
    float theta_v_max = -3.f * M_PI / 8.f;
    float theta_v_min = -theta_v_max;
    float vc = OFFSET + HEIGHT + RADIUS * std::sin(theta_u_bot_min);
    float uc = OFFSET - RADIUS * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), RADIUS);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_v_min - theta_v_max) + (theta_u_bot_max - theta_u_bot_min))) * RADIUS;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the bottom and the right border, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing each axis twice in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) > umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) >=  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 5.f * M_PI / 8.f;
    float theta_u_top_max = 3.f * M_PI / 8.f;
    float theta_v_min = 7.f * M_PI / 8.f;
    float theta_v_max = -theta_v_min;
    float theta_u_bottom_min = -5.f * M_PI / 8.f;
    float theta_u_bottom_max = -3.f * M_PI / 8.f;
    float vc = OFFSET + HEIGHT / 2.f;
    float radius = -(OFFSET - vc)/ std::sin(theta_u_top_min);
    float uc = OFFSET - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = ((theta_v_min - theta_u_top_min) + (theta_u_top_max - theta_u_bottom_max) + (theta_u_bottom_min - theta_v_max)) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing only the top and bottom axes in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) <= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) >= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_max = M_PI / 6.f;
    float theta_u_top_min = M_PI - theta_u_top_max;
    float theta_v_min = M_PI / 3.f;
    float theta_u_bottom_max = -theta_u_top_max;
    float radius = HEIGHT;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_u_top_max - theta_u_bottom_max) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing only the top and bottom axes in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing the top and bottom axes and touching the left axis in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi && v_cross_min <= vmin_roi + height
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = v_cross_min
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) >=  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 4.f * M_PI / 6.f;
    float theta_u_top_max = M_PI - theta_u_top_min;
    float theta_v_min = M_PI;
    float theta_u_bottom_min = -theta_u_top_min;
    float theta_u_bottom_max = -theta_u_top_max;
    float radius = HEIGHT / (2.f * std::sin(theta_u_top_min)); // vmin + h - vmin = (vc - r sin(-theta_u_top_min)) - (vc - r sin(theta_top_min))
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_u_top_min -  theta_u_top_max) + (theta_u_bottom_max - theta_u_bottom_min))) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing the top and bottom axes and touching the left axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing only the left axis in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <= umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = M_PI_2;
    float theta_v_min = M_PI_4;
    float theta_v_max = -theta_v_min;
    float radius = HEIGHT / 2.f;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_v_min - theta_v_max) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing only the left axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing the left axis and touching the two others in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) >=  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = M_PI_2;
    float theta_v_min = 3.f * M_PI_4;
    float theta_v_max = -theta_v_min;
    float radius = HEIGHT / 2.f;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_v_min - theta_v_max) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing the left axis and touching the two others in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing only the top and left axes once in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) < vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) > umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) > & &&  <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_max = 0.f;
    float theta_u_bot_max = -M_PI / 3.f;
    float theta_v_max = -M_PI / 6.f;
    float radius = HEIGHT / (std::sin(theta_u_top_max)  - std::sin(theta_u_bot_max));
    float uc = OFFSET - radius * std::cos(theta_v_max);
    float vc = OFFSET + radius * std::sin(theta_u_top_max);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_u_top_max - theta_v_max) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing only the top and left axes once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the left border
  // crossing the bottom and left axis once in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin_roi ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) >= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <=  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_max = M_PI / 3.f;
    float theta_u_bot_max = 0.f;
    float theta_v_min = M_PI / 6.f;
    float radius = HEIGHT / (std::sin(theta_u_top_max) - std::sin(theta_u_bot_max));
    float uc = OFFSET - radius * std::cos(theta_v_min);
    float vc = OFFSET + radius * std::sin(theta_u_top_max);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_v_min - theta_u_bot_max) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the left border, crossing the bottom and left axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing each axis twice in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 5.f * M_PI / 8.f;
    float theta_u_top_max = 3.f * M_PI / 8.f;
    float theta_v_min = 1.f * M_PI / 8.f;
    float theta_v_max = -theta_v_min;
    float theta_u_bottom_min = -5.f * M_PI / 8.f;
    float theta_u_bottom_max = -3.f * M_PI / 8.f;
    float vc = OFFSET + HEIGHT / 2.f;
    float radius = -(OFFSET - vc)/ std::sin(theta_u_top_min);
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_u_top_min - theta_u_top_max) + (theta_v_min - theta_v_max) + (theta_u_bottom_max - theta_u_bottom_min))) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing only the top and bottom axes in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) <= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) > umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) >= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >   umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 5.f * M_PI / 6.f;
    float theta_v_min = 2.f * M_PI / 3.f;
    float theta_u_bottom_min = -theta_u_top_min;
    float radius = HEIGHT;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - (theta_u_top_min - theta_u_bottom_min)) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing only the top and bottom axes in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing the top and bottom axes and touching the right axis in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi && <= vmin_roi + height
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <= umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = v_cross_min
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <=   umin_roi ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 4.f * M_PI / 6.f;
    float theta_u_top_max = M_PI - theta_u_top_min;
    float theta_v_min = 0;
    float theta_u_bottom_min = -theta_u_top_min;
    float theta_u_bottom_max = -theta_u_top_max;
    float radius = HEIGHT / (2.f * std::sin(theta_u_top_min)); // vmin + h - vmin = (vc - r sin(-theta_u_top_min)) - (vc - r sin(theta_top_min))
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_u_top_min -  theta_u_top_max) + (theta_u_bottom_max - theta_u_bottom_min))) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing the top and bottom axes and touching the right axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing only the right axis in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) > umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) >=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >   umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = M_PI_2;
    float theta_v_min = 3.f * M_PI_4;
    float theta_v_max = -theta_v_min;
    float radius = HEIGHT / 2.f;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - (theta_v_min - theta_v_max)) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing only the right axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing the right axis and touching the two others in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <= umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <=   umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = M_PI_2;
    float theta_v_min = M_PI_4;
    float theta_v_max = -theta_v_min;
    float radius = HEIGHT / 2.f;
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - (theta_v_min - theta_v_max)) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing the right axis and touching the two others in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing the top and right axis once in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) <= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) <= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) >=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >   umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = M_PI;
    float theta_u_bot_min = -2.f * M_PI / 3.f;
    float theta_v_max = -5.f * M_PI / 6.f;
    float radius = HEIGHT / (std::sin(theta_u_top_min) - std::sin(theta_u_bot_min));
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_max);
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - (theta_u_top_min - theta_v_max)) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing the top and right axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top, bottom and the right border
  // crossing the bottom and right axis once in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) >=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (2): umin_roi + width = uc + r cos(theta_v_min); v_cross_min = vc - r sin(theta_v_min) >= vmin_roi
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) >= umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (4): umin_roi + width = uc + r cos(theta_v_max); v_cross_max = vc - r sin(theta_v_max) >= vmin_roi + height
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) >=   umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (2) & (4) theta_v_min = - theta_v_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 2.f * M_PI / 3.f;
    float theta_u_bot_min = M_PI;
    float theta_v_min = 5.f * M_PI / 6.f;
    float radius = HEIGHT / (std::sin(theta_u_top_min) - std::sin(theta_u_bot_min));
    float uc = OFFSET + WIDTH - radius * std::cos(theta_v_min);
    float vc = OFFSET + radius * std::sin(theta_u_top_min);
    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (theta_u_bot_min - theta_v_min) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top, bottom and the right border, crossing the bottom and right axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and bottom only,
  // crossing each axis twice in the RoI
  {
    // (1): u_cross_top_min = uc + r cos(theta_u_top_min) <  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_min)
    // (3): u_cross_top_max = uc + r cos(theta_u_top_max) <=  umin_roi + width ; vmin_roi = vc - r sin(theta_u_top_max)
    // (5): u_cross_bot_min = uc + r cos(theta_u_bottom_min) <  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_min)
    // (6): u_cross_bot_max = uc + r cos(theta_u_bottom_max) <=  umin_roi + width ; vmin_roi + height = vc - r sin(theta_u_bottom_max)
    // (1) & (3) theta_u_top_min = PI - theta_u_top_max
    // (5) & (6) theta_u_bottom_min = PI - theta_u_bottom_max
    float theta_u_top_min = 2.f * M_PI / 3.f;
    float theta_u_top_max = M_PI / 3.f;
    float theta_u_bottom_min = -2.f * M_PI / 3.f;
    float theta_u_bottom_max = -M_PI / 3.f;
    float uc = OFFSET + WIDTH / 2.f;
    float vc = OFFSET + HEIGHT / 2.f;
    float radius = -(OFFSET - vc)/ std::sin(theta_u_top_min);

    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(roi);
    float theoreticalValue = (2.f * M_PI - ((theta_u_top_min - theta_u_top_max) + (theta_u_bottom_max - theta_u_bottom_min))) * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and bottom borders only, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test intersections with all the axis
  {
    // Choosing theta_v_left_min = 7 PI / 8 and circle at the center of the RoI
    // umin = uc + r cos(theta_v_left_min) => r = (umin - uc) / cos(theta_v_left_min)
    vpRect squareRoI(OFFSET, OFFSET, HEIGHT, HEIGHT);
    float theta_v_left_min = 7.f * M_PI / 8.f;
    float uc = OFFSET + HEIGHT  / 2.f;
    float vc = OFFSET + HEIGHT / 2.f;
    float radius = (OFFSET - uc) / std::cos(theta_v_left_min);
    std::cout << "uc = " << uc << "\tvc = " << vc << "\tr = " << radius << std::endl;
    std::cout << "roi = " << squareRoI << std::endl;

    vpImageCircle circle(vpImagePoint(vc, uc), radius);
    float arcLengthCircle = circle.computeArcLengthInRoI(squareRoI);
    float theoreticalValue = M_PI * radius;
    bool isValueOK = compareAngles(arcLengthCircle, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and bottom borders only, crossing each axis twice in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthCircle << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  if (hasSucceeded) {
    std::cout << "testImageCircle overall result: SUCCESS";
    return EXIT_SUCCESS;
  }
  std::cout << "testImageCircle overall result: FAILED";
  return EXIT_FAILURE;
}
