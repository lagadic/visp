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
    vpImageCircle noIntersect(vpImagePoint(HEIGHT / 2.f, WIDTH / 2.f), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test no intersection." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test circle touching borders of the RoI
  {
    vpRect roiSquare(OFFSET, OFFSET, HEIGHT, HEIGHT);
    vpImageCircle noIntersect(vpImagePoint(OFFSET + HEIGHT / 2.f, OFFSET + HEIGHT / 2.f), HEIGHT / 2.f);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test circle touching borders of the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the left border, more than half a circle visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    float uc = OFFSET + 24.f;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the left border, less than half a circle visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    float uc = OFFSET - 24.f;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the left border, all the circle is visible
  {
    // Formula: uc = OFFSET - RADIUS * cos(theta)
    float uc = OFFSET + RADIUS;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the left border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the right border, more than half a circle visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    float uc = OFFSET + 616.f;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the right border, less than half a circle visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    float uc = OFFSET + 664.f;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the right border, all the circle is visible
  {
    // Formula: uc = OFFSET + WIDTH - RADIUS * cos(theta)
    float uc = OFFSET + WIDTH - RADIUS;
    float vc = OFFSET + 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the right border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
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
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
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
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
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
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the top border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
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
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
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
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with circle touching the bottom border, all the circle is visible
  {
    // Formula: vc = OFFSET + HEIGHT - RADIUS * sin(theta)
    float uc = OFFSET + 100.f;
    float vc = OFFSET + HEIGHT - RADIUS;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with circle touching the bottom border, all the circle is visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border, crossing each axis once in the RoI
  {
    // Formula: u1   = uc + r cos (theta1) ; vmin = vc + r sin(theta1)
    // Formula: umin = uc + r cos(theta 2) ; v    = vc + r sin(theta2)
    // Choice: theta1 - theta2 = pi / 2 => for theta1 = 0 theta2 = -pi/2
    //      => uc = umin - r cos(theta2) vc = vmin - r sin(theta1)
    float uc = OFFSET;
    float vc = OFFSET;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border, crossing each axis once in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // but crossing only the left axis in the RoI
  {
    // (1): u1   = uc + rcos(theta1) <= umin ; vmin = vc - r sin(theta1)
    // (2): u2   = uc + rcos(theta2) <= umin ; vmin = vc - r sin(theta2)
    // (3): umin = uc + r cos(theta3)        ; v3   = vc - r sin(theta3)
    // (4): umin = uc + r cos(theta4)        ; v4   = vc - r sin(theta4)
    // (3) & (4) => uc = umin - r cos(theta3) = umin - r cos(theta4) <=> theta3 = - theta4
    // (3) & (4) => vc >= vmin + r sin(theta3) && vc >= vmin + r sin (theta4)
    float theta = M_PI / 4.f;
    float uc = OFFSET - RADIUS * std::cos(theta);
    float vc = OFFSET + RADIUS * std::sin(theta) + 1.f;
    vc = std::max(vc, OFFSET + RADIUS * std::sin(-theta) + 1.f);
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border but crossing only the left axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // but crossing only the top axis in the RoI
  {
    // (1): u1   = uc + rcos(theta1) >= umin ; vmin = vc - r sin(theta1)
    // (2): u2   = uc + rcos(theta2) >= umin ; vmin = vc - r sin(theta2)
    // (3): umin = uc + r cos(theta3)        ; v3   = vc - r sin(theta3) <= vmin
    // (4): umin = uc + r cos(theta4)        ; v4   = vc - r sin(theta4) <= vmin
    // (1) & (2) => vmin = vc - r sin (theta1) = vc - r sin(theta2) <=> theta1 = PI - theta2
    // (1)       => uc + r cos(theta1) >= umin <=> uc >= umin - r cos(theta1)
    // (2)       => uc + r cos(theta2) >= umin <=> uc >= umin - r cos(theta2)

    float theta = -1.1f * M_PI_2;
    float uc = OFFSET - RADIUS * std::cos(theta) + 1.f;
    uc = std::max(uc, OFFSET - RADIUS * std::cos((float)M_PI - theta) + 1.f);
    float vc = OFFSET + RADIUS * std::sin(theta);
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 0.2f * M_PI_2 * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border but crossing only the top axis in the RoI." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest status = " << statusTest << std::endl;

    hasSucceeded &= isValueOK;
  }

  // Test with intersections with the top and the left border
  // crossing twice each axis
  {
    // (1): u1   = uc + r cos(theta1) >= umin ; vmin = vc - r sin(theta1)
    // (2): u2   = uc + r cos(theta2) >= umin ; vmin = vc - r sin(theta2)
    // (3): umin = uc + r cos(theta3)        ; v3   = vc - r sin(theta3) >= vmin
    // (4): umin = uc + r cos(theta4)        ; v4   = vc - r sin(theta4) >= vmin
    // (1) & (2) => vmin = vc - r sin(theta1) = vc - r sin(theta2) <=> theta1 = PI - theta2
    // (1) & (2) =>{ uc >= umin - r cos(theta1) & { uc >= umin - r cos(PI - theta1)
    // (1) & (2)   { vc  = vmin + r sin(theta1) & { vc  = vmin + r sin(PI - theta1)
    // (3) & (4) =>{ uc  = umin - r cos(theta3) & { uc  = umin - r cos(   - theta3)
    // (3) & (4)   { vc >= vmin - r sin(theta3) & { vc >= vmin - r cos(   - theta3)

    float theta1 = 5.f * M_PI / 8.f;
    float theta2 = M_PI - theta1;
    float uc = OFFSET - RADIUS * std::cos(theta1) + 1.f;
    uc = std::max(uc, OFFSET - RADIUS * std::cos((float)M_PI - theta1) + 1.f);
    float vc = OFFSET + RADIUS * std::sin(theta1);
    float theta3 = std::acos((OFFSET - uc)/RADIUS);
    if (theta3 > M_PI) {
      theta3 -= 2.0 * M_PI;
    }
    else if (theta3 > M_PI) {
      theta3 += 2.0 * M_PI;
    }
    float theta4 = -theta3;
    if (theta4 < 0) {
      float temp = theta4;
      theta4 = theta3;
      theta3 = temp;
    }
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = ((theta4 - theta1) + (theta2 - theta3)) * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "SUCCESS";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test with intersections with the top and the left border crossing twice each axis ." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length = " << theoreticalValue << std::endl;
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
