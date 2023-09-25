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
  const float WIDTH = 640.f;
  const float HEIGHT = 480.f;
  const float RADIUS = std::min(WIDTH, HEIGHT) / 10.f;
  vpRect roi(0, 0, WIDTH, HEIGHT);

  // Test with no intersections
  {
    vpImageCircle noIntersect(vpImagePoint(HEIGHT / 2.f, WIDTH / 2.f), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test no intersection." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the left border, more than half a circle visible
  {
    // Formula: uc = - RADIUS * cos(theta)
    float uc = 24.f;
    float vc = 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the left border, less than half a circle visible
  {
    // Formula: uc = - RADIUS * cos(theta)
    float uc = -24.f;
    float vc = 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection left border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the right border, more than half a circle visible
  {
    // Formula: uc = WIDTH - RADIUS * cos(theta)
    float uc = 616.f;
    float vc = 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 4.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the right border, less than half a circle visible
  {
    // Formula: uc = WIDTH - RADIUS * cos(theta)
    float uc = 664.f;
    float vc = 100.f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 2.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection right border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the top border, more than half a circle visible
  {
    // Formula: vc = - RADIUS * sin(theta)
    float uc = 100.f;
    float vc = -41.56921938f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the top border, less than half a circle visible
  {
    // Formula: vc = - RADIUS * sin(theta)
    float uc = 100.f;
    float vc = 41.56921938f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection top border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the bottom border, more than half a circle visible
  {
    // Formula: vc = HEIGHT - RADIUS * sin(theta)
    float uc = 100.f;
    float vc = 521.569219381653f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = 5.f * M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, more than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Test with intersections with the bottom border, less than half a circle visible
  {
    // Formula: vc = HEIGHT - RADIUS * sin(theta)
    float uc = 100.f;
    float vc = 438.430780618347f;
    vpImageCircle noIntersect(vpImagePoint(vc, uc), RADIUS);
    float arcLengthNoIntersect = noIntersect.computeArcLengthInRoI(roi);
    float theoreticalValue = M_PI * RADIUS /3.f;
    bool isValueOK = compareAngles(arcLengthNoIntersect, theoreticalValue);
    std::string statusTest;
    if (isValueOK) {
      statusTest = "OK";
    }
    else {
      statusTest = "FAILED";
    }
    std::cout << "Test intersection bottom border, less than half a circle visible." << std::endl;
    std::cout << "\tarc length =" << arcLengthNoIntersect << std::endl;
    std::cout << "\ttheoretical length =" << theoreticalValue << std::endl;
    std::cout << "\ttest OK = " << statusTest << std::endl;

    if (!isValueOK) {
      std::cerr << "Problem with the computeArcLengthInRoI function!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::cout << "vpImageCircle is ok." << std::endl;
  return EXIT_SUCCESS;
}
