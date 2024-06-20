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
 * Test line fitting.
 */

/*!
  \example testLineFitting.cpp

  Test line fitting.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMath.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void convertLineEquation(double A, double B, double C, double &a, double &b)
{
  a = -A / B;
  b = C / B;
}
} // namespace

TEST_CASE("Line fitting - Horizontal", "[line_fitting]")
{
  std::cout << "\nLine fitting - Horizontal" << std::endl;
  double a = 0, b = 10;
  std::vector<vpImagePoint> imPts;
  for (int i = 0; i < 3; i++) {
    double x = i * 10;
    imPts.push_back(vpImagePoint(a * x + b, x));
    std::cout << "imPts: (" << imPts.back().get_u() << ", " << imPts.back().get_v() << ")" << std::endl;
  }

  double A = 0, B = 0, C = 0;
  double error = vpMath::lineFitting(imPts, A, B, C);
  std::cout << "error: " << error << std::endl;
  std::cout << "a: " << a << " ; b: " << b << std::endl;
  std::cout << "A: " << A << " ; B: " << B << " ; C: " << C << std::endl;
  double a_est = 0, b_est = 0;
  convertLineEquation(A, B, C, a_est, b_est);
  std::cout << "-A/B: " << a_est << " ; -C/B: " << b_est << std::endl;

  CHECK(a == Approx(a_est).margin(1e-6));
  CHECK(b == Approx(b_est).epsilon(1e-6));
}

TEST_CASE("Line fitting", "[line_fitting]")
{
  std::cout << "\nLine fitting" << std::endl;
  double a = -4.68, b = 21.456;
  std::vector<vpImagePoint> imPts;
  const int nbPoints = 10;
  for (int i = 0; i < nbPoints; i++) {
    double x = i * 10;
    double y = a * x + b;
    imPts.push_back(vpImagePoint(y, x));
    std::cout << "imPts: (" << imPts.back().get_u() << ", " << imPts.back().get_v() << ")" << std::endl;
  }

  double A = 0, B = 0, C = 0;
  double error = vpMath::lineFitting(imPts, A, B, C);
  std::cout << "error: " << error << std::endl;
  std::cout << "a: " << a << " ; b: " << b << std::endl;
  std::cout << "A: " << A << " ; B: " << B << " ; C: " << C << std::endl;
  double a_est = 0, b_est = 0;
  convertLineEquation(A, B, C, a_est, b_est);
  std::cout << "-A/B: " << a_est << " ; -C/B: " << b_est << std::endl;

  CHECK(a == Approx(a_est).epsilon(1e-6));
  CHECK(b == Approx(b_est).epsilon(1e-6));
}

TEST_CASE("Line fitting - Gaussian noise", "[line_fitting]")
{
  std::cout << "\nLine fitting - Gaussian noise" << std::endl;
  const double sigma = 3, mean = 0;
  vpGaussRand gauss(sigma, mean);

  double a = -4.68, b = 21.456;
  std::vector<vpImagePoint> imPts;
  const int nbPoints = 10;
  for (int i = 0; i < nbPoints; i++) {
    double x = i * 10;
    double y = a * x + b;
    imPts.push_back(vpImagePoint(y + gauss(), x + gauss()));
    std::cout << "x: " << x << " ; y: " << y << " ; imPts: (" << imPts.back().get_u() << ", " << imPts.back().get_v()
      << ")" << std::endl;
  }

  double A = 0, B = 0, C = 0;
  double error = vpMath::lineFitting(imPts, A, B, C);
  std::cout << "error: " << error << std::endl;
  std::cout << "a: " << a << " ; b: " << b << std::endl;
  std::cout << "A: " << A << " ; B: " << B << " ; C: " << C << std::endl;
  double a_est = 0, b_est = 0;
  convertLineEquation(A, B, C, a_est, b_est);
  std::cout << "-A/B: " << a_est << " ; -C/B: " << b_est << std::endl;

  REQUIRE(error < sigma);
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
