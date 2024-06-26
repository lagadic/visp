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
 * Test some vpHomogeneousMatrix functionalities.
 */

/*!
  \example testHomogeneousMatrix.cpp

  Test some vpHomogeneousMatrix functionalities.
 */
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#include <visp3/core/vpHomogeneousMatrix.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool test_matrix_equal(const vpHomogeneousMatrix &M1, const vpHomogeneousMatrix &M2, double epsilon = 1e-10)
{
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      if (!vpMath::equal(M1[i][j], M2[i][j], epsilon)) {
        return false;
      }
    }
  }
  return true;
}

TEST_CASE("vpHomogeneousMatrix re-orthogonalize rotation matrix", "[vpHomogeneousMatrix]")
{
  CHECK_NOTHROW([]() {
    vpHomogeneousMatrix M { 0.9835,  -0.0581, 0.1716, 0.0072, -0.0489, -0.9972,
                          -0.0571, 0.0352,  0.1744, 0.0478, -0.9835, 0.9470 };
  }());

  SECTION("check re-orthogonalize rotation part")
  {
    vpHomogeneousMatrix M1 { 0.9835,  -0.0581, 0.1716, 0.0072, -0.0489, -0.9972,
                           -0.0571, 0.0352,  0.1744, 0.0478, -0.9835, 0.9470 };

    vpHomogeneousMatrix M2;
    M2[0][0] = 0.9835;
    M2[0][1] = -0.0581;
    M2[0][2] = 0.1716;
    M2[0][3] = 0.0072;
    M2[1][0] = -0.0489;
    M2[1][1] = -0.9972;
    M2[1][2] = -0.0571;
    M2[1][3] = 0.0352;
    M2[2][0] = 0.1744;
    M2[2][1] = 0.0478;
    M2[2][2] = -0.9835;
    M2[2][3] = 0.9470;
    M2.orthogonalizeRotation();

    for (unsigned int i = 0; i < 4; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        CHECK(M1[i][j] == Approx(M2[i][j]).margin(std::numeric_limits<double>::epsilon()));
      }
    }
  }

  CHECK_NOTHROW([]() {
    vpHomogeneousMatrix M { 0.9835, -0.0581, 0.1716, 0.0072,  -0.0937, -0.9738,
                          0.2072, 0.0481,  0.1551, -0.2199, -0.9631, 0.9583 };

  std::cout << "Original data:" << std::endl;
  std::cout << "0.9835 -0.0581  0.1716 0.0072" << std::endl;
  std::cout << " -0.0937 -0.9738  0.2072 0.0481" << std::endl;
  std::cout << "0.1551 -0.2199 -0.9631 0.9583" << std::endl;
  std::cout << "0 0 0 1" << std::endl;
  std::cout << "M after rotation re-orthogonalization:\n" << M << std::endl;
  }());

  CHECK_NOTHROW([]() {
    vpHomogeneousMatrix M1 { 0.9835, -0.0581, 0.1716, 0.0072,  -0.0937, -0.9738,
                           0.2072, 0.0481,  0.1551, -0.2199, -0.9631, 0.9583 };

  // following R init should not throw an exception
  vpRotationMatrix R { M1[0][0], M1[0][1], M1[0][2], M1[1][0], M1[1][1], M1[1][2], M1[2][0], M1[2][1], M1[2][2] };
  }());

  CHECK_THROWS([]() {
    vpHomogeneousMatrix M { 0.983, -0.058, 0.171, 0.0072, -0.093, -0.973, 0.207, 0.0481, 0.155, -0.219, -0.963, 0.9583 };
  }());
}

TEST_CASE("vpRotationMatrix re-orthogonalize rotation matrix", "[vpRotationMatrix]")
{
  CHECK_NOTHROW(
      []() { vpRotationMatrix R { 0.9835, -0.0581, 0.1716, -0.0489, -0.9972, -0.0571, 0.1744, 0.0478, -0.9835 }; }());

  CHECK_NOTHROW([]() {
    vpRotationMatrix R { 0.9835, -0.0581, 0.1716, -0.0937, -0.9738, 0.2072, 0.1551, -0.2199, -0.9631 };

    std::cout << "Original data:" << std::endl;
    std::cout << "0.9835 -0.0581  0.1716" << std::endl;
    std::cout << " -0.0937 -0.9738  0.2072" << std::endl;
    std::cout << "0.1551 -0.2199 -0.9631" << std::endl;
    std::cout << "R after rotation re-orthogonalization:\n" << R << std::endl;
  }());

  CHECK_NOTHROW([]() {
    vpRotationMatrix R {
        0.46682, -0.74434, 0.47754, -0.83228, -0.55233, -0.04733, 0.29899, -0.37535, -0.87734,
    };

    std::cout << "Original data:" << std::endl;
    std::cout << "0.46682, -0.74434,  0.47754" << std::endl;
    std::cout << "-0.83228, -0.55233, -0.04733" << std::endl;
    std::cout << "0.29899, -0.37535, -0.87734" << std::endl;
    std::cout << "R after rotation re-orthogonalization:\n" << R << std::endl;
  }());

  CHECK_NOTHROW([]() {
    vpRotationMatrix R;
    R = {
        0.46682, -0.74434, 0.47754, -0.83228, -0.55233, -0.04733, 0.29899, -0.37535, -0.87734,
    };

    std::cout << "Original data:" << std::endl;
    std::cout << "0.46682, -0.74434,  0.47754" << std::endl;
    std::cout << "-0.83228, -0.55233, -0.04733" << std::endl;
    std::cout << "0.29899, -0.37535, -0.87734" << std::endl;
    std::cout << "R after rotation re-orthogonalization:\n" << R << std::endl;
  }());

  CHECK_THROWS([]() { vpRotationMatrix R { 0.983, -0.058, 0.171, -0.093, -0.973, 0.207, 0.155, -0.219, -0.963 }; }());
}

TEST_CASE("ENU to NED conversion", "[enu2ned]")
{
  vpHomogeneousMatrix enu_M_flu { 0, -1, 0, 0.2, 1, 0, 0, 1., 0, 0, 1, 0.3 };
  std::cout << "enu_M_flu:\n" << enu_M_flu << std::endl;

  vpHomogeneousMatrix enu_M_ned { 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0 };
  std::cout << "enu_M_ned:\n" << enu_M_ned << std::endl;

  vpHomogeneousMatrix flu_M_frd { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0 };
  std::cout << "flu_M_frd:\n" << flu_M_frd << std::endl;

  vpHomogeneousMatrix enu_M_frd = enu_M_flu * flu_M_frd;

  // Test1
  {
    vpHomogeneousMatrix ned_M_frd = enu_M_ned.inverse() * enu_M_flu * flu_M_frd;
    std::cout << "ned_M_frd:\n" << ned_M_frd << std::endl;

    vpHomogeneousMatrix ned_M_frd_est = vpMath::enu2ned(enu_M_frd);
    std::cout << "ned_M_frd_est:\n" << ned_M_frd_est << std::endl;

    bool success = test_matrix_equal(ned_M_frd, ned_M_frd_est);
    std::cout << "Test enu2ned 1 " << (success ? "succeed" : "failed") << std::endl;

    CHECK(success);
  }
  // Test2
  {
    vpHomogeneousMatrix ned_M_flu = enu_M_ned.inverse() * enu_M_flu;
    std::cout << "ned_M_flu:\n" << ned_M_flu << std::endl;

    vpHomogeneousMatrix ned_M_flu_est = vpMath::enu2ned(enu_M_flu);
    std::cout << "ned_M_flu_est:\n" << ned_M_flu_est << std::endl;

    bool success = test_matrix_equal(ned_M_flu, ned_M_flu_est);
    std::cout << "Test enu2ned 2 " << (success ? "succeed" : "failed") << std::endl;

    CHECK(success);
  }
}

TEST_CASE("vpHomogenousMatrix * vpRotationMatrix", "[operator*]")
{
  // Test rotation_matrix * homogeneous_matrix
  vpHomogeneousMatrix _1_M_2_ {
       0.9835, -0.0581,  0.1716, 0.0072,
      -0.0489, -0.9972, -0.0571, 0.0352,
       0.1744,  0.0478, -0.9835, 0.9470
  };
  vpHomogeneousMatrix  _2_M_3_truth {
       0.9835, -0.0581,  0.1716, 0,
      -0.0489, -0.9972, -0.0571, 0,
       0.1744,  0.0478, -0.9835, 0
  };

  vpRotationMatrix _2_R_3_ = _2_M_3_truth.getRotationMatrix();
  vpHomogeneousMatrix _1_M_3_(_1_M_2_* _2_R_3_);
  vpHomogeneousMatrix _1_M_3_truth(_1_M_2_ * _2_M_3_truth);
  bool success = test_matrix_equal(_1_M_3_, _1_M_3_truth);
  std::cout << "Test vpHomogeneousMatrix vpHomogeneousMatrix::operator*(vpRotationMatrix) " << (success ? "succeed" : "failed") << std::endl;
  CHECK(success);
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
