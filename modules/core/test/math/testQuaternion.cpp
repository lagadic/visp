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
 * Test quaternion interpolation.
 */

/*!
  \example testQuaternion.cpp

  Test quaternion interpolation.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpQuaternionVector.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Quaternion interpolation", "[quaternion]")
{
  const double angle0 = vpMath::rad(-37.14);
  const double angle1 = vpMath::rad(57.96);
  vpColVector axis({ 1.2, 6.4, -3.7 });
  axis.normalize();
  const vpThetaUVector tu0(angle0 * axis);
  const vpThetaUVector tu1(angle1 * axis);
  const vpQuaternionVector q0(tu0);
  const vpQuaternionVector q1(tu1);
  const double t = 0.5;

  const double ref_angle_middle = t * (angle0 + angle1);
  const double margin = 1e-3;
  const double marginLerp = 1e-1;

  // From:
  // https://github.com/google/mathfu/blob/a75f852f2d76f6f14d5697e0d09ce509a2e3bfc6/unit_tests/quaternion_test/quaternion_test.cpp#L319-L329
  // This will verify that interpolating two quaternions corresponds to interpolating the angle.
  SECTION("LERP")
  {
    vpQuaternionVector qLerp = vpQuaternionVector::lerp(q0, q1, t);
    CHECK(vpThetaUVector(qLerp).getTheta() == Approx(ref_angle_middle).margin(marginLerp));
  }

  SECTION("NLERP")
  {
    vpQuaternionVector qNlerp = vpQuaternionVector::nlerp(q0, q1, t);
    CHECK(vpThetaUVector(qNlerp).getTheta() == Approx(ref_angle_middle).margin(margin));
  }

  SECTION("SERP")
  {
    vpQuaternionVector qSlerp = vpQuaternionVector::slerp(q0, q1, t);
    CHECK(vpThetaUVector(qSlerp).getTheta() == Approx(ref_angle_middle).margin(margin));
  }
}

TEST_CASE("Quaternion operators", "[quaternion]")
{

  SECTION("Addition and subtraction")
  {
    const vpQuaternionVector q1(2.1, -1, -3.7, 1.5);
    const vpQuaternionVector q2(0.5, 1.4, 0.7, 2.5);
    const vpQuaternionVector q3 = q1 + q2;
    const double margin = std::numeric_limits<double>::epsilon();
    std::cout << "q3=" << q3 << std::endl;
    CHECK(q3.x() == Approx(2.6).margin(margin));
    CHECK(q3.y() == Approx(0.4).margin(margin));
    CHECK(q3.z() == Approx(-3.0).margin(margin));
    CHECK(q3.w() == Approx(4.0).margin(margin));


    // Test subtraction of two quaternions
    const vpQuaternionVector q4 = q3 - q1;
    std::cout << "q4=" << q4 << std::endl;
    CHECK(q4.x() == Approx(q2.x()).margin(margin));
    CHECK(q4.y() == Approx(q2.y()).margin(margin));
    CHECK(q4.z() == Approx(q2.z()).margin(margin));
    CHECK(q4.w() == Approx(q2.w()).margin(margin));
  }

  SECTION("Multiplication")
  {
    //// https://www.wolframalpha.com/input/?i=quaternion+-Sin%5BPi%5D%2B3i%2B4j%2B3k+multiplied+by+-1j%2B3.9i%2B4-3k&lk=3
    const vpQuaternionVector q1(3.0, 4.0, 3.0, -sin(M_PI));
    const vpQuaternionVector q2(3.9, -1.0, -3.0, 4.0);
    const vpQuaternionVector q3 = q1 * q2;
    const double margin = std::numeric_limits<double>::epsilon() * 1e4;
    CHECK(q3.x() == Approx(3.0).margin(margin));
    CHECK(q3.y() == Approx(36.7).margin(margin));
    CHECK(q3.z() == Approx(-6.6).margin(margin));
    CHECK(q3.w() == Approx(1.3).margin(margin));
  }

  SECTION("Conjugate")
  {
    const vpQuaternionVector q1(3.0, 36.7, -6.6, 1.3);
    const vpQuaternionVector q1_conj = q1.conjugate();
    const double margin = std::numeric_limits<double>::epsilon();
    CHECK(q1_conj.x() == Approx(-q1.x()).margin(margin));
    CHECK(q1_conj.y() == Approx(-q1.y()).margin(margin));
    CHECK(q1_conj.z() == Approx(-q1.z()).margin(margin));
    CHECK(q1_conj.w() == Approx(q1.w()).margin(margin));
  }

  SECTION("Inverse")
  {
    const vpQuaternionVector q1(3.0, 36.7, -6.6, 1.3);
    const vpQuaternionVector q1_inv = q1.inverse();
    const double margin = 1e-6;
    CHECK(q1_inv.x() == Approx(-0.00214111).margin(margin));
    CHECK(q1_inv.y() == Approx(-0.026193).margin(margin));
    CHECK(q1_inv.z() == Approx(0.00471045).margin(margin));
    CHECK(q1_inv.w() == Approx(0.000927816).margin(margin));
  }

  SECTION("Norm")
  {
    const vpQuaternionVector q1(3.0, 36.7, -6.6, 1.3);
    const double norm = q1.magnitude();
    CHECK(norm == Approx(37.4318).margin(1e-4));
  }

  SECTION("Normalization")
  {
    vpQuaternionVector q1(3.0, 36.7, -6.6, 1.3);
    q1.normalize();
    const double margin = 1e-6;
    const double norm = q1.magnitude();
    CHECK(norm == Approx(1.0).margin(1e-4));
    CHECK(q1.x() == Approx(0.0801457).margin(margin));
    CHECK(q1.y() == Approx(0.98045).margin(margin));
    CHECK(q1.z() == Approx(-0.176321).margin(margin));
    CHECK(q1.w() == Approx(0.0347298).margin(margin));
  }

  SECTION("Copy constructor")
  {
    vpQuaternionVector q_copy1 = vpQuaternionVector(0, 0, 1, 1);
    std::cout << "q_copy1=" << q_copy1 << std::endl;
    const vpQuaternionVector q_copy2 = q_copy1;
    CHECK_FALSE((!vpMath::equal(q_copy2.x(), q_copy1.x()) || !vpMath::equal(q_copy2.y(), q_copy1.y()) ||
                 !vpMath::equal(q_copy2.z(), q_copy1.z()) || !vpMath::equal(q_copy2.w(), q_copy1.w())));

    // compare data pointers: verify that they're not the same
    CHECK(q_copy2.data != q_copy1.data);
    q_copy1.set(1, 0, 1, 10);
    CHECK((vpMath::equal(q_copy2.x(), q_copy1.x()) || vpMath::equal(q_copy2.y(), q_copy1.y()) ||
           vpMath::equal(q_copy2.z(), q_copy1.z()) || vpMath::equal(q_copy2.w(), q_copy1.w())));
    std::cout << "q_copy1 after set = " << q_copy1 << std::endl;
    std::cout << "q_copy2=" << q_copy2 << std::endl;
  }

  SECTION("operator=")
  {
    const vpQuaternionVector q1 = vpQuaternionVector(0, 0, 1, 1);
    vpQuaternionVector q_same(10, 10, 10, 10);
    q_same = q1;

    CHECK_FALSE((!vpMath::equal(q_same.x(), q1.x()) || !vpMath::equal(q_same.y(), q1.y()) ||
                 !vpMath::equal(q_same.z(), q1.z()) || !vpMath::equal(q_same.w(), q1.w())));
    // compare data pointers: verify that they're not the same
    CHECK(q_same.data != q1.data);
  }

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
