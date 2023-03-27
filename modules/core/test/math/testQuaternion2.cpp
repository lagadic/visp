/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Test quaternion interpolation.
 *
 *****************************************************************************/

/*!
  \example testQuaternion2.cpp

  Test quaternion interpolation.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpQuaternionVector.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

TEST_CASE("Quaternion interpolation", "[quaternion]")
{
  const double angle0 = vpMath::rad(-37.14);
  const double angle1 = vpMath::rad(57.96);
  vpColVector axis({1.2, 6.4, -3.7});
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
