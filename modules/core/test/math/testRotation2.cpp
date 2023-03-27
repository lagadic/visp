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
 * Test theta.u and quaternion multiplication.
 *
 *****************************************************************************/

/*!
  \example testRotation2.cpp

  Test theta.u and quaternion multiplication.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpUniRand.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

namespace
{
vpThetaUVector generateThetaU(vpUniRand &rng)
{
  return vpThetaUVector(
      vpMath::rad(rng.uniform(-180.0, 180.0)) *
      vpColVector({rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)}).normalize());
}

vpQuaternionVector generateQuat(vpUniRand &rng)
{
  const double angle = vpMath::rad(rng.uniform(-180.0, 180.0));
  const double ctheta = std::cos(angle);
  const double stheta = std::sin(angle);
  const double ax = rng.uniform(-1.0, 1.0);
  const double ay = rng.uniform(-1.0, 1.0);
  const double az = rng.uniform(-1.0, 1.0);
  return vpQuaternionVector(stheta * ax, stheta * ay, stheta * az, ctheta);
}
} // namespace

TEST_CASE("Theta u multiplication", "[theta.u]")
{
  const int nTrials = 100;
  const uint64_t seed = 0x123456789;
  vpUniRand rng(seed);
  for (int iter = 0; iter < nTrials; iter++) {
    const vpThetaUVector tu0 = generateThetaU(rng);
    const vpThetaUVector tu1 = generateThetaU(rng);

    const vpRotationMatrix c1Rc2(tu0);
    const vpRotationMatrix c2Rc3(tu1);
    const vpRotationMatrix c1Rc3_ref = c1Rc2 * c2Rc3;
    const vpThetaUVector c1_tu_c3 = tu0 * tu1;
    // two rotation vectors can represent the same rotation,
    // that is why we compare the rotation matrices
    const vpRotationMatrix c1Rc3(c1_tu_c3);

    const double tolerance = 1e-9;
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        CHECK(c1Rc3_ref[i][j] == Approx(c1Rc3[i][j]).epsilon(0).margin(tolerance));
      }
    }
  }
}

TEST_CASE("Quaternion multiplication", "[quaternion]")
{
  const int nTrials = 100;
  const uint64_t seed = 0x123456789;
  vpUniRand rng(seed);
  for (int iter = 0; iter < nTrials; iter++) {
    const vpQuaternionVector q0 = generateQuat(rng);
    const vpQuaternionVector q1 = generateQuat(rng);

    const vpRotationMatrix c1Rc2(q0);
    const vpRotationMatrix c2Rc3(q1);
    const vpRotationMatrix c1Rc3_ref = c1Rc2 * c2Rc3;

    const vpQuaternionVector c1_q_c3 = q0 * q1;
    // two quaternions of opposite sign can represent the same rotation,
    // that is why we compare the rotation matrices
    const vpRotationMatrix c1Rc3(c1_q_c3);

    const double tolerance = 1e-9;
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        CHECK(c1Rc3_ref[i][j] == Approx(c1Rc3[i][j]).epsilon(0).margin(tolerance));
      }
    }
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
