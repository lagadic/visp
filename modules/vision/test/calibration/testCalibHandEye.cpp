/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Eye-in-hand calibration test to estimate hand to eye transformation.
 */

/*!
  \example testCalibHandEye.cpp
  \brief Test of eye-in-hand calibration to estimate extrinsic camera parameters,
  ie hand-eye homogeneous transformation corresponding to the transformation between
  the robot end-effector and the camera.
*/
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#include <visp3/core/vpExponentialMap.h>
#include <visp3/vision/vpHandEyeCalibration.h>

#include <catch_amalgamated.hpp>

#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Eye-in-hand calibration", "[vpHandEyeCalibration]")
{
  // We want to calibrate the hand-eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  const unsigned int N = 6;
  // Input: six couple of poses used as input in the calibration process
  // - eye (camera) to object transformation. The object frame is attached to the calibration grid
  std::vector<vpHomogeneousMatrix> cMo(N);
  // - world to hand (end-effector) transformation
  std::vector<vpHomogeneousMatrix> wMe(N);
  // Output: Result of the calibration
  // - ground truth hand (end-effector) to eye (camera) transformation
  vpHomogeneousMatrix eMc_gt;

  // Initialize an eMc transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector etc_gt(0.1, 0.2, 0.3);
  vpThetaUVector erc_gt;
  erc_gt[0] = vpMath::rad(10);  //  10 deg
  erc_gt[1] = vpMath::rad(-10); // -10 deg
  erc_gt[2] = vpMath::rad(25);  //  25 deg

  eMc_gt.buildFrom(etc_gt, erc_gt);
  std::cout << "Ground truth hand-eye transformation: eMc " << std::endl;
  std::cout << eMc_gt << std::endl;
  std::cout << "Theta U rotation: "
    << vpMath::deg(erc_gt[0]) << " "
    << vpMath::deg(erc_gt[1]) << " "
    << vpMath::deg(erc_gt[2])
    << std::endl;

  vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
  for (unsigned int i = 0; i < N; i++) {
    v_c = 0;
    if (i == 0) {
      // Initialize first poses
      cMo[0].buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
      wMe[0].buildFrom(0, 0, 0, 0, 0, 0);   // Id
    }
    else if (i == 1)
      v_c[3] = M_PI / 8;
    else if (i == 2)
      v_c[4] = M_PI / 8;
    else if (i == 3)
      v_c[5] = M_PI / 10;
    else if (i == 4)
      v_c[0] = 0.5;
    else if (i == 5)
      v_c[1] = 0.8;

    vpHomogeneousMatrix cMc;             // camera displacement
    cMc = vpExponentialMap::direct(v_c); // Compute the camera displacement
    // due to the velocity applied to
    // the camera
    if (i > 0) {
      // From the camera displacement cMc, compute the wMe and cMo matrices
      cMo[i] = cMc.inverse() * cMo[i - 1];
      wMe[i] = wMe[i - 1] * eMc_gt * cMc * eMc_gt.inverse();
    }
  }

  if (0) {
    for (unsigned int i = 0; i < N; i++) {
      vpHomogeneousMatrix wMo;
      wMo = wMe[i] * eMc_gt * cMo[i];
      std::cout << std::endl << "wMo[" << i << "] " << std::endl;
      std::cout << wMo << std::endl;
      std::cout << "cMo[" << i << "] " << std::endl;
      std::cout << cMo[i] << std::endl;
      std::cout << "wMe[" << i << "] " << std::endl;
      std::cout << wMe[i] << std::endl;
    }
  }

  // Eye-in-hand homogeneous transformation to estimate
  vpHomogeneousMatrix eMc;

  // Compute the eMc hand to eye transformation from six poses
  // - cMo[6]: camera to object poses as six homogeneous transformations
  // - wMe[6]: world to hand (end-effector) poses as six homogeneous transformations
  int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

  bool success = true;
  if (ret == 0) {
    std::cout << std::endl << "** Hand-eye (eMc) transformation estimated:" << std::endl;
    std::cout << eMc << std::endl;
    std::cout << "** Corresponding pose vector: " << vpPoseVector(eMc).t() << std::endl;
    vpThetaUVector erc;
    vpTranslationVector etc;
    eMc.extract(erc);
    eMc.extract(etc);
    std::cout << std::endl
      << "** Translation [m]: " << etc[0] << " " << etc[1] << " " << etc[2] << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
      << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;

    // Compare with ground truth
    for (unsigned int i = 0; i < 3; ++i) {
      if (!vpMath::equal(etc[i], etc_gt[i])) {
        std::cout << "Error: Translation " << i << " differ from ground truth" << std::endl;
        success = false;
      }
      if (!vpMath::equal(erc[i], erc_gt[i])) {
        std::cout << "Error: Theta-u axis-angle rotation " << i << " differ from ground truth" << std::endl;
        success = false;
      }
    }
    if (success) {
      std::cout << std::endl << "** Hand-eye calibration succeed" << std::endl;
    }
  }
  else {
    std::cout << std::endl << "** Hand-eye calibration failed" << std::endl;
    success = false;
  }

  CHECK(success);
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}

#else
int main()
{
  std::cout << "This test needs catch2 that is not enabled..." << std::endl;
}
#endif
