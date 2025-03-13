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
  \example catchCalibHandEye.cpp
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

bool homogeneous_equal(const vpHomogeneousMatrix &M1, const vpHomogeneousMatrix &M2)
{
  bool equal = true;
  vpTranslationVector t1 = M1.getTranslationVector();
  vpTranslationVector t2 = M2.getTranslationVector();
  vpThetaUVector      tu1 = M1.getThetaUVector();
  vpThetaUVector      tu2 = M2.getThetaUVector();
  for (unsigned int i = 0; i < 3; ++i) {
    if (!vpMath::equal(t1[i], t2[i])) {
      std::cout << "Error: Translation " << i << " differ " << std::endl;
      equal = false;
    }
    if (!vpMath::equal(tu1[i], tu2[i])) {
      std::cout << "Error: Theta-u axis-angle rotation " << i << " differ" << std::endl;
      equal = false;
    }
  }
  return equal;
}

SCENARIO("Eye-in-hand calibration", "[eye-in-hand]")
{
  GIVEN("Eye-in-hand data")
  {
    std::cout << "-- First part: Eye-in-hand configuration -- " << std::endl;
    // We want to calibrate the hand-eye extrinsic camera parameters from 6 couple of poses: cMo and rMe
    const unsigned int N = 6;
    // Input: six couple of poses used as input in the calibration process
    // - eye (camera) to object transformation. The object frame is attached to the calibration grid
    std::vector<vpHomogeneousMatrix> cMo(N);
    // - robot reference to hand (end-effector) transformation
    std::vector<vpHomogeneousMatrix> rMe(N);
    // Output: Result of the calibration
    // - ground truth hand (end-effector) to eye (camera) transformation
    vpHomogeneousMatrix eMc_gt;
    // - ground truth robot reference to object transformation
    vpHomogeneousMatrix rMo_gt;

    // Initialize eMc and rMo transformations used to produce the simulated input transformations cMo and rMe
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

    vpTranslationVector rto_gt(-0.1, 0.2, 0.5);
    vpThetaUVector rwo_gt;
    rwo_gt[0] = vpMath::rad(-10); // -10 deg
    rwo_gt[1] = vpMath::rad(10);  //  10 deg
    rwo_gt[2] = vpMath::rad(30);  //  30 deg

    rMo_gt.buildFrom(rto_gt, rwo_gt);
    std::cout << "Ground truth robot reference-object: rMo " << std::endl;
    std::cout << rMo_gt << std::endl;
    std::cout << "Theta U rotation: "
      << vpMath::deg(rwo_gt[0]) << " "
      << vpMath::deg(rwo_gt[1]) << " "
      << vpMath::deg(rwo_gt[2])
      << std::endl;

    vpColVector v_e(6); // end effector velocity used to produce 6 simulated poses
    for (unsigned int i = 0; i < N; ++i) {
      v_e = 0;
      if (i == 0) {
        // Initialize first poses
        rMe[0].buildFrom(0, 0, 0, 0, 0, 0);   // Id
      }
      else if (i == 1) {
        v_e[3] = M_PI / 8;
      }
      else if (i == 2) {
        v_e[4] = M_PI / 8;
      }
      else if (i == 3) {
        v_e[5] = M_PI / 10;
      }
      else if (i == 4) {
        v_e[0] = 0.5;
      }
      else if (i == 5) {
        v_e[1] = 0.8;
      }

      vpHomogeneousMatrix eMe;             // end effector displacement
      eMe = vpExponentialMap::direct(v_e); // Compute the end effectot displacement
      // due to the velocity applied to
      // the end effector
      if (i > 0) {
        // From the end effector displacement eMe, compute the rMe matrix
        rMe[i] = rMe[i - 1] * eMe;
      }
      // Deduce the cMo corresponding matrix
      cMo[i] = eMc_gt.inverse() * rMe[i].inverse() * rMo_gt;
    }

    if (0) {
      for (unsigned int i = 0; i < N; ++i) {
        vpHomogeneousMatrix rMo;
        rMo = rMe[i] * eMc_gt * cMo[i];
        std::cout << std::endl << "rMo[" << i << "] " << std::endl;
        std::cout << rMo << std::endl;
        std::cout << "cMo[" << i << "] " << std::endl;
        std::cout << cMo[i] << std::endl;
        std::cout << "rMe[" << i << "] " << std::endl;
        std::cout << rMe[i] << std::endl;
      }
    }

    WHEN("Estimating eMc and rMo")
    {
      // robot end-effector to camera frames transformation to estimate
      vpHomogeneousMatrix eMc;
      // Robot reference to object transformation frames to estimate
      vpHomogeneousMatrix rMo;

      // Compute eMc and rMo from six poses
      // - cMo[6]: camera to object poses as six homogeneous transformations
      // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous transformations
      CHECK(vpHandEyeCalibration::calibrate(cMo, rMe, eMc, rMo) == 0);
      CHECK(homogeneous_equal(eMc, eMc_gt));
      CHECK(homogeneous_equal(rMo, rMo_gt));
    }
    WHEN("Estimating eMc")
    {
      // robot end-effector to camera frames transformation to estimate
      vpHomogeneousMatrix eMc;

      // Compute eMc hand to eye transformation from six poses
      // - cMo[6]: camera to object poses as six homogeneous transformations
      // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous transformations
      CHECK(vpHandEyeCalibration::calibrate(cMo, rMe, eMc) == 0);
      CHECK(homogeneous_equal(eMc, eMc_gt));
    }
  }
}

SCENARIO("Eye-to-hand calibration", "[eye-to-hand]")
{
  GIVEN("Eye-to-hand data")
  {
    std::cout << "\n-- Second part: Eye-to-hand configuration -- " << std::endl;
    // We want to calibrate the transformation from the robot reference frame
    // to the camera frame rMc using an object rigidly attached to the
    // end effector that is observed by the camera, as well as the
    // transformation eMo from the end effector frame to the camera frame
    // using 6 couples of poses: cMo and rMe. This corresponds to the eye-to-hand
    // configuration
    const unsigned int N = 6;
    // Input: six couple of poses used as input in the calibration process
    std::vector<vpHomogeneousMatrix> oMc(N); //object to camera transformation. The object
    // frame is attached to the calibration grid

    std::vector<vpHomogeneousMatrix> rMe(N); // robot reference to end-effector transformation
    // Output: Result of the calibration
    vpHomogeneousMatrix eMo_gt; // end-effector to object transformation
    vpHomogeneousMatrix rMc_gt; // robot reference to camera transformation

    // Initialize eMo and rMc transformations used to produce the simulated input
    // transformations cMo and rMe
    vpTranslationVector eto_gt(0.2, -0.1, 0.15);
    vpThetaUVector ero_gt;
    ero_gt[0] = vpMath::rad(-15);  // -15 deg
    ero_gt[1] = vpMath::rad(10);   //  10 deg
    ero_gt[2] = vpMath::rad(-25);  // -25 deg

    eMo_gt.buildFrom(eto_gt, ero_gt);
    std::cout << "Ground truth end effector to objet transformation : eMo " << std::endl;
    std::cout << eMo_gt << std::endl;
    std::cout << "Theta U rotation: " << vpMath::deg(ero_gt[0]) << " " << vpMath::deg(ero_gt[1]) << " " << vpMath::deg(ero_gt[2])
      << std::endl;

    vpTranslationVector rtc_gt(1.0, 0.0, 0.0);
    vpThetaUVector rrc_gt;
    rrc_gt[0] = vpMath::rad(10);  // 10 deg
    rrc_gt[1] = vpMath::rad(20);  // 20 deg
    rrc_gt[2] = vpMath::rad(90);  // 90 deg

    rMc_gt.buildFrom(rtc_gt, rrc_gt);
    std::cout << "Ground truth robot reference to camera transformation: rMc " << std::endl;
    std::cout << rMc_gt << std::endl;
    std::cout << "Theta U rotation: " << vpMath::deg(rrc_gt[0]) << " " << vpMath::deg(rrc_gt[1]) << " " << vpMath::deg(rrc_gt[2])
      << std::endl;

    vpColVector v_e(6); // end-effector velocity used to produce 6 simulated poses
    for (unsigned int i = 0; i < N; ++i) {
      v_e = 0;
      if (i == 0) {
        // Initialize first poses
        rMe[0].buildFrom(0.1, -0.1, 1.0, 0.1, -0.2, 0.3);   // general pose
      }
      else if (i == 1) {
        v_e[3] = M_PI / 4;
      }
      else if (i == 2) {
        v_e[4] = -M_PI / 8;
      }
      else if (i == 3) {
        v_e[5] = M_PI / 3;
      }
      else if (i == 4) {
        v_e[0] = -0.5;
      }
      else if (i == 5) {
        v_e[1] = 0.4;
      }

      vpHomogeneousMatrix eMe;             // end-effector displacement
      eMe = vpExponentialMap::direct(v_e); // Compute the end effector displacement
      // due to the velocity applied to
      // the end effector
      if (i > 0) {
        // From the end effector displacement eMe, compute the rMe matrix
        rMe[i] = rMe[i - 1] * eMe;
      }
      // Deduce the cMo and oMc matrices
      vpHomogeneousMatrix cMo = rMc_gt.inverse() * rMe[i] * eMo_gt;
      oMc[i] = cMo.inverse();
    }

    if (0) {
      for (unsigned int i = 0; i < N; ++i) {
        std::cout << "rMe[" << i << "] " << std::endl;
        std::cout << rMe[i] << std::endl;
        std::cout << "cMo[" << i << "] " << std::endl;
        std::cout << oMc[i].inverse() << std::endl;
      }
    }

    WHEN("Estimating eMo and rMc")
    {
      // Robot end-effector to object frames transformation to estimate
      vpHomogeneousMatrix eMo;
      // Robot reference to camera frames transformation to estimate
      vpHomogeneousMatrix rMc;

      // Compute eMo and rMc from six poses
      // - cMo[6]: camera to object poses as six homogeneous transformations
      // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous transformations
      CHECK(vpHandEyeCalibration::calibrate(oMc, rMe, eMo, rMc) == 0);
      CHECK(homogeneous_equal(eMo, eMo_gt));
      CHECK(homogeneous_equal(rMc, rMc_gt));
    }
    WHEN("Estimating eMo")
    {
      // Robot end-effector to object frames transformation to estimate
      vpHomogeneousMatrix eMo;

      // Compute eMo hand to object transformation from six poses
      // - cMo[6]: camera to object poses as six homogeneous transformations
      // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous transformations
      CHECK(vpHandEyeCalibration::calibrate(oMc, rMe, eMo) == 0);
      CHECK(homogeneous_equal(eMo, eMo_gt));
    }
  }
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
