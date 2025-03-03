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

TEST_CASE("Eye-in-hand calibration", "[vpHandEyeCalibration]")
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

  // Eye-in-hand homogeneous transformation to estimate
  vpHomogeneousMatrix eMc;
  // Robot reference to object transformation also given as output
  vpHomogeneousMatrix rMo;

  // Compute the eMc hand to eye transformation from six poses
  // - cMo[6]: camera to object poses as six homogeneous transformations
  // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous transformations
  int ret = vpHandEyeCalibration::calibrate(cMo, rMe, eMc, rMo);

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
      << "** Translation [m]: " << etc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
      << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;

    std::cout << std::endl << "** Robot reference-object (rMo) transformation estimated:" << std::endl;
    std::cout << rMo << std::endl;
    std::cout << "** Corresponding pose vector: " << vpPoseVector(rMo).t() << std::endl;
    vpThetaUVector rwo;
    vpTranslationVector rto;
    rMo.extract(rwo);
    rMo.extract(rto);
    std::cout << std::endl
      << "** Translation [m]: " << rto.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << rwo.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(rwo[0]) << " " << vpMath::deg(rwo[1])
      << " " << vpMath::deg(rwo[2]) << std::endl;
    vpQuaternionVector quaternion2(rMo.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion2.t() << std::endl;

    // Compare with ground truth
    for (unsigned int i = 0; i < 3; ++i) {
      if (!vpMath::equal(etc[i], etc_gt[i])) {
        std::cout << "Error: Translation " << i << " in eMc differ from ground truth" << std::endl;
        success = false;
      }
      if (!vpMath::equal(erc[i], erc_gt[i])) {
        std::cout << "Error: Theta-u axis-angle rotation " << i << " in eMc differ from ground truth" << std::endl;
        success = false;
      }
    }
    for (unsigned int i = 0; i < 3; ++i) {
      if (!vpMath::equal(rto[i], rto_gt[i])) {
        std::cout << "Error: Translation " << i << " in rMo differ from ground truth" << std::endl;
        success = false;
      }
      if (!vpMath::equal(rwo[i], rwo_gt[i])) {
        std::cout << "Error: Theta-u axis-angle rotation " << i << " in rMo differ from ground truth" << std::endl;
        success = false;
      }
    }
    if (success) {
      std::cout << std::endl << "** eye-in-hand calibration succeed" << std::endl;
    }
  }
  else {
    std::cout << std::endl << "** eye-in-hand calibration failed" << std::endl;
    success = false;
  }

  CHECK(success);
}

TEST_CASE("Eye-to-hand calibration", "[vpHandEyeCalibration]")
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
  std::vector<vpHomogeneousMatrix> oMc(N); //object to camera
 // transformation. The object
  // frame is attached to the
  // calibration grid

  std::vector<vpHomogeneousMatrix> rMe(N); // robot reference to end-effector transformation
  // Output: Result of the calibration
  vpHomogeneousMatrix eMo; // end-effector to object transformation
  vpHomogeneousMatrix rMc; // robot reference to camera transformation

  // Initialize eMo and rMc transformations used to produce the simulated input
  // transformations cMo and rMe
  vpTranslationVector eto_gt(0.2, -0.1, 0.15);
  vpThetaUVector ero_gt;
  ero_gt[0] = vpMath::rad(-15);  // -15 deg
  ero_gt[1] = vpMath::rad(10);   //  10 deg
  ero_gt[2] = vpMath::rad(-25);  // -25 deg

  eMo.buildFrom(eto_gt, ero_gt);
  std::cout << "Ground truth end effector to objet transformation : eMo " << std::endl;
  std::cout << eMo << std::endl;
  std::cout << "Theta U rotation: " << vpMath::deg(ero_gt[0]) << " " << vpMath::deg(ero_gt[1]) << " " << vpMath::deg(ero_gt[2])
    << std::endl;

  vpTranslationVector rtc_gt(1.0, 0.0, 0.0);
  vpThetaUVector rrc_gt;
  rrc_gt[0] = vpMath::rad(10);  // 10 deg
  rrc_gt[1] = vpMath::rad(20);  // 20 deg
  rrc_gt[2] = vpMath::rad(90);  // 90 deg

  rMc.buildFrom(rtc_gt, rrc_gt);
  std::cout << "Ground truth robot reference to camera transformation: rMc " << std::endl;
  std::cout << rMc << std::endl;
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
    vpHomogeneousMatrix cMo = rMc.inverse() * rMe[i] * eMo;
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

  // Reset the rMc and eMo matrices to eye
  rMc.eye();
  eMo.eye();

  // Compute the eMo and rMc transformations from six poses
  // - rMe[6]: robot reference to hand (end-effector) poses as six homogeneous
  // - oMc[6]: object to camera poses as six homogeneous transformations
  // transformations
  int ret = vpHandEyeCalibration::calibrate(oMc, rMe, eMo, rMc);
  bool success = true;
  if (ret == 0) {
    std::cout << std::endl << "** End effector to object transformation eMo estimated:" << std::endl;
    std::cout << eMo << std::endl;
    std::cout << "** Corresponding pose vector: " << vpPoseVector(eMo).t() << std::endl;
    vpThetaUVector ero;
    vpTranslationVector eto;
    eMo.extract(ero);
    eMo.extract(eto);
    std::cout << std::endl
      << "** Translation [m]: " << eto.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << ero.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(ero[0]) << " " << vpMath::deg(ero[1])
      << " " << vpMath::deg(ero[2]) << std::endl;
    vpQuaternionVector quaternion(eMo.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;

    std::cout << std::endl << "** Robot reference to camera transformation rMc estimated:" << std::endl;
    std::cout << rMc << std::endl;
    std::cout << "** Corresponding pose vector: " << vpPoseVector(rMc).t() << std::endl;
    vpThetaUVector rrc;
    vpTranslationVector rtc;
    rMc.extract(rrc);
    rMc.extract(rtc);
    std::cout << std::endl
      << "** Translation [m]: " << rtc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << rrc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(rrc[0]) << " " << vpMath::deg(rrc[1])
      << " " << vpMath::deg(rrc[2]) << std::endl;
    vpQuaternionVector quaternion2(rMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion2.t() << std::endl;

    // Compare with ground truth
    for (unsigned int i = 0; i < 3; ++i) {
      if (!vpMath::equal(eto[i], eto_gt[i])) {
        std::cout << "Error: Translation " << i << " in eMo differ from ground truth" << std::endl;
        success = false;
      }
      if (!vpMath::equal(ero[i], ero_gt[i])) {
        std::cout << "Error: Theta-u axis-angle rotation " << i << " in eMo differ from ground truth" << std::endl;
        success = false;
      }
    }
    for (unsigned int i = 0; i < 3; ++i) {
      if (!vpMath::equal(rtc[i], rtc_gt[i])) {
        std::cout << "Error: Translation " << i << " in rMc differ from ground truth" << std::endl;
        success = false;
      }
      if (!vpMath::equal(rrc[i], rrc_gt[i])) {
        std::cout << "Error: Theta-u axis-angle rotation " << i << " in rMc differ from ground truth" << std::endl;
        success = false;
      }
    }
  }

  if (success) {
    std::cout << std::endl << "** eye-to-hand calibration succeed" << std::endl;
  }
  else {
    std::cout << std::endl << "** eye-to-hand calibration failed" << std::endl;
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
