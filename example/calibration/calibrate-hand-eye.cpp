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
 * Hand-eye calibration example to estimate hand to eye transformation.
 *
*****************************************************************************/

/*!
  \example calibrate-hand-eye.cpp
  \brief Example of hand-eye calibration to estimate extrinsic camera parameters,
  ie hand-eye homogeneous transformation corresponding to the transformation between
  the robot end-effector and the camera.

*/
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpHandEyeCalibration.h>

int main()
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
#if defined(ENABLE_VISP_NAMESPACE)
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    // We want to calibrate the hand-eye extrinsic camera parameters from 6
    // couple of poses: cMo and wMe
    const unsigned int N = 6;
    // Input: six couple of poses used as input in the calibration proces
    std::vector<vpHomogeneousMatrix> cMo(N); // eye (camera) to object
    // transformation. The object
    // frame is attached to the
    // calibrartion grid
    std::vector<vpHomogeneousMatrix> wMe(N); // world to hand (end-effector) transformation
    // Output: Result of the calibration
    vpHomogeneousMatrix eMc; // hand (end-effector) to eye (camera) transformation

    // Initialize an eMc transformation used to produce the simulated input
    // transformations cMo and wMe
    vpTranslationVector etc(0.1, 0.2, 0.3);
    vpThetaUVector erc;
    erc[0] = vpMath::rad(10);  // 10 deg
    erc[1] = vpMath::rad(-10); // -10 deg
    erc[2] = vpMath::rad(25);  // 25 deg

    eMc.build(etc, erc);
    std::cout << "Simulated hand-eye transformation: eMc " << std::endl;
    std::cout << eMc << std::endl;
    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2])
      << std::endl;

    vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
    for (unsigned int i = 0; i < N; i++) {
      v_c = 0;
      if (i == 0) {
        // Initialize first poses
        cMo[0].build(0, 0, 0.5, 0, 0, 0); // z=0.5 m
        wMe[0].build(0, 0, 0, 0, 0, 0);   // Id
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
        wMe[i] = wMe[i - 1] * eMc * cMc * eMc.inverse();
      }
    }

    //    if (1) {
    if (1) {
      for (unsigned int i = 0; i < N; i++) {
        vpHomogeneousMatrix wMo;
        wMo = wMe[i] * eMc * cMo[i];
        std::cout << std::endl << "wMo[" << i << "] " << std::endl;
        std::cout << wMo << std::endl;
        std::cout << "cMo[" << i << "] " << std::endl;
        std::cout << cMo[i] << std::endl;
        std::cout << "wMe[" << i << "] " << std::endl;
        std::cout << wMe[i] << std::endl;
      }
    }

    // Reset the eMc matrix to eye
    eMc.eye();

    // Compute the eMc hand to eye transformation from six poses
    // - cMo[6]: camera to object poses as six homogeneous transformations
    // - wMe[6]: world to hand (end-effector) poses as six homogeneous
    // transformations
    int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

    if (ret == 0) {
      std::cout << std::endl << "** Hand-eye calibration succeed" << std::endl;
      std::cout << std::endl << "** Hand-eye (eMc) transformation estimated:" << std::endl;
      std::cout << eMc << std::endl;
      std::cout << "** Corresponding pose vector: " << vpPoseVector(eMc).t() << std::endl;
      eMc.extract(erc);
      std::cout << std::endl
        << "** Translation [m]: " << eMc[0][3] << " " << eMc[1][3] << " " << eMc[2][3] << std::endl;
      std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
      std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
        << " " << vpMath::deg(erc[2]) << std::endl;
      vpQuaternionVector quaternion(eMc.getRotationMatrix());
      std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;
    }
    else {
      std::cout << std::endl << "** Hand-eye calibration failed" << std::endl;
      std::cout << std::endl
        << "Check your input data and ensure they are covering the half sphere over the chessboard."
        << std::endl;
      std::cout << std::endl
        << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html" << std::endl;
    }

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
  }
