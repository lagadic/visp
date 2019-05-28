/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Hand-eye calibration example to estimate hand to eye transformation.
 *
 * Authors:
 * Fabien Spindler
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

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpHandEyeCalibration.h>

int main()
{
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

    eMc.buildFrom(etc, erc);
    std::cout << "Simulated hand-eye transformation: eMc " << std::endl;
    std::cout << eMc << std::endl;
    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2])
        << std::endl;

    vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
    for (unsigned int i = 0; i < N; i++) {
      v_c = 0;
      if (i == 0) {
        // Initialize first poses
        cMo[0].buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
        wMe[0].buildFrom(0, 0, 0, 0, 0, 0);   // Id
      } else if (i == 1)
        v_c[3] = M_PI/8 ;
      else if (i == 2)
        v_c[4] = M_PI/8 ;
      else if (i == 3)
        v_c[5] = M_PI/10 ;
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
    vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

    std::cout << std::endl << "Output: hand-eye calibration result: eMc estimated " << std::endl;
    std::cout << eMc << std::endl;
    eMc.extract(erc);
    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2])
        << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
