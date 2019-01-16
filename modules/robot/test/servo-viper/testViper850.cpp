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
 * Test for Viper850 6 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testViper850.cpp

  Example of a real robot control, the Viper robot (arm robot, with 6
  degrees of freedom).
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpViper850.h>

#include <iostream>

int main()
{
  try {

    std::cout << "a test for vpViper850 class..." << std::endl;

    vpViper850 viper850;
    vpCameraParameters cam;

    std::cout << "-- Default settings for Viper 850  ---" << std::endl;
    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the Marlin F033C camera without "
                 "distortion ---"
              << std::endl;
    viper850.init(vpViper850::TOOL_MARLIN_F033C_CAMERA);

    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    std::cout << "-- Settings associated to the Marlin F033C camera with "
                 "distortion ------"
              << std::endl;
    viper850.init(vpViper850::TOOL_MARLIN_F033C_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << viper850 << std::endl;
    viper850.getCameraParameters(cam, 640, 480);
    std::cout << cam << std::endl;

    vpColVector qmotor(6);
#if 1
    qmotor[0] = vpMath::rad(30);
    qmotor[1] = vpMath::rad(-100);
    qmotor[2] = vpMath::rad(180);
    qmotor[3] = vpMath::rad(20);
    qmotor[4] = vpMath::rad(90);
    qmotor[5] = vpMath::rad(13.37);
#else
    qmotor[0] = vpMath::rad(0);
    qmotor[1] = vpMath::rad(0 - 90);
    qmotor[2] = vpMath::rad(0);
    qmotor[3] = vpMath::rad(0);
    qmotor[4] = vpMath::rad(0);
    qmotor[5] = vpMath::rad(0);
#endif
    vpHomogeneousMatrix fMe;
    viper850.get_fMe(qmotor, fMe);

    vpTranslationVector t;
    fMe.extract(t);
    vpRotationMatrix R;
    fMe.extract(R);
    vpRzyzVector r;
    r.buildFrom(R);

    std::cout << "fMe:" << std::endl
              << "\tt: " << t.t() << std::endl
              << "\trzyz (rad): " << r.t() << std::endl
              << "\trzyz (deg): " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2])
              << std::endl;

    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return 1;
  }
}
