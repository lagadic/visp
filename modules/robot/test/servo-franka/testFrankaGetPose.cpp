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
 *   Test Franka robot behavior
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testFrankaGetPose.cpp

  Test Panda robot from Franka Emika getting robot state implemented in vpRobotFranka.
*/


#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>


int main(int argc, char **argv)
{
  std::string robot_ip = "192.168.1.1";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip 192.168.1.1] [--help] [-h]"
                           << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    std::cout << "-- Start test 1/4" << std::endl;
    vpRobotFranka robot;
    robot.connect(robot_ip);

    vpColVector q;

    for (unsigned i = 0; i < 10; i++) {
      robot.getPosition(vpRobot::JOINT_STATE, q);
      std::cout << "Joint position: " << q.t() << std::endl;
      vpTime::wait(10);
    }

    vpPoseVector fPe;
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
    std::cout << "fMe pose vector: " << fPe.t() << std::endl;
    std::cout << "fMe pose matrix: \n" << vpHomogeneousMatrix(fPe) << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command"
              << " line option set by default to 192.168.1.1. " << std::endl;
     return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    std::cout << "-- Start test 2/4" << std::endl;
    vpRobotFranka robot(robot_ip);

    franka::Robot *handler = robot.getHandler();

    // Get end-effector cartesian position
    std::array<double, 16> pose = handler->readOnce().O_T_EE;
    vpHomogeneousMatrix oMee;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        oMee[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "oMee: \n" << oMee << std::endl;

    // Get flange to end-effector frame transformation
    pose = handler->readOnce().F_T_EE;
    vpHomogeneousMatrix fMee;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        fMee[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "fMee: \n" << fMee << std::endl;

    // Get end-effector to K frame transformation
    pose = handler->readOnce().EE_T_K;
    vpHomogeneousMatrix eeMk;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        eeMk[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "eeMk: \n" << eeMk << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    std::cout << "-- Start test 3/4" << std::endl;
    vpRobotFranka robot;
    robot.connect(robot_ip);

    vpMatrix mass;
    robot.getMass(mass);
    std::cout << "Mass matrix:\n" << mass << std::endl;

    vpColVector gravity;
    robot.getGravity(gravity);
    std::cout << "Gravity vector: " << gravity.t() << std::endl;

    vpColVector coriolis;
    robot.getCoriolis(coriolis);
    std::cout << "Coriolis vector: " << coriolis.t() << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    std::cout << "-- Start test 4/4" << std::endl;
    vpRobotFranka robot;
    robot.connect(robot_ip);

    vpColVector q;
    robot.getPosition(vpRobot::JOINT_STATE, q);
    std::cout << "Joint position: " << q.t() << std::endl;

    vpMatrix fJe;
    robot.get_fJe(fJe);
    std::cout << "Jacobian fJe:\n" << fJe << std::endl;

    robot.get_fJe(q, fJe);
    std::cout << "Jacobian fJe:\n" << fJe << std::endl;

    vpMatrix eJe;
    robot.get_eJe(eJe);
    std::cout << "Jacobian eJe:\n" << eJe << std::endl;

    robot.get_eJe(q, eJe);
    std::cout << "Jacobian eJe:\n" << eJe << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libfranka..." << std::endl;
}
#endif
