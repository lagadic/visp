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
 * Test Universal Robots behavior
 */

/*!
  \example testUniversalRobotsGetData.cpp

  Test robot from Universal Robot getting robot state implemented in vpRobotUniversalRobots.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_UR_RTDE)

#include <visp3/robot/vpRobotUniversalRobots.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string robot_ip = "192.168.0.100";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip " << robot_ip << "] [--help] [-h]"
        << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    std::cout << "-- Start test 1/3" << std::endl;
    vpRobotUniversalRobots robot;
    robot.connect(robot_ip);

    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_interface = robot.getRTDEReceiveInterfaceHandler();
    std::shared_ptr<ur_rtde::DashboardClient> db_client = robot.getDashboardClientHandler();

    std::cout << "Robot connected  : " << (rtde_receive_interface->isConnected() ? "yes" : "no") << std::endl;
    std::cout << "Robot mode       : " << rtde_receive_interface->getRobotMode() << std::endl;
    std::cout << "Robot model      : " << db_client->getRobotModel() << std::endl;
    std::cout << "PolyScope version: " << db_client->polyscopeVersion() << std::endl;
    robot.disconnect();
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "ur_rtde exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    std::cout << "-- Start test 2/3" << std::endl;
    vpRobotUniversalRobots robot;
    robot.connect(robot_ip);

    // Next test is only done when robot powered on
    // See robot mode doc:
    // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
    if (robot.getRobotMode() >= 4) {
      vpColVector q, q_init;

      for (unsigned i = 0; i < 10; i++) {
        robot.getPosition(vpRobot::JOINT_STATE, q_init);
        q = q_init;
        std::cout << "Joint position [deg]: " << q.rad2deg().t() << std::endl;
        vpTime::wait(10);
      }

      vpPoseVector fPe;
      robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
      std::cout << "fMe pose vector: " << fPe.t() << std::endl;
      vpHomogeneousMatrix fMe_1(fPe);
      std::cout << "fMe pose matrix: \n" << fMe_1 << std::endl;
      vpColVector position;
      robot.getPosition(vpRobot::END_EFFECTOR_FRAME, position);
      for (size_t i = 0; i < fPe.size(); i++) {
        if (!vpMath::equal(fPe[i], position[i])) {
          std::cout << "Wrong end-effector pose returned by getPosition(). Test failed" << std::endl;
          return EXIT_FAILURE;
        }
      }
      vpHomogeneousMatrix fMe_2 = robot.get_fMe();
      std::cout << "fMe pose matrix: \n" << fMe_2 << std::endl;
      for (size_t i = 0; i < fMe_2.size(); i++) {
        if (!vpMath::equal(fMe_1.data[i], fMe_2.data[i])) {
          std::cout << "Wrong end-effector pose returned by get_fMe(). Test failed" << std::endl;
          return EXIT_FAILURE;
        }
      }

      // Next test is only done when brakes are released
      // See robot mode doc:
      // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
      if (robot.getRobotMode() == 7) {
        vpHomogeneousMatrix fMe_3 = robot.get_fMe(q_init);
        std::cout << "fMe pose matrix: \n" << fMe_3 << std::endl;
        for (size_t i = 0; i < fMe_3.size(); i++) {
          if (!vpMath::equal(fMe_2.data[i], fMe_3.data[i])) {
            std::cout << "Wrong end-effector forward kinematics . Test failed" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }

      vpPoseVector fPc;
      robot.getPosition(vpRobot::TOOL_FRAME, fPc);
      std::cout << "fMc pose vector: " << fPc.t() << std::endl;
      std::cout << "fMc pose matrix: \n" << vpHomogeneousMatrix(fPc) << std::endl;
      robot.getPosition(vpRobot::TOOL_FRAME, position);
      for (size_t i = 0; i < fPc.size(); i++) {
        if (!vpMath::equal(fPc[i], position[i])) {
          std::cout << "Wrong tool pose. Test failed" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
    else {
      std::cout << "To proceed with this test you need to power on the robot" << std::endl;
    }
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "ur_rtde exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    std::cout << "-- Start test 3/3" << std::endl;
    vpRobotUniversalRobots robot(robot_ip);
    robot.disconnect();
    robot.connect(robot_ip);

    vpColVector eFe;
    for (unsigned i = 0; i < 10; i++) {
      robot.getForceTorque(vpRobot::END_EFFECTOR_FRAME, eFe);
      std::cout << "End-effector force/torque: " << eFe.t() << std::endl;
      vpTime::wait(10);
    }

    vpColVector cFc;
    for (unsigned i = 0; i < 10; i++) {
      robot.getForceTorque(vpRobot::TOOL_FRAME, cFc);
      std::cout << "Camera or tool frame force/torque: " << cFc.t() << std::endl;
      vpTime::wait(10);
    }
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "ur_rtde exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libur_rtde 3rd party used to control a robot from Universal Robots..."
    << std::endl;

}
#endif
