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
 * Test for Virtuose SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 * Nicolò Pedemonte
 *
 *****************************************************************************/

/*!
  \example testVirtuoseAfma6.cpp

  Test Haption Virtuose SDK wrapper to control Afma6 robot velocity from
  haptic device velocity. Movements are allowed inside a cube of
  pre-determined side.
*/

#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/robot/vpVirtuose.h>

int main()
{
#if defined(VISP_HAVE_VIRTUOSE) && defined(VISP_HAVE_AFMA6)
  vpRobotAfma6 robot;
  try {
    vpVirtuose virtuose;
    virtuose.setVerbose(true);
    virtuose.setCommandType(COMMAND_TYPE_IMPEDANCE);
    virtuose.setPowerOn();
    //    virtuose.setSaturation(1.0f,0.0f);

    vpColVector virt_velocity;
    vpColVector robot_velocity;
    vpColVector robot_joint_position;
    vpColVector robot_cart_position;
    vpColVector robot_cart_position_init;
    vpColVector force_feedback_robot(3);
    float force_limit = 15;
    int force_increase_rate = 500;

    double cube_size = 0.15;

    vpHomogeneousMatrix rMv;
    rMv[0][0] = rMv[0][2] = 0;
    rMv[1][1] = rMv[1][2] = 0;
    rMv[2][0] = rMv[2][1] = 0;
    rMv[0][1] = rMv[1][0] = rMv[2][2] = -1;
    std::cout << "rMv:\n" << rMv << std::endl;
    vpVelocityTwistMatrix rVv(rMv);

    // Set the extrinsic camera parameters obtained with a perpective
    // projection model including a distortion parameter
    robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithDistortion);
    // Initialize the controller to position control
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    // Moves the robot in the joint space
    vpColVector q(6, 0);
    robot.setPositioningVelocity(10);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);

    robot.getPosition(vpRobot::REFERENCE_FRAME, robot_cart_position_init);
    vpColVector min(3), max(3);
    for (unsigned int i = 0; i < 3; i++) {
      min[i] = robot_cart_position_init[i] - cube_size / 2;
      max[i] = robot_cart_position_init[i] + cube_size / 2;
    }
    std::cout << "min: " << min.t() << std::endl;
    std::cout << "max: " << max.t() << std::endl;

    // Initialize the controller to position control
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    for (unsigned int iter = 0; iter < 10000; iter++) {
      virt_velocity = virtuose.getVelocity();
      std::cout << "Virtuose velocity: " << virt_velocity.t() << std::endl;

      robot.getPosition(vpRobot::REFERENCE_FRAME, robot_cart_position);

      for (int i = 0; i < 3; i++) {
        if (robot_cart_position[i] >= max[i]) {
          force_feedback_robot[i] = (max[i] - robot_cart_position[i]) * force_increase_rate;
          if (force_feedback_robot[i] <= -force_limit)
            force_feedback_robot[i] = -force_limit;
        } else if (robot_cart_position[i] <= min[i]) {
          force_feedback_robot[i] = (min[i] - robot_cart_position[i]) * force_increase_rate;
          if (force_feedback_robot[i] >= force_limit)
            force_feedback_robot[i] = force_limit;
        } else
          force_feedback_robot[i] = 0;
      }
      vpColVector force_feedback_virt = rMv.getRotationMatrix().inverse() * force_feedback_robot;

      // Printing force feedback
      std::cout << "Force feedback: " << force_feedback_virt.t() << std::endl;

      robot_velocity = rVv * virt_velocity;
      robot.setVelocity(vpRobot::CAMERA_FRAME, robot_velocity);

      // Set force feedback
      vpColVector force_feedback(6, 0);
      force_feedback.insert(0, force_feedback_virt);

      virtuose.setForce(force_feedback);

      vpTime::wait(10);
    }
    robot.stopMotion();
    virtuose.setPowerOff();
    std::cout << "The end" << std::endl;
  } catch (const vpException &e) {
    robot.stopMotion();
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
}
