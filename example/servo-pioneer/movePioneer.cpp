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
 * Example that shows how to control a Pioneer mobile robot in ViSP.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRobotPioneer.h>

#ifndef VISP_HAVE_PIONEER
int main()
{
  std::cout << "\nThis example requires Aria 3rd party library. You should "
               "install it.\n"
            << std::endl;
  return EXIT_SUCCESS;
}

#else

/*!
  \example movePioneer.cpp example showing how to connect and send
  direct basic motion commands to a Pioneer mobile robot.

  WARNING: this program does no sensing or avoiding of obstacles, the robot
  WILL collide with any objects in the way!   Make sure the robot has about
  2-3 meters of free space around it before starting the program.

  This program will work either with the MobileSim simulator or on a real
  robot's onboard computer.  (Or use -remoteHost to connect to a wireless
  ethernet-serial bridge.)
*/
int main(int argc, char **argv)
{
  try {
    std::cout << "\nWARNING: this program does no sensing or avoiding of "
                 "obstacles, \n"
                 "the robot WILL collide with any objects in the way! Make sure "
                 "the \n"
                 "robot has approximately 3 meters of free space on all sides.\n"
              << std::endl;

    vpRobotPioneer robot;

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();

    // ArRobotConnector connects to the robot, get some initial data from it
    // such as type and name, and then loads parameter files for this robot.
    ArRobotConnector robotConnector(&parser, &robot);
    if (!robotConnector.connectRobot()) {
      ArLog::log(ArLog::Terse, "Could not connect to the robot.");
      if (parser.checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
      }
    }
    if (!Aria::parseArgs()) {
      Aria::logOptions();
      Aria::shutdown();
      return false;
    }

    std::cout << "Robot connected" << std::endl;
    robot.useSonar(false); // disable the sonar device usage

    // Robot velocities
    vpColVector v(2), v_mes(2);

    for (int i = 0; i < 100; i++) {
      double t = vpTime::measureTimeMs();

      v = 0;
      v[0] = i / 1000.; // Translational velocity in m/s
      // v[1] = vpMath::rad(i/5.); // Rotational velocity in rad/sec
      robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

      v_mes = robot.getVelocity(vpRobot::REFERENCE_FRAME);
      std::cout << "Trans. vel= " << v_mes[0] << " m/s, Rot. vel=" << vpMath::deg(v_mes[1]) << " deg/s" << std::endl;
      v_mes = robot.getVelocity(vpRobot::ARTICULAR_FRAME);
      std::cout << "Left wheel vel= " << v_mes[0] << " m/s, Right wheel vel=" << v_mes[1] << " m/s" << std::endl;
      std::cout << "Battery=" << robot.getBatteryVoltage() << std::endl;

      vpTime::wait(t, 40);
    }

    ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
    robot.lock();
    robot.stop();
    robot.unlock();
    ArUtil::sleep(1000);

    robot.lock();
    ArLog::log(ArLog::Normal,
               "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. "
               "Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
               robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
    robot.unlock();

    std::cout << "Ending robot thread..." << std::endl;
    robot.stopRunning();

    // wait for the thread to stop
    robot.waitForRunExit();

    // exit
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#endif
