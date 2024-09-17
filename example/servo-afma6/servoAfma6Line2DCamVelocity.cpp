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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 */

/*!
  \file servoAfma6Line2DCamVelocity.cpp
  \example servoAfma6Line2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  The visual feature is a line.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_AFMA6)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpRobotAfma6 robot;
  vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

  // Load the end-effector to camera frame transformation obtained
  // using a camera intrinsic model with distortion
  robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

  try {
    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480, fps = 60;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);

    // Warm up camera
    vpImage<unsigned char> I;
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);
    std::cout << "cam:\n" << cam << std::endl;

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(10);
    me.setSampleStep(10);

    vpMeLine line;
    line.setDisplay(vpMeSite::RANGE_RESULT);
    line.setMe(&me);

    // Initialize the tracking. Define the line to track.
    line.initTracking(I);
    line.track(I);
    vpDisplay::flush(I);

    // Sets the current position of the visual feature
    vpFeatureLine s_line;
    vpFeatureBuilder::create(s_line, cam, line);

    // Sets the desired position of the visual feature
    vpLine line_d;
    line_d.setWorldCoordinates(1, 0, 0, 0, 0, 0, 1, 0);
    vpHomogeneousMatrix c_M_o(0, 0, 0.3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    line_d.project(c_M_o);

    vpFeatureLine s_line_d;
    vpFeatureBuilder::create(s_line_d, line_d);

    // Define the task
    vpServo task;
    // - We want an eye-in-hand control law
    // - Robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    // - We want to see a line on a line
    task.addFeature(s_line, s_line_d);
    // - Set the gain
    task.setLambda(0.5);
    // - Display task information
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    bool final_quit = false;
    bool send_velocities = false;

    while (!final_quit) {
      double t_start = vpTime::measureTimeMs();
      rs.acquire(I);
      vpDisplay::display(I);

      // Track the line
      line.track(I);
      line.display(I, vpColor::red);

      // Update the current line feature
      vpFeatureBuilder::create(s_line, cam, line);

      // Display the current and the desired features
      s_line.display(cam, I, vpColor::red);
      s_line_d.display(cam, I, vpColor::green);

      vpColVector v = task.computeControlLaw();

      if (!send_velocities) {
        v = 0;
      }

      // Send camera frame velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      {
        std::stringstream ss;
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
        ss.clear();
        ss.str("");
        ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      }

      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        switch (button) {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          break;

        default:
          break;
        }
      }
    }

    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);

    if (!final_quit) {
      while (!final_quit) {
        rs.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }

    // Display task information
    task.print();
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an afma6 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
