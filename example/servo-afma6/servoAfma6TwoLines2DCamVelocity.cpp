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
  \file servoAfma6TwoLines2DCamVelocity.cpp
  \example servoAfma6TwoLines2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame. Visual features are the two lines.
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

  try {
    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480, fps = 60;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);

    vpImage<unsigned char> I;

    // Warm up camera
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a point " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    int nb_lines = 2;

    std::vector<vpMeLine> line(nb_lines);

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(15);
    me.setSampleStep(10);

    // Initialize the tracking. Define the two lines to track
    // The two lines to track must be parallels
    for (int i = 0; i < nb_lines; ++i) {
      line[i].setDisplay(vpMeSite::RANGE_RESULT);
      line[i].setMe(&me);

      line[i].initTracking(I);
      line[i].track(I);
      vpDisplay::flush(I);
    }

    vpRobotAfma6 robot;
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, vpCameraParameters::perspectiveProjWithoutDistortion);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    // Sets the current position of the visual feature
    std::vector<vpFeatureLine> s_line(nb_lines);
    for (int i = 0; i < nb_lines; ++i) {
      vpFeatureBuilder::create(s_line[i], cam, line[i]);
    }

    // Sets the desired position of the visual feature
    std::vector<vpLine> line_d(2);
    line_d[0].setWorldCoordinates(1, 0, 0, -0.05, 0, 0, 1, 0);
    line_d[1].setWorldCoordinates(1, 0, 0, 0.05, 0, 0, 1, 0);

    vpHomogeneousMatrix c_M_o(0, 0, 0.5, 0, 0, vpMath::rad(0));

    line_d[0].project(c_M_o);
    line_d[1].project(c_M_o);

    // Those lines are needed to keep the conventions define in vpMeLine
    // (Those in vpLine are less restrictive)  Another way to have the
    // coordinates of the desired features is to learn them before executing
    // the program.
    line_d[0].setRho(-fabs(line_d[0].getRho()));
    line_d[0].setTheta(0);
    line_d[1].setRho(-fabs(line_d[1].getRho()));
    line_d[1].setTheta(M_PI);

    std::vector<vpFeatureLine> s_line_d(nb_lines);
    vpFeatureBuilder::create(s_line_d[0], line_d[0]);
    vpFeatureBuilder::create(s_line_d[1], line_d[1]);

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);

    // - we want to see a line on a line
    for (int i = 0; i < nb_lines; ++i) {
      task.addFeature(s_line[i], s_line_d[i]);
    }

    // - set the gain
    task.setLambda(0.2);

    // -Display task information
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    bool quit = false;
    while (!quit) {
      rs.acquire(I);
      vpDisplay::display(I);

      // Track the lines and update the features
      for (int i = 0; i < nb_lines; ++i) {
        line[i].track(I);
        line[i].display(I, vpColor::red);

        vpFeatureBuilder::create(s_line[i], cam, line[i]);

        s_line[i].display(cam, I, vpColor::red);
        s_line_d[i].display(cam, I, vpColor::green);
      }

      vpColVector v_c = task.computeControlLaw();

      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      vpDisplay::flush(I);
    }

    // Display task information
    task.print();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Visual servo failed with exception: " << e << std::endl;
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
