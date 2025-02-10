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
   \file servoAfma6Cylinder2DCamVelocity.cpp
   \example servoAfma6Cylinder2DCamVelocity.cpp

   Example of eye-in-hand control law. We control here a real robot,
   the Afma6 robot (cartesian robot, with 6 degrees of freedom). The
   velocity is computed in the camera frame. Visual features are the
   two lines corresponding to the edges of a cylinder.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_AFMA6)

#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/vs/vpServo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/**
 * To run this example:
 * - Put a 8 cm diameter gray cylinder on the table
 *   - orientate the cylinder along 11:00 and 17:00 (10 or 15 deg)
 * - Launch: Afma6_move zero
 * - Launch ./servoAfma6Cylinder2DCamVelocity
 *   - initialize image processing on the right line first, then the left line
 */
int main(int argc, char **argv)
{
  bool opt_verbose = false;
  bool opt_adaptive_gain = false;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--verbose") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--adaptive-gain") {
      opt_adaptive_gain = true;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout
        << argv[0]
        << " [--adaptive-gain]"
        << " [--verbose]"
        << " [--help] [-h]"
        << std::endl;;
      return EXIT_SUCCESS;
    }
  }

  vpRobotAfma6 robot;
  vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

  // Load the end-effector to camera frame transformation obtained
  // using a camera intrinsic model with distortion
  robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif

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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 100, 100, "Current image");
#else
    vpDisplay *display = vpDisplayFactory::allocateDisplay(I, 100, 100, "Current image");
#endif
    vpDisplay::display(I);
    vpDisplay::flush(I);

    int nblines = 2;
    std::vector<vpMeLine> line(nblines);

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(15);
    me.setSampleStep(10);

    // Initialize the tracking of the two edges of the cylinder
    for (int i = 0; i < nblines; ++i) {
      line[i].setDisplay(vpMeSite::RANGE_RESULT);
      line[i].setMe(&me);

      line[i].initTracking(I);
      line[i].track(I);
      vpDisplay::flush(I);
    }

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);
    std::cout << "cam:\n" << cam << std::endl;

    // Sets the current position of the visual feature ");
    std::vector<vpFeatureLine> s_line(nblines);
    for (int i = 0; i < nblines; ++i) {
      vpFeatureBuilder::create(s_line[i], cam, line[i]);
    }

    // Sets the desired position of the visual feature ");
    vpCylinder cylinder(0, 1, 0, 0, 0, 0, 0.04);

    vpHomogeneousMatrix c_M_o(0, 0, 0.4, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));

    cylinder.project(c_M_o);

    std::vector<vpFeatureLine> s_line_d(nblines);
    vpFeatureBuilder::create(s_line_d[0], cylinder, vpCylinder::line1);
    vpFeatureBuilder::create(s_line_d[1], cylinder, vpCylinder::line2);

    // {
    //   std::cout << "Desired features: " << std::endl;
    //   std::cout << " - line 1: rho: " << s_line_d[0].getRho() << " theta: " << vpMath::deg(s_line_d[0].getTheta()) << "deg" << std::endl;
    //   std::cout << " - line 2: rho: " << s_line_d[1].getRho() << " theta: " << vpMath::deg(ss_line_d_d[1].getTheta()) << "deg" << std::endl;
    // }

    // Next 2 lines are needed to keep the conventions defined in vpMeLine
    s_line_d[0].setRhoTheta(+fabs(s_line_d[0].getRho()), 0);
    s_line_d[1].setRhoTheta(+fabs(s_line_d[1].getRho()), M_PI);
    // {
    //   std::cout << "Modified desired features: " << std::endl;
    //   std::cout << " - line 1: rho: " << s_line_d[0].getRho() << " theta: " << vpMath::deg(s_line_d[0].getTheta()) << "deg" << std::endl;
    //   std::cout << " - line 2: rho: " << s_s_line_dd[1].getRho() << " theta: " << vpMath::deg(s_line_d[1].getTheta()) << "deg" << std::endl;
    // }

    // Define the task
    vpServo task;
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    // - we want to see a two lines on two lines
    for (int i = 0; i < nblines; ++i) {
      task.addFeature(s_line[i], s_line_d[i]);
    }

    // Set the gain
    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(0.5);
    }

    // Display task information
    task.print();
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    vpColVector v_c(6);
    bool final_quit = false;
    bool send_velocities = false;

    while (!final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);
      vpDisplay::display(I);

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      // Track the two edges and update the features
      for (int i = 0; i < nblines; ++i) {
        line[i].track(I);
        line[i].display(I, vpColor::red);

        vpFeatureBuilder::create(s_line[i], cam, line[i]);
        //std::cout << "line " << i << " rho: " << s[i].getRho() << " theta: " << vpMath::deg(s[i].getTheta()) << " deg" << std::endl;

        s_line[i].display(cam, I, vpColor::red);
        s_line_d[i].display(cam, I, vpColor::green);
      }

      v_c = task.computeControlLaw();

      if (opt_verbose) {
        std::cout << "v: " << v_c.t() << std::endl;
        std::cout << "\t\t || s - s* || = " << task.getError().sumSquare() << std::endl;;
      }

      if (!send_velocities) {
        v_c = 0;
      }

      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);

      ss.str("");
      ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
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
        vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
    task.print();
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "You do not have an afma6 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
