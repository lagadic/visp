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
  \file servoAfma62DhalfCamVelocity.cpp
  \example servoAfma62DhalfCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame. Visual features are given thanks to four lines
  and are the x and y coordinates of the rectangle center, log(Z/Z*) the
  current depth relative to the desired depth and the thetau rotations.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// Define the object CAD model
// Here we consider 4 black blobs whose centers are located on the corners of a square.
#define L 0.06 // To deal with a 12cm by 12cm square

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

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 100, 100, "Current image");

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpRobotAfma6 robot;
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a line " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    int nbline = 4;
    int nbpoint = 4;

    vpTRACE("sets the desired position of the visual feature ");
    vpPoint pointd[nbpoint]; // position of the fours corners
    vpPoint pointcd;         // position of the center of the square

    pointd[0].setWorldCoordinates(+L, -L, 0);
    pointd[1].setWorldCoordinates(+L, +L, 0);
    pointd[2].setWorldCoordinates(-L, +L, 0);
    pointd[3].setWorldCoordinates(-L, -L, 0);

    // The coordinates in the object frame of the point used as a feature ie
    // the center of the square
    pointcd.setWorldCoordinates(0, 0, 0);

    // The desired homogeneous matrix.
    vpHomogeneousMatrix cd_M_o(0, 0, 0.4, 0, 0, vpMath::rad(10));

    pointd[0].project(cd_M_o);
    pointd[1].project(cd_M_o);
    pointd[2].project(cd_M_o);
    pointd[3].project(cd_M_o);

    pointcd.project(cd_M_o);

    vpFeaturePoint s_pd;
    vpFeatureBuilder::create(s_pd, pointcd);

    // Tracking initialization
    vpMeLine line[nbline];
    vpPoint point[nbpoint];

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(15);
    me.setSampleStep(10);

    // Initialize the tracking. Define the four lines to track
    for (int i = 0; i < nbline; ++i) {
      line[i].setMe(&me);

      line[i].initTracking(I);
      line[i].track(I);
    }

    // Compute the position of the four corners. The goal is to compute the pose
    vpImagePoint ip;
    for (int i = 0; i < nbline; ++i) {
      double x = 0, y = 0;

      if (!vpMeLine::intersection(line[i % nbline], line[(i + 1) % nbline], ip)) {
        return EXIT_FAILURE;
      }

      vpPixelMeterConversion::convertPoint(cam, ip, x, y);

      point[i].set_x(x);
      point[i].set_y(y);
    }

    // Compute the pose c_M_o
    vpPose pose;
    pose.clearPoint();
    vpHomogeneousMatrix c_M_o;

    point[0].setWorldCoordinates(+L, -L, 0);
    point[1].setWorldCoordinates(+L, +L, 0);
    point[2].setWorldCoordinates(-L, +L, 0);
    point[3].setWorldCoordinates(-L, -L, 0);

    for (int i = 0; i < nbline; ++i) {
      pose.addPoint(point[i]); // and added to the pose computation point list
    }

    // Pose by Dementhon or Lagrange provides an initialization of the non linear virtual visual-servoing pose estimation
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, c_M_o);

    // The first features are the position in the camera frame x and y of the square center
    vpPoint pointc; // The current position of the center of the square
    double xc = (point[0].get_x() + point[2].get_x()) / 2;
    double yc = (point[0].get_y() + point[2].get_y()) / 2;
    pointc.set_x(xc);
    pointc.set_y(yc);

    // Sets the current position of the visual feature
    vpFeaturePoint s_p;
    pointc.project(c_M_o);
    vpFeatureBuilder::create(s_p, pointc);

    // The second feature is the depth of the current square center relative
    // to the depth of the desired square center.
    vpFeatureDepth s_logZ;
    s_logZ.build(pointc.get_x(), pointc.get_y(), pointc.get_Z(), log(pointc.get_Z() / pointcd.get_Z()));

    // The last three features are the rotations thetau between the current
    // pose and the desired pose.
    vpHomogeneousMatrix cd_M_c;
    cd_M_c = cd_M_o * c_M_o.inverse();
    vpFeatureThetaU s_tu(vpFeatureThetaU::cdRc);
    s_tu.build(cd_M_c);

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    // - we want to see a point on a point
    task.addFeature(s_p, s_pd);
    task.addFeature(s_logZ);
    task.addFeature(s_tu);

    // - set the gain
    vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
    task.setLambda(lambda);

    // - display task information ");
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    bool quit = false;

    while (!quit) {
      rs.acquire(I);
      vpDisplay::display(I);

      pose.clearPoint();

      // Track the lines and find the current position of the corners
      for (int i = 0; i < nbline; ++i) {
        line[i].track(I);

        line[i].display(I, vpColor::green);

        double x = 0, y = 0;

        if (!vpMeLine::intersection(line[i % nbline], line[(i + 1) % nbline], ip)) {
          return EXIT_FAILURE;
        }

        vpPixelMeterConversion::convertPoint(cam, ip, x, y);

        point[i].set_x(x);
        point[i].set_y(y);

        pose.addPoint(point[i]);
      }

      // Compute the pose
      pose.computePose(vpPose::VIRTUAL_VS, c_M_o);

      // Update the two first features x and y (position of the square center)
      xc = (point[0].get_x() + point[2].get_x()) / 2;
      yc = (point[0].get_y() + point[2].get_y()) / 2;
      pointc.set_x(xc);
      pointc.set_y(yc);
      pointc.project(c_M_o);
      vpFeatureBuilder::create(s_p, pointc);
      // Print the current and the desired position of the center of the
      // square  Print the desired position of the four corners
      s_p.display(cam, I, vpColor::green);
      s_pd.display(cam, I, vpColor::red);
      for (int i = 0; i < nbpoint; ++i) {
        pointd[i].display(I, cam, vpColor::red);
      }

      // Update the second feature
      s_logZ.build(pointc.get_x(), pointc.get_y(), pointc.get_Z(), log(pointc.get_Z() / pointcd.get_Z()));

      // Update the last three features
      cd_M_c = cd_M_o * c_M_o.inverse();
      s_tu.build(cd_M_c);

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
