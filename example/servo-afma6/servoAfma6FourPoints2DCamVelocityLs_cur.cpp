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
  \file servoAfma6FourPoints2DCamVelocityLs_cur.cpp
  \example servoAfma6FourPoints2DCamVelocityLs_cur.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  Visual features are the image coordinates of
  4 vpDot2 points. The interaction matrix is computed using the current visual
  features.
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
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// Define the object CAD model
// Here we consider 4 black blobs whose centers are located on the corners of a square.
#define L 0.06 // To deal with a 12cm by 12cm square

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
  Compute the pose \e c_M_o from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot. The pose is computed using a Lowe
  non linear method.

  \param point : 3D coordinates of the points.
  \param dot : 2D coordinates of the points.
  \param cam : Intrinsic camera parameters.
  \param c_M_o : Homogeneous matrix in output describing the transformation
  between the camera and object frame.
  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.
*/
void compute_pose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot, const vpCameraParameters &cam,
                  vpHomogeneousMatrix &c_M_o, bool init)
{
  vpRotationMatrix c_R_o;
  vpPose pose;
  vpImagePoint cog;

  for (size_t i = 0; i < point.size(); ++i) {
    double x = 0, y = 0;
    cog = dot[i].getCog();
    vpPixelMeterConversion::convertPoint(cam, cog, x, y); // Pixel to meter conversion
    point[i].set_x(x);                                    // Perspective projection
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, c_M_o);
  }
  else { // init = false; use of the previous pose to initialise VIRTUAL_VS
    pose.computePose(vpPose::VIRTUAL_VS, c_M_o);
  }
}

int main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed camera velocities (m/s, rad/s) to achieve the task
  // - the 6 measured joint velocities (m/s, rad/s)
  // - the 6 measured joint positions (m, rad)
  // - the 8 values of s - s*
  // - the 6 values of the pose c_M_o (tx,ty,tz, rx,ry,rz) with translation
  //   in meters and rotations in radians

  // Get the user login name
  std::string username = vpIoTools::getUserName();

  // Create a log filename to save velocities...
  std::string logdirname = "/tmp/" + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(logdirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(logdirname);
    }
    catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << logdirname << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::string logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

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

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Use of the Afma6 robot " << std::endl;
    std::cout << " Interaction matrix computed with the current features " << std::endl;
    std::cout << " task : servo 4 points on a square with dimension " << L << " meters" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    std::vector<vpDot2> dot(4);

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;
    for (size_t i = 0; i < dot.size(); ++i) {
      dot[i].initTracking(I);
      vpImagePoint cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    vpRobotAfma6 robot;
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    // Sets the current position of the visual feature
    std::vector<vpFeaturePoint> s(4);
    for (size_t i = 0; i < s.size(); ++i) {
      vpFeatureBuilder::create(s[i], cam, dot[i]); // retrieve x,y  of the vpFeaturePoint structure
    }

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates(+L, -L, 0);
    point[2].setWorldCoordinates(+L, +L, 0);
    point[3].setWorldCoordinates(-L, +L, 0);

    // Initialise a desired pose to compute s*, the desired 2D point features
    vpHomogeneousMatrix c_M_o;
    vpTranslationVector c_t_o(0, 0, 0.5);   // tz = 0.5 meter
    vpRxyzVector c_r_o(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); // No rotations
    vpRotationMatrix c_R_o(c_r_o);          // Build the rotation matrix
    c_M_o.build(c_t_o, c_R_o);                // Build the homogeneous matrix

    // Sets the desired position of the 2D visual feature
    std::vector<vpFeaturePoint> s_d(4);
    // Compute the desired position of the features from the desired pose
    for (size_t i = 0; i < s_d.size(); ++i) {
      vpColVector cP, p;
      point[i].changeFrame(c_M_o, cP);
      point[i].projection(cP, p);

      s_d[i].set_x(p[0]);
      s_d[i].set_y(p[1]);
      s_d[i].set_Z(cP[2]);
    }

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    // - Interaction matrix is computed with the current visual features
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    // We want to see a point on a point
    for (size_t i = 0; i < s.size(); ++i) {
      task.addFeature(s[i], s_d[i]);
    }

    // Set the proportional gain
    task.setLambda(0.3);

    // Display task information
    task.print();

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;

    bool init_pose_from_linear_method = true;

    bool quit = false;
    while (!quit) {
      // Acquire a new image from the camera
      rs.acquire(I);

      // Display this image
      vpDisplay::display(I);

      // For each point...
      for (size_t i = 0; i < dot.size(); ++i) {
        // Achieve the tracking of the dot in the image
        dot[i].track(I);
      }

      // At first iteration, we initialise non linear pose estimation with a linear approach.
      // For the other iterations, non linear pose estimation is initialized with the pose estimated at previous iteration of the loop
      compute_pose(point, dot, cam, c_M_o, init_pose_from_linear_method);
      if (init_pose_from_linear_method) {
        init_pose_from_linear_method = false;
      }

      for (size_t i = 0; i < dot.size(); ++i) {
        // Update the point feature from the dot location
        vpFeatureBuilder::create(s[i], cam, dot[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(c_M_o, cP);

        s[i].set_Z(cP[2]);
      }

      // Compute the visual servoing skew vector
      vpColVector v = task.computeControlLaw();

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed camera velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to camera translation velocities in m/s
      // v[3], v[4], v[5] correspond to camera rotation velocities in rad/s
      flog << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5] << " ";

      // Get the measured joint velocities of the robot
      vpColVector qvel;
      robot.getVelocity(vpRobot::ARTICULAR_FRAME, qvel);
      // Save measured joint velocities of the robot in the log file:
      // - qvel[0], qvel[1], qvel[2] correspond to measured joint translation
      //   velocities in m/s
      // - qvel[3], qvel[4], qvel[5] correspond to measured joint rotation
      //   velocities in rad/s
      flog << qvel[0] << " " << qvel[1] << " " << qvel[2] << " " << qvel[3] << " " << qvel[4] << " " << qvel[5] << " ";

      // Get the measured joint positions of the robot
      vpColVector q;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
      // Save measured joint positions of the robot in the log file
      // - q[0], q[1], q[2] correspond to measured joint translation
      //   positions in m
      // - q[3], q[4], q[5] correspond to measured joint rotation
      //   positions in rad
      flog << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " ";

      // Save feature error (s-s*) for the 4 feature points. For each feature
      // point, we have 2 errors (along x and y axis).  This error is
      // expressed in meters in the camera frame
      flog << (task.getError()).t() << " "; // s-s* for points

      // Save the current c_M_o pose: translations in meters, rotations (rx, ry,
      // rz) in radians
      flog << c_t_o[0] << " " << c_t_o[1] << " " << c_t_o[2] << " "     // translation
        << c_r_o[0] << " " << c_r_o[1] << " " << c_r_o[2] << std::endl; // rot

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      // Flush the display
      vpDisplay::flush(I);
    }

    // Close the log file
    flog.close();

    // Display task information
    task.print();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    // Close the log file
    flog.close();

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
