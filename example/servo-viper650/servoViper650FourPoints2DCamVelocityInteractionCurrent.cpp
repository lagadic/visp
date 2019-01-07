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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoViper650FourPoints2DCamVelocityInteractionCurrent.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Viper S650 robot (arm with 6 degrees of freedom). The velocity is
  computed in the camera frame. The inverse jacobian that converts cartesian
  velocities in joint velocities is implemented in the robot low level
  controller. Visual features are the image coordinates of 4 points. The
  target is made of 4 dots arranged as a 10cm by 10cm square.

  The device used to acquire images is a firewire camera (PointGrey Flea2)

  Camera extrinsic (eMc) and intrinsic parameters are retrieved from the robot
  low level driver that is not public.

*/

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_VIPER650) && defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_X11)

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotViper650.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define L 0.05 // to deal with a 10cm by 10cm square

/*!

  Compute the pose \e cMo from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot.

  \param point : 3D coordinates of the points.

  \param dot : 2D coordinates of the points.

  \param cam : Intrinsic camera parameters.

  \param cMo : Homogeneous matrix in output describing the transformation
  between the camera and object frame.

  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.

*/
void compute_pose(std::vector<vpPoint> &point, std::vector<vpDot2> &dot, vpCameraParameters cam,
                  vpHomogeneousMatrix &cMo, bool init)
{
  vpHomogeneousMatrix cMo_dementhon; // computed pose with dementhon method
  vpHomogeneousMatrix cMo_lagrange;  // computed pose with lagrange method
  vpPose pose;

  for (size_t i = 0; i < point.size(); i++) {

    double x = 0, y = 0;
    vpImagePoint cog = dot[i].getCog();
    vpPixelMeterConversion::convertPoint(cam, cog, x,
                                         y); // pixel to meter conversion
    point[i].set_x(x);                       // projection perspective          p
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
    // Compute and return the residual expressed in meter for the pose matrix
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);

    // Select the best pose to initialize the lowe pose computation
    if (residual_lagrange < residual_dementhon)
      cMo = cMo_lagrange;
    else
      cMo = cMo_dementhon;
  }

  pose.computePose(vpPose::LOWE, cMo);
}

int main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed camera velocities (m/s, rad/s) to achieve the task
  // - the 6 mesured joint velocities (m/s, rad/s)
  // - the 6 mesured joint positions (m, rad)
  // - the 8 values of s - s*
  std::string username;
  // Get the user login name
  vpIoTools::getUserName(username);

  // Create a log filename to save velocities...
  std::string logdirname;
  logdirname = "/tmp/" + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(logdirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(logdirname);
    } catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << logdirname << std::endl;
      return (-1);
    }
  }
  std::string logfilename;
  logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

  try {
    vpRobotViper650 robot;
    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;
    robot.init(vpRobotViper650::TOOL_PTGREY_FLEA2_CAMERA, projModel);
    vpHomogeneousMatrix eMc;
    robot.get_eMc(eMc);
    std::cout << "Camera extrinsic parameters (eMc): \n" << eMc << std::endl;

    vpServo task;

    vpImage<unsigned char> I;

    bool reset = false;
    vp1394TwoGrabber g(reset);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

    g.acquire(I);

    vpDisplayX display(I, 100, 100, "Current image");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::vector<vpDot2> dot(4);

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;

    for (size_t i = 0; i < dot.size(); i++) {
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      vpImagePoint cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    vpCameraParameters cam;

    // Update camera parameters
    robot.getCameraParameters(cam, I);
    std::cout << "Camera intrinsic parameters: \n" << cam << std::endl;

    // Sets the current position of the visual feature
    vpFeaturePoint p[4];
    for (size_t i = 0; i < dot.size(); i++)
      vpFeatureBuilder::create(p[i], cam, dot[i]); // retrieve x,y  of the vpFeaturePoint structure

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates(L, -L, 0);
    point[2].setWorldCoordinates(L, L, 0);
    point[3].setWorldCoordinates(-L, L, 0);

    // Compute target initial pose
    vpHomogeneousMatrix cMo;
    compute_pose(point, dot, cam, cMo, true);
    std::cout << "Initial camera pose (cMo): \n" << cMo << std::endl;

    // Initialise a desired pose to compute s*, the desired 2D point features
    vpHomogeneousMatrix cMo_d(vpTranslationVector(0, 0, 0.5), // tz = 0.5 meter
                              vpRotationMatrix());            // no rotation

    // Sets the desired position of the 2D visual feature
    vpFeaturePoint pd[4];
    // Compute the desired position of the features from the desired pose
    for (int i = 0; i < 4; i++) {
      vpColVector cP, p;
      point[i].changeFrame(cMo_d, cP);
      point[i].projection(cP, p);

      pd[i].set_x(p[0]);
      pd[i].set_y(p[1]);
      pd[i].set_Z(cP[2]);
    }

    // We want to see a point on a point
    for (size_t i = 0; i < dot.size(); i++)
      task.addFeature(p[i], pd[i]);

    // Set the proportional gain
    task.setLambda(0.3);

    // Define the task
    // - we want an eye-in-hand control law
    // - camera velocities are computed
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.print();

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C or click in the image to stop the loop...\n" << std::flush;
    for (;;) {
      // Acquire a new image from the camera
      g.acquire(I);

      // Display this image
      vpDisplay::display(I);

      try {
        // For each point...
        for (size_t i = 0; i < dot.size(); i++) {
          // Achieve the tracking of the dot in the image
          dot[i].track(I);
          // Display a green cross at the center of gravity position in the
          // image
          vpImagePoint cog = dot[i].getCog();
          vpDisplay::displayCross(I, cog, 10, vpColor::green);
        }
      } catch (...) {
        std::cout << "Error detected while tracking visual features.." << std::endl;
        break;
      }

      // During the servo, we compute the pose using a non linear method. For
      // the initial pose used in the non linear minimisation we use the pose
      // computed at the previous iteration.
      compute_pose(point, dot, cam, cMo, false);

      for (size_t i = 0; i < dot.size(); i++) {
        // Update the point feature from the dot location
        vpFeatureBuilder::create(p[i], cam, dot[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP);

        p[i].set_Z(cP[2]);
      }

      // Compute the visual servoing skew vector
      vpColVector v = task.computeControlLaw();

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
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
      flog << task.getError() << std::endl;

      vpDisplay::displayText(I, 10, 10, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false))
        break;

      // Flush the display
      vpDisplay::flush(I);

      // std::cout << "\t\t || s - s* || = " << ( task.getError()
      // ).sumSquare() << std::endl;
    }

    robot.stopMotion();

    std::cout << "Display task information: " << std::endl;
    task.print();
    task.kill();
    flog.close(); // Close the log file
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    flog.close(); // Close the log file
    std::cout << "Catched an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an Viper 650 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
