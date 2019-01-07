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
 *   velocity computed in the articular frame
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoViper850FourPoints2DArtVelocityInteractionDesired.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Viper S850 robot (arm with 6 degrees of freedom). The velocities resulting
  from visual servo are here joint velocities. Visual features are the image
  coordinates of 4 points. The target is made of 4 dots arranged as a 10cm by
  10cm square.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#if (defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394))

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed joint velocities (m/s, rad/s) to achieve the task
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
// Define the square CAD model
// Square dimention
//#define L 0.075
#define L 0.05
// Distance between the camera and the square at the desired
// position after visual servoing convergence
#define D 0.5

    vpRobotViper850 robot;
    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;
    robot.init(vpRobotViper850::TOOL_PTGREY_FLEA2_CAMERA, projModel);

    vpServo task;

    vpImage<unsigned char> I;
    int i;

    bool reset = false;
    vp1394TwoGrabber g(reset);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

    g.acquire(I);

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the joint space" << std::endl;
    std::cout << " Use of the Afma6 robot " << std::endl;
    std::cout << " task : servo 4 points on a square with dimention " << L << " meters" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    vpDot dot[4];
    vpImagePoint cog;

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;

    for (i = 0; i < 4; i++) {
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    vpCameraParameters cam;

    // Update camera parameters
    robot.getCameraParameters(cam, I);

    cam.printParameters();

    // Sets the current position of the visual feature
    vpFeaturePoint p[4];
    for (i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], cam, dot[i]); // retrieve x,y and Z of the vpPoint structure

    // sets the desired position of the visual feature
    vpFeaturePoint pd[4];

    pd[0].buildFrom(-L, -L, D);
    pd[1].buildFrom(L, -L, D);
    pd[2].buildFrom(L, L, D);
    pd[3].buildFrom(-L, L, D);

    // We want to see a point on a point
    std::cout << std::endl;
    for (i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    // Set the proportional gain
    task.setLambda(0.4);

    // Display task information
    task.print();

    // Define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.print();

    vpVelocityTwistMatrix cVe;
    robot.get_cVe(cVe);
    task.set_cVe(cVe);
    task.print();

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);
    task.print();

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    for (;;) {
      // Acquire a new image from the camera
      g.acquire(I);

      // Display this image
      vpDisplay::display(I);

      try {
        // For each point...
        for (i = 0; i < 4; i++) {
          // Achieve the tracking of the dot in the image
          dot[i].track(I);
          // Display a green cross at the center of gravity position in the
          // image
          cog = dot[i].getCog();
          vpDisplay::displayCross(I, cog, 10, vpColor::green);
        }
      } catch (...) {
        flog.close(); // Close the log file
        vpTRACE("Error detected while tracking visual features");
        robot.stopMotion();
        exit(1);
      }

      // Update the point feature from the dot location
      for (i = 0; i < 4; i++)
        vpFeatureBuilder::create(p[i], cam, dot[i]);

      // Get the jacobian of the robot
      robot.get_eJe(eJe);
      // Update this jacobian in the task structure. It will be used to
      // compute the velocity skew (as an articular velocity) qdot = -lambda *
      // L^+ * cVe * eJe * (s-s*)
      task.set_eJe(eJe);

      vpColVector v;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw();

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to joint translation velocities in m/s
      // v[3], v[4], v[5] correspond to joint rotation velocities in rad/s
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
      flog << (task.getError()).t() << std::endl;

      // Flush the display
      vpDisplay::flush(I);

      // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() <<
      // std::endl;
    }

    std::cout << "Display task information: " << std::endl;
    task.print();
    task.kill();
    flog.close(); // Close the log file
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    flog.close(); // Close the log file
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an Viper 850 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
