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
 *   eye-to-hand control
 *   velocity computed in the camera frame
 */

/*!
  \file servoAfma6Points2DCamVelocityEyeToHand.cpp
  \example servoAfma6Points2DCamVelocityEyeToHand.cpp

  \brief Example of a eye-to-hand control law. We control here a real robot,
  the Afma6 robot (cartesian robot, with 6 degrees of freedom). The robot is
  controlled in the camera frame.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define SAVE 0
#define L 0.006
#define D 0

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    std::string username = vpIoTools::getUserName();
    std::string logdirname = "/tmp/" + username;
    if (SAVE) {
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
    }

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

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 100, 100, "Current image");

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-to-hand task control" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a point " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    int nbPoint = 7;

    vpDot dot[nbPoint];
    vpImagePoint cog;

    for (int i = 0; i < nbPoint; ++i) {
      dot[i].initTracking(I);
      dot[i].setGraphics(true);
      dot[i].track(I);
      vpDisplay::flush(I);
      dot[i].setGraphics(false);
    }

    // Compute the pose 3D model
    vpPoint point[nbPoint];
    point[0].setWorldCoordinates(-2 * L, D, -3 * L);
    point[1].setWorldCoordinates(0, D, -3 * L);
    point[2].setWorldCoordinates(2 * L, D, -3 * L);

    point[3].setWorldCoordinates(-L, D, -L);
    point[4].setWorldCoordinates(L, D, -L);
    point[5].setWorldCoordinates(L, D, L);
    point[6].setWorldCoordinates(-L, D, L);

    vpRobotAfma6 robot;
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, vpCameraParameters::perspectiveProjWithoutDistortion);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    vpHomogeneousMatrix c_M_o, cd_M_o;
    vpPose pose;
    pose.clearPoint();
    for (int i = 0; i < nbPoint; ++i) {
      cog = dot[i].getCog();
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(cam, cog, x, y);
      point[i].set_x(x);
      point[i].set_y(y);
      pose.addPoint(point[i]);
    }

    // compute the initial pose using Dementhon method followed by a non
    // linear minimization method
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, c_M_o);

    std::cout << "c_M_o: \n" << c_M_o << std::endl;

    /*
     *  Learning or reading the desired position
     */
    std::cout << "Learning (0/1)? " << std::endl;
    std::string filename = "cd_M_o.dat";
    int learning;
    std::cin >> learning;
    if (learning == 1) {
      // save the object position
      std::cout << "Save the location of the object cMo in " << filename << std::endl;
      c_M_o.save(filename);
      return EXIT_SUCCESS;
    }

    std::cout << "Loading desired location of the object cMo from " << filename << std::endl;
    cd_M_o.load(filename);

    vpFeaturePoint s[nbPoint], s_d[nbPoint];

    // Set the desired position of the point by forward projection using the pose cd_M_o
    for (int i = 0; i < nbPoint; ++i) {
      vpColVector cP, p;
      point[i].changeFrame(cd_M_o, cP);
      point[i].projection(cP, p);

      s_d[i].set_x(p[0]);
      s_d[i].set_y(p[1]);
    }

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    vpServo task;
    task.setServo(vpServo::EYETOHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT);

    // - we want to see a point on a point
    for (int i = 0; i < nbPoint; ++i) {
      task.addFeature(s[i], s_d[i]);
    }

    // - display task information
    task.print();

    double convergence_threshold = 0.00; // 025 ;

    double error = 1;
    unsigned int iter = 0;
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // position of the object in the effector frame
    vpHomogeneousMatrix o_M_camrobot;
    o_M_camrobot[0][3] = -0.05;

    int it = 0;

    std::list<vpImagePoint> Lcog;
    bool quit = false;
    while ((error > convergence_threshold) && (!quit)) {
      std::cout << "---------------------------------------------" << iter++ << std::endl;

      rs.acquire(I);
      vpDisplay::display(I);

      try {
        for (int i = 0; i < nbPoint; ++i) {
          dot[i].track(I);
          Lcog.push_back(dot[i].getCog());
        }
      }
      catch (...) {
        std::cout << "Error detected while tracking visual features" << std::endl;
        robot.stopMotion();
        return EXIT_FAILURE;
      }

      // compute the initial pose using  a non linear minimization method
      pose.clearPoint();

      for (int i = 0; i < nbPoint; ++i) {
        double x = 0, y = 0;
        cog = dot[i].getCog();
        vpPixelMeterConversion::convertPoint(cam, cog, x, y);
        point[i].set_x(x);
        point[i].set_y(y);

        vpColVector cP;
        point[i].changeFrame(cd_M_o, cP);

        s[i].set_x(x);
        s[i].set_y(y);
        s[i].set_Z(cP[2]);

        pose.addPoint(point[i]);

        point[i].display(I, c_M_o, cam, vpColor::green);
        point[i].display(I, cd_M_o, cam, vpColor::blue);
      }
      pose.computePose(vpPose::LOWE, c_M_o);

      // - set the camera to end-effector velocity twist matrix transformation
      vpHomogeneousMatrix c_M_e, camrobot_M_e;
      robot.get_cMe(camrobot_M_e);
      c_M_e = c_M_o * o_M_camrobot * camrobot_M_e;

      task.set_cVe(c_M_e);

      // - set the Jacobian (expressed in the end-effector frame)
      vpMatrix e_J_e;
      robot.get_eJe(e_J_e);
      task.set_eJe(e_J_e);

      // - set the task adaptive gain
      vpAdaptiveGain lambda_adaptive;
      lambda_adaptive.initStandard(1.7, 0.3, 1.5); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda_adaptive);

      vpColVector qdot = task.computeControlLaw();

      // Display points trajectory
      for (std::list<vpImagePoint>::const_iterator it_cog = Lcog.begin(); it_cog != Lcog.end(); ++it_cog) {
        vpDisplay::displayPoint(I, *it_cog, vpColor::red);
      }

      // Display task visual features feature
      vpServoDisplay::display(task, cam, I);

      // Apply joint velocity to the robot
      robot.setVelocity(vpRobot::JOINT_STATE, qdot);

      error = (task.getError()).sumSquare();
      std::cout << "|| s - s* || = " << error << std::endl;

      if (error > 7) {
        std::cout << "Error detected while tracking visual features" << std::endl;
        robot.stopMotion();
        return EXIT_FAILURE;
      }

      if ((SAVE == 1) && (iter % 3 == 0)) {
        vpImage<vpRGBa> Ic;
        vpDisplay::getImage(I, Ic);
        std::string filename = vpIoTools::formatString(logdirname + "/image.%04d.png", it++);
        vpImageIo::write(Ic, filename);
      }

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      vpDisplay::flush(I);
    }

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
