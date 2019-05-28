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
 * Nicolas Melchior
 *
 *****************************************************************************/
/*!
  \file servoAfma62DhalfCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame. Visual features are given thanks to four lines
  and are the x and y coordinates of the rectangle center, log(Z/Z*) the
  current depth relative to the desired depth and the thetau rotations.

*/

/*!
  \example servoAfma62DhalfCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. Visual features are given thanks to four lines and are
  the x and y coordinates of the rectangle center, log(Z/Z*) the current depth
  relative to the desired depth and the thetau rotations.

*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpMath.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpGenericFeature.h>
#include <visp3/vs/vpServo.h>

#include <visp3/robot/vpRobotAfma6.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoint.h>

int main()
{
  try {
    vpImage<unsigned char> I;

    vp1394TwoGrabber g;
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

    vpServo task;

    vpRobotAfma6 robot;
    // robot.move("zero.pos") ;

    vpCameraParameters cam;
    // Update camera parameters
    robot.getCameraParameters(cam, I);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a line " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    int nbline = 4;
    int nbpoint = 4;

    vpTRACE("sets the desired position of the visual feature ");
    vpPoint pointd[nbpoint]; // position of the fours corners
    vpPoint pointcd;         // position of the center of the square
    vpFeaturePoint pd;

    double L = 0.05;
    pointd[0].setWorldCoordinates(L, -L, 0);
    pointd[1].setWorldCoordinates(L, L, 0);
    pointd[2].setWorldCoordinates(-L, L, 0);
    pointd[3].setWorldCoordinates(-L, -L, 0);

    // The coordinates in the object frame of the point used as a feature ie
    // the center of the square
    pointcd.setWorldCoordinates(0, 0, 0);

    // The desired homogeneous matrix.
    vpHomogeneousMatrix cMod(0, 0, 0.4, 0, 0, vpMath::rad(10));

    pointd[0].project(cMod);
    pointd[1].project(cMod);
    pointd[2].project(cMod);
    pointd[3].project(cMod);

    pointcd.project(cMod);

    vpFeatureBuilder::create(pd, pointcd);

    vpTRACE("Initialization of the tracking");
    vpMeLine line[nbline];
    vpPoint point[nbpoint];
    int i;

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setThreshold(50000);
    me.setSampleStep(10);

    // Initialize the tracking. Define the four lines to track
    for (i = 0; i < nbline; i++) {
      line[i].setMe(&me);

      line[i].initTracking(I);
      line[i].track(I);
    }

    // Compute the position of the four corners. The goal is to
    // compute the pose
    vpImagePoint ip;
    for (i = 0; i < nbline; i++) {
      double x = 0, y = 0;

      if (!vpMeLine::intersection(line[i % nbline], line[(i + 1) % nbline], ip)) {
        exit(-1);
      }

      vpPixelMeterConversion::convertPoint(cam, ip, x, y);

      point[i].set_x(x);
      point[i].set_y(y);
    }

    // Compute the pose cMo
    vpPose pose;
    pose.clearPoint();
    vpHomogeneousMatrix cMo;

    point[0].setWorldCoordinates(L, -L, 0);
    point[1].setWorldCoordinates(L, L, 0);
    point[2].setWorldCoordinates(-L, L, 0);
    point[3].setWorldCoordinates(-L, -L, 0);

    for (i = 0; i < nbline; i++) {
      pose.addPoint(point[i]); // and added to the pose computation point list
    }

    pose.computePose(vpPose::LAGRANGE, cMo);
    pose.computePose(vpPose::VIRTUAL_VS, cMo);

    vpTRACE("sets the current position of the visual feature ");

    // The first features are the position in the camera frame x and y of the
    // square center
    vpPoint pointc; // The current position of the center of the square
    double xc = (point[0].get_x() + point[2].get_x()) / 2;
    double yc = (point[0].get_y() + point[2].get_y()) / 2;
    pointc.set_x(xc);
    pointc.set_y(yc);
    vpFeaturePoint p;
    pointc.project(cMo);
    vpFeatureBuilder::create(p, pointc);

    // The second feature is the depth of the current square center relative
    // to the depth of the desired square center.
    vpFeatureDepth logZ;
    logZ.buildFrom(pointc.get_x(), pointc.get_y(), pointc.get_Z(), log(pointc.get_Z() / pointcd.get_Z()));

    // The last three features are the rotations thetau between the current
    // pose and the desired pose.
    vpHomogeneousMatrix cdMc;
    cdMc = cMod * cMo.inverse();
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    tu.buildFrom(cdMc);

    vpTRACE("define the task");
    vpTRACE("\t we want an eye-in-hand control law");
    vpTRACE("\t robot is controlled in the camera frame");
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    vpTRACE("\t we want to see a point on a point..");
    std::cout << std::endl;
    task.addFeature(p, pd);
    task.addFeature(logZ);
    task.addFeature(tu);

    vpTRACE("\t set the gain");
    task.setLambda(0.2);

    vpTRACE("Display task information ");
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    unsigned int iter = 0;
    vpTRACE("\t loop");
    vpColVector v;
    vpImage<vpRGBa> Ic;
    double lambda_av = 0.05;
    double alpha = 0.05;
    double beta = 3;

    for (;;) {
      std::cout << "---------------------------------------------" << iter << std::endl;

      try {
        g.acquire(I);
        vpDisplay::display(I);

        pose.clearPoint();

        // Track the lines and find the current position of the corners
        for (i = 0; i < nbline; i++) {
          line[i].track(I);

          line[i].display(I, vpColor::green);

          double x = 0, y = 0;

          if (!vpMeLine::intersection(line[i % nbline], line[(i + 1) % nbline], ip)) {
            exit(-1);
          }

          vpPixelMeterConversion::convertPoint(cam, ip, x, y);

          point[i].set_x(x);
          point[i].set_y(y);

          pose.addPoint(point[i]);
        }

        // Compute the pose
        pose.computePose(vpPose::VIRTUAL_VS, cMo);

        // Update the two first features x and y (position of the square
        // center)
        xc = (point[0].get_x() + point[2].get_x()) / 2;
        yc = (point[0].get_y() + point[2].get_y()) / 2;
        pointc.set_x(xc);
        pointc.set_y(yc);
        pointc.project(cMo);
        vpFeatureBuilder::create(p, pointc);
        // Print the current and the desired position of the center of the
        // square  Print the desired position of the four corners
        p.display(cam, I, vpColor::green);
        pd.display(cam, I, vpColor::red);
        for (i = 0; i < nbpoint; i++)
          pointd[i].display(I, cam, vpColor::red);

        // Update the second feature
        logZ.buildFrom(pointc.get_x(), pointc.get_y(), pointc.get_Z(), log(pointc.get_Z() / pointcd.get_Z()));

        // Update the last three features
        cdMc = cMod * cMo.inverse();
        tu.buildFrom(cdMc);

        // Adaptive gain
        double gain;
        {
          if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
            gain = lambda_av;
          else {
            gain = alpha * exp(-beta * (task.getError()).sumSquare()) + lambda_av;
          }
        }

        task.setLambda(gain);

        v = task.computeControlLaw();

        vpDisplay::flush(I);
        std::cout << v.sumSquare() << std::endl;
        if (iter == 0)
          vpDisplay::getClick(I);
        if (v.sumSquare() > 0.5) {
          v = 0;
          robot.setVelocity(vpRobot::CAMERA_FRAME, v);
          robot.stopMotion();
          vpDisplay::getClick(I);
        }

        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      } catch (...) {
        v = 0;
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        robot.stopMotion();
        exit(1);
      }

      vpTRACE("\t\t || s - s* || = %f ", (task.getError()).sumSquare());
      iter++;
    }

    vpTRACE("Display task information ");
    task.print();
    task.kill();
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
