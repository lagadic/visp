/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 *
*****************************************************************************/

/*!
  \file servoAfma6Line2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  The visual feature is a line.
*/

/*!
  \example servoAfma6Line2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  The visual feature is a line.
*/

#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/vs/vpServo.h>

#include <visp3/robot/vpRobotAfma6.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    vpImage<unsigned char> I;

    vpRealSense2 rs;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    rs.open(config);

    // Warm up camera
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpServo task;

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a line " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    vpMeLine line;

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(50);
    me.setSampleStep(10);
    line.setDisplay(vpMeSite::RANGE_RESULT);

    line.setMe(&me);

    // Initialize the tracking. Define the line to track.
    line.initTracking(I);
    line.track(I);

    vpRobotAfma6 robot;
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, vpCameraParameters::perspectiveProjWithoutDistortion);
    // robot.move("pos-init.pos");

    vpCameraParameters cam;
    // Update camera parameters
    robot.getCameraParameters(cam, I);

    vpTRACE("sets the current position of the visual feature ");
    vpFeatureLine p;
    vpFeatureBuilder::create(p, cam, line);

    vpTRACE("sets the desired position of the visual feature ");
    vpLine lined;
    lined.setWorldCoordinates(1, 0, 0, 0, 0, 0, 1, 0);
    vpHomogeneousMatrix cMo(0, 0, 0.3, 0, 0, vpMath::rad(0));
    lined.project(cMo);
    lined.setRho(-fabs(lined.getRho()));
    lined.setTheta(0);

    vpFeatureLine pd;
    vpFeatureBuilder::create(pd, lined);

    vpTRACE("define the task");
    vpTRACE("\t we want an eye-in-hand control law");
    vpTRACE("\t robot is controlled in the camera frame");
    task.setServo(vpServo::EYEINHAND_CAMERA);

    vpTRACE("\t we want to see a point on a point..");
    std::cout << std::endl;
    task.addFeature(p, pd);

    vpTRACE("\t set the gain");
    task.setLambda(0.2);

    vpTRACE("Display task information ");
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    unsigned int iter = 0;
    vpTRACE("\t loop");
    vpColVector v;
    bool quit = false;
    while (!quit) {
      std::cout << "---------------------------------------------" << iter << std::endl;

      try {
        rs.acquire(I);
        vpDisplay::display(I);

        // Track the line
        line.track(I);
        line.display(I, vpColor::red);

        // Update the current line feature
        vpFeatureBuilder::create(p, cam, line);

        // displqy the current and the desired features
        p.display(cam, I, vpColor::red);
        pd.display(cam, I, vpColor::green);

        v = task.computeControlLaw();

        if (iter == 0) {
          vpDisplay::getClick(I);
        }
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
        if (vpDisplay::getClick(I, false)) {
          quit = true;
        }

        vpDisplay::flush(I);
      }
      catch (...) {
        v = 0;
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        robot.stopMotion();
        return EXIT_FAILURE;
      }

      vpTRACE("\t\t || s - s* || = %f ", (task.getError()).sumSquare());
      iter++;
    }

    vpTRACE("Display task information ");
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
