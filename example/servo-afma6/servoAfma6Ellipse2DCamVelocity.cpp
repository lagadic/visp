/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
  \file servoAfma6Ellipse2DCamVelocity.cpp
  \example servoAfma6Ellipse2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame. The used visual feature is a circle.
*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_REALSENSE2))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vpRealSense2.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>
#include <visp3/vs/vpServo.h>

#include <visp3/robot/vpRobotAfma6.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp3/blob/vpDot.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    vpServo task;

    vpImage<unsigned char> I;
    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480, fps = 60;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);

    // Warm up camera
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I, 100, 100, "Current image");
#else
    display = vpDisplayFactory::allocateDisplay(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a point " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    vpDot dot;

    dot.setMaxDotSize(0.30); // Max dot size is 30 % of the image size
    // dot.setGraphics(true) ;
    dot.setComputeMoments(true);
    std::cout << "Click on an ellipse..." << std::endl;
    dot.initTracking(I);
    vpImagePoint cog = dot.getCog();
    vpDisplay::displayCross(I, cog, 10, vpColor::blue);
    vpDisplay::flush(I);

    dot.track(I);

    vpRobotAfma6 robot;
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, vpCameraParameters::perspectiveProjWithoutDistortion);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    vpTRACE("sets the current position of the visual feature ");
    vpFeatureEllipse c;
    vpFeatureBuilder::create(c, cam, dot);

    std::cout << " Learning 0/1 " << std::endl;
    int learning;
    std::cin >> learning;
    std::string name = "dat/ellipse.dat";
    if (learning == 1) {
      // save the object position
      vpTRACE("Save the location of the object in a file dat/ellipse.dat");
      std::ofstream f(name.c_str());
      f << c.get_s().t();
      f.close();
      exit(1);
    }

    vpTRACE("sets the desired position of the visual feature ");
    vpFeatureEllipse cd;
    std::ifstream f("dat/ellipse.dat");
    double x, y, n20, n11, n02;
    f >> x;
    f >> y;
    f >> n20;
    f >> n11;
    f >> n02;
    f.close();
    cd.buildFrom(x, y, n20, n11, n02);
    cd.setABC(0, 0, 10);

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    task.addFeature(c, cd);

    task.setLambda(0.01);

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    unsigned int iter = 0;
    double lambda_av = 0.01;
    double alpha = 0.1; // 1 ;
    double beta = 3;    // 3 ;

    std::cout << "alpha 0.7" << std::endl;
    std::cin >> alpha;
    std::cout << "beta 5" << std::endl;
    std::cin >> beta;
    bool quit = false;
    while (!quit) {
      std::cout << "---------------------------------------------" << iter++ << std::endl;

      rs.acquire(I);
      vpDisplay::display(I);

      dot.track(I);

      // Get the dot cog
      cog = dot.getCog();

      vpDisplay::displayCross(I, cog, 10, vpColor::green);

      vpFeatureBuilder::create(c, cam, dot);
      // Compute the adaptative gain (speed up the convergence)
      double gain;
      if (iter > 2) {
        if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
          gain = lambda_av;
        else {
          gain = alpha * exp(-beta * (task.getError()).sumSquare()) + lambda_av;
        }
      }
      else
        gain = lambda_av;

      vpTRACE("%f %f", (task.getError()).sumSquare(), gain);
      task.setLambda(gain);
      vpColVector v;
      v = task.computeControlLaw();
      std::cout << "rank " << task.getTaskRank() << std::endl;
      vpServoDisplay::display(task, cam, I);
      std::cout << v.t();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }
      vpDisplay::flush(I);
      vpTRACE("\t\t || s - s* || = %f ", (task.getError()).sumSquare());
    }

    vpTRACE("Display task information ");
    task.print();
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
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
