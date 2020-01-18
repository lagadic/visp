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
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file servoAfma6Ellipse2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame. The used visual feature is a circle.

*/

/*!
  \example servoAfma6Ellipse2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. The used visual feature is a circle.

*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

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
  try {
    vpServo task;

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

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a point " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

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

    vpCameraParameters cam;

    vpRobotAfma6 robot;

    // Update camera parameters
    robot.getCameraParameters(cam, I);

    vpTRACE("sets the current position of the visual feature ");
    vpFeatureEllipse c;
    vpFeatureBuilder::create(c, cam, dot);

    std::cout << " Learning 0/1 " << std::endl;
    int learning;
    std::cin >> learning;
    char name[FILENAME_MAX];
    sprintf(name, "dat/ellipse.dat");
    if (learning == 1) {
      // save the object position
      vpTRACE("Save the location of the object in a file dat/ellipse.dat");
      std::ofstream f(name);
      f << c.get_s().t();
      f.close();
      exit(1);
    }

    vpTRACE("sets the desired position of the visual feature ");
    vpFeatureEllipse cd;
    std::ifstream f("dat/ellipse.dat");
    double x, y, mu20, mu11, mu02;
    f >> x;
    f >> y;
    f >> mu20;
    f >> mu11;
    f >> mu02;
    f.close();
    cd.buildFrom(x, y, mu20, mu11, mu02);
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
    for (;;) {
      std::cout << "---------------------------------------------" << iter++ << std::endl;

      g.acquire(I);
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
      } else
        gain = lambda_av;

      vpTRACE("%f %f", (task.getError()).sumSquare(), gain);
      task.setLambda(gain);
      vpColVector v;
      v = task.computeControlLaw();
      std::cout << "rank " << task.getTaskRank() << std::endl;
      vpServoDisplay::display(task, cam, I);
      std::cout << v.t();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpDisplay::flush(I);
      vpTRACE("\t\t || s - s* || = %f ", (task.getError()).sumSquare());
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
