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
 *   eye-to-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/
/*!
  \file servoAfma6Points2DCamVelocityEyeToHand.cpp

  \brief Example of a eye-to-hand control law. We control here a real robot,
  the Afma6 robot (cartesian robot, with 6 degrees of freedom). The robot is
  controlled in the camera frame.

*/

/*!
  \example servoAfma6Points2DCamVelocityEyeToHand.cpp

  Example of a eye-to-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The robot is
  controlled in the camera frame.

*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <list>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394))

#define SAVE 0

#include <visp3/blob/vpDot.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#define L 0.006
#define D 0

int main()
{
  try {
    vpServo task;

    vpCameraParameters cam;
    vpImage<unsigned char> I;
    int i;

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
    std::cout << " Eye-to-hand task control" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo a point " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    int nbPoint = 7;

    vpDot dot[nbPoint];
    vpImagePoint cog;

    for (i = 0; i < nbPoint; i++) {
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
    // Update camera parameters
    robot.getCameraParameters(cam, I);

    vpHomogeneousMatrix cMo, cdMo;
    vpPose pose;
    pose.clearPoint();
    for (i = 0; i < nbPoint; i++) {
      cog = dot[i].getCog();
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(cam, cog, x, y);
      point[i].set_x(x);
      point[i].set_y(y);
      pose.addPoint(point[i]);
    }

    // compute the initial pose using Dementhon method followed by a non
    // linear minimisation method
    pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

    std::cout << cMo << std::endl;
    cMo.print();

    /*------------------------------------------------------------------
    --  Learning the desired position
    --  or reading the desired position
    ------------------------------------------------------------------
    */
    std::cout << " Learning 0/1 " << std::endl;
    char name[FILENAME_MAX];
    sprintf(name, "cdMo.dat");
    int learning;
    std::cin >> learning;
    if (learning == 1) {
      // save the object position
      vpTRACE("Save the location of the object in a file cdMo.dat");
      std::ofstream f(name);
      cMo.save(f);
      f.close();
      exit(1);
    }

    {
      vpTRACE("Loading desired location from cdMo.dat");
      std::ifstream f("cdMo.dat");
      cdMo.load(f);
      f.close();
    }

    vpFeaturePoint p[nbPoint], pd[nbPoint];

    // set the desired position of the point by forward projection using
    // the pose cdMo
    for (i = 0; i < nbPoint; i++) {
      vpColVector cP, p;
      point[i].changeFrame(cdMo, cP);
      point[i].projection(cP, p);

      pd[i].set_x(p[0]);
      pd[i].set_y(p[1]);
    }

    //------------------------------------------------------------------

    vpTRACE("define the task");
    vpTRACE("\t we want an eye-in-hand control law");
    vpTRACE("\t robot is controlled in the camera frame");
    task.setServo(vpServo::EYETOHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT);

    for (i = 0; i < nbPoint; i++) {
      task.addFeature(p[i], pd[i]);
    }

    vpTRACE("Display task information ");
    task.print();

    //------------------------------------------------------------------

    double convergence_threshold = 0.00; // 025 ;
    vpDisplay::getClick(I);

    //-------------------------------------------------------------
    double error = 1;
    unsigned int iter = 0;
    vpTRACE("\t loop");
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    vpColVector v; // computed robot velocity

    // position of the object in the effector frame
    vpHomogeneousMatrix oMcamrobot;
    oMcamrobot[0][3] = -0.05;

    vpImage<vpRGBa> Ic;
    int it = 0;

    double lambda_av = 0.1;
    double alpha = 1; // 1 ;
    double beta = 3;  // 3 ;

    std::cout << "alpha 0.7" << std::endl;
    std::cin >> alpha;
    std::cout << "beta 5" << std::endl;
    std::cin >> beta;
    std::list<vpImagePoint> Lcog;
    vpImagePoint ip;
    while (error > convergence_threshold) {
      std::cout << "---------------------------------------------" << iter++ << std::endl;

      g.acquire(I);
      vpDisplay::display(I);
      ip.set_i(265);
      ip.set_j(150);
      vpDisplay::displayText(I, ip, "Eye-To-Hand Visual Servoing", vpColor::green);
      ip.set_i(280);
      ip.set_j(150);
      vpDisplay::displayText(I, ip, "IRISA-INRIA Rennes, Lagadic project", vpColor::green);
      try {
        for (i = 0; i < nbPoint; i++) {
          dot[i].track(I);
          Lcog.push_back(dot[i].getCog());
        }
      } catch (...) {
        vpTRACE("Error detected while tracking visual features");
        robot.stopMotion();
        exit(1);
      }

      // compute the initial pose using  a non linear minimisation method
      pose.clearPoint();

      for (i = 0; i < nbPoint; i++) {
        double x = 0, y = 0;
        cog = dot[i].getCog();
        vpPixelMeterConversion::convertPoint(cam, cog, x, y);
        point[i].set_x(x);
        point[i].set_y(y);

        vpColVector cP;
        point[i].changeFrame(cdMo, cP);

        p[i].set_x(x);
        p[i].set_y(y);
        p[i].set_Z(cP[2]);

        pose.addPoint(point[i]);

        point[i].display(I, cMo, cam, vpColor::green);
        point[i].display(I, cdMo, cam, vpColor::blue);
      }
      pose.computePose(vpPose::LOWE, cMo);
      vpDisplay::flush(I);

      //! set up the Jacobian
      vpHomogeneousMatrix cMe, camrobotMe;
      robot.get_cMe(camrobotMe);
      cMe = cMo * oMcamrobot * camrobotMe;

      task.set_cVe(cMe);

      vpMatrix eJe;
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

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
      if (SAVE == 1)
        gain = gain / 5;

      vpTRACE("%f %f %f %f  %f", alpha, beta, lambda_av, (task.getError()).sumSquare(), gain);
      task.setLambda(gain);

      v = task.computeControlLaw();

      // display points trajectory
      for (std::list<vpImagePoint>::const_iterator it_cog = Lcog.begin(); it_cog != Lcog.end(); ++it_cog) {
        vpDisplay::displayPoint(I, *it_cog, vpColor::red);
      }
      vpServoDisplay::display(task, cam, I);
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

      error = (task.getError()).sumSquare();
      std::cout << "|| s - s* || = " << error << std::endl;

      if (error > 7) {
        vpTRACE("Error detected while tracking visual features");
        robot.stopMotion();
        exit(1);
      }

      // display the pose
      // pose.display(I,cMo,cam, 0.04, vpColor::red) ;
      // display the pose
      //   pose.display(I,cdMo,cam, 0.04, vpColor::blue) ;
      if ((SAVE == 1) && (iter % 3 == 0)) {

        vpDisplay::getImage(I, Ic);
        sprintf(name, "/tmp/marchand/image.%04d.ppm", it++);
        vpImageIo::write(Ic, name);
      }
    }
    v = 0;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    vpDisplay::getClick(I);
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
