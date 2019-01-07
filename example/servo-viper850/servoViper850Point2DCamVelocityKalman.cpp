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
 *   velocity computed in camera frame
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoViper850Point2DCamVelocityKalman.cpp

  Example of eye-in-hand control law. We control here a real robot, the ADEPT
  Viper 850 robot (arm, with 6 degrees of freedom). The velocity is computed
  in the camera frame. The visual feature is the center of gravity of a point.
  We use here a linear Kalman filter with a constant velocity state model to
  estimate the moving target motion.

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
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed joint velocities (m/s, rad/s) to achieve the task
  // - the 6 mesured joint velocities (m/s, rad/s)
  // - the 6 mesured joint positions (m, rad)
  // - the 2 values of s - s*
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
      exit(-1);
    }
  }
  std::string logfilename;
  logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

  vpServo task;

  try {
    // Initialize linear Kalman filter
    vpLinearKalmanFilterInstantiation kalman;

    // Initialize the kalman filter
    unsigned int nsignal = 2; // The two values of dedt
    double rho = 0.3;
    vpColVector sigma_state;
    vpColVector sigma_measure(nsignal);
    unsigned int state_size = 0; // Kalman state vector size

    kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel);
    state_size = kalman.getStateSize();
    sigma_state.resize(state_size * nsignal);
    sigma_state = 0.00001; // Same state variance for all signals
    sigma_measure = 0.05;  // Same measure variance for all the signals
    double dummy = 0;      // non used parameter dt for the velocity state model
    kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dummy);

    // Initialize the robot
    vpRobotViper850 robot;

    vpImage<unsigned char> I;

    bool reset = false;
    vp1394TwoGrabber g(reset);

#if 1
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
#else
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_0);
    g.setColorCoding(vp1394TwoGrabber::vpCOLOR_CODING_MONO8);
#endif
    g.open(I);

    double Tloop = 1. / 80.f;

    vp1394TwoGrabber::vp1394TwoFramerateType fps;
    g.getFramerate(fps);
    switch (fps) {
    case vp1394TwoGrabber::vpFRAMERATE_15:
      Tloop = 1.f / 15.f;
      break;
    case vp1394TwoGrabber::vpFRAMERATE_30:
      Tloop = 1.f / 30.f;
      break;
    case vp1394TwoGrabber::vpFRAMERATE_60:
      Tloop = 1.f / 60.f;
      break;
    case vp1394TwoGrabber::vpFRAMERATE_120:
      Tloop = 1.f / 120.f;
      break;
    default:
      break;
    }

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, (int)(100 + I.getWidth() + 30), 200, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, (int)(100 + I.getWidth() + 30), 200, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, (int)(100 + I.getWidth() + 30), 200, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDot2 dot;
    vpImagePoint cog;

    dot.setGraphics(true);

    for (int i = 0; i < 10; i++)
      g.acquire(I);

    std::cout << "Click on a dot..." << std::endl;
    dot.initTracking(I);

    cog = dot.getCog();
    vpDisplay::displayCross(I, cog, 10, vpColor::blue);
    vpDisplay::flush(I);

    vpCameraParameters cam;
    // Update camera parameters
    robot.getCameraParameters(cam, I);

    // sets the current position of the visual feature
    vpFeaturePoint p;
    // retrieve x,y and Z of the vpPoint structure
    vpFeatureBuilder::create(p, cam, dot);

    // sets the desired position of the visual feature
    vpFeaturePoint pd;
    pd.buildFrom(0, 0, 1);

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    // - we want to see a point on a point
    task.addFeature(p, pd);

    // - set the constant gain
    vpAdaptiveGain lambda;
    lambda.initStandard(4, 0.2, 30);
    task.setLambda(lambda);

    // Display task information
    task.print();

    // Now the robot will be controlled in velocity
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    vpColVector v, v1, v2;
    int iter = 0;
    vpColVector vm(6);
    double t_0, t_1, Tv;
    vpColVector err(2), err_1(2);
    vpColVector dedt_filt(2), dedt_mes(2);
    dc1394video_frame_t *frame = NULL;

    t_1 = vpTime::measureTimeMs();

    for (;;) {
      try {
        t_0 = vpTime::measureTimeMs(); // t_0: current time

        // Update loop time in second
        Tv = (double)(t_0 - t_1) / 1000.0;

        // Update time for next iteration
        t_1 = t_0;

        vm = robot.getVelocity(vpRobot::CAMERA_FRAME);

        // Acquire a new image from the camera
        frame = g.dequeue(I);

        // Display this image
        vpDisplay::display(I);

        // Achieve the tracking of the dot in the image
        dot.track(I);

        // Get the dot cog
        cog = dot.getCog();

        // Display a green cross at the center of gravity position in the
        // image
        vpDisplay::displayCross(I, cog, 10, vpColor::green);

        // Update the point feature from the dot location
        vpFeatureBuilder::create(p, cam, dot);

        // Compute the visual servoing skew vector
        v1 = task.computeControlLaw();

        // Get the error ||s-s*||
        err = task.getError();

        //! terme correctif : de/dt = Delta s / Delta t - L*vc
        if (iter == 0) {
          err_1 = 0;
          dedt_mes = 0;
        } else {
          vpMatrix J1 = task.getTaskJacobian();
          dedt_mes = (err - err_1) / (Tv)-J1 * vm;
          err_1 = err;
        }

        // Filter de/dt
        if (iter < 2)
          dedt_mes = 0;
        kalman.filter(dedt_mes);
        // Get the filtered values
        for (unsigned int i = 0; i < nsignal; i++) {
          dedt_filt[i] = kalman.Xest[i * state_size];
        }
        if (iter < 2)
          dedt_filt = 0;

        vpMatrix J1p = task.getTaskJacobianPseudoInverse();
        v2 = -J1p * dedt_filt;

        // Update the robot camera velocity
        v = v1 + v2;

        // Display the current and desired feature points in the image display
        vpServoDisplay::display(task, cam, I);

        // Apply the computed camera velocities to the robot
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        iter++;
        // Synchronize the loop with the image frame rate
        vpTime::wait(t_0, 1000. * Tloop);
        // Release the ring buffer used for the last image to start a new acq
        g.enqueue(frame);
      } catch (...) {
        std::cout << "Tracking failed... Stop the robot." << std::endl;
        v = 0;
        // Stop robot
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        // Kill the task
        task.kill();
        return 0;
      }

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

      // Save feature error (s-s*) for the feature point. For this feature
      // point, we have 2 errors (along x and y axis).  This error is
      // expressed in meters in the camera frame
      flog << (task.getError()).t() << std::endl; // s-s* for point

      // Flush the display
      vpDisplay::flush(I);
    }

    flog.close(); // Close the log file

    // Display task information
    task.print();

    // Kill the task
    task.kill();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    flog.close(); // Close the log file
    // Kill the task
    task.kill();
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
