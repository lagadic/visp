/*
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
 *   velocity computed in joint
 */

/*!
 * \file servoPololuPtuPoint2DJointVelocity.cpp
 * \example servoPololuPtuPoint2DJointVelocity.cpp
 *
 * Example of eye-in-hand control law. We control here a real robot, a pan-tilt head controlled using a Pololu Maestro
 * board where pan axis servo a connected to channel 0 and tilt axis to channel 1. The velocity is computed
 * in joint. The visual feature is a 2D point corresponding to the center of gravity of an AprilTag.
 * A Realsense camera is mounted on the pan-tilt unit.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_POLOLU) && defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpTime.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotPololuPtu.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServoDisplay.h>

void usage(const char **argv, int error, const std::string &device, int baudrate)
{
  std::cout << "Name" << std::endl
    << "  Example of eye-in-hand control law. We control here a real robot, a pan-tilt unit" << std::endl
    << "  controlled using a Pololu Maestro board equipped.The PTU is equipped with a Realsense" << std::endl
    << "  camera mounted on its end-effector.The velocity to apply to the PT head is a joint" << std::endl
    << "  velocity.The visual feature is a point corresponding to the center of gravity" << std::endl
    << "  of an AprilTag." << std::endl
    << std::endl;
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0] << " [--device <name>] [--baud <rate>] [--verbose, -v] [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  --device <name>  Device name." << std::endl
    << "    Default: " << device << std::endl
    << std::endl
    << "  --baud <rate>  Serial link baud rate." << std::endl
    << "    Default: " << baudrate << std::endl
    << std::endl
    << "  --verbose, -v  Enable verbosity." << std::endl
    << std::endl
    << "  --help, -h  Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#ifdef _WIN32
  std::string opt_device = "COM4";
#else
  std::string opt_device = "/dev/ttyACM0";
  // Example for Mac OS, the Maestro creates two devices, use the one with the lowest number (the command port)
  //std::string opt_device = "/dev/cu.usbmodem00031501";
#endif
  int opt_baudrate = 38400;
  bool opt_verbose = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0, opt_device, opt_baudrate);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i, opt_device, opt_baudrate);
      return EXIT_FAILURE;
    }
  }

  try {
    // Creating the servo object on channel 0
    vpRobotPololuPtu robot(opt_device, opt_baudrate, opt_verbose);

    /*
     * Pololu PTU has the following axis orientation (rear view)
     *
     *  tilt + <----  (end-effector-frame)
     *             |
     *             \/ pan +
     *
     * The PTU end-effector-frame is the following (rear view)
     *
     *             /\ x
     *             |
     *         (e) ----> y
     *
     * The camera frame attached to the PT unit is the following (rear view)
     *
     *         (c) ----> x
     *             |
     *             \/ y
     *
     * The corresponding cRe (camera to end-effector rotation matrix) is then the following
     *
     *       ( 0  1  0)
     * cRe = (-1  0  0)
     *       ( 0  0  1)
     *
     * Translation cte (camera to end-effector) can be neglected
     *
     *       (0)
     * cte = (0)
     *       (0)
     */

    vpRotationMatrix cRe({ 0, 1, 0, -1, 0, 0, 0, 0, 1 });
    vpTranslationVector cte; // By default set to 0

    // Robot Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    // Camera to end-effector frame transformation
    vpHomogeneousMatrix cMe(cte, cRe);
    // Velocity twist transformation to express a velocity from end-effector to camera frame
    vpVelocityTwistMatrix cVe(cMe);

    vpColVector q(robot.getNDof());
    q = 0;
    std::cout << "Move PT to initial position: " << q.t() << std::endl;
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPositioningVelocityPercentage(10.f);
    robot.setPosition(vpRobot::JOINT_STATE, q);

    vpTime::wait(1500); // TODO make setPosition() blocking

    std::cout << "Min velocity resolution: " << vpMath::deg(robot.getAngularVelocityResolution()) << " deg/s" << std::endl;

    // Initialize grabber
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

    vpImage<unsigned char> I;
    g.acquire(I);

    // We open a window using either X11 or GTK or GDI.
    // Its size is automatically defined by the image (I) size
#if defined(VISP_HAVE_X11)
    vpDisplayX display(I, 100, 100, "Display X...");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Display GTK...");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I, 100, 100, "Display GDI...");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDetectorAprilTag detector;

    vpServo task;

    // Create current and desired point visual feature
    vpFeaturePoint p, pd;
    // Sets the desired position of the visual feature
    // Here we set Z desired to 1 meter, and (x,y)=(0,0) to center the tag in the image
    pd.build(0, 0, 1);

    // Define the task
    // - we want an eye-in-hand control law
    // - joint velocities are computed
    // - interaction matrix is the one at desired position
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.set_cVe(cVe);
    // We want to see a point on a point
    task.addFeature(p, pd);
    // Set the gain
    //task.setLambda(2.0);
    //vpAdaptiveGain lambda(2, 0.7, 30);
    vpAdaptiveGain lambda(3.5, 2, 50);
    task.setLambda(lambda);

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // {
    //   vpColVector ve(6);
    //   ve = 0;
    //   ve[5] = vpMath::rad(5);
    //   double t_start = vpTime::measureTimeMs();
    //   while (vpTime::measureTimeMs() - t_start < 3000) {
    //     robot.get_eJe(eJe);
    //     vpColVector q_dot = (cVe * eJe).pseudoInverse() * ve;
    //     robot.setVelocity(vpRobot::JOINT_STATE, q_dot);
    //     vpTime::wait(40);
    //   }

    //   return EXIT_SUCCESS;
    // }


    bool quit = false;
    bool send_velocities = false;
    vpColVector q_dot(robot.getNDof());
    double min_pix_error = 10; // In pixels
    double min_error = vpMath::sqr(min_pix_error / cam.get_px());

    while (!quit) {
      g.acquire(I);
      vpDisplay::display(I);

      {
        std::stringstream ss;
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
      }

      if (detector.detect(I)) {
        // We consider the first tag only
        vpImagePoint cog = detector.getCog(0); // 0 is the id of the first tag

        vpFeatureBuilder::create(p, cam, cog);

        // Get the jacobian
        robot.get_eJe(eJe);
        task.set_eJe(eJe);

        q_dot = task.computeControlLaw();

        vpServoDisplay::display(task, cam, I);
        vpDisplay::flush(I);

        double error = (task.getError()).sumSquare();
        if (opt_verbose) {
          std::cout << "|| s - s* || = " << error << std::endl;
        }
        if (error < min_error) {
          if (opt_verbose) {
            std::cout << "Stop the robot" << std::endl;
          }
          q_dot = 0;
        }
      }
      else {
        q_dot = 0;
      }
      if (!send_velocities) {
        q_dot = 0;
      }

      robot.setVelocity(vpRobot::JOINT_STATE, q_dot);

      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        switch (button) {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          quit = true;
          q_dot = 0;
          break;

        default:
          break;
        }
      }
    }

    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have a Pololu PTU connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
