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
 *   velocity computed in articular
 */

/*!
 * \file servoBiclopsPoint2DArtVelocity.cpp
 * \example servoBiclopsPoint2DArtVelocity.cpp
 *
 * Example of eye-in-hand control law. We control here a real robot, the
 * Biclops robot (pan-tilt head provided by Traclabs). The velocity is computed
 * in articular. The visual feature is the center of gravity of a point.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_BICLOPS) && defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS "c:d:h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
 * Print the program options.
 *
 * \param name : Program name.
 * \param badparam : Bad parameter name.
 * \param conf : Robot configuration file.
 */
void usage(const char *name, const char *badparam, std::string &conf)
{
  fprintf(stdout, "\n\
  Example of eye-in-hand control law. We control here a real robot, the biclops\n\
  robot (pan-tilt head provided by Traclabs) equipped with a Realsense camera\n\
  mounted on its end-effector. The velocity to apply to the PT head is joint\n\
  velocity. The visual feature is a point corresponding to the center of\n\
  gravity of an AprilTag. \n\
\n\
SYNOPSIS\n\
  %s [-c <Biclops configuration file>] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c <Biclops configuration file>                      %s\n\
     Sets the Biclops robot configuration file.\n",
          conf.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!
 * Set the program options.
 *
 * \param argc : Command line number of parameters.
 * \param argv : Array of command line parameters.
 * \param conf : Robot configuration file.
 *
 * \return false if the program has to be stopped, true otherwise.
 *
 */
bool getOptions(int argc, const char **argv, std::string &conf)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      conf = optarg_;
      break;
    case 'h':
      usage(argv[0], nullptr, conf);
      return false;
      break;

    default:
      usage(argv[0], optarg_, conf);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, conf);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    // Default unix configuration file path
    std::string opt_conf = "/usr/share/BiclopsDefault.cfg";

    // Read the command line options
    if (getOptions(argc, argv, opt_conf) == false) {
      return EXIT_FAILURE;
    }

    // Initialize PTU
    vpRobotBiclops robot(opt_conf);

    /*
     * Biclops DH2 has the following axis orientation
     *
     *  tilt + <----  (end-effector-frame)
     *             |
     *             \/ pan +
     *
     * The end-effector-frame from PT unit rear view is the following
     *
     *             /\ x
     *             |
     *         (e) ----> y
     *
     *
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

    robot.setDenavitHartenbergModel(vpBiclops::DH2);
    vpRotationMatrix cRe;
    cRe[0][0] = 0;  cRe[0][1] = 1; cRe[0][2] = 0;
    cRe[1][0] = -1; cRe[1][1] = 0; cRe[1][2] = 0;
    cRe[2][0] = 0;  cRe[2][1] = 0; cRe[2][2] = 1;
    vpTranslationVector cte; // By default set to 0

    // Robot Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    // Camera to end-effector frame transformation
    vpHomogeneousMatrix cMe(cte, cRe);
    // Velocity twist transformation to express a velocity from end-effector to camera frame
    vpVelocityTwistMatrix cVe(cMe);

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

    vpColVector q(vpBiclops::ndof);
    q = 0;
    std::cout << "Move PT to initial position: " << q.t() << std::endl;
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::JOINT_STATE, q);

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
    task.setLambda(0.2);

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    bool quit = false;
    bool send_velocities = false;
    vpColVector q_dot;

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

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
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
  std::cout << "You do not have an Biclops PT robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
