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
  \example tutorial-flir-ptu-ibvs.cpp

  Example of eye-in-hand image-based control law. We control here a real robot, the
  FLIR PTU that has 2 degrees of freedom. The velocity is computed in the joint space.
  Visual features are the image coordinates of the center of gravity of an AprilTag.
  The goal is here to center the tag in the image acquired from a FLIR camera mounted
  on the PTU.

  Camera extrinsic (eMc) parameters are set by default to a value that will not match
  Your configuration. Use --eMc command line option to read the values from a file.
  This file could be obtained following extrinsic camera calibration tutorial:
  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html

  Camera intrinsic parameters are retrieved from the Realsense SDK.

  The target is an AprilTag. It's size doesn't matter since we are using the
  center of gravity position.

*/

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotFlirPtu.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#if defined(VISP_HAVE_FLIR_PTU_SDK) && defined(VISP_HAVE_FLYCAPTURE) &&                                                \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_portname;
  int opt_baudrate = 9600;
  bool opt_network = false;
  std::string opt_extrinsic;
  double opt_tag_size = 0.120; // Used to compute the distance of the cog wrt the camera
  double opt_constant_gain = 0.5;

  if (argc == 1) {
    std::cout << "To see how to use this example, run: " << argv[0] << " --help" << std::endl;
    return EXIT_SUCCESS;
  }

  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--portname" || std::string(argv[i]) == "-p") && (i + 1 < argc)) {
      opt_portname = std::string(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--baudrate" || std::string(argv[i]) == "-b") && (i + 1 < argc)) {
      opt_baudrate = std::atoi(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--network" || std::string(argv[i]) == "-n")) {
      opt_network = true;
    }
    else if (std::string(argv[i]) == "--extrinsic" && i + 1 < argc) {
      opt_extrinsic = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--constant-gain" || std::string(argv[i]) == "-g") {
      opt_constant_gain = std::stod(argv[i + 1]);
      ;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSIS" << std::endl
        << "  " << argv[0] << " [--portname <portname>] [--baudrate <rate>] [--network] "
        << "[--extrinsic <extrinsic.yaml>] [--constant-gain] [--help] [-h]" << std::endl
        << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  --portname, -p <portname>" << std::endl
        << "    Set serial or tcp port name." << std::endl
        << std::endl
        << "  --baudrate, -b <rate>" << std::endl
        << "    Set serial communication baud rate. Default: " << opt_baudrate << "." << std::endl
        << std::endl
        << "  --network, -n" << std::endl
        << "    Get PTU network information (Hostname, IP, Gateway) and exit. " << std::endl
        << std::endl
        << "  --extrinsic <extrinsic.yaml>" << std::endl
        << "    YAML file containing extrinsic camera parameters as a vpHomogeneousMatrix." << std::endl
        << "    It corresponds to the homogeneous transformation eMc, between end-effector" << std::endl
        << "    and camera frame." << std::endl
        << std::endl
        << "  --constant-gain, -g" << std::endl
        << "    Constant gain value. Default value: " << opt_constant_gain << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Print this helper message. " << std::endl
        << std::endl;
      std::cout << "EXAMPLE" << std::endl
        << "  - How to get network IP" << std::endl
#ifdef _WIN32
        << "    $ " << argv[0] << " --portname COM1 --network" << std::endl
        << "    Try to connect FLIR PTU to port: COM1 with baudrate: 9600" << std::endl
#else
        << "    $ " << argv[0] << " --portname /dev/ttyUSB0 --network" << std::endl
        << "    Try to connect FLIR PTU to port: /dev/ttyUSB0 with baudrate: 9600" << std::endl
#endif
        << "       PTU HostName: PTU-5" << std::endl
        << "       PTU IP      : 169.254.110.254" << std::endl
        << "       PTU Gateway : 0.0.0.0" << std::endl
        << "  - How to run this binary using network communication" << std::endl
        << "    $ " << argv[0] << " --portname tcp:169.254.110.254 --tag-size 0.1 --gain 0.1" << std::endl;

      return EXIT_SUCCESS;
    }
  }

  vpRobotFlirPtu robot;

  try {
    std::cout << "Try to connect FLIR PTU to port: " << opt_portname << " with baudrate: " << opt_baudrate << std::endl;
    robot.connect(opt_portname, opt_baudrate);

    if (opt_network) {
      std::cout << "PTU HostName: " << robot.getNetworkHostName() << std::endl;
      std::cout << "PTU IP      : " << robot.getNetworkIP() << std::endl;
      std::cout << "PTU Gateway : " << robot.getNetworkGateway() << std::endl;
      return EXIT_SUCCESS;
    }

    vpImage<unsigned char> I;

    vpFlyCaptureGrabber g;
    g.open(I);

    // Get camera extrinsics
    vpTranslationVector etc;
    vpRotationMatrix eRc;
    eRc << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    etc << -0.1, -0.123, 0.035;
    vpHomogeneousMatrix eMc(etc, eRc);

    if (!opt_extrinsic.empty()) {
      vpPoseVector ePc;
      ePc.loadYAML(opt_extrinsic, ePc);
      eMc.build(ePc);
    }

    std::cout << "Considered extrinsic transformation eMc:\n" << eMc << std::endl;

    // Get camera intrinsics
    vpCameraParameters cam(900, 900, I.getWidth() / 2., I.getHeight() / 2.);
    std::cout << "Considered intrinsic camera parameters:\n" << cam << "\n";

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector.setDisplayTag(true);
    detector.setAprilTagQuadDecimate(2);

    // Create visual features
    vpFeaturePoint p, pd; // We use 1 point, the tag cog

    // Set desired position to the image center
    pd.set_x(0);
    pd.set_y(0);

    vpServo task;
    // Add the visual feature point
    task.addFeature(p, pd);
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(opt_constant_gain);

    bool final_quit = false;
    bool send_velocities = false;
    vpMatrix eJe;

    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame

    vpVelocityTwistMatrix cVe = robot.get_cVe();
    task.set_cVe(cVe);

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::vector<vpHomogeneousMatrix> cMo_vec;
    vpColVector qdot(2);

    while (!final_quit) {
      g.acquire(I);

      vpDisplay::display(I);

      detector.detect(I, opt_tag_size, cam, cMo_vec);

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      // Only one tag has to be detected
      if (detector.getNbObjects() == 1) {

        vpImagePoint cog = detector.getCog(0);
        double Z = cMo_vec[0][2][3];

        // Update current feature from measured cog position
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, cog, x, y);
        p.set_xyZ(x, y, Z);
        pd.set_Z(Z);

        // Get robot Jacobian
        robot.get_eJe(eJe);
        task.set_eJe(eJe);

        qdot = task.computeControlLaw();

        // Display the current and desired feature points in the image display
        vpServoDisplay::display(task, cam, I);
      } // end if (cMo_vec.size() == 1)
      else {
        qdot = 0;
      }

      if (!send_velocities) {
        qdot = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::JOINT_STATE, qdot);

      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        switch (button) {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          qdot = 0;
          break;

        default:
          break;
        }
      }
    }
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
  }
  catch (const vpRobotException &e) {
    std::cout << "Catch Flir Ptu exception: " << e.getMessage() << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_FLYCAPTURE)
  std::cout << "Install FLIR Flycapture" << std::endl;
#endif
#if !defined(VISP_HAVE_FLIR_PTU_SDK)
  std::cout << "Install FLIR PTU SDK." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
