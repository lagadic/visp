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
  \example servoFlirPtuIBVS.cpp

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
#include <visp3/core/vpXmlParserCamera.h>
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

#if defined(VISP_HAVE_FLIR_PTU_SDK) && defined(VISP_HAVE_FLYCAPTURE) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PUGIXML)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void display_point_trajectory(const vpImage<unsigned char> &I, const vpImagePoint &ip,
                              std::vector<vpImagePoint> &traj_ip)
{
  if (traj_ip.size()) {
    // Add the point only if distance with the previous > 2 pixel
    if (vpImagePoint::distance(ip, traj_ip.back()) > 2.) {
      traj_ip.push_back(ip);
    }
  }
  else {
    traj_ip.push_back(ip);
  }
  for (size_t j = 1; j < traj_ip.size(); j++) {
    vpDisplay::displayLine(I, traj_ip[j - 1], traj_ip[j], vpColor::green, 2);
  }
}

int main(int argc, char **argv)
{
  std::string opt_portname;
  int opt_baudrate = 9600;
  bool opt_network = false;
  std::string opt_extrinsic;
  std::string opt_intrinsic;
  std::string opt_camera_name;
  bool display_tag = true;
  int opt_quad_decimate = 2;
  double opt_tag_size = 0.120; // Used to compute the distance of the cog wrt the camera
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double opt_constant_gain = 0.5;
  bool opt_display_trajectory = true;
  double convergence_threshold = 0.00005;

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
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot" || std::string(argv[i]) == "-p") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--display-image-trajectory" || std::string(argv[i]) == "-traj") {
      opt_display_trajectory = true;
    }
    else if (std::string(argv[i]) == "--adaptive-gain" || std::string(argv[i]) == "-a") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--constant-gain" || std::string(argv[i]) == "-g") {
      opt_constant_gain = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--task-sequencing") {
      opt_task_sequencing = true;
    }
    else if (std::string(argv[i]) == "--quad-decimate" && i + 1 < argc) {
      opt_quad_decimate = std::stoi(argv[i + 1]);
    }
    if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      convergence_threshold = 0.;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSIS" << std::endl
        << "  " << argv[0] << " [--portname <portname>] [--baudrate <rate>] [--network] "
        << "[--extrinsic <extrinsic.yaml>] [--intrinsic <intrinsic.xml>] [--camera-name <name>] "
        << "[--quad-decimate <decimation>] [--tag-size <size>] "
        << "[--adaptive-gain] [--constant-gain] [--display-image-trajectory] [--plot] [--task-sequencing] "
        << "[--no-convergence-threshold] [--verbose] [--help] [-h]" << std::endl
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
        << "  --reset, -r" << std::endl
        << "    Reset PTU axis and exit. " << std::endl
        << std::endl
        << "  --extrinsic <extrinsic.yaml>" << std::endl
        << "    YAML file containing extrinsic camera parameters as a vpHomogeneousMatrix." << std::endl
        << "    It corresponds to the homogeneous transformation eMc, between end-effector" << std::endl
        << "    and camera frame." << std::endl
        << std::endl
        << "  --intrinsic <intrinsic.xml>" << std::endl
        << "    Intrinsic camera parameters obtained after camera calibration." << std::endl
        << std::endl
        << "  --camera-name <name>" << std::endl
        << "    Name of the camera to consider in the xml file provided for intrinsic camera parameters."
        << std::endl
        << std::endl
        << "  --quad-decimate <decimation>" << std::endl
        << "    Decimation factor used to detect AprilTag. Default " << opt_quad_decimate << "." << std::endl
        << std::endl
        << "  --tag-size <size>" << std::endl
        << "    Width in meter or the black part of the AprilTag used as target. Default " << opt_tag_size
        << "." << std::endl
        << std::endl
        << "  --adaptive-gain, -a" << std::endl
        << "    Enable adaptive gain instead of constant gain to speed up convergence. " << std::endl
        << std::endl
        << "  --constant-gain, -g" << std::endl
        << "    Constant gain value. Default value: " << opt_constant_gain << std::endl
        << std::endl
        << "  --display-image-trajectory, -traj" << std::endl
        << "    Display the trajectory of the target cog in the image. " << std::endl
        << std::endl
        << "  --plot, -p" << std::endl
        << "    Enable curve plotter. " << std::endl
        << std::endl
        << "  --task-sequencing" << std::endl
        << "    Enable task sequencing that allows to smoothly control the velocity of the robot. " << std::endl
        << std::endl
        << "  --no-convergence-threshold" << std::endl
        << "    Disable ending servoing when it reaches the desired position." << std::endl
        << std::endl
        << "  --verbose, -v" << std::endl
        << "    Additional printings. " << std::endl
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
        << "    $ " << argv[0] << " -p /dev/ttyUSB0 --network" << std::endl
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

    // If provided, read camera extrinsics from command line option
    if (!opt_extrinsic.empty()) {
      vpPoseVector ePc;
      ePc.loadYAML(opt_extrinsic, ePc);
      eMc.build(ePc);
    }
    else {
      std::cout << "***************************************************************" << std::endl;
      std::cout << "Warning, use hard coded values for extrinsic camera parameters." << std::endl;
      std::cout << "Create a yaml file that contains the extrinsic:" << std::endl
        << std::endl
        << "$ cat eMc.yaml" << std::endl
        << "rows: 4" << std::endl
        << "cols: 4" << std::endl
        << "data:" << std::endl
        << "  - [0, 0, 1, -0.1]" << std::endl
        << "  - [-1, 0, 0, -0.123]" << std::endl
        << "  - [0, -1, 0, 0.035]" << std::endl
        << "  - [0, 0, 0, 1]" << std::endl
        << std::endl
        << "and load this file with [--extrinsic <extrinsic.yaml] command line option, like:" << std::endl
        << std::endl
        << "$ " << argv[0] << "-p " << opt_portname << " --extrinsic eMc.yaml" << std::endl
        << std::endl;
      std::cout << "***************************************************************" << std::endl;
    }

    std::cout << "Considered extrinsic transformation eMc:\n" << eMc << std::endl;

    // Get camera intrinsics
    vpCameraParameters cam(900, 900, I.getWidth() / 2., I.getHeight() / 2.);

    // If provided, read camera intrinsics from command line option
    if (!opt_intrinsic.empty() && !opt_camera_name.empty()) {
      vpXmlParserCamera parser;
      parser.parse(cam, opt_intrinsic, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    }
    else {
      std::cout << "***************************************************************" << std::endl;
      std::cout << "Warning, use hard coded values for intrinsic camera parameters." << std::endl;
      std::cout << "Calibrate your camera and load the parameters from command line options, like:" << std::endl
        << std::endl
        << "$ " << argv[0] << "-p " << opt_portname << " --intrinsic camera.xml --camera-name \"Camera\""
        << std::endl
        << std::endl;
      std::cout << "***************************************************************" << std::endl;
    }

    std::cout << "Considered intrinsic camera parameters:\n" << cam << "\n";

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

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

    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(opt_constant_gain);
    }

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Joint velocities");
      plotter->initGraph(0, 2);
      plotter->initGraph(1, 2);
      plotter->setLegend(0, 0, "error_feat_p_x");
      plotter->setLegend(0, 1, "error_feat_p_y");
      plotter->setLegend(1, 0, "qdot_pan");
      plotter->setLegend(1, 1, "qdot_tilt");
    }

    bool final_quit = false;
    bool has_converged = false;
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpImagePoint> traj_cog;
    vpMatrix eJe;

    static double t_init_servo = vpTime::measureTimeMs();

    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame

    vpVelocityTwistMatrix cVe = robot.get_cVe();
    std::cout << cVe << std::endl;
    task.set_cVe(cVe);

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      g.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, opt_tag_size, cam, cMo_vec);

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      vpColVector qdot(2);

      // Only one tag has to be detected
      if (detector.getNbObjects() == 1) {

        vpImagePoint cog = detector.getCog(0);
        double Z = cMo_vec[0][2][3];

        // Update current feature from measured cog position
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, cog, x, y);
        if (opt_verbose) {
          std::cout << "Z: " << Z << std::endl;
        }
        p.set_xyZ(x, y, Z);
        pd.set_Z(Z);

        // Get robot Jacobian
        robot.get_eJe(eJe);
        task.set_eJe(eJe);

        if (opt_task_sequencing) {
          if (!servo_started) {
            if (send_velocities) {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          qdot = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
        }
        else {
          qdot = task.computeControlLaw();
        }

        // Display the current and desired feature points in the image display
        vpServoDisplay::display(task, cam, I, vpColor::green, vpColor::red, 3);

        // Display the trajectory of the points used as features
        if (opt_display_trajectory) {
          display_point_trajectory(I, cog, traj_cog);
        }

        if (opt_plot) {
          plotter->plot(0, iter_plot, task.getError());
          plotter->plot(1, iter_plot, qdot);
          iter_plot++;
        }

        if (opt_verbose) {
          std::cout << "qdot: " << qdot.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        ss.str("");
        ss << "error: " << error;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error: " << error << std::endl;

        if (error < convergence_threshold) {
          has_converged = true;
          std::cout << "Servo task has converged"
            << "\n";
          vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }
      } // end if (cMo_vec.size() == 1)
      else {
        qdot = 0;
      }

      if (!send_velocities) {
        qdot = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::JOINT_STATE, qdot);
      ss.str("");
      ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

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

    if (opt_plot && plotter != nullptr) {
      delete plotter;
      plotter = nullptr;
    }

    if (!final_quit) {
      while (!final_quit) {
        g.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
        vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
  }
  catch (const vpRobotException &e) {
    std::cout << "Catch Flir Ptu signal exception: " << e.getMessage() << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
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
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
