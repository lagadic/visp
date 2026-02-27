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
 *   eye-to-hand control
 *   velocity computed in the camera frame
 */

/*!
  \example servoFrankaIBVS-EyeToHand-Lcur_cVf_fVe_eJe.cpp

  Example of eye-to-hand image-based control law. We control here a real robot, the
  Franka Emika Panda robot (arm with 7 degrees of freedom).
  An Apriltag is attached to the robot end-effector. A camera mounted on a fixed tripod is observing the Apriltag.
  The velocity is computed in the camera frame. The inverse jacobian that converts cartesian
  velocities in joint velocities is implemented in the robot low level
  controller. Visual features are the image coordinates of 4 points corresponding
  to the corners of an AprilTag.

  The device used to acquire images is a Realsense D435 device.

  Camera extrinsic (eMo) transformation is set by default to a value that will not match
  Your configuration.
  - Use `--rMc` command line option to read the robot reference to camera frames constant transformation from
    a file. This file could be obtained following extrinsic camera calibration tutorial:
    https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic-eye-to-hand.html

  - Camera intrinsic parameters are retrieved from the Realsense SDK.

  The target is an AprilTag that is by default 12 cm large. To print your own tag, see
  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-detection-apriltag.html
  You can specify the size of your tag using --tag-size command line option.

  In this example we are estimating the pose of the tag:
  - to compute the interaction matrix with the current visual features L_s = f(x, y, Z)
  - cVf is the constant velocity twist matrix between the camera and the robot reference frames. This matrix is
    obtained by extrinsic calibration
  - fVe is the velocity twist transformation matrix between the robot reference and end-effector frames. This matrix
    is obtained thanks to the robot odometry.
*/

#include <iostream>
#include <fstream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_FRANKA) && defined(VISP_HAVE_PUGIXML)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool save_desired_features(const std::string &filename, const std::vector<vpFeaturePoint> &desired_features)
{
  std::ofstream file(filename);
  if (file.is_open()) {
    for (size_t i = 0; i < desired_features.size(); ++i) {
      file << desired_features[i].get_x() << " " << desired_features[i].get_y() << " " << desired_features[i].get_Z() << std::endl;
    }

    file.close();
    return true;
  }
  else {
    return false;
  }
}

bool read_desired_features(const std::string &filename, std::vector<vpFeaturePoint> &desired_features)
{
  desired_features.clear();
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      double x, y, Z;
      if (!(iss >> x >> y >> Z)) {
        return false;
      }
      vpFeaturePoint s_d;
      s_d.buildFrom(x, y, Z);
      desired_features.push_back(s_d);
    }
    file.close();

    return true;
  }
  else {
    return false;
  }
}

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); ++i) {
    if (traj_vip[i].size()) {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); ++i) {
    for (size_t j = 1; j < traj_vip[i].size(); j++) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

int main(int argc, char **argv)
{
  double opt_tag_size = 0.120;
  bool opt_tag_z_aligned = false;
  std::string opt_robot_ip = "192.168.1.1";
  std::string opt_rMc_filename = "";
  std::string opt_intrinsic_filename = "";
  std::string opt_camera_name = "Camera";
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  bool opt_learn_desired_features = false;
  std::string desired_features_filename = "learned_desired_features.txt";
  double convergence_threshold = 0.00005;

  for (int i = 1; i < argc; ++i) {
    if ((std::string(argv[i]) == "--tag-size") && (i + 1 < argc)) {
      opt_tag_size = std::stod(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-z-aligned") {
      opt_tag_z_aligned = true;
    }
    else if ((std::string(argv[i]) == "--ip") && (i + 1 < argc)) {
      opt_robot_ip = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--rMc") && (i + 1 < argc)) {
      opt_rMc_filename = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--verbose") || (std::string(argv[i]) == "-v")) {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--adaptive-gain") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--task-sequencing") {
      opt_task_sequencing = true;
    }
    else if ((std::string(argv[i]) == "--tag-quad-decimate") && (i + 1 < argc)) {
      opt_quad_decimate = std::stoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      convergence_threshold = 0.;
    }
    else if (std::string(argv[i]) == "--learn-desired-features") {
      opt_learn_desired_features = true;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout << "SYNOPSYS" << std::endl
        << "  " << argv[0]
        << " [--ip <controller ip>]"
        << " [--intrinsic <xml file>]"
        << " [--camera-name <name>]"
        << " [--tag-size <size>]"
        << " [--tag-quad-decimate <decimation factor>]"
        << " [--tag-z-aligned]"
        << " [--learn-desired-features]"
        << " [--rMc <file.yaml>]"
        << " [--adaptive-gain]"
        << " [--plot]"
        << " [--task-sequencing]"
        << " [--no-convergence-threshold]"
        << " [--verbose, -v]"
        << " [--help, -h]\n"
        << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  Use an image-based visual-servoing scheme to position the camera in front of an Apriltag." << std::endl
        << std::endl
        << "  --ip <controller ip>" << std::endl
        << "    Franka controller ip address" << std::endl
        << "    Default: " << opt_robot_ip << std::endl
        << std::endl
        << "  --intrinsic <xml file>" << std::endl
        << "    XML file that contains camera intrinsic parameters. " << std::endl
        << "    If no file is specified, use Realsense camera factory intrinsic parameters." << std::endl
        << std::endl
        << "  --camera-name <name>" << std::endl
        << "    Camera name in the XML file that contains camera intrinsic parameters." << std::endl
        << "    Default: \"Camera\"" << std::endl
        << std::endl
        << "  --tag-size <size>" << std::endl
        << "    Apriltag size in [m]." << std::endl
        << "    Default: " << opt_tag_size << " [m]" << std::endl
        << std::endl
        << "  --tag-quad-decimate <decimation factor>" << std::endl
        << "    Decimation factor used during Apriltag detection." << std::endl
        << "    Default: " << opt_quad_decimate << std::endl
        << std::endl
        << "  --tag-z-aligned" << std::endl
        << "    When enabled, tag z-axis and camera z-axis are aligned." << std::endl
        << "    Default: false" << std::endl
        << std::endl
        << "  --rMc <file.yaml>" << std::endl
        << "    Yaml file containing the extrinsic transformation between" << std::endl
        << "    robot reference frame and camera frames." << std::endl
        << std::endl
        << "  --adaptive-gain" << std::endl
        << "    Flag to enable adaptive gain to speed up visual servo near convergence." << std::endl
        << std::endl
        << "  --plot" << std::endl
        << "    Flag to enable curve plotter." << std::endl
        << std::endl
        << "  --task-sequencing" << std::endl
        << "    Flag to enable task sequencing scheme." << std::endl
        << std::endl
        << "  --no-convergence-threshold" << std::endl
        << "    Flag to disable convergence threshold used to stop the visual servo." << std::endl
        << std::endl
        << "  --learn-desired-pose" << std::endl
        << "    Flag to enable desired pose learning." << std::endl
        << "    Data are saved in learned-desired-pose.yaml file." << std::endl
        << std::endl
        << "  --verbose, -v" << std::endl
        << "    Flag to enable extra verbosity." << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Print this helper message." << std::endl
        << std::endl;

      return EXIT_SUCCESS;
    }
    else {
      std::cout << "\nERROR" << std::endl
        << std::string(argv[i]) << " command line option is not supported." << std::endl
        << "Use " << std::string(argv[0]) << " --help" << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }
  }

  vpRealSense2 rs;
  rs2::config config;
  unsigned int width = 640, height = 480;
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
  rs.open(config);

  vpImage<unsigned char> I(height, width);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay(I, 10, 10, "Current image");
#endif

  std::cout << "Parameters:" << std::endl;
  std::cout << "  Apriltag                  " << std::endl;
  std::cout << "    Size [m]              : " << opt_tag_size << std::endl;
  std::cout << "    Z aligned             : " << (opt_tag_z_aligned ? "true" : "false") << std::endl;
  std::cout << "  Camera intrinsics         " << std::endl;
  std::cout << "    Factory parameters    : " << (opt_intrinsic_filename.empty() ? "yes" : "no") << std::endl;

  // Get camera intrinsics
  vpCameraParameters cam;
  if (opt_intrinsic_filename.empty()) {
    std::cout << "Use Realsense camera intrinsic factory parameters: " << std::endl;
    cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << "cam:\n" << cam << std::endl;
  }
  else if (!vpIoTools::checkFilename(opt_intrinsic_filename)) {
    std::cout << "Camera parameters file " << opt_intrinsic_filename << " doesn't exist." << std::endl;
    return EXIT_FAILURE;
  }
  else {
    vpXmlParserCamera parser;
    if (!opt_camera_name.empty()) {

      std::cout << "    Param file name [.xml]: " << opt_intrinsic_filename << std::endl;
      std::cout << "    Camera name           : " << opt_camera_name << std::endl;

      if (parser.parse(cam, opt_intrinsic_filename, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) !=
        vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to parse parameters with distortion for camera \"" << opt_camera_name << "\" from "
          << opt_intrinsic_filename << " file" << std::endl;
        std::cout << "Attempt to find parameters without distortion" << std::endl;

        if (parser.parse(cam, opt_intrinsic_filename, opt_camera_name,
                         vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
          std::cout << "Unable to parse parameters without distortion for camera \"" << opt_camera_name << "\" from "
            << opt_intrinsic_filename << " file" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
  }

  std::cout << "Camera parameters used to compute the pose:\n" << cam << std::endl;

  // Setup Apriltag detector
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
#if defined(VISP_HAVE_APRILTAG_EXTENDED_API)
  vpDetectorAprilTag::vpPoseEstimationMethod pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
#else
  vpDetectorAprilTag::vpPoseEstimationMethod pose_estimation_method = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
#endif
  vpDetectorAprilTag detector(tagFamily);
  detector.setAprilTagPoseEstimationMethod(pose_estimation_method);
  detector.setDisplayTag(display_tag);
  detector.setAprilTagQuadDecimate(opt_quad_decimate);
  detector.setZAlignedWithCameraAxis(opt_tag_z_aligned);

  // Define 4 3D points corresponding to the CAD model of the Apriltag
  std::vector<vpPoint> point(4);
  point[0].setWorldCoordinates(-opt_tag_size / 2., -opt_tag_size / 2., 0);
  point[1].setWorldCoordinates(+opt_tag_size / 2., -opt_tag_size / 2., 0);
  point[2].setWorldCoordinates(+opt_tag_size / 2., +opt_tag_size / 2., 0);
  point[3].setWorldCoordinates(-opt_tag_size / 2., +opt_tag_size / 2., 0);

  if (opt_learn_desired_features) {

    bool quit = false;
    while (!quit) {
      rs.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> c_M_o_vec;
      // To learn the desired visual features (x*, y*, Z*) we will use the tag pose to estimate Z*
      bool ret = detector.detect(I, opt_tag_size, cam, c_M_o_vec);

      vpDisplay::displayText(I, 20, 20, "Move the robot to the desired tag pose...", vpColor::red);
      vpDisplay::displayText(I, 40, 20, "Left click to learn desired features, right click to quit", vpColor::red);

      std::vector< std::vector<vpImagePoint> > tags_corners;
      if (ret) {
        if (detector.getNbObjects() == 1) {
          tags_corners = detector.getTagsCorners();
          for (size_t i = 0; i < 4; ++i) {
            std::stringstream ss;
            ss << i;
            vpDisplay::displayText(I, tags_corners[0][i]+vpImagePoint(15, -15), ss.str(), vpColor::red);
            vpDisplay::displayCross(I, tags_corners[0][i], 15, vpColor::red, 2);
          }
        }
      }

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        switch (button) {
        case vpMouseButton::button1:
          if (ret) {
            if (detector.getNbObjects() == 1) {
              vpHomogeneousMatrix cd_M_o = c_M_o_vec[0];
              std::vector<vpFeaturePoint> p_d(4);

              // Update x*, y* for each desired visual feature
              for (size_t i = 0; i < 4; ++i) {
                double x = 0, y = 0;
                vpPixelMeterConversion::convertPoint(cam, tags_corners[0][i], x, y);
                std::stringstream ss;
                ss << i;
                vpDisplay::displayText(I, tags_corners[0][i]+vpImagePoint(10, 10), ss.str(), vpColor::red);
                p_d[i].set_x(x);
                p_d[i].set_y(y);
              }

              // Update Z* for each desired visual feature
              for (size_t i = 0; i < point.size(); ++i) {
                vpColVector c_P, p;
                point[i].changeFrame(cd_M_o, c_P);
                p_d[i].set_Z(c_P[2]);
              }
              if (save_desired_features(desired_features_filename, p_d)) {
                std::cout << "Desired visual features saved in: " << desired_features_filename << std::endl;
              }
              else {
                std::cout << "Error: Unable to save desired features in " << desired_features_filename << std::endl;
                return EXIT_FAILURE;
              }
            }
            else {
              std::cout << "Cannot save desired features. More than 1 tag is visible in the image..." << std::endl;
            }
          }
          else {
            std::cout << "Cannot save desired features. Tag is not visible in the image..." << std::endl;
          }
          break;
        case vpMouseButton::button3:
          quit = true;
          break;
        default:
          break;
        }
      }

      vpDisplay::flush(I);
    }
    return EXIT_SUCCESS;
  }

  // Load desired features to reach by visual servo

  std::vector<vpFeaturePoint> p_d; // Desired visual features
  // Sanity options check
  if (desired_features_filename.empty() || (!vpIoTools::checkFilename(desired_features_filename))) {
    std::cout << "Cannot start eye-to-hand visual-servoing. Desired features are not available." << std::endl;
    std::cout << "use --learn-desired-features flag to learn the desired features." << std::endl;
    return EXIT_FAILURE;
  }
  else {
    if (!read_desired_features(desired_features_filename, p_d)) {
      std::cout << "Error: Unable to read desired features from: " << desired_features_filename << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Get camera extrinsics
  vpPoseVector r_P_c;

  // Read camera extrinsics from --eMo <file>
  if (!opt_rMc_filename.empty()) {
    r_P_c.loadYAML(opt_rMc_filename, r_P_c);
  }
  else {
    std::cout << "Warning, rMc transformation is not specified using --rMc parameter." << std::endl;
    return EXIT_FAILURE;
  }
  vpHomogeneousMatrix r_M_c(r_P_c);
  std::cout << "r_M_c:\n" << r_M_c << std::endl;

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    // Create current visual features
    std::vector<vpFeaturePoint> p(4); // We use 4 points

    vpServo task;
    // Add the 4 visual feature points
    for (size_t i = 0; i < p.size(); ++i) {
      task.addFeature(p[i], p_d[i]);
    }
    task.setServo(vpServo::EYETOHAND_L_cVf_fVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT);

    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1, 0.4, 30); // lambda(0)=1, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(0.2);
    }

    // Set the camera to robot reference frame velocity twist matrix constant transformation
    task.set_cVf(r_M_c.inverse());

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Joint velocities");
      plotter->initGraph(0, 8);
      plotter->initGraph(1, 7);
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "q_1");
      plotter->setLegend(1, 1, "q_2");
      plotter->setLegend(1, 2, "q_3");
      plotter->setLegend(1, 3, "q_4");
      plotter->setLegend(1, 4, "q_5");
      plotter->setLegend(1, 5, "q_6");
      plotter->setLegend(1, 6, "q_7");
    }

    bool final_quit = false;
    bool has_converged = false;
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

    static double t_init_servo = vpTime::measureTimeMs();

    //robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> c_M_o_vec;
      // To update the current visual feature (x,y,Z) parameters, we need the tag pose to estimate Z
      bool ret = detector.detect(I, opt_tag_size, cam, c_M_o_vec);

      {
        std::stringstream ss;
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
      }

      vpColVector qdot(robot.getNDof());

      // Only one tag is detected
      if (ret && detector.getNbObjects() == 1) {
        static bool first_time = true;
        vpHomogeneousMatrix c_M_o = c_M_o_vec[0];

        // Get tag corners
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        // Update visual features
        for (size_t i = 0; i < corners.size(); ++i) {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create(p[i], cam, corners[i]);
          // Set the feature Z coordinate from the pose
          vpColVector c_P;
          point[i].changeFrame(c_M_o, c_P);

          p[i].set_Z(c_P[2]); // Update Z value of each visual feature thanks to the tag pose
        }

        // Set the robot reference frame to end-effector frame velocity twist matrix transformation
        vpPoseVector r_P_e;
        robot.getPosition(vpRobot::END_EFFECTOR_FRAME, r_P_e);
        vpHomogeneousMatrix r_M_e(r_P_e);
        task.set_fVe(r_M_e);

        // Set the Jacobian (expressed in the end-effector frame)
        vpMatrix e_J_e;
        robot.get_eJe(e_J_e);
        task.set_eJe(e_J_e);

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
        vpServoDisplay::display(task, cam, I);
        for (size_t i = 0; i < corners.size(); ++i) {
          std::stringstream ss;
          ss << i;
          // Display current point indexes
          vpDisplay::displayText(I, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
          // Display desired point indexes
          vpImagePoint ip;
          vpMeterPixelConversion::convertPoint(cam, p_d[i].get_x(), p_d[i].get_y(), ip);
          vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        }
        if (first_time) {
          traj_corners = new std::vector<vpImagePoint>[corners.size()];
        }
        // Display the trajectory of the points used as features
        display_point_trajectory(I, corners, traj_corners);

        if (opt_plot) {
          plotter->plot(0, iter_plot, task.getError());
          plotter->plot(1, iter_plot, qdot);
          iter_plot++;
        }

        if (opt_verbose) {
          std::cout << "qdot: " << qdot.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        std::stringstream ss;
        ss << "error: " << error;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error: " << error << std::endl;

        if (error < convergence_threshold) {
          has_converged = true;
          std::cout << "Servo task has converged" << std::endl;
          vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }
        if (first_time) {
          first_time = false;
        }
      } // end if (c_M_o_vec.size() == 1)
      else {
        qdot = 0;
      }

      if (!send_velocities) {
        qdot = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::JOINT_STATE, qdot);

      {
        std::stringstream ss;
        ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      }
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
        rs.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
        vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
    if (traj_corners) {
      delete[] traj_corners;
    }
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }
  catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
      << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
      << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x and rebuild ViSP." << std::endl;
#endif
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka and rebuild ViSP." << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "Build ViSP with pugixml support enabled." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
