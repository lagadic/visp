/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
  \example servoAfma6AprilTagIBVS.cpp

  Example of eye-in-hand image-based control law. We control here the Afma6 robot
  at Inria. The velocity is computed in the camera frame. Visual features
  are the image coordinates of 4 points corresponding to the corners of an AprilTag.

  The device used to acquire images is a Realsense D435 device.

  Camera intrinsic parameters are retrieved from the Realsense SDK.

  The target is an AprilTag that is by default 12cm large. To print your own tag, see
  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-detection-apriltag.html
  You can specify the size of your tag using --tag-size command line option.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_AFMA6)

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void display_point_trajectory(const vpImage<unsigned char> &I,
                              const std::vector<vpImagePoint> &vip,
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
    for (size_t j = 1; j < traj_vip[i].size(); ++j) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

int main(int argc, char **argv)
{
  double opt_tagSize = 0.120;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double opt_convergence_threshold = 0.00005;

  bool display_tag = true;

  for (int i = 1; i < argc; ++i) {
    if ((std::string(argv[i]) == "--tag-size") && (i + 1 < argc)) {
      opt_tagSize = std::stod(argv[i + 1]);
      ++i;
    }
    else if (std::string(argv[i]) == "--verbose") {
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
    else if ((std::string(argv[i]) == "--quad-decimate") && (i + 1 < argc)) {
      opt_quad_decimate = std::stoi(argv[i + 1]);
      ++i;
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      opt_convergence_threshold = 0.;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout
        << argv[0]
        << " [--tag-size <marker size in meter; default " << opt_tagSize << ">]"
        << " [--quad-decimate <decimation; default " << opt_quad_decimate << ">]"
        << " [--adaptive-gain]"
        << " [--plot]"
        << " [--task-sequencing]"
        << " [--no-convergence-threshold]"
        << " [--verbose]"
        << " [--help] [-h]"
        << std::endl;;
      return EXIT_SUCCESS;
    }
  }

  vpRobotAfma6 robot;
  vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

  // Load the end-effector to camera frame transformation obtained
  // using a camera intrinsic model with distortion
  robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

  try {
    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480, fps = 60;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);

    // Warm up camera
    vpImage<unsigned char> I;
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);
    std::cout << "cam:\n" << cam << std::endl;

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    detector.setZAlignedWithCameraAxis(true);

    // Tag frame for pose estimation is the following
    // - When using
    //   detector.setZAlignedWithCameraAxis(false);
    //   detector.detect();
    //   we consider the tag frame (o) such as z_o axis is not aligned with camera frame
    //   (meaning z_o axis is facing the camera)
    //
    //   3    y    2
    //        |
    //        o--x
    //
    //   0         1
    //
    //   In that configuration, it is more difficult to set a desired camera pose c_M_o.
    //   To ease this step we can introduce an extra transformation matrix o'_M_o to align the axis
    //   with the camera frame:
    //
    //         o'--x
    //         |
    //         y
    //
    //   where
    //            | 1  0  0  0 |
    //   o'_M_o = | 0 -1  0  0 |
    //            | 0  0 -1  0 |
    //            | 0  0  0  1 |
    //
    //   Defining the desired camera pose in frame o' becomes than easier.
    //
    // - When using rather
    //   detector.setZAlignedWithCameraAxis(true);
    //   detector.detect();
    //   we consider the tag frame (o) such as z_o axis is aligned with camera frame
    //
    //   3         2
    //
    //        o--x
    //        |
    //   0    y    1
    //
    //   In that configuration, it is easier to define a desired camera pose c_M_o since all the axis
    //   (camera frame and tag frame are aligned)

    // Servo
    vpHomogeneousMatrix cd_M_c, c_M_o, o_M_o;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cd_M_o(vpTranslationVector(0, 0, opt_tagSize * 3.5), // 3.5 times tag with along camera z axis
                               vpThetaUVector(vpMath::rad(10), vpMath::rad(3), vpMath::rad(5)));
    if (!detector.isZAlignedWithCameraAxis()) {
      vpHomogeneousMatrix oprim_M_o = { 1,  0,  0, 0,
                                        0, -1,  0, 0,
                                        0,  0, -1, 0,
                                        0,  0,  0, 1 };
      cd_M_o *= oprim_M_o;
    }
    // Create visual features
    std::vector<vpFeaturePoint> s_point(4), s_point_d(4); // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector<vpPoint> point(4);
    if (detector.isZAlignedWithCameraAxis()) {
      point[0].setWorldCoordinates(-opt_tagSize / 2., +opt_tagSize / 2., 0);
      point[1].setWorldCoordinates(+opt_tagSize / 2., +opt_tagSize / 2., 0);
      point[2].setWorldCoordinates(+opt_tagSize / 2., -opt_tagSize / 2., 0);
      point[3].setWorldCoordinates(-opt_tagSize / 2., -opt_tagSize / 2., 0);
    }
    else {
      point[0].setWorldCoordinates(-opt_tagSize / 2., -opt_tagSize / 2., 0);
      point[1].setWorldCoordinates(+opt_tagSize / 2., -opt_tagSize / 2., 0);
      point[2].setWorldCoordinates(+opt_tagSize / 2., +opt_tagSize / 2., 0);
      point[3].setWorldCoordinates(-opt_tagSize / 2., +opt_tagSize / 2., 0);
    }

    vpServo task;
    // Add the 4 visual feature points
    for (size_t i = 0; i < s_point.size(); ++i) {
      task.addFeature(s_point[i], s_point_d[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    // Set the gain
    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(0.2);
    }

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->initGraph(0, 8);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
    }

    bool final_quit = false;
    bool has_converged = false;
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

    static double t_init_servo = vpTime::measureTimeMs();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> c_M_o_vec;
      detector.detect(I, opt_tagSize, cam, c_M_o_vec);

      {
        std::stringstream ss;
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
      }

      vpColVector v_c(6);

      // Ensure that only one tag is detected during servoing
      if (c_M_o_vec.size() == 1) {
        c_M_o = c_M_o_vec[0];

        static bool first_time = true;
        if (first_time) {
          // Introduce security wrt tag positioning in order to avoid PI rotation
          std::vector<vpHomogeneousMatrix> v_o_M_o(2), v_cd_M_c(2);
          v_o_M_o[1].buildFrom(0, 0, 0, 0, 0, M_PI);
          for (size_t i = 0; i < 2; ++i) {
            v_cd_M_c[i] = cd_M_o * v_o_M_o[i] * c_M_o.inverse();
          }
          if (std::fabs(v_cd_M_c[0].getThetaUVector().getTheta()) < std::fabs(v_cd_M_c[1].getThetaUVector().getTheta())) {
            o_M_o = v_o_M_o[0];
          }
          else {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            o_M_o = v_o_M_o[1]; // Introduce PI rotation
          }

          // Compute the desired position of the features from the desired pose
          for (size_t i = 0; i < point.size(); ++i) {
            vpColVector cP, p;
            point[i].changeFrame(cd_M_o * o_M_o, cP);
            point[i].projection(cP, p);

            s_point_d[i].set_x(p[0]);
            s_point_d[i].set_y(p[1]);
            s_point_d[i].set_Z(cP[2]);
          }
        }

        // Get tag corners
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        // Update visual features
        for (size_t i = 0; i < corners.size(); ++i) {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create(s_point[i], cam, corners[i]);
          // Set the feature Z coordinate from the pose
          vpColVector c_P;
          point[i].changeFrame(c_M_o, c_P);

          s_point[i].set_Z(c_P[2]);
        }

        if (opt_task_sequencing) {
          if (!servo_started) {
            if (send_velocities) {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
        }
        else {
          v_c = task.computeControlLaw();
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
          vpMeterPixelConversion::convertPoint(cam, s_point_d[i].get_x(), s_point_d[i].get_y(), ip);
          vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        }
        if (first_time) {
          traj_corners = new std::vector<vpImagePoint>[corners.size()];
        }
        // Display the trajectory of the points used as features
        display_point_trajectory(I, corners, traj_corners);

        if (opt_plot) {
          plotter->plot(0, iter_plot, task.getError());
          plotter->plot(1, iter_plot, v_c);
          iter_plot++;
        }

        if (opt_verbose) {
          std::cout << "v_c: " << v_c.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        std::stringstream ss;
        ss << "error: " << error;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error: " << error << std::endl;

        if (error < opt_convergence_threshold) {
          has_converged = true;
          std::cout << "Servo task has converged" << std::endl;
          vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }
        if (first_time) {
          first_time = false;
        }
      } // end if (c_M_o_vec.size() == 1)
      else {
        v_c = 0;
      }

      if (!send_velocities) {
        v_c = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);

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
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x" << std::endl;
#endif
#if !defined(VISP_HAVE_AFMA6)
  std::cout << "ViSP is not build with Afma6 robot support..." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
