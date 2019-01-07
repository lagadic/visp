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
 * Data acquisition with RealSense RGB-D sensor and Franka robot.
 *
 *****************************************************************************/

/*!
  \example servoFrankaPBVS.cpp
  This example shows how to achieve a position-based visual servoing with a RealSense RGB-D sensor attached
  to a Panda robot from Franka Emika.
*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/gui/vpPlot.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_CPP11_COMPATIBILITY) &&                                         \
  (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_FRANKA)

int main(int argc, char **argv)
{
  double opt_tagSize = 0.120;
  std::string opt_robot_ip = "192.168.1.1";
  std::string opt_eMc_filename = "";
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      opt_tagSize = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      opt_robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--eMc" && i + 1 < argc) {
      opt_eMc_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--verbose") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--adaptive_gain") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--task_sequencing") {
      opt_task_sequencing = true;
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      opt_quad_decimate = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--tag_size <marker size in meter; default " << opt_tagSize << ">] [--eMc <eMc extrinsic file>] "
                           << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] [--adaptive_gain] [--plot] [--task_sequencing] [--verbose] [--help] [-h]"
                           << "\n";
      return EXIT_SUCCESS;
    }
  }

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    rs.open(config);

    // Get camera extrinsics
    vpPoseVector ePc;
    ePc[0] = 0.0337731; ePc[1] = -0.00535012; ePc[2] = -0.0523339;
    ePc[3] = -0.247294; ePc[4] = -0.306729; ePc[5] = 1.53055;

    if (!opt_eMc_filename.empty()) {
      ePc.loadYAML(opt_eMc_filename, ePc);
    }
    else {
      std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << "\n";
    }
    vpHomogeneousMatrix eMc(ePc);
    std::cout << "eMc:\n" << eMc << "\n";

    // Get camera intrinsics
    vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << "cam:\n" << cam << "\n";

    vpImage<unsigned char> I(height, width);

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    // Servo
    vpHomogeneousMatrix cdMc, cMo, cdMo, oMo;

    // Desired pose to reach
    cdMo[0][0] = 1; cdMo[0][1] =  0; cdMo[0][2] =  0;
    cdMo[1][0] = 0; cdMo[1][1] = -1; cdMo[1][2] =  0;
    cdMo[2][0] = 0; cdMo[2][1] =  0; cdMo[2][2] = -1;
    cdMo[0][3] = 0;
    cdMo[1][3] = 0;
    cdMo[2][3] = 0.3; // 30 cm along camera z axis

    cdMc = cdMo * cMo.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc);

    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);

    vpServo task;
    task.addFeature(t, td);
    task.addFeature(tu, tud);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(0.5);
    }

    vpPlot *plotter = NULL;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, 250 * 2, 500, I.getWidth() + 80, 10, "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->initGraph(0, 6);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_tx");
      plotter->setLegend(0, 1, "error_feat_ty");
      plotter->setLegend(0, 2, "error_feat_tz");
      plotter->setLegend(0, 3, "error_feat_theta_ux");
      plotter->setLegend(0, 4, "error_feat_theta_uy");
      plotter->setLegend(0, 5, "error_feat_theta_uz");
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

    static double t_init_servo = vpTime::measureTimeMs();

    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, opt_tagSize, cam, cMo_vec);

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      vpColVector v_c(6);

      // Only one tag is detected
      if (cMo_vec.size() == 1) {
        cMo = cMo_vec[0];

        static bool first_time = true;
        if (first_time) {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
          v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
          for (size_t i = 0; i < 2; i++) {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
          if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
            oMo = v_oMo[0];
          }
          else {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo = v_oMo[1];   // Introduce PI rotation
          }

          first_time = false;
        }
        // Display desired and current pose
        vpDisplay::displayFrame(I, cdMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2);
        vpDisplay::displayFrame(I, cMo,  cam, opt_tagSize / 2,   vpColor::none,   3);

        // VVS
        cdMc = cdMo * oMo * cMo.inverse();
        t.buildFrom(cdMc);
        tu.buildFrom(cdMc);

        if (opt_task_sequencing) {
          if (! servo_started) {
            if (send_velocities) {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo)/1000.);
        }
        else {
          v_c = task.computeControlLaw();
        }

        if (opt_plot) {
          plotter->plot(0, iter_plot, task.getError());
          plotter->plot(1, iter_plot, v_c);
          iter_plot++;
        }

        if (opt_verbose) {
          std::cout << "v_c: " << v_c.t() << std::endl;
        }

        vpTranslationVector cd_t_c = cdMc.getTranslationVector();
        vpThetaUVector cd_tu_c = cdMc.getThetaUVector();
        double error_t = sqrt(cd_t_c.sumSquare());
        double error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));

        ss.str("");
        ss << "error_t: " << error_t;
        vpDisplay::displayText(I, 20, I.getWidth() - 150, ss.str(), vpColor::red);
        ss.str("");
        ss << "error_tu: " << error_tu;
        vpDisplay::displayText(I, 40, I.getWidth() - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error translation: " << error_t << " ; error rotation: " << error_tu << "\n";

        if (error_t < convergence_threshold_t && error_tu < convergence_threshold_tu) {
          has_converged = true;
          std::cout << "Servo task has converged" << "\n";
          vpDisplay::displayText(I, 100, 20, "Servo task has converged, wait for robot go to init position", vpColor::red);
        }
      } //if (cMo_vec.size() == 1)
      else {
        v_c = 0;
      }

      if (!send_velocities) {
        v_c = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);

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
          v_c = 0;
          break;

        default:
          break;
        }
      }     
    }
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);

    if (opt_plot && plotter != NULL) {
      delete plotter;
      plotter = NULL;
    }

    task.kill();

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
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x" << std::endl;
#endif
#if !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "Build ViSP with C++11 compiler flag (cmake -DUSE_CPP11=ON)." << std::endl;
#endif
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka." << std::endl;
#endif
  return 0;
}
#endif
