/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
   \example servoAfma6MegaposePBVS.cpp
   Example of eye-in-hand image-based control law. We control here the Afma6 robot
   at Inria. The velocity is computed in the camera frame. Visual features
   correspond to the 3D pose of the target (an AprilTag) in the camera frame.

   The device used to acquire images is a Realsense D435 device.

   Camera intrinsic parameters are retrieved from the Realsense SDK.

   The target is an object of which we have the 3D model (in .obj format). We use MegaPose to estimate the object pose in the camera frame,
   which we plug into the Pose-Based control law.
 */

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/io/vpJsonArgumentParser.h>
#include <optional>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) &&                                    \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_MODULE_DNN_TRACKER)


void overlayRender(vpImage<vpRGBa> &I, const vpImage<vpRGBa> &overlay, bool contour)
{
  if (!contour) {
    const vpRGBa black = vpRGBa(0, 0, 0);
    for (unsigned int i = 0; i < I.getHeight(); ++i) {
      for (unsigned int j = 0; j < I.getWidth(); ++j) {
        if (overlay[i][j] != black) {
          I[i][j] = overlay[i][j];
        }
      }
    }
  }
  else {
    vpImage<unsigned char> overlayGray, canny;
    vpImageConvert::convert(overlay, overlayGray);
    vpImageFilter::canny(overlayGray, canny, 3, 30, 3);
    for (unsigned int i = 0; i < I.getHeight(); ++i) {
      for (unsigned int j = 0; j < I.getWidth(); ++j) {
        if (canny[i][j] > 0) {
          I[i][j] = vpColor::green;
        }
      }
    }
  }
}

std::optional<vpRect> detectObjectForInitMegaposeClick(const vpImage<vpRGBa> &I)
{
  const bool startLabelling = vpDisplay::getClick(I, false);

  const vpImagePoint textPosition(10.0, 20.0);

  if (startLabelling) {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(I, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, topLeft, true);
    vpDisplay::display(I);
    vpDisplay::displayCross(I, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(I, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else {
    vpDisplay::display(I);
    vpDisplay::displayText(I, textPosition, "Click when the object is visible and static to start reinitializing megapose.", vpColor::red);
    vpDisplay::flush(I);
    return std::nullopt;
  }
}

int main(int argc, char **argv)
{
  bool opt_verbose = false;
  double convergence_threshold_t = 0.0005; // Value in [m]
  double convergence_threshold_tu = 0.5;   // Value in [deg]

  unsigned width = 640, height = 480;
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  int refinerIterations = 1, coarseNumSamples = 576;
  std::string objectName = "cube";

  std::string desiredPosFile = "cdTw.pos";
  std::string initialPosFile = "cTw.pos";


  vpRobotAfma6 robot;

  try {

    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /*
     * Move to a safe position
     */
    vpColVector q(6, 0);

    // Go to desired pose, save true camera pose wrt world frame
    robot.setPositioningVelocity(10.0); // In %
    robot.readPosFile(desiredPosFile, q);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
    std::cout << "Move to joint position: " << q.t() << std::endl;
    vpHomogeneousMatrix cdTw = robot.get_fMc(q).inverse();


    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, 60);
    rs.open(config);

    // Get camera intrinsics
    vpCameraParameters cam =
      rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    std::cout << "cam:\n" << cam << "\n";

    std::shared_ptr<vpMegaPose> megapose;
    try {
      megapose = std::make_shared<vpMegaPose>(megaposeAddress, megaposePort, cam, height, width);
    }
    catch (...) {
      throw vpException(vpException::ioError, "Could not connect to Megapose server at " + megaposeAddress + " on port " + std::to_string(megaposePort));
    }

    vpMegaPoseTracker megaposeTracker(megapose, objectName, refinerIterations);
    megapose->setCoarseNumSamples(coarseNumSamples);
    const std::vector<std::string> allObjects = megapose->getObjectNames();
    if (std::find(allObjects.begin(), allObjects.end(), objectName) == allObjects.end()) {
      throw vpException(vpException::badValue, "Object " + objectName + " is not known by the Megapose server!");
    }
    std::future<vpMegaPoseEstimate> trackerFuture;

    vpImage<vpRGBa> I(height, width);

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    std::optional<vpRect> detection;
    while (!detection) {
      rs.acquire(I);
      vpDisplay::display(I);
      detection = detectObjectForInitMegaposeClick(I);
      vpDisplay::flush(I);
    }

    vpHomogeneousMatrix cdTo = megaposeTracker.init(I, detection).get().cTo; //get camera pose relative to object, not world


    // Go to desired pose, save true desired pose
    robot.readPosFile(initialPosFile, q);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
    std::cout << "Move to joint position: " << q.t() << std::endl;
    vpHomogeneousMatrix cTw = robot.get_fMc(q).inverse();

    vpHomogeneousMatrix cdTc_true = cdTw * cTw.inverse();



    detection = std::nullopt;
    while (!detection) {
      rs.acquire(I);
      vpDisplay::display(I);
      detection = detectObjectForInitMegaposeClick(I);
      vpDisplay::flush(I);
    }
    vpHomogeneousMatrix cTo = megaposeTracker.init(I, detection).get().cTo;

    cdTc = cdTo * cTo.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.buildFrom(cdTc);
    tu.buildFrom(cdTc);

    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);

    vpServo task;
    task.addFeature(t, td);
    task.addFeature(tu, tud);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.05);

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
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

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    vpColVector vLastUpdate(6);

    vpHomogeneousMatrix prev_cTo = cTo;

    vpColVector v(6);

    bool callMegapose = true;
    bool updatedThisIter = true;
    double timeLastUpdate = vpTime::measureMs();

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);
      vpDisplay::display(I);
      if (!callMegapose && trackerFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        vpMegaPoseEstimate megaposeEstimate = trackerFuture.get();

        cTo = megaposeEstimate.cTo;        
        callMegapose = true;
        updateThisIter = true;
        timeLastUpdate = vpTime::measureMs();

        if (megaposeEstimate.score < reinitThreshold) { // If confidence is low, exit
          final_quit = true;
        }
      }

      if(callMegapose) {
        megaposeEstimate = megaposeTracker.track(I);
        callMegapose = false;
      }

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);




      // Update visual features

      cdTc = cdTo * cTo.inverse();
      t.buildFrom(cdTc);
      tu.buildFrom(cdTc);
      v_c = task.computeControlLaw();


      // Update true pose
      cTw = robot.get_fMc(q).inverse();
      cdTc_true = cdTw * cTw.inverse();

      updatedThisIter = false;

      // Display desired and current pose features
      vpDisplay::displayFrame(I, cdTo, cam, 5, vpColor::yellow, 2);
      vpDisplay::displayFrame(I, cTo, cam, 5, vpColor::none, 3);
      // Get tag corners
      
      if (opt_plot) {
        plotter->plot(0, iter_plot, task.getError());
        plotter->plot(1, iter_plot, v);
        iter_plot++;
      }

      if (opt_verbose) {
        std::cout << "v_c: " << v_c.t() << std::endl;
      }

      vpTranslationVector cd_t_c = cdTc.getTranslationVector();
      vpThetaUVector cd_tu_c = cdTc.getThetaUVector();
      double error_tr = sqrt(cd_t_c.sumSquare());
      double error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));
      vpTranslationVector cd_t_c_true = cdTc_true.getTranslationVector();
      vpThetaUVector cd_tu_c_true = cdTc_true.getThetaUVector();
      double error_tr_true = sqrt(cd_t_c_true.sumSquare());
      double error_tu_true = vpMath::deg(sqrt(cd_tu_c_true.sumSquare()));

      ss.str("");
      ss << "Predicted error_t: " << error_tr << ", True error_t:" << error_tr_true;
      vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
      ss.str("");
      ss << "Predicted error_tu: " << error_tu << ", True error_tu:" << error_tu_true;
      vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

      if (opt_verbose)
        std::cout << "error translation: " << error_tr << " ; error rotation: " << error_tu << std::endl;

      if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
        has_converged = true;
        std::cout << "Servo task has converged" << std::endl;
        vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
      }

      if (first_time) {
        first_time = false;
      }


      // Send to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

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
          v = 0;
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

    
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "ur_rtde exception: " << e.what() << std::endl;
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
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
#endif
#if !defined(VISP_HAVE_AFMA6)
  std::cout << "ViSP is not build with Afma-6 robot support..." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
