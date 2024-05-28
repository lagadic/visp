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
 * Pose-based visual servoing using MegaPose, on an Afma6 platform.
 *
 *****************************************************************************/

 /*!
   \example servoAfma6MegaposePBVS.cpp
   Example of eye-in-hand pose-based control law. We control here the Afma6 robot
   at Inria. The velocity is computed in the camera frame. Visual features
   correspond to the 3D pose of the target (a known of object, for which we have the 3D model) in the camera frame.

   The device used to acquire images is a Realsense D435 device. Camera intrinsic parameters are retrieved from the Realsense SDK.

   The target is an object for which we have the 3D model (in .obj format). We use MegaPose to estimate the object pose in the camera frame,
   which we plug into the Pose-Based control law.

  To install and use megapose, see \ref tutorial-tracking-megapose.

  This example was used to validate Megapose: as such, we provide the initial and desired poses in world frame and use megapose to match them with the object's pose in the camera at the initial and desired locations.
  Thus, this example takes as input two pose files, acquired with Afma6_office, where the poses are expressed in the world frame. The robot is then moved to these poses and Megapose is used to estimate the object pose in the camera frames. The object detection in the image is performed by click.
  This allows to compare the ground truth pose error (computed in world frame) with the one estimated thanks to megapose.

  In a more practical example, the desired pose would directly be given in the camera frame (as used by megapose) and the robot would thus not need to move to the desired pose before actually servoing.

  To start this example enter:
  \code
  $ ./servoAfma6MegaposePBVS initialPose init.pos desiredPose desired.pos object myObjectName megapose/address 127.0.0.1 megapose/port 5555
  \endcode
  where init.pos and desired.pos are files obtained through Afma6_office, and myObjectName is the name of an object known by the megapose server.

 */

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
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
#include <visp3/core/vpImageFilter.h>
#include <visp3/io/vpVideoWriter.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_REALSENSE2) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_MODULE_DNN_TRACKER)

#include <optional>
#include <visp3/io/vpJsonArgumentParser.h>
#include <visp3/dnn_tracker/vpMegaPoseTracker.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
using json = nlohmann::json; //! json namespace shortcut
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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

int main(int argc, const char *argv[])
{
  bool opt_verbose = true;
  bool opt_plot = true;
  double convergence_threshold_t = 0.0005; // Value in [m]
  double convergence_threshold_tu = 0.5;   // Value in [deg]

  unsigned width = 640, height = 480;
  std::string megaposeAddress = "127.0.0.1";
  unsigned megaposePort = 5555;
  int refinerIterations = 1, coarseNumSamples = 1024;
  std::string objectName = "";

  std::string desiredPosFile = "desired.pos";
  std::string initialPosFile = "init.pos";

#ifdef VISP_HAVE_NLOHMANN_JSON
  vpJsonArgumentParser parser("Pose-based visual servoing with Megapose on an Afma6, with a Realsense D435.", "--config", "/");
  parser
    .addArgument("initialPose", initialPosFile, true, "Path to the file that contains that the desired pose. Can be acquired with Afma6_office.")
    .addArgument("desiredPose", desiredPosFile, true, "Path to the file that contains that the desired pose. Can be acquired with Afma6_office.")
    .addArgument("object", objectName, true, "Name of the object to track with megapose.")
    .addArgument("megapose/address", megaposeAddress, true, "IP address of the Megapose server.")
    .addArgument("megapose/port", megaposePort, true, "Port on which the Megapose server listens for connections.")
    .addArgument("megapose/refinerIterations", refinerIterations, false, "Number of Megapose refiner model iterations."
                 "A higher count may lead to better accuracy, at the cost of more processing time")
    .addArgument("megapose/initialisationNumSamples", coarseNumSamples, false, "Number of Megapose renderings used for the initial pose estimation.");
  parser.parse(argc, argv);
#endif

  vpRobotAfma6 robot;

  try {
    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::vector<vpColVector> velocities;
    std::vector<vpPoseVector> error;
    /*
     * Move to a safe position
     */
    vpColVector q(6, 0);

    vpVideoWriter writer;

    // Go to desired pose, save true camera pose wrt world frame
    robot.setPositioningVelocity(10.0); // In %
    robot.readPosFile(desiredPosFile, q);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
    std::cout << "Move to joint position: " << q.t() << std::endl;
    vpHomogeneousMatrix cdTw = robot.get_fMc(q).inverse();

    // Setup camera
    vpRealSense2 rs;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, 30);
    rs.open(config);

    // Get camera intrinsics
    vpCameraParameters cam =
      rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    std::cout << "cam:\n" << cam << "\n";
    // Initialize Megapose
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

    vpHomogeneousMatrix cdTo = megaposeTracker.init(I, *detection).get().cTo; //get camera pose relative to object, not world

    // Go to starting pose, save true starting pose in world frame
    robot.readPosFile(initialPosFile, q);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
    std::cout << "Move to joint position: " << q.t() << std::endl;
    vpHomogeneousMatrix cTw = robot.get_fMc(q).inverse();
    vpHomogeneousMatrix cdTc_true = cdTw * cTw.inverse(); // ground truth error

    detection = std::nullopt;
    while (!detection) {
      rs.acquire(I);
      vpDisplay::display(I);
      detection = detectObjectForInitMegaposeClick(I);
      vpDisplay::flush(I);
    }
    auto est = megaposeTracker.init(I, *detection).get();
    vpHomogeneousMatrix cTo = est.cTo;
    std::cout << "Estimate score = " << est.score << std::endl;
    writer.setFileName("video/I%05d.png");
    //writer.setFramerate(60.0);
    writer.open(I);

    //vpHomogeneousMatrix oTw = cTo.inverse() * cTw;
    vpHomogeneousMatrix cdTc = cdTo * cTo.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.build(cdTc);
    tu.build(cdTc);

    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);

    vpServo task;
    task.addFeature(t, td);
    task.addFeature(tu, tud);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.2);

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

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    vpColVector vLastUpdate(6);

    vpHomogeneousMatrix prev_cTo = cTo;

    vpColVector v(6);

    bool callMegapose = true;
    vpMegaPoseEstimate  megaposeEstimate;

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);
      vpDisplay::display(I);
      if (!callMegapose && trackerFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        megaposeEstimate = trackerFuture.get();

        cTo = megaposeEstimate.cTo;
        callMegapose = true;
        if (megaposeEstimate.score < 0.2) { // If confidence is low, exit
          final_quit = true;
          std::cout << "Low confidence, exiting" << std::endl;
        }
      }

      if (callMegapose) {
        std::cout << "Calling megapose" << std::endl;
        trackerFuture = megaposeTracker.track(I);
        callMegapose = false;
      }

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      // Update visual features
      cdTc = cdTo * cTo.inverse();
      t.build(cdTc);
      tu.build(cdTc);
      v = task.computeControlLaw();
      velocities.push_back(v);

      // Update true pose
      vpPoseVector p;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
      cTw = robot.get_fMc(q).inverse();
      cdTc_true = cdTw * cTw.inverse();
      vpPoseVector cdrc(cdTc_true);
      error.push_back(cdrc);

      // Display desired and current pose features
      vpDisplay::displayFrame(I, cdTo, cam, 0.05, vpColor::yellow, 2);
      vpDisplay::displayFrame(I, cTo, cam, 0.05, vpColor::none, 3);

      if (opt_plot) {
        plotter->plot(0, iter_plot, task.getError());
        plotter->plot(1, iter_plot, v);
        iter_plot++;
      }

      if (opt_verbose) {
        std::cout << "v: " << v.t() << std::endl;
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
      vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 300, ss.str(), vpColor::red);
      ss.str("");
      ss << "Predicted error_tu: " << error_tu << ", True error_tu:" << error_tu_true;
      vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 300, ss.str(), vpColor::red);

      if (opt_verbose)
        std::cout << "error translation: " << error_tr << " ; error rotation: " << error_tu << std::endl;

      if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
        has_converged = true;
        std::cout << "Servo task has converged" << std::endl;
        vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
      }

      // Send to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      ss.str("");
      ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      vpDisplay::flush(I);
      vpImage<vpRGBa> displayImage;
      vpDisplay::getImage(I, displayImage);
      writer.saveFrame(displayImage);

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

#ifdef VISP_HAVE_NLOHMANN_JSON
    // Save results to JSON
    json j = json {
      {"velocities", velocities},
      {"error", error}
    };
    std::ofstream jsonFile;
    jsonFile.open("results.json");
    jsonFile << j.dump(4);
    jsonFile.close();
#endif

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
#if !((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  std::cout << "Build ViSP with c++17 or higher compiler flag (cmake -DUSE_CXX_STANDARD=17)." << std::endl;
#endif
#if !defined(VISP_HAVE_AFMA6)
  std::cout << "ViSP is not built with Afma-6 robot support..." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
