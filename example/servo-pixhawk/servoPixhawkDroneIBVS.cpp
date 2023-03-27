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
 * Example that shows how to do visual servoing with a drone equipped with a Pixhawk.
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && defined(VISP_HAVE_REALSENSE2)

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotMavsdk.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// Comment next line to disable sending commands to the robot
#define CONTROL_UAV

bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
  return (p1.second.get_v() < p2.second.get_v());
};

/*!
 * \example servoPixhawkDroneIBVS.cpp
 *
 * Example that shows how to how to achieve an image-based visual servo a drone
 * equipped with a Pixhawk connected to a Jetson TX2. An Intel Realsense camera
 * is also attached to the drone and connected to the Jetson. The drone is localized
 * thanks to Qualisys Mocap. Communication between the Jetson and the Pixhawk
 * is based on Mavlink using MAVSDK 3rd party.
 *
 * This program makes the drone detect and follow an AprilTag from the 36h11 family.
 *
 * \warning this program does no sensing or avoiding of obstacles, the drone
 * will collide with any objects in the way! Make sure the drone has about
 * 3-4 meters of free space around it before starting the program.
 *
 */
int main(int argc, char **argv)
{
  try {
    std::string opt_connecting_info = "udp://:192.168.30.111:14552";
    double tagSize = -1;
    double opt_distance_to_tag = -1;
    bool opt_has_distance_to_tag = false;
    int opt_display_fps = 10;
    bool opt_verbose = false;

    int acq_fps = 30;

    if (argc >= 3 && std::string(argv[1]) == "--tag-size") {
      tagSize = std::atof(argv[2]); // Tag size option is required
      if (tagSize <= 0) {
        std::cout << "Error : invalid tag size." << std::endl << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
      for (int i = 3; i < argc; i++) {
        if (std::string(argv[i]) == "--co" && i + 1 < argc) {
          opt_connecting_info = std::string(argv[i + 1]);
          i++;
        } else if (std::string(argv[i]) == "--distance-to-tag" && i + 1 < argc) {
          opt_distance_to_tag = std::atof(argv[i + 1]);
          if (opt_distance_to_tag <= 0) {
            std::cout << "Error : invalid distance to tag." << std::endl << "See " << argv[0] << " --help" << std::endl;
            return EXIT_FAILURE;
          }
          opt_has_distance_to_tag = true;
          i++;

        } else if (std::string(argv[i]) == "--display-fps" && i + 1 < argc) {
          opt_display_fps = std::stoi(std::string(argv[i + 1]));
          i++;
        } else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
          opt_verbose = true;
        } else {
          std::cout << "Error : unknown parameter " << argv[i] << std::endl
                    << "See " << argv[0] << " --help" << std::endl;
          return EXIT_FAILURE;
        }
      }
    } else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << "\nUsage:\n"
                << "  " << argv[0]
                << " [--tag-size <tag size [m]>] [--co <connection information>] [--distance-to-tag <distance>]"
                << " [--display-fps <display fps>] [--verbose] [-v] [--help] [-h]\n"
                << std::endl
                << "Description:\n"
                << "  --tag-size <size>\n"
                << "      The size of the tag to detect in meters, required.\n\n"
                << "  --co <connection information>\n"
                << "      - UDP: udp://[host][:port]\n"
                << "      - TCP: tcp://[host][:port]\n"
                << "      - serial: serial://[path][:baudrate]\n"
                << "      - Default: udp://192.168.30.111:14552).\n\n"
                << "  --distance-to-tag <distance>\n"
                << "      The desired distance to the tag in meters (default: 1 meter).\n\n"
                << "  --display-fps <display_fps>\n"
                << "      The desired fps rate for the video display (default: 10 fps).\n\n"
                << "  --verbose, -v\n"
                << "      Enables verbosity (drone information messages and velocity commands\n"
                << "      are then displayed).\n\n"
                << "  --help, -h\n"
                << "      Print help message.\n"
                << std::endl;
      return EXIT_SUCCESS;

    } else {
      std::cout << "Error : tag size parameter required." << std::endl << "See " << argv[0] << " --help" << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << std::endl
              << "WARNING:" << std::endl
              << " - This program does no sensing or avoiding of obstacles, " << std::endl
              << "   the drone WILL collide with any objects in the way! Make sure the " << std::endl
              << "   drone has approximately 3 meters of free space on all sides." << std::endl
              << " - The drone uses a forward-facing camera for Apriltag detection," << std::endl
              << "   make sure the drone flies  above a non-uniform flooring," << std::endl
              << "   or its movement will be inacurate and dangerous !" << std::endl
              << std::endl;

    // Connect to the drone
    vpRobotMavsdk drone(opt_connecting_info);

    if (drone.isRunning()) {
      vpRealSense2 rs;

      std::string product_line = rs.getProductLine();
      if (opt_verbose) {
        std::cout << "Product line: " << product_line << std::endl;
      }

      if (product_line == "T200") {
        std::cout << "This example doesn't support T200 product line family !" << std::endl;
        return EXIT_SUCCESS;
      }
      rs2::config config;

      config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, acq_fps);
      config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, acq_fps);
      config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, acq_fps);

      rs.open(config);
      vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR);

      if (opt_verbose) {
        cam.printParameters();
      }

#ifdef CONTROL_UAV
      drone.doFlatTrim(); // Flat trim calibration
      drone.takeOff();    // Take off
#endif

      vpImage<unsigned char> I(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);

#if defined VISP_HAVE_X11
      vpDisplayX display;
#elif defined VISP_HAVE_GTK
      vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
      vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
      vpDisplayOpenCV display;
#endif
      int orig_displayX = 100;
      int orig_displayY = 100;
      display.init(I, orig_displayX, orig_displayY, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);
      double time_since_last_display = vpTime::measureTimeMs();

      vpPlot plotter(1, 700, 700, orig_displayX + static_cast<int>(I.getWidth()) + 20, orig_displayY,
                     "Visual servoing tasks");
      unsigned int iter = 0;

      vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
      vpDetectorAprilTag detector(tagFamily); // The detector used to detect Apritags
      detector.setAprilTagQuadDecimate(4.0);
      detector.setAprilTagNbThreads(4);
      detector.setDisplayTag(true);

      vpServo task; // Visual servoing task

      // double lambda = 0.5;
      vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
      task.setInteractionMatrixType(vpServo::CURRENT);
      task.setLambda(lambda);

      /*
       * In the following section, camera 1 refers to camera coordinates system of the drone,
       * but without taking camera orientation into account. This orientation is taken into
       * consideration in camera 2. E is the effective coordinate system of the drone, the one
       * in which we need to convert every velocity command.
       *
       * We can easily compute homogeneous matrix between camera 1 and camera 2, and between
       * camera 1 and effective coordonate system E of the drone.
       *
       * Using those matrices, we can in the end obtain the matrix between c2 and E
       */
      vpRxyzVector c1_rxyz_c2(vpMath::rad(-10.0), vpMath::rad(0), 0);
      vpRotationMatrix c1Rc2(c1_rxyz_c2); // Rotation between camera 1 and 2
      vpHomogeneousMatrix c1Mc2(vpTranslationVector(),
                                c1Rc2); // Homogeneous matrix between c1 and c2

      vpRotationMatrix c1Re{1, 0, 0, 0, 0, 1, 0, -1, 0}; // Rotation between camera 1 and E
      vpTranslationVector c1te(0, -0.03, -0.07);         // Translation between camera 1 and E
      vpHomogeneousMatrix c1Me(c1te, c1Re);              // Homogeneous matrix between c1 and E

      vpHomogeneousMatrix c2Me = c1Mc2.inverse() * c1Me; // Homogeneous matrix between c2 and E

      vpVelocityTwistMatrix cVe(c2Me);

      task.set_cVe(cVe);

      vpMatrix eJe(6, 4, 0);

      eJe[0][0] = 1;
      eJe[1][1] = 1;
      eJe[2][2] = 1;
      eJe[5][3] = 1;

      //      double Z_d = 1.; // Desired distance to the target
      double Z_d = (opt_has_distance_to_tag ? opt_distance_to_tag : 1.);

      // Define the desired polygon corresponding the the AprilTag CLOCKWISE
      double X[4] = {tagSize / 2., tagSize / 2., -tagSize / 2., -tagSize / 2.};
      double Y[4] = {tagSize / 2., -tagSize / 2., -tagSize / 2., tagSize / 2.};
      std::vector<vpPoint> vec_P, vec_P_d;

      for (int i = 0; i < 4; i++) {
        vpPoint P_d(X[i], Y[i], 0);
        vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
        P_d.track(cdMo); //
        vec_P_d.push_back(P_d);
      }
      vpMomentObject m_obj(3), m_obj_d(3);
      vpMomentDatabase mdb, mdb_d;
      vpMomentBasic mb_d; // Here only to get the desired area m00
      vpMomentGravityCenter mg, mg_d;
      vpMomentCentered mc, mc_d;
      vpMomentAreaNormalized man(0, Z_d), man_d(0, Z_d); // Declare normalized area updated below with m00
      vpMomentGravityCenterNormalized mgn, mgn_d;        // Declare normalized gravity center

      // Desired moments
      m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
      m_obj_d.fromVector(vec_P_d);                    // Initialize the object with the points coordinates

      mb_d.linkTo(mdb_d);       // Add basic moments to database
      mg_d.linkTo(mdb_d);       // Add gravity center to database
      mc_d.linkTo(mdb_d);       // Add centered moments to database
      man_d.linkTo(mdb_d);      // Add area normalized to database
      mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
      mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just an_d
      mg_d.compute();           // Compute gravity center moment
      mc_d.compute();           // Compute centered moments AFTER gravity center

      double area = 0;
      if (m_obj_d.getType() == vpMomentObject::DISCRETE)
        area = mb_d.get(2, 0) + mb_d.get(0, 2);
      else
        area = mb_d.get(0, 0);
      // Update moment with the desired area
      man_d.setDesiredArea(area);

      man_d.compute(); // Compute area normalized moment AFTER centered moments
      mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized
                       // moment

      // Desired plane
      double A = 0.0;
      double B = 0.0;
      double C = 1.0 / Z_d;

      // Construct area normalized features
      vpFeatureMomentGravityCenterNormalized s_mgn(mdb, A, B, C), s_mgn_d(mdb_d, A, B, C);
      vpFeatureMomentAreaNormalized s_man(mdb, A, B, C), s_man_d(mdb_d, A, B, C);
      vpFeatureVanishingPoint s_vp, s_vp_d;

      // Add the features
      task.addFeature(s_mgn, s_mgn_d);
      task.addFeature(s_man, s_man_d);
      task.addFeature(s_vp, s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());

      plotter.initGraph(0, 4);
      plotter.setLegend(0, 0, "Xn");          // Distance from center on X axis feature
      plotter.setLegend(0, 1, "Yn");          // Distance from center on Y axis feature
      plotter.setLegend(0, 2, "an");          // Tag area feature
      plotter.setLegend(0, 3, "atan(1/rho)"); // Vanishing point feature

      // Update desired gravity center normalized feature
      s_mgn_d.update(A, B, C);
      s_mgn_d.compute_interaction();
      // Update desired area normalized feature
      s_man_d.update(A, B, C);
      s_man_d.compute_interaction();

      // Update desired vanishing point feature for the horizontal line
      s_vp_d.setAtanOneOverRho(0);
      s_vp_d.setAlpha(0);

      bool condition;
      bool runLoop = true;
      bool vec_ip_has_been_sorted = false;
      bool send_velocities = false;
      std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;

      //** Visual servoing loop **//
      while (drone.isRunning() && runLoop) {

        double startTime = vpTime::measureTimeMs();

        // drone.getGrayscaleImage(I);
        rs.acquire(I);

        condition = (startTime - time_since_last_display) > 1000. / opt_display_fps ? true : false;
        if (condition) {
          vpDisplay::display(I);
          time_since_last_display = vpTime::measureTimeMs();
        }

        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, tagSize, cam, cMo_vec); // Detect AprilTags in current image
        double t = vpTime::measureTimeMs() - startTime;

        if (condition) {
          std::stringstream ss;
          ss << "Detection time: " << t << " ms";
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        }

        if (detector.getNbObjects() != 0) {

          // Update current points used to compute the moments
          std::vector<vpImagePoint> vec_ip = detector.getPolygon(0);
          vec_P.clear();
          for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam, vec_ip[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            vec_P.push_back(P);
          }

          // Current moments
          m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
          m_obj.fromVector(vec_P);                      // Initialize the object with the points coordinates

          mg.linkTo(mdb);           // Add gravity center to database
          mc.linkTo(mdb);           // Add centered moments to database
          man.linkTo(mdb);          // Add area normalized to database
          mgn.linkTo(mdb);          // Add gravity center normalized to database
          mdb.updateAll(m_obj);     // All of the moments must be updated, not just an_d
          mg.compute();             // Compute gravity center moment
          mc.compute();             // Compute centered moments AFTER gravity center
          man.setDesiredArea(area); // Desired area was init at 0 (unknow at contruction),
                                    // need to be updated here
          man.compute();            // Compute area normalized moment AFTER centered moment
          mgn.compute();            // Compute gravity center normalized moment AFTER area normalized
                                    // moment

          s_mgn.update(A, B, C);
          s_mgn.compute_interaction();
          s_man.update(A, B, C);
          s_man.compute_interaction();

          /* Sort points from their height in the image, and keep original indexes.
          This is done once, in order to be independent from the orientation of the tag
          when detecting vanishing points. */
          if (!vec_ip_has_been_sorted) {
            for (size_t i = 0; i < vec_ip.size(); i++) {

              // Add the points and their corresponding index
              std::pair<size_t, vpImagePoint> index_pair = std::pair<size_t, vpImagePoint>(i, vec_ip[i]);
              vec_ip_sorted.push_back(index_pair);
            }

            // Sort the points and indexes from the v value of the points
            std::sort(vec_ip_sorted.begin(), vec_ip_sorted.end(), compareImagePoint);

            vec_ip_has_been_sorted = true;
          }

          // Use the two highest points for the first line, and the two others for the second
          // line.
          vpFeatureBuilder::create(s_vp, cam, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first],
                                   vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first],
                                   vpFeatureVanishingPoint::selectAtanOneOverRho());

          task.set_cVe(cVe);
          task.set_eJe(eJe);

          // Compute the control law. Velocities are computed in the mobile robot reference
          // frame
          vpColVector ve = task.computeControlLaw();
          if (!send_velocities) {
            ve = 0;
          }

          // Sending the control law to the drone
          if (opt_verbose) {
            std::cout << "ve: " << ve.t() << std::endl;
          }

#ifdef CONTROL_UAV
          drone.setVelocity(ve);
#endif

          if (condition) {
            for (size_t i = 0; i < 4; i++) {
              vpDisplay::displayCross(I, vec_ip[i], 15, vpColor::red, 1);
              std::stringstream ss;
              ss << i;
              vpDisplay::displayText(I, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
            }

            // Display visual features
            vpDisplay::displayPolygon(I, vec_ip, vpColor::green,
                                      3); // Current polygon used to compure an moment
            vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::green,
                                    3); // Current polygon used to compute a moment
            vpDisplay::displayLine(I, 0, static_cast<int>(cam.get_u0()), static_cast<int>(I.getHeight()) - 1,
                                   static_cast<int>(cam.get_u0()), vpColor::red,
                                   3); // Vertical line as desired x position
            vpDisplay::displayLine(I, static_cast<int>(cam.get_v0()), 0, static_cast<int>(cam.get_v0()),
                                   static_cast<int>(I.getWidth()) - 1, vpColor::red,
                                   3); // Horizontal line as desired y position

            // Display lines corresponding to the vanishing point for the horizontal lines
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first], vpColor::red, 1,
                                   false);
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first], vpColor::red, 1,
                                   false);
          }

        } else {

          std::stringstream sserr;
          sserr << "Failed to detect an Apriltag, or detected multiple ones";
          if (condition) {
            vpDisplay::displayText(I, 120, 20, sserr.str(), vpColor::red);
            vpDisplay::flush(I);
          } else {
            std::cout << sserr.str() << std::endl;
          }
#ifdef CONTROL_UAV
          drone.stopMoving(); // In this case, we stop the drone
#endif
        }

        if (condition) {
          {
            std::stringstream ss;
            ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot")
               << ", right click to quit.";
            vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
          }
          vpDisplay::flush(I);

          plotter.plot(0, iter, task.getError());
        }

        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(I, button, false)) {
          switch (button) {
          case vpMouseButton::button1:
            send_velocities = !send_velocities;
            break;

          case vpMouseButton::button3:
            drone.land();
            runLoop = false;
            break;

          default:
            break;
          }
        }

        double totalTime = vpTime::measureTimeMs() - startTime;
        std::stringstream sstime;
        sstime << "Total time: " << totalTime << " ms";
        if (condition) {
          vpDisplay::displayText(I, 80, 20, sstime.str(), vpColor::red);
          vpDisplay::flush(I);
        }

        iter++;
        vpTime::wait(startTime, 1000. / acq_fps);
      }

      return EXIT_SUCCESS;
    } else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
      return EXIT_FAILURE;
    }
  } catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n"
            << std::endl;
#endif
#ifndef VISP_HAVE_REALSENSE2
  std::cout << "\nThis example requires librealsense2 library. You should install it, configure and rebuid ViSP.\n"
            << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  std::cout
      << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
         "rebuild ViSP.\n"
      << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
