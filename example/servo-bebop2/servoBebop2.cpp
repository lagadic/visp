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
 * Example that shows how to do visual servoing with Parrot Bebop 2 drone in ViSP.
 *
 * Authors:
 * Gatien Gaumerais
 *
*****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>
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
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotBebop2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#if !defined(VISP_HAVE_ARSDK)
int main()
{
  std::cout << "\nThis example requires Parrot ARSDK3 library. You should install it.\n" << std::endl;
  return EXIT_SUCCESS;
}
#elif !defined(VISP_HAVE_FFMPEG)
int main()
{
  std::cout << "\nThis example requires ffmpeg library. You should install it.\n" << std::endl;
  return EXIT_SUCCESS;
}
#elif !defined(VISP_HAVE_PUGIXML)
int main()
{
  std::cout << "\nThis example requires pugixml built-in 3rdparty library. You should enable it.\n" << std::endl;
  return EXIT_SUCCESS;
}

#else

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
  return (p1.second.get_v() < p2.second.get_v());
};

/*!

  \example servoBebop2.cpp example showing how to do visual servoing of
  Parrot Bebop 2 drone.

  WARNING: this program does no sensing or avoiding of obstacles, the drone
  WILL collide with any objects in the way!   Make sure the drone has about
  3-4 meters of free space around it before starting the program.

  This program makes the drone detect and follow an AprilTag from the 36h11 family.
*/
int main(int argc, char **argv)
{
  try {

    std::string ip_address = "192.168.42.1";
    std::string opt_cam_parameters;
    bool opt_has_cam_parameters = false;

    double tagSize = -1;

    double opt_distance_to_tag = -1;
    bool opt_has_distance_to_tag = false;

    int stream_res = 0; // Default 480p resolution

    bool verbose = false;

    if (argc >= 3 && std::string(argv[1]) == "--tag_size") {
      tagSize = std::atof(argv[2]); // Tag size option is required
      if (tagSize <= 0) {
        std::cout << "Error : invalid tag size." << std::endl << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
      for (int i = 3; i < argc; i++) {
        if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
          ip_address = std::string(argv[i + 1]);
          i++;
        }
        else if (std::string(argv[i]) == "--distance_to_tag" && i + 1 < argc) {
          opt_distance_to_tag = std::atof(argv[i + 1]);
          if (opt_distance_to_tag <= 0) {
            std::cout << "Error : invalid distance to tag." << std::endl << "See " << argv[0] << " --help" << std::endl;
            return EXIT_FAILURE;
          }
          opt_has_distance_to_tag = true;
          i++;
        }
        else if (std::string(argv[i]) == "--intrinsic") {

          opt_cam_parameters = std::string(argv[i + 1]);
          opt_has_cam_parameters = true;
          i++;
        }
        else if (std::string(argv[i]) == "--hd_stream") {
          stream_res = 1;
        }
        else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
          verbose = true;
        }
        else {
          std::cout << "Error : unknown parameter " << argv[i] << std::endl
            << "See " << argv[0] << " --help" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
    else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << "\nUsage:\n"
        << "  " << argv[0]
        << " [--tag_size <size>] [--ip <drone ip>] [--distance_to_tag <distance>] [--intrinsic <xml file>] "
        << "[--hd_stream] [--verbose] [-v] [--help] [-h]\n"
        << std::endl
        << "Description:\n"
        << "  --tag_size <size>\n"
        << "      The size of the tag to detect in meters, required.\n\n"
        << "  --ip <drone ip>\n"
        << "      Ip address of the drone to which you want to connect (default : 192.168.42.1).\n\n"
        << "  --distance_to_tag <distance>\n"
        << "      The desired distance to the tag in meters (default: 1 meter).\n\n"
        << "  --intrinsic <xml file>\n"
        << "      XML file containing computed intrinsic camera parameters (default: empty).\n\n"
        << "  --hd_stream\n"
        << "      Enables HD 720p streaming instead of default 480p.\n"
        << "      Allows to increase range and accuracy of the tag detection,\n"
        << "      but increases latency and computation time.\n"
        << "      Caution: camera calibration settings are different for the two resolutions.\n"
        << "      Make sure that if you pass custom intrinsic camera parameters,\n"
        << "      they were obtained with the correct resolution.\n\n"
        << "  --verbose, -v\n"
        << "      Enables verbose (drone information messages and velocity commands\n"
        << "      are then displayed).\n\n"
        << "  --help, -h\n"
        << "      Print help message.\n"
        << std::endl;
      return EXIT_SUCCESS;
    }
    else {
      std::cout << "Error : tag size parameter required." << std::endl << "See " << argv[0] << " --help" << std::endl;
      return EXIT_FAILURE;
    }

    std::cout
      << "\nWARNING: \n - This program does no sensing or avoiding of "
      "obstacles, \n"
      "the drone WILL collide with any objects in the way! Make sure "
      "the \n"
      "drone has approximately 3 meters of free space on all sides.\n"
      "  - The drone uses a downward-facing camera for horizontal speed estimation,\n make sure the drone flies "
      "above a non-uniform flooring,\n or its movement will be inacurate and dangerous !\n"

      << std::endl;

    vpRobotBebop2 drone(
        verbose, true, ip_address); // Create the drone with desired verbose level, settings reset, and corresponding IP

    if (drone.isRunning()) {

      drone.setVideoResolution(stream_res); // Set video resolution to 480p (default) or 720p

      drone.setStreamingMode(0);          // Set lowest latency stream mode
      drone.setVideoStabilisationMode(0); // Disable video stabilisation

      drone.doFlatTrim(); // Flat trim calibration

      drone.startStreaming(); // Start streaming and decoding video data

      drone.setExposure(1.5f); // Set exposure to max so that the aprilTag detection is more efficient

      drone.setCameraOrientation(-5., 0.,
                                 true); // Set camera to look slightly down so that the drone is slightly above the tag

      drone.takeOff(true); // Take off

      vpImage<unsigned char> I;
      drone.getGrayscaleImage(I);

#if defined(VISP_HAVE_X11)
      vpDisplayX display;
#elif defined(VISP_HAVE_GTK)
      vpDisplayGTK display;
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI display;
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV display;
#endif
      int orig_displayX = 100;
      int orig_displayY = 100;
      display.init(I, orig_displayX, orig_displayY, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);

      vpPlot plotter(1, 700, 700, orig_displayX + static_cast<int>(I.getWidth()) + 20, orig_displayY,
                     "Visual servoing tasks");
      unsigned int iter = 0;

      vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
      vpDetectorAprilTag detector(tagFamily); // The detector used to detect Apritags
      detector.setAprilTagQuadDecimate(4.0);
      detector.setAprilTagNbThreads(4);
      detector.setDisplayTag(true);

      vpCameraParameters cam;

      if (opt_has_cam_parameters) {

        vpXmlParserCamera p;
        vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion;

        if (p.parse(cam, opt_cam_parameters, "Camera", projModel, I.getWidth(), I.getHeight()) !=
            vpXmlParserCamera::SEQUENCE_OK) {
          std::cout << "Cannot find parameters in XML file " << opt_cam_parameters << std::endl;
          if (drone.getVideoHeight() == 720) { // 720p streaming
            cam.initPersProjWithoutDistortion(785.6412585, 785.3322447, 637.9049857, 359.7524531);
          }
          else { // 480p streaming
            cam.initPersProjWithoutDistortion(531.9213063, 520.8495788, 429.133986, 240.9464457);
          }
        }
      }
      else {
        std::cout << "Setting default camera parameters ... " << std::endl;
        if (drone.getVideoHeight() == 720) { // 720p streaming
          cam.initPersProjWithoutDistortion(785.6412585, 785.3322447, 637.9049857, 359.7524531);
        }
        else { // 480p streaming
          cam.initPersProjWithoutDistortion(531.9213063, 520.8495788, 429.133986, 240.9464457);
        }
      }
      cam.printParameters();

      vpServo task; // Visual servoing task

      // double lambda = 0.5;
      vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
      task.setInteractionMatrixType(vpServo::CURRENT);
      task.setLambda(lambda);

      /*
       In the following section, camera 1 refers to camera coordonates system of the drone, but without taking camera
       pan and camera tilt into account.
       Those values are taken into consideration in Camera 2.
       E is the effective coordinate system of the drone, the one in which we need to convert every velocity command.

       We can easily compute homogeneous matrix between camera 1 and camera 2, and between camera 1
         and effective coordonate system E of the drone.

      Using those matrices, we can in the end obtain the matrix between c2 and E
      */
      vpRxyzVector c1_rxyz_c2(vpMath::rad(drone.getCurrentCameraTilt()), vpMath::rad(drone.getCurrentCameraPan()), 0);
      vpRotationMatrix c1Rc2(c1_rxyz_c2);                      // Rotation between camera 1 and 2
      vpHomogeneousMatrix c1Mc2(vpTranslationVector(), c1Rc2); // Homogeneous matrix between c1 and c2

      vpRotationMatrix c1Re { 0, 1, 0, 0, 0, 1, 1, 0, 0 }; // Rotation between camera 1 and E
      vpTranslationVector c1te(0, 0, -0.09);            // Translation between camera 1 and E
      vpHomogeneousMatrix c1Me(c1te, c1Re);             // Homogeneous matrix between c1 and E

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
      double X[4] = { tagSize / 2., tagSize / 2., -tagSize / 2., -tagSize / 2. };
      double Y[4] = { tagSize / 2., -tagSize / 2., -tagSize / 2., tagSize / 2. };
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
      mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized moment

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

      bool runLoop = true;
      bool vec_ip_has_been_sorted = false;
      std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;

      //** Visual servoing loop **//
      while (drone.isRunning() && drone.isStreaming() && runLoop) {

        double startTime = vpTime::measureTimeMs();

        drone.getGrayscaleImage(I);
        vpDisplay::display(I);

        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, tagSize, cam, cMo_vec); // Detect AprilTags in current image
        double t = vpTime::measureTimeMs() - startTime;

        {
          std::stringstream ss;
          ss << "Detection time: " << t << " ms";
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        }

        if (detector.getNbObjects() == 1) {

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
          man.setDesiredArea(area); // Desired area was init at 0 (unknow at construction), need to be updated here
          man.compute();            // Compute area normalized moment AFTER centered moment
          mgn.compute();            // Compute gravity center normalized moment AFTER area normalized moment

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

          // Use the two highest points for the first line, and the two others for the second line.
          vpFeatureBuilder::create(s_vp, cam, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first],
                                   vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first],
                                   vpFeatureVanishingPoint::selectAtanOneOverRho());

          task.set_cVe(cVe);
          task.set_eJe(eJe);

          // Compute the control law. Velocities are computed in the mobile robot reference frame
          vpColVector ve = task.computeControlLaw();

          // Sending the control law to the drone
          if (verbose) {
            std::cout << "ve: " << ve.t() << std::endl;
          }
          drone.setVelocity(ve, 1.0);

          for (size_t i = 0; i < 4; i++) {
            vpDisplay::displayCross(I, vec_ip[i], 15, vpColor::red, 1);
            std::stringstream ss;
            ss << i;
            vpDisplay::displayText(I, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
          }

          // Display visual features
          vpDisplay::displayPolygon(I, vec_ip, vpColor::green, 3); // Current polygon used to compure an moment
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
        else {
          std::stringstream sserr;
          sserr << "Failed to detect an Apriltag, or detected multiple ones";
          vpDisplay::displayText(I, 120, 20, sserr.str(), vpColor::red);
          vpDisplay::flush(I);
          drone.stopMoving(); // In this case, we stop the drone
        }

        vpDisplay::displayText(I, 10, 10, "Click to exit", vpColor::red);
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false)) {
          drone.land();
          runLoop = false;
        }

        plotter.plot(0, iter, task.getError());

        double totalTime = vpTime::measureTimeMs() - startTime;
        std::stringstream sstime;
        sstime << "Total time: " << totalTime << " ms";
        vpDisplay::displayText(I, 80, 20, sstime.str(), vpColor::red);
        vpDisplay::flush(I);

        iter++;
        vpTime::wait(startTime, 40.0); // We wait a total of 40 milliseconds
      }

      return EXIT_SUCCESS;

    }
    else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#endif // #elif !defined(VISP_HAVE_FFMPEG)
