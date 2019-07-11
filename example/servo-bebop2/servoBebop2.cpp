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
 * Example that shows how to visual servoing with Parrot Bebop 2 drone in ViSP.
 *
 * Authors:
 * Fabien Spindler
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
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotBebop2.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/vs/vpServo.h>

#ifdef VISP_HAVE_PUGIXML
#include <visp3/core/vpXmlParserCamera.h>
#endif

#if !defined(VISP_HAVE_ARSDK)
int main()
{
  std::cout << "\nThis example requires Parrot ARSDK3 library. You should install it.\n" << std::endl;
  return EXIT_SUCCESS;
}
#elif !defined(VISP_HAVE_OPENCV)
int main()
{
  std::cout << "\nThis example requires OpenCV library. You should install it.\n" << std::endl;
  return EXIT_SUCCESS;
}
#else

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

    std::string opt_cam_parameters;
    bool has_opt_cam_parameters = false;

    for (int i = 0; i < argc; i++) {

      if (std::string(argv[i]) == "--intrinsic") {

#ifdef VISP_HAVE_PUGIXML
        opt_cam_parameters = std::string(argv[i + 1]);
        has_opt_cam_parameters = true;
#else
        std::cout << "PUGIXML is required for custom camera parameters input" << std::endl;
        return 0;
#endif

      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--intrinsic <XML file containing computed intrinsic camera parameters (default: empty>]"
                     " [--help] [-h]\n"
                  << std::endl;
        return 0;
      }
    }

    std::cout << "\nWARNING: this program does no sensing or avoiding of "
                 "obstacles, \n"
                 "the drone WILL collide with any objects in the way! Make sure "
                 "the \n"
                 "drone has approximately 3 meters of free space on all sides.\n"
              << std::endl;

    vpRobotBebop2 drone(false); // Create the drone with low verbose level

    if (drone.isRunning()) {

      drone.setStreamingMode(0);          // Set lowest latency stream mode
      drone.setVideoStabilisationMode(0); // Disable video stabilisation

      drone.doFlatTrim();  // Flat trim calibration
      drone.takeOff(true); // Take off

      drone.startStreaming(); // Start streaming and decoding video data

      drone.setExposure(1.5f);

      vpImage<unsigned char> I;
      drone.getGrayscaleImage(I);
      vpDisplayX display(I, 100, 100, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);

      vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
      vpDetectorAprilTag detector(tagFamily); // The detector used to detect Apritags
      detector.setAprilTagQuadDecimate(4.0);
      detector.setAprilTagNbThreads(4);
      detector.setDisplayTag(true);

      vpCameraParameters cam;

      if (has_opt_cam_parameters) {

        vpXmlParserCamera p;
        vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion;

        if (p.parse(cam, opt_cam_parameters, "Camera", projModel, I.getWidth(), I.getHeight()) !=
            vpXmlParserCamera::SEQUENCE_OK) {
          std::cout << "Cannot find parameters in XML file " << opt_cam_parameters << std::endl;
          cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);
        }
      } else {
        cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);
      }

      cam.printParameters();
      double tagSize = 0.14;

      vpServo task; // Visual servoing task

      // double lambda = 0.5;
      vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
      task.setInteractionMatrixType(vpServo::CURRENT);
      task.setLambda(lambda);

      vpRotationMatrix cRe{0, 1, 0, 0, 0, 1, 1, 0, 0};

      vpTranslationVector cte(0, 0, -0.09);

      vpHomogeneousMatrix cMe(cte, cRe);

      vpVelocityTwistMatrix cVe(cMe);
      task.set_cVe(cVe);

      vpMatrix eJe(6, 4, 0);

      eJe[0][0] = 1;
      eJe[1][1] = 1;
      eJe[2][2] = 1;
      eJe[5][3] = 1;

      double Z_d = 1.5; // Desired distance to the target

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
      vpMomentAreaNormalized man(0, Z_d),
          man_d(0, Z_d); // Declare normalized area. Desired area parameter will be updated below with m00
      vpMomentGravityCenterNormalized mgn, mgn_d; // Declare normalized gravity center

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
      // Update an moment with the desired area
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

      // Add the features
      task.addFeature(s_mgn, s_mgn_d); //, vpFeatureMomentGravityCenterNormalized::selectXn());
      task.addFeature(s_man, s_man_d);

      // Update desired gravity center normalized feature
      s_mgn_d.update(A, B, C);
      s_mgn_d.compute_interaction();
      // Update desired area normalized feature
      s_man_d.update(A, B, C);
      s_man_d.compute_interaction();

      bool runLoop = true;

      // Visual servoing loop
      while (drone.isRunning() && runLoop) {
        double startTime = vpTime::measureTimeMs();

        drone.getGrayscaleImage(I);
        vpDisplay::display(I);

        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, tagSize, cam, cMo_vec); // Detect AprilTags in current image
        double t = vpTime::measureTimeMs() - startTime;

        std::stringstream ss;
        ss << "Detection time: " << t << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

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

          // Current moments
          m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
          m_obj.fromVector(vec_P);                      // Initialize the object with the points coordinates

          mg.linkTo(mdb);       // Add gravity center to database
          mc.linkTo(mdb);       // Add centered moments to database
          man.linkTo(mdb);      // Add area normalized to database
          mgn.linkTo(mdb);      // Add gravity center normalized to database
          mdb.updateAll(m_obj); // All of the moments must be updated, not just an_d
          mg.compute();         // Compute gravity center moment
          mc.compute();         // Compute centered moments AFTER gravity center
          man.setDesiredArea(
              area);     // Since desired area was init at 0, because unknow at contruction, need to be updated here
          man.compute(); // Compute area normalized moment AFTER centered moment
          mgn.compute(); // Compute gravity center normalized moment AFTER area normalized moment

          s_mgn.update(A, B, C);
          s_mgn.compute_interaction();
          s_man.update(A, B, C);
          s_man.compute_interaction();

          task.set_cVe(cVe);
          task.set_eJe(eJe);

          // Compute the control law. Velocities are computed in the mobile robot reference frame
          vpColVector ve = task.computeControlLaw();

          // Sending the control law to the drone
          drone.setVelocity(ve, 1.0);
        } else {
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

        double totalTime = vpTime::measureTimeMs() - startTime;
        std::stringstream sstime;
        sstime << "Total time: " << totalTime << " ms";
        vpDisplay::displayText(I, 80, 20, sstime.str(), vpColor::red);
        vpDisplay::flush(I);

        vpTime::wait(startTime, 40.0); // We wait a total of 40 milliseconds
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

#endif // #elif !defined(VISP_HAVE_OPENCV)
