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
 * Acquisition of odometry data with RealSense T265 sensor and librealsense2.
 *
 *****************************************************************************/

/*!
  \example grabT265-odometry.cpp
  This example shows how to retrieve odometry data from a RealSense T265 sensor
  and librealsense2 and draws the trajectory of the sensor.
*/

#include <iostream>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main()
{
  vpHomogeneousMatrix pose, pose_0;
  vpTranslationVector pos, pos_0, v_pos, v_pos_0; // v for visualization
  vpRotationMatrix rot, rot_0, v_rot, v_rot_0; // v for visualization
  vpRotationMatrix rot_z_mat(vpRxyzVector(0, 0, vpMath::rad(180))); // Matrix used to rotate frame around Z-axis
  unsigned int tracker_confidence;
  double ts;
  vpImagePoint origin;
  std::vector< std::pair<unsigned int, vpImagePoint> > h_origin; // origin history (for trajectory visualization)

  try {
    vpRealSense2 rs;
    rs2::config config;
    vpCameraParameters cam(600., 600., 300., 300.); // for visualization
    
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    rs.open(config);

    // Creating black image for trajectory visualization
    vpImage<unsigned char> vI(600, 600, 0);

    vpDisplayX v_display(vI, 10, 10, "Pose visualizer");

    tracker_confidence = rs.getOdometryData(&pose_0, NULL, NULL, &ts); // Acquire first frame (pose only)
    pos_0 = pose_0.getTranslationVector();
    rot_0 = pose_0.getRotationMatrix();

    v_pos_0 = pos_0; // for visualization only
    v_pos_0[2] += 1.5; // for visualization only (consider the frame is in Z = 1.5) (because 0 by default)
    vpMeterPixelConversion::convertPoint(cam, - v_pos_0[0] / v_pos_0[2], - v_pos_0[1] / v_pos_0[2], origin);
    h_origin.push_back(std::make_pair(tracker_confidence,origin));

    v_rot_0 = rot_0 * rot_z_mat; // for visualization only
    v_pos_0[2] = 1.5; // for visualization only

    while (true) {
      double t = vpTime::measureTimeMs();
      tracker_confidence = rs.getOdometryData(&pose, NULL, NULL, &ts); // Acquire timestamped pose only

      vpDisplay::display(vI);

      pos = pose.getTranslationVector();
      rot = pose.getRotationMatrix();

      v_rot = rot * rot_z_mat; // for visualization only

      v_pos = pos; // for visualization only
      v_pos[2] += 1.5; // for visualization only (consider the frame is in Z = 1.5) (because 0 by default)
      vpMeterPixelConversion::convertPoint(cam, - v_pos[0] / v_pos[2], - v_pos[1] / v_pos[2], origin);
      h_origin.push_back(std::make_pair(tracker_confidence,origin));

      v_pos[0] = -v_pos[0]; // for visualization only
      v_pos[1] = -v_pos[1]; // for visualization only
      v_pos[2] = 1.5; // for visualization only

      vpDisplay::displayText(vI, 15, 15, "Click to quit...", vpColor::red);
      vpDisplay::displayFrame(vI, vpHomogeneousMatrix(v_pos, v_rot), cam, 0.1, vpColor::none, 2);
      vpDisplay::displayFrame(vI, vpHomogeneousMatrix(v_pos_0, v_rot_0), cam, 0.1, vpColor::none, 2);

      for(int i = 0; i < h_origin.size(); i++)
        vpDisplay::displayPoint(vI, h_origin[i].second, h_origin[i].first == 3 ? vpColor::green : vpColor::yellow, 2);

      if(vpDisplay::getClick(vI, false))
          break;

      vpDisplay::flush(vI);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }
  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#endif