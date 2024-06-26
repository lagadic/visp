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
 * Acquisition of images and odometry information with RealSense T265 sensor
 * and librealsense2.
 */

/*!
  \example testRealSense2_T265_images_odometry.cpp
  This example shows how to retrieve both images and odometry data from a
  RealSense T265 sensor with librealsense2.
*/

#include <iostream>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))

#include <thread>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpHomogeneousMatrix cMw, cMw_0;
  vpHomogeneousMatrix cextMw(0, 0, 2, 0, 0, 0); // External camera view for pose visualization
  vpColVector odo_vel, odo_acc, imu_acc, imu_vel;
  unsigned int confidence;
  double ts;
  vpImagePoint frame_origin;
  std::list<std::pair<unsigned int, vpImagePoint> >
    frame_origins; // Frame origin's history for trajectory visualization
  unsigned int display_scale = 2;

  try {
    vpRealSense2 rs;
    std::string product_line = rs.getProductLine();
    std::cout << "Product line: " << product_line << std::endl;

    if (product_line != "T200") {
      std::cout << "This example doesn't support devices that are not part of T200 product line family !" << std::endl;
      return EXIT_SUCCESS;
    }
    rs2::config config;
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    rs.open(config);

    // Creating images for left and right cameras, and for visualizing trajectory
    vpImage<unsigned char> I_left, I_right;
    vpImage<unsigned char> I_pose(300, 300, 0);

    vpCameraParameters cam(300., 300., I_pose.getWidth() / 2, I_pose.getHeight() / 2); // For pose visualization

    rs.acquire(&I_left, &I_right, &cMw_0, nullptr, nullptr, &confidence, &ts);

#if defined(VISP_HAVE_X11)
    vpDisplayX display_left;  // Left image
    vpDisplayX display_right; // Right image
    vpDisplayX display_pose;  // Pose visualization
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_left;  // Left image
    vpDisplayGDI display_right; // Right image
    vpDisplayGDI display_pose;  // Pose visualization
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
    display_left.setDownScalingFactor(display_scale);
    display_right.setDownScalingFactor(display_scale);
    display_left.init(I_left, 10, 10, "Left image");
    display_right.init(I_right, static_cast<int>(I_left.getWidth() / display_scale) + 80, 10, "Right image"); // Right
    display_pose.init(I_pose, 10, static_cast<int>(I_left.getHeight() / display_scale) + 80,
                      "Pose visualizer"); // visualization
#endif

    vpHomogeneousMatrix cextMc_0 = cextMw * cMw_0.inverse();
    vpMeterPixelConversion::convertPoint(cam, cextMc_0[0][3] / cextMc_0[2][3], cextMc_0[1][3] / cextMc_0[2][3],
                                         frame_origin);
    frame_origins.push_back(std::make_pair(confidence, frame_origin));

    while (true) {
      double t = vpTime::measureTimeMs();

      rs.acquire(&I_left, &I_right, &cMw, &odo_vel, &odo_acc, &confidence, &ts);

      vpDisplay::display(I_left);
      vpDisplay::display(I_right);
      vpDisplay::display(I_pose);

      vpHomogeneousMatrix cextMc = cextMw * cMw.inverse();
      vpMeterPixelConversion::convertPoint(cam, cextMc[0][3] / cextMc[2][3], cextMc[1][3] / cextMc[2][3], frame_origin);
      frame_origins.push_back(std::make_pair(confidence, frame_origin));

      vpDisplay::displayText(I_left, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::displayText(I_right, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::displayText(I_pose, 15, 15, "Click to quit", vpColor::red);

      vpDisplay::displayFrame(I_pose, cextMc_0, cam, 0.1, vpColor::none, 2); // First frame
      vpDisplay::displayFrame(I_pose, cextMc, cam, 0.1, vpColor::none, 2);

      // Display frame origin trajectory
      {
        std::list<std::pair<unsigned int, vpImagePoint> >::const_iterator it = frame_origins.begin();
        std::pair<unsigned int, vpImagePoint> frame_origin_pair_prev = *(it++);
        for (; it != frame_origins.end(); ++it) {
          if (vpImagePoint::distance(frame_origin_pair_prev.second, (*it).second) > 1) {
            vpDisplay::displayLine(
                I_pose, frame_origin_pair_prev.second, (*it).second,
                (*it).first == 3 ? vpColor::green : ((*it).first == 2 ? vpColor::yellow : vpColor::red), 2);
            frame_origin_pair_prev = *it;
          }
        }
      }
      if (vpDisplay::getClick(I_left, false) || vpDisplay::getClick(I_right, false) ||
          vpDisplay::getClick(I_pose, false)) {
        break;
      }
      vpDisplay::flush(I_left);
      vpDisplay::flush(I_right);
      vpDisplay::flush(I_pose);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }

  }
  catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "You do not realsense2 SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install librealsense2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
  std::cout << "You don't have X11 or GDI display capabilities" << std::endl;
#elif !(RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  std::cout << "Install librealsense version > 2.31.0" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
