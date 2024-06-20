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
 * Asynchronous acquisition of images and odometry information with
 * RealSense T265 sensor and librealsense2.
 */

/*!
  \example testRealSense2_T265_images_odometry_async.cpp
  This example shows how to retrieve asynchronous data from a RealSense T265
  sensor with librealsense2. Odometry at 200Hz and images at 30Hz.
*/

#include <iostream>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))

#include <functional>
#include <thread>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpHomogeneousMatrix cMw, cMw_0;
  vpHomogeneousMatrix cextMw(0, 0, 2, 0, 0, 0); // External camera view for pose visualization.
  vpColVector odo_vel, odo_acc, imu_acc, imu_vel;
  unsigned int confidence;
  vpImagePoint frame_origin;
  std::list<std::pair<unsigned int, vpImagePoint> >
    frame_origins; // Frame origin's history for trajectory visualization.
  unsigned int display_scale = 2;

  try {
    vpRealSense2 g;

    rs2::config config;
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // Creating images for left and right cameras, and for visualizing trajectory.
    vpImage<unsigned char> I_left, I_right;
    vpImage<unsigned char> I_pose(300, 300, 0);

    vpCameraParameters cam(300., 300., I_pose.getWidth() / 2, I_pose.getHeight() / 2); // For pose visualization.

    // Define frame callback.
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors.
    std::function<void(rs2::frame)> callback = [&](const rs2::frame &frame) {
      if (rs2::frameset fs = frame.as<rs2::frameset>()) {
        // With callbacks, all synchronized stream will arrive in a single frameset.
        rs2::video_frame left_frame = fs.get_fisheye_frame(1);
        size_t size = left_frame.get_width() * left_frame.get_height();
        memcpy(I_left.bitmap, left_frame.get_data(), size);

        rs2::video_frame right_frame = fs.get_fisheye_frame(2);
        size = right_frame.get_width() * right_frame.get_height();
        memcpy(I_right.bitmap, right_frame.get_data(), size);

        rs2_pose pose_data = fs.get_pose_frame().get_pose_data();

        vpTranslationVector ctw(static_cast<double>(pose_data.translation.x),
                                static_cast<double>(pose_data.translation.y),
                                static_cast<double>(pose_data.translation.z));
        vpQuaternionVector cqw(static_cast<double>(pose_data.rotation.x), static_cast<double>(pose_data.rotation.y),
                               static_cast<double>(pose_data.rotation.z), static_cast<double>(pose_data.rotation.w));

        cMw.build(ctw, cqw);

        odo_vel.resize(6, false);
        odo_vel[0] = static_cast<double>(pose_data.velocity.x);
        odo_vel[1] = static_cast<double>(pose_data.velocity.y);
        odo_vel[2] = static_cast<double>(pose_data.velocity.z);
        odo_vel[3] = static_cast<double>(pose_data.angular_velocity.x);
        odo_vel[4] = static_cast<double>(pose_data.angular_velocity.y);
        odo_vel[5] = static_cast<double>(pose_data.angular_velocity.z);

        odo_acc.resize(6, false);
        odo_acc[0] = static_cast<double>(pose_data.acceleration.x);
        odo_acc[1] = static_cast<double>(pose_data.acceleration.y);
        odo_acc[2] = static_cast<double>(pose_data.acceleration.z);
        odo_acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
        odo_acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
        odo_acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

        confidence = pose_data.tracker_confidence;
      }
      else {
     // Stream that bypass synchronization (such as IMU, Pose, ...) will produce single frames.
        rs2_pose pose_data = frame.as<rs2::pose_frame>().get_pose_data();
        vpTranslationVector ctw(static_cast<double>(pose_data.translation.x),
                                static_cast<double>(pose_data.translation.y),
                                static_cast<double>(pose_data.translation.z));
        vpQuaternionVector cqw(static_cast<double>(pose_data.rotation.x), static_cast<double>(pose_data.rotation.y),
                               static_cast<double>(pose_data.rotation.z), static_cast<double>(pose_data.rotation.w));

        cMw.build(ctw, cqw);

        odo_vel.resize(6, false);
        odo_vel[0] = static_cast<double>(pose_data.velocity.x);
        odo_vel[1] = static_cast<double>(pose_data.velocity.y);
        odo_vel[2] = static_cast<double>(pose_data.velocity.z);
        odo_vel[3] = static_cast<double>(pose_data.angular_velocity.x);
        odo_vel[4] = static_cast<double>(pose_data.angular_velocity.y);
        odo_vel[5] = static_cast<double>(pose_data.angular_velocity.z);

        odo_acc.resize(6, false);
        odo_acc[0] = static_cast<double>(pose_data.acceleration.x);
        odo_acc[1] = static_cast<double>(pose_data.acceleration.y);
        odo_acc[2] = static_cast<double>(pose_data.acceleration.z);
        odo_acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
        odo_acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
        odo_acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

        confidence = pose_data.tracker_confidence;
      }

      // Calculate the frame's origin to be projected on the image I_pose and append it to frame_origins
      vpHomogeneousMatrix cextMc = cextMw * cMw.inverse();
      vpMeterPixelConversion::convertPoint(cam, cextMc[0][3] / cextMc[2][3], cextMc[1][3] / cextMc[2][3], frame_origin);
      frame_origins.push_back(std::make_pair(confidence, frame_origin));
      };

      // Open vpRealSense2 object according to configuration and with the callback to be called.
    g.open(config, callback);

    I_left.resize(g.getIntrinsics(RS2_STREAM_FISHEYE, 1).height, g.getIntrinsics(RS2_STREAM_FISHEYE, 1).width);

    I_right.resize(g.getIntrinsics(RS2_STREAM_FISHEYE, 2).height, g.getIntrinsics(RS2_STREAM_FISHEYE, 2).width);

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
      // Sleep for 1 millisecond to reduce the number of iterations
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
