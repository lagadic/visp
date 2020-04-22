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
 * Asynchronous acquisition of images and odometry information with 
 * RealSense T265 sensor and librealsense2.
 *
 *****************************************************************************/

/*!
  \example grabT265-images-and-odometry-async.cpp
  This example shows how to retrieve asynchronous data from a RealSense T265 
  sensor with librealsense2. Odometry at 200Hz and images at 30Hz.
*/

#include <iostream>
#include <thread>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main()
{
  vpQuaternionVector quat;
  vpHomogeneousMatrix pose;
  vpTranslationVector pos, v_pos; // v for visualization
  vpRotationMatrix rot, v_rot; // v for visualization
  vpRotationMatrix rot_z_mat(vpRxyzVector(0, 0, vpMath::rad(180)));
  vpColVector vel(6), acc(6);
  unsigned int tracker_confidence;
  int size;
  vpImagePoint origin;
  std::vector< std::pair<unsigned int, vpImagePoint> > h_origin; // Frame origin's history for trajectory visualization

  try {
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    
    vpCameraParameters cam(300., 300., 150., 150.); // for visualization

    vpImage<unsigned char> Il, Ir; // for left & right cameras of T265 device

    vpImage<unsigned char> vI(300, 300, 0); // for pose visualization

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
      if (rs2::frameset fs = frame.as<rs2::frameset>())
      {
        // With callbacks, all synchronized stream will arrive in a single frameset
        rs2::video_frame left_frame = fs.get_fisheye_frame(1);
        size = left_frame.get_width() * left_frame.get_height();
        memcpy(Il.bitmap, left_frame.get_data(), size);

        rs2::video_frame right_frame = fs.get_fisheye_frame(2);
        size = right_frame.get_width() * right_frame.get_height();
        memcpy(Ir.bitmap, right_frame.get_data(), size);

        rs2_pose pose_data = fs.get_pose_frame().get_pose_data();
        pos[0] = static_cast<double>(pose_data.translation.x);
        pos[1] = static_cast<double>(pose_data.translation.y);
        pos[2] = static_cast<double>(pose_data.translation.z);

        quat[0] = static_cast<double>(pose_data.rotation.x);
        quat[1] = static_cast<double>(pose_data.rotation.y);
        quat[2] = static_cast<double>(pose_data.rotation.z);
        quat[3] = static_cast<double>(pose_data.rotation.w);

        pose.buildFrom(pos,quat);

        vel[0] = static_cast<double>(pose_data.velocity.x);
        vel[1] = static_cast<double>(pose_data.velocity.y);
        vel[2] = static_cast<double>(pose_data.velocity.z);
        vel[3] = static_cast<double>(pose_data.angular_velocity.x);
        vel[4] = static_cast<double>(pose_data.angular_velocity.y);
        vel[5] = static_cast<double>(pose_data.angular_velocity.z);

        acc[0] = static_cast<double>(pose_data.acceleration.x);
        acc[1] = static_cast<double>(pose_data.acceleration.y);
        acc[2] = static_cast<double>(pose_data.acceleration.z);
        acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
        acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
        acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

        tracker_confidence = pose_data.tracker_confidence;
      }
      else
      {
        // Stream that bypass synchronization (such as IMU, Pose, ...) will produce single frames
        rs2_pose pose_data = frame.as<rs2::pose_frame>().get_pose_data();
        pos[0] = static_cast<double>(pose_data.translation.x);
        pos[1] = static_cast<double>(pose_data.translation.y);
        pos[2] = static_cast<double>(pose_data.translation.z);

        quat[0] = static_cast<double>(pose_data.rotation.x);
        quat[1] = static_cast<double>(pose_data.rotation.y);
        quat[2] = static_cast<double>(pose_data.rotation.z);
        quat[3] = static_cast<double>(pose_data.rotation.w);

        pose.buildFrom(pos,quat);

        vel[0] = static_cast<double>(pose_data.velocity.x);
        vel[1] = static_cast<double>(pose_data.velocity.y);
        vel[2] = static_cast<double>(pose_data.velocity.z);
        vel[3] = static_cast<double>(pose_data.angular_velocity.x);
        vel[4] = static_cast<double>(pose_data.angular_velocity.y);
        vel[5] = static_cast<double>(pose_data.angular_velocity.z);

        acc[0] = static_cast<double>(pose_data.acceleration.x);
        acc[1] = static_cast<double>(pose_data.acceleration.y);
        acc[2] = static_cast<double>(pose_data.acceleration.z);
        acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
        acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
        acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

        tracker_confidence = pose_data.tracker_confidence;
      }

      // Calculate the frame's origin to be projected on the image vI and append it to h_origins
      vpMeterPixelConversion::convertPoint(cam, - pos[0] / (pos[2]+1.5), - pos[1] / (pos[2]+1.5), origin);
      h_origin.push_back(std::make_pair(tracker_confidence,origin));
      v_pos[0] = -pos[0];
      v_pos[1] = -pos[1];
      v_pos[2] = 1.5;
      v_rot = rot * rot_z_mat;
    };

    // Start the pipline streaming according to configuration
    rs2::pipeline_profile profiles = pipe.start(config, callback);

    Il.resize(profiles.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().height(),
              profiles.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().width());

    Ir.resize(profiles.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().height(),
              profiles.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().width());

#if defined(VISP_HAVE_X11)
    vpDisplayX fe_l(Il, 10, 10, "Left image"); // Left
    vpDisplayX fe_r(Ir, (int)Il.getWidth() + 80, 10, "Right image"); // Right
    vpDisplayX vf(vI, (int)Il.getWidth() - 150, (int)Il.getHeight(), "Pose visualizer"); // Pose visualization
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI fe_l(Il, 10, 10, "Right image");
    vpDisplayGDI fe_r(Ir, Il.getWidth() + 80, 10, "Left image");
    vpDisplayGDI vf(vI, (int)Il.getWidth() - 150, (int)Il.getHeight(), "Pose visualizer");
#endif

    while (true) {
      // Sleep for 1 millisecond to reduce the number of iterations
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      vpDisplay::display(Il);
      vpDisplay::display(Ir);
      vpDisplay::display(vI);

      vpDisplay::displayText(Il, 15, 15, "Click to quit", vpColor::red);
      vpDisplay::displayText(Ir, 15, 15, "Click to quit", vpColor::red);
      vpDisplay::displayText(vI, 15, 15, "Click to quit", vpColor::red);

      if (vpDisplay::getClick(Il, false) || vpDisplay::getClick(Ir, false) || vpDisplay::getClick(vI, false)) {
        break;
      }

      if(h_origin.size() != 0)
      {
        vpDisplay::displayFrame(vI, vpHomogeneousMatrix(v_pos, v_rot), cam, 0.1, vpColor::none, 2);

        for(int i = 0; i < h_origin.size(); i++)
          vpDisplay::displayPoint(vI, h_origin[i].second, h_origin[i].first == 3 ? vpColor::green : vpColor::yellow, 2);
      }

      vpDisplay::flush(Il);
      vpDisplay::flush(Ir);
      vpDisplay::flush(vI);
    }

  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#endif