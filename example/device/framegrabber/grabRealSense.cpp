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
 * Test RealSense RGB-D sensor.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example grabRealSense.cpp
  This example shows how to retrieve data from a RealSense RGB-D sensor.

*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

// Using a thread to display the pointcloud with PCL produces a segfault on
// OSX
#if (!defined(__APPLE__) && !defined(__MACH__))     // Not OSX
#if (defined(VISP_HAVE_PTHREAD) || defined(_WIN32)) // Threading available
#define USE_THREAD
#endif
#endif

#ifdef VISP_HAVE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
// Shared vars
typedef enum { capture_waiting, capture_started, capture_stopped } t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
vpMutex s_mutex_capture;

vpThread::Return displayPointcloudFunction(vpThread::Args args)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ = *((pcl::PointCloud<pcl::PointXYZRGB>::Ptr *)args);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud_);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setPosition(640 + 80, 480 + 80);
  viewer->setCameraPosition(0, 0, -0.5, 0, -1, 0);
  viewer->setSize(640, 480);

  t_CaptureState capture_state_;

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    if (capture_state_ == capture_started) {
      static bool update = false;
      if (!update) {
        viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud_, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        update = true;
      } else {
        viewer->updatePointCloud<pcl::PointXYZRGB>(pointcloud_, rgb, "sample cloud");
      }

      viewer->spinOnce(10);
    }
  } while (capture_state_ != capture_stopped);

  std::cout << "End of point cloud display thread" << std::endl;
  return 0;
}
#endif
#endif
#endif

int main()
{
#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)
  try {
    vpRealSense rs;
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30));
    rs.open();

    std::cout << rs.getCameraParameters(rs::stream::color, vpCameraParameters::perspectiveProjWithoutDistortion)
              << std::endl;
    std::cout << rs.getCameraParameters(rs::stream::color, vpCameraParameters::perspectiveProjWithDistortion)
              << std::endl;
    std::cout << "Extrinsics cMd: \n" << rs.getTransformation(rs::stream::color, rs::stream::depth) << std::endl;
    std::cout << "Extrinsics dMc: \n" << rs.getTransformation(rs::stream::depth, rs::stream::color) << std::endl;
    std::cout << "Extrinsics cMi: \n" << rs.getTransformation(rs::stream::color, rs::stream::infrared) << std::endl;
    std::cout << "Extrinsics dMi: \n" << rs.getTransformation(rs::stream::depth, rs::stream::infrared) << std::endl;

    vpImage<vpRGBa> color((unsigned int)rs.getIntrinsics(rs::stream::color).height,
                          (unsigned int)rs.getIntrinsics(rs::stream::color).width);

    vpImage<unsigned char> infrared_display((unsigned int)rs.getIntrinsics(rs::stream::infrared).height,
                                            (unsigned int)rs.getIntrinsics(rs::stream::infrared).width);
    vpImage<uint16_t> infrared_y16;
    rs::device *dev = rs.getHandler();
    bool use_infrared_y16 = dev->get_stream_format(rs::stream::infrared) == rs::format::y16;

    vpImage<vpRGBa> depth_display((unsigned int)rs.getIntrinsics(rs::stream::depth).height,
                                  (unsigned int)rs.getIntrinsics(rs::stream::depth).width);
    vpImage<uint16_t> depth(depth_display.getHeight(), depth_display.getWidth());

#ifdef VISP_HAVE_PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

#ifdef USE_THREAD
    vpThread thread_display_pointcloud(displayPointcloudFunction, (vpThread::Args)&pointcloud);
#else
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -0.5, 0, -1, 0);
#endif

#else
    std::vector<vpColVector> pointcloud;
#endif

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(color, 10, 10, "Color image");
    vpDisplayX di(infrared_display, (int)color.getWidth() + 80, 10, "Infrared image");
    vpDisplayX dd(depth_display, 10, (int)color.getHeight() + 80, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(color, 10, 10, "Color image");
    vpDisplayGDI di(infrared_display, color.getWidth() + 80, 10, "Infrared image");
    vpDisplayGDI dd(depth_display, 10, color.getHeight() + 80, "Depth image");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while (1) {
      double t = vpTime::measureTimeMs();

      if (use_infrared_y16) {
        rs.acquire(color, infrared_y16, depth, pointcloud);
        vpImageConvert::convert(infrared_y16, infrared_display);
      } else {
#ifdef VISP_HAVE_PCL
        rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud,
                   (unsigned char *)infrared_display.bitmap);
#else
        rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth.bitmap, NULL,
                   (unsigned char *)infrared_display.bitmap);
#endif
      }

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        s_capture_state = capture_started;
      }
#else
      static bool update = false;
      if (!update) {
        viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->setPosition(color.getWidth() + 80, color.getHeight() + 80);
        update = true;
      } else {
        viewer->updatePointCloud<pcl::PointXYZRGB>(pointcloud, rgb, "sample cloud");
      }

      viewer->spinOnce(10);
#endif
#endif

      vpImageConvert::createDepthHistogram(depth, depth_display);

      vpDisplay::display(color);
      vpDisplay::display(infrared_display);
      vpDisplay::display(depth_display);

      vpDisplay::displayText(color, 15, 15, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(color, false) || vpDisplay::getClick(infrared_display, false) ||
          vpDisplay::getClick(depth_display, false)) {
        break;
      }
      vpDisplay::flush(color);
      vpDisplay::flush(infrared_display);
      vpDisplay::flush(depth_display);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }

    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      s_capture_state = capture_stopped;
    }
#endif
#endif

    rs.close();
  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const rs::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args()
              << "): " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

#elif !defined(VISP_HAVE_REALSENSE)
  std::cout << "Install RealSense SDK to make this test working" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- After installation, configure again ViSP using cmake and build again this example" << std::endl;
#elif !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "You do not build ViSP with C++11 compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CPP11=ON, and build again this example" << std::endl;
            << std::endl;
#endif
  return EXIT_SUCCESS;
}
