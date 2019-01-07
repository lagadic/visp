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
 * Acquisition with RealSense RGB-D sensor and librealsense2.
 *
 *****************************************************************************/

/*!
  \example grabRealSense2.cpp
  This example shows how to retrieve data from a RealSense RGB-D sensor with
  librealsense2.
*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_CPP11_COMPATIBILITY) &&                                         \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#ifdef VISP_HAVE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>

namespace
{
// Global variables
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
bool cancelled = false, update_pointcloud = false;

class ViewerWorker
{
public:
  explicit ViewerWorker(const bool color_mode, std::mutex &mutex) : m_colorMode(color_mode), m_mutex(mutex) {}

  void run()
  {
    std::string date = vpTime::getDateTime();
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer " + date));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud_color);
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setPosition(640 + 80, 480 + 80);
    viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
    viewer->setSize(640, 480);

    bool init = true;
    bool local_update = false, local_cancelled = false;
    while (!local_cancelled) {
      {
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);

        if (lock.owns_lock()) {
          local_update = update_pointcloud;
          update_pointcloud = false;
          local_cancelled = cancelled;

          if (local_update) {
            if (m_colorMode) {
              local_pointcloud_color = pointcloud_color->makeShared();
            } else {
              local_pointcloud = pointcloud->makeShared();
            }
          }
        }
      }

      if (local_update && !local_cancelled) {
        local_update = false;

        if (init) {
          if (m_colorMode) {
            viewer->addPointCloud<pcl::PointXYZRGB>(local_pointcloud_color, rgb, "RGB sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                     "RGB sample cloud");
          } else {
            viewer->addPointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
          }
          init = false;
        } else {
          if (m_colorMode) {
            viewer->updatePointCloud<pcl::PointXYZRGB>(local_pointcloud_color, rgb, "RGB sample cloud");
          } else {
            viewer->updatePointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
          }
        }
      }

      viewer->spinOnce(5);
    }

    std::cout << "End of point cloud display thread" << std::endl;
  }

private:
  bool m_colorMode;
  std::mutex &m_mutex;
};
}
#endif

int main()
{
  try {
    vpRealSense2 rs;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    rs.open(config);

    std::cout << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion)
              << std::endl;
    std::cout << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion)
              << std::endl;
    std::cout << "Extrinsics cMd: \n" << rs.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH) << std::endl;
    std::cout << "Extrinsics dMc: \n" << rs.getTransformation(RS2_STREAM_DEPTH, RS2_STREAM_COLOR) << std::endl;
    std::cout << "Extrinsics cMi: \n" << rs.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_INFRARED) << std::endl;
    std::cout << "Extrinsics dMi: \n" << rs.getTransformation(RS2_STREAM_DEPTH, RS2_STREAM_INFRARED) << std::endl;

    vpImage<vpRGBa> color((unsigned int)rs.getIntrinsics(RS2_STREAM_COLOR).height,
                          (unsigned int)rs.getIntrinsics(RS2_STREAM_COLOR).width);
    vpImage<unsigned char> infrared((unsigned int)rs.getIntrinsics(RS2_STREAM_INFRARED).height,
                                    (unsigned int)rs.getIntrinsics(RS2_STREAM_INFRARED).width);
    vpImage<vpRGBa> depth_display((unsigned int)rs.getIntrinsics(RS2_STREAM_DEPTH).height,
                                  (unsigned int)rs.getIntrinsics(RS2_STREAM_DEPTH).width);
    vpImage<uint16_t> depth(depth_display.getHeight(), depth_display.getWidth());

#ifdef VISP_HAVE_PCL
    std::mutex mutex;
    ViewerWorker viewer(true, mutex);
    std::thread viewer_thread(&ViewerWorker::run, &viewer);
#endif

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(color, 10, 10, "Color image");
    vpDisplayX di(infrared, (int)color.getWidth() + 80, 10, "Infrared image");
    vpDisplayX dd(depth_display, 10, (int)color.getHeight() + 80, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(color, 10, 10, "Color image");
    vpDisplayGDI di(infrared, color.getWidth() + 80, 10, "Infrared image");
    vpDisplayGDI dd(depth_display, 10, color.getHeight() + 80, "Depth image");
#endif

    while (true) {
      double t = vpTime::measureTimeMs();

#ifdef VISP_HAVE_PCL
      {
        std::lock_guard<std::mutex> lock(mutex);
        rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud_color,
                   (unsigned char *)infrared.bitmap);
        update_pointcloud = true;
      }
#else
      rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth.bitmap, NULL, (unsigned char *)infrared.bitmap);
#endif

      vpImageConvert::createDepthHistogram(depth, depth_display);

      vpDisplay::display(color);
      vpDisplay::display(infrared);
      vpDisplay::display(depth_display);

      vpDisplay::displayText(color, 15, 15, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(color, false) || vpDisplay::getClick(infrared, false) ||
          vpDisplay::getClick(depth_display, false)) {
        break;
      }
      vpDisplay::flush(color);
      vpDisplay::flush(infrared);
      vpDisplay::flush(depth_display);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }

    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

#ifdef VISP_HAVE_PCL
    {
      std::lock_guard<std::mutex> lock(mutex);
      cancelled = true;
    }

    viewer_thread.join();
#endif

  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
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
#endif
#if !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "You do not build ViSP with C++11 compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CPP11=ON, and build again this example" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
