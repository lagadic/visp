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
 * Test Intel RealSense acquisition with librealsense2 (PCL demo).
 *
 *****************************************************************************/
/*!
  \example testRealSense2_D435_pcl.cpp
  Test Intel RealSense D435 acquisition with librealsense2 (PCL demo).
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) &&                                         \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PCL)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

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

int main(int argc, char *argv[])
{
  bool pcl_color = false;
  bool show_infrared2 = false;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pcl_color") {
      pcl_color = true;
    } else if (std::string(argv[i]) == "--show_infrared2") {
      show_infrared2 = true;
    }
  }

  const int width = 640, height = 480, fps = 30;
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
  config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
  config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
  rs.open(config);

  vpImage<vpRGBa> color(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  vpImage<vpRGBa> depth_color(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  vpImage<uint16_t> depth_raw(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  vpImage<unsigned char> infrared1(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  vpImage<unsigned char> infrared2(static_cast<unsigned int>(height), static_cast<unsigned int>(width));

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2, d3, d4;
#else
  vpDisplayGDI d1, d2, d3, d4;
#endif
  d1.init(color, 0, 0, "Color");
  d2.init(depth_color, color.getWidth(), 0, "Depth");
  d3.init(infrared1, 0, color.getHeight() + 100, "Infrared left");
  if (show_infrared2) {
    d4.init(infrared2, color.getWidth(), color.getHeight() + 100, "Infrared right");
  }

  std::mutex mutex;
  ViewerWorker viewer_pointcloud(pcl_color, mutex);
  std::thread viewer_thread(&ViewerWorker::run, &viewer_pointcloud);

  std::vector<double> time_vector;
  vpChrono chrono;
  while (true) {
    chrono.start();
    {
      std::lock_guard<std::mutex> lock(mutex);

      if (pcl_color) {
        rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap),
                   reinterpret_cast<unsigned char *>(depth_raw.bitmap),
                   NULL, pointcloud_color,
                   reinterpret_cast<unsigned char *>(infrared1.bitmap),
                   show_infrared2 ? reinterpret_cast<unsigned char *>(infrared2.bitmap) : NULL,
                   NULL);
      } else {
        rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap),
                   reinterpret_cast<unsigned char *>(depth_raw.bitmap),
                   NULL, pointcloud,
                   reinterpret_cast<unsigned char *>(infrared1.bitmap),
                   show_infrared2 ? reinterpret_cast<unsigned char *>(infrared2.bitmap) : NULL,
                   NULL);
      }

      update_pointcloud = true;
    }

    vpImageConvert::createDepthHistogram(depth_raw, depth_color);

    vpDisplay::display(color);
    vpDisplay::display(depth_color);
    vpDisplay::display(infrared1);
    vpDisplay::display(infrared2);

    vpDisplay::displayText(color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::displayText(depth_color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::displayText(infrared1, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::displayText(infrared2, 20, 20, "Click to quit.", vpColor::red);

    vpDisplay::flush(color);
    vpDisplay::flush(depth_color);
    vpDisplay::flush(infrared1);
    vpDisplay::flush(infrared2);

    chrono.stop();
    time_vector.push_back(chrono.getDurationMs());
    if (vpDisplay::getClick(color, false) ||
        vpDisplay::getClick(depth_color, false) ||
        vpDisplay::getClick(infrared1, false) ||
        vpDisplay::getClick(infrared2, false)) {
      break;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex);
    cancelled = true;
  }
  viewer_thread.join();

  std::cout << "Acquisition - Mean time: " << vpMath::getMean(time_vector)
            << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

  return EXIT_SUCCESS;
}

#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense2 to make this test work." << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11) "
               "to make this test work"
            << std::endl;
#endif
#if !defined(VISP_HAVE_X11) && !defined(VISP_HAVE_GDI)
  std::cout << "X11 or GDI are needed." << std::endl;
#endif
#if !defined(VISP_HAVE_PCL)
  std::cout << "Install PCL to make this test work." << std::endl;
#endif
  return 0;
}
#endif
