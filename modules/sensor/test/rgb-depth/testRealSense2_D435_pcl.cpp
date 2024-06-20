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
 * Test Intel RealSense acquisition with librealsense2 (PCL demo).
 */
/*!
  \example testRealSense2_D435_pcl.cpp
  Test Intel RealSense D435 acquisition with librealsense2 (PCL demo).
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_PCL_VISUALIZATION)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayPCL.h>
#include <visp3/sensor/vpRealSense2.h>

int main(int argc, char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  bool opt_pcl_color = false;
  bool opt_show_infrared2 = false;
  bool display_helper = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pcl-color") {
      opt_pcl_color = true;
    }
    else if (std::string(argv[i]) == "--show-infrared2") {
      opt_show_infrared2 = true;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      display_helper = true;
    }
    else {
      display_helper = true;
      std::cout << "\nERROR" << std::endl;
      std::cout << "  Wrong command line option." << std::endl;
    }
    if (display_helper) {
      std::cout << "\nSYNOPSIS " << std::endl
        << "  " << argv[0]
        << " [--pcl-color]"
        << " [--show-infrared2]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --pcl-color" << std::endl
        << "    Enable textured point cloud visualization." << std::endl
        << std::endl
        << "  --show-infrared2" << std::endl
        << "    Display also the infrared2 stream." << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
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
  d2.init(depth_color, color.getWidth() + 80, 0, "Depth");
  d3.init(infrared1, 0, color.getHeight() + 70, "Infrared left");
  if (opt_show_infrared2) {
    d4.init(infrared2, color.getWidth(), color.getHeight() + 100, "Infrared right");
  }

  std::mutex mutex;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
  vpDisplayPCL pcl_viewer(color.getWidth() + 80, color.getHeight() + 70, "3D viewer " + vpTime::getDateTime());
  if (opt_pcl_color) {
    pcl_viewer.startThread(std::ref(mutex), pointcloud_color);
  }
  else {
    pcl_viewer.startThread(std::ref(mutex), pointcloud);
  }
  std::vector<double> time_vector;
  vpChrono chrono;
  while (true) {
    chrono.start();
    {
      std::lock_guard<std::mutex> lock(mutex);

      if (opt_pcl_color) {
        rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth_raw.bitmap),
                   nullptr, pointcloud_color, reinterpret_cast<unsigned char *>(infrared1.bitmap),
                   opt_show_infrared2 ? reinterpret_cast<unsigned char *>(infrared2.bitmap) : nullptr, nullptr);
      }
      else {
        rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth_raw.bitmap),
                   nullptr, pointcloud, reinterpret_cast<unsigned char *>(infrared1.bitmap),
                   opt_show_infrared2 ? reinterpret_cast<unsigned char *>(infrared2.bitmap) : nullptr, nullptr);
      }
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
    if (vpDisplay::getClick(color, false) || vpDisplay::getClick(depth_color, false) ||
        vpDisplay::getClick(infrared1, false) || vpDisplay::getClick(infrared2, false)) {
      break;
    }
  }

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
#if !defined(VISP_HAVE_X11) && !defined(VISP_HAVE_GDI)
  std::cout << "X11 or GDI are needed." << std::endl;
#endif
#if !defined(VISP_HAVE_PCL)
  std::cout << "Install PCL to make this test work." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
