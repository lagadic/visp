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
 * Acquisition with RealSense RGB-D sensor and librealsense2.
 *
*****************************************************************************/

/*!
  \example grabRealSense2.cpp
  This example shows how to retrieve data from a RealSense RGB-D sensor with
  librealsense2.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpTime.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayPCL.h>
#include <visp3/sensor/vpRealSense2.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    vpRealSense2 rs;

    std::string product_line = rs.getProductLine();
    std::cout << "Product line: " << product_line << std::endl;

    if (product_line == "T200") {
      std::cout << "This example doesn't support T200 product line family !" << std::endl;
      return EXIT_SUCCESS;
    }
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

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)
    std::mutex mutex;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
#if defined(VISP_HAVE_PCL_VISUALIZATION)
    vpDisplayPCL pcl_viewer(color.getWidth() + 80, color.getHeight() + 70, "3D viewer " + vpTime::getDateTime());
    pcl_viewer.startThread(std::ref(mutex), pointcloud_color);
#endif
#endif

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(color, 10, 10, "Color image");
    vpDisplayX di(infrared, static_cast<int>(color.getWidth()) + 80, 10, "Infrared image");
    vpDisplayX dd(depth_display, 10, static_cast<int>(color.getHeight()) + 70, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(color, 10, 10, "Color image");
    vpDisplayGDI di(infrared, color.getWidth() + 80, 10, "Infrared image");
    vpDisplayGDI dd(depth_display, 10, color.getHeight() + 70, "Depth image");
#endif

    while (true) {
      double t = vpTime::measureTimeMs();

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)
      {
        std::lock_guard<std::mutex> lock(mutex);
        rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth.bitmap), nullptr, pointcloud_color,
                   reinterpret_cast<unsigned char *>(infrared.bitmap));
      }
#else
      rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth.bitmap), nullptr, reinterpret_cast<unsigned char *>(infrared.bitmap));
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
  std::cout << "You do not have realsense2 SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install librealsense2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#endif
  return EXIT_SUCCESS;
}
#endif
