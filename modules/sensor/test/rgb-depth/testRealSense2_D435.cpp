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
 * Test Intel RealSense acquisition with librealsense2.
 */
/*!
  \example testRealSense2_D435.cpp
  Test Intel RealSense D435 acquisition with librealsense2.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

int main(int argc, char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  bool show_info = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--show_info") {
      show_info = true;
    }
  }

  if (show_info) {
    vpRealSense2 rs;
    rs.open();
    std::cout << "RealSense:\n" << rs << std::endl;
    return EXIT_SUCCESS;
  }

  int width = 1280, height = 720, fps = 30;
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
  d4.init(infrared2, color.getWidth(), color.getHeight() + 100, "Infrared right");

  std::vector<vpColVector> pointcloud_colvector;

  std::vector<double> time_vector;
  double t_begin = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - t_begin < 10000) {
    double t = vpTime::measureTimeMs();

    rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth_raw.bitmap),
               &pointcloud_colvector, reinterpret_cast<unsigned char *>(infrared1.bitmap),
               reinterpret_cast<unsigned char *>(infrared2.bitmap), nullptr);

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

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (vpDisplay::getClick(color, false) || vpDisplay::getClick(depth_color, false) ||
        vpDisplay::getClick(infrared1, false) || vpDisplay::getClick(infrared2, false)) {
      break;
    }
  }

  // test open -> close -> open sequence
  rs.close();
  d1.close(color);
  d2.close(depth_color);
  d3.close(infrared1);
  d4.close(infrared2);

  std::cout << "Acquisition1 - Mean time: " << vpMath::getMean(time_vector)
    << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

  width = 640;
  height = 480;
  fps = 60;
  config.disable_all_streams();
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
  config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
  rs.open(config);

  color.init(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  depth_color.init(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  depth_raw.init(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
  infrared1.init(static_cast<unsigned int>(height), static_cast<unsigned int>(width));

  d1.init(color, 0, 0, "Color");
  d2.init(depth_color, color.getWidth(), 0, "Depth");
  d3.init(infrared1, 0, color.getHeight() + 100, "Infrared");

  time_vector.clear();
  t_begin = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - t_begin < 10000) {
    double t = vpTime::measureTimeMs();

    rs.acquire(reinterpret_cast<unsigned char *>(color.bitmap), reinterpret_cast<unsigned char *>(depth_raw.bitmap),
               nullptr, reinterpret_cast<unsigned char *>(infrared1.bitmap));

    vpImageConvert::createDepthHistogram(depth_raw, depth_color);

    vpDisplay::display(color);
    vpDisplay::display(depth_color);
    vpDisplay::display(infrared1);

    vpDisplay::displayText(color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::displayText(depth_color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::displayText(infrared1, 20, 20, "Click to quit.", vpColor::red);

    vpDisplay::flush(color);
    vpDisplay::flush(depth_color);
    vpDisplay::flush(infrared1);

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (vpDisplay::getClick(color, false) || vpDisplay::getClick(depth_color, false) ||
        vpDisplay::getClick(infrared1, false)) {
      break;
    }
  }

  std::cout << "Acquisition2 - Mean time: " << vpMath::getMean(time_vector)
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
  return EXIT_SUCCESS;
}
#endif
