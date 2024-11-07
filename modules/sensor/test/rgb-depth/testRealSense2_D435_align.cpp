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
 * Test Intel RealSense D435 acquisition with librealsense2.
 */
/*!
  \example testRealSense2_D435_align.cpp
  Test Intel RealSense D435 acquisition with librealsense2.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) \
  && defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void createDepthHist(std::vector<uint32_t> &histogram2, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
                     double depth_scale)
{
  std::fill(histogram2.begin(), histogram2.end(), 0);

  for (uint32_t i = 0; i < pointcloud->height; i++) {
    for (uint32_t j = 0; j < pointcloud->width; j++) {
      const pcl::PointXYZ &pcl_pt = pointcloud->at(j, i);
      ++histogram2[static_cast<uint32_t>(pcl_pt.z * depth_scale)];
    }
  }

  for (int i = 2; i < 0x10000; i++)
    histogram2[i] += histogram2[i - 1]; // Build a cumulative histogram for
                                        // the indices in [1,0xFFFF]
}

void createDepthHist(std::vector<uint32_t> &histogram2, const std::vector<vpColVector> &pointcloud, double depth_scale)
{
  std::fill(histogram2.begin(), histogram2.end(), 0);

  for (size_t i = 0; i < pointcloud.size(); i++) {
    const vpColVector &pt = pointcloud[i];
    ++histogram2[static_cast<uint32_t>(pt[2] * depth_scale)];
  }

  for (int i = 2; i < 0x10000; i++)
    histogram2[i] += histogram2[i - 1]; // Build a cumulative histogram for
                                        // the indices in [1,0xFFFF]
}

unsigned char getDepthColor(const std::vector<uint32_t> &histogram2, double z, double depth_scale)
{
  // 0-255 based on histogram location
  return static_cast<unsigned char>(histogram2[static_cast<uint32_t>(z * depth_scale)] * 255 / histogram2[0xFFFF]);
}
} // namespace

int main(int argc, char *argv[])
{
  bool align_to_depth = false;
  bool color_pointcloud = false;
  bool col_vector = false;
  bool no_align = false;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--align_to_depth") {
      align_to_depth = true;
    }
    else if (std::string(argv[i]) == "--color") {
      color_pointcloud = true;
    }
    else if (std::string(argv[i]) == "--col_vector") {
      col_vector = true;
    }
    else if (std::string(argv[i]) == "--no_align") {
      no_align = true;
    }
  }

  std::cout << "align_to_depth: " << align_to_depth << std::endl;
  std::cout << "color_pointcloud: " << color_pointcloud << std::endl;
  std::cout << "col_vector: " << col_vector << std::endl;
  std::cout << "no_align: " << no_align << std::endl;

  vpRealSense2 rs;
  rs2::config config;
  const int width = 640, height = 480, fps = 30;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
  config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
  rs.open(config);
  const double depth_scale = 1.0 / rs.getDepthScale();

  vpImage<vpRGBa> I_color(height, width), I_depth(height, width), I_pcl(height, width), I_pcl2(height, width);
  vpImage<vpRGBa> I_display(height * 2, width), I_display2(height * 2, width), I_display3(height * 2, width);
  vpImage<uint16_t> I_depth_raw(height, width);

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2, d3;
#else
  vpDisplayGDI d1, d2, d3;
#endif
  d1.init(I_display, 0, 0, "Color + depth");
  d2.init(I_display2, width, 0, "Color + ROS pointcloud");
  d3.init(I_display3, 2 * width, 0, "Color + pointcloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<vpColVector> vp_pointcloud;
  std::vector<uint32_t> histogram(0x10000), histogram2(0x10000);

  rs2::align align_to(align_to_depth ? RS2_STREAM_DEPTH : RS2_STREAM_COLOR);
  vpCameraParameters cam_projection =
    align_to_depth ? rs.getCameraParameters(RS2_STREAM_DEPTH) : rs.getCameraParameters(RS2_STREAM_COLOR);

  while (true) {
    if (color_pointcloud) {
      rs.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
                 reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), &vp_pointcloud, pointcloud_color, nullptr,
                 no_align ? nullptr : &align_to);
    }
    else {
      rs.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
                 reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), &vp_pointcloud, pointcloud, nullptr,
                 no_align ? nullptr : &align_to);
    }

    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    I_pcl = vpRGBa(0, 0, 0);
    if (color_pointcloud) {
      for (uint32_t i = 0; i < pointcloud_color->height; i++) {
        for (uint32_t j = 0; j < pointcloud_color->width; j++) {
          const pcl::PointXYZRGB &pcl_pt = pointcloud_color->at(j, i);
          double Z = pcl_pt.z;
          if (Z > 1e-2) {
            double x = pcl_pt.x / Z;
            double y = pcl_pt.y / Z;

            vpImagePoint imPt;
            vpMeterPixelConversion::convertPoint(cam_projection, x, y, imPt);
            unsigned int u =
              std::min<unsigned int>(static_cast<unsigned int>(width - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_u())));
            unsigned int v =
              std::min<unsigned int>(static_cast<unsigned int>(height - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_v())));
            I_pcl[v][u] = vpRGBa(pcl_pt.r, pcl_pt.g, pcl_pt.b);
          }
        }
      }
    }
    else {
      createDepthHist(histogram, pointcloud, depth_scale);

      for (uint32_t i = 0; i < pointcloud->height; i++) {
        for (uint32_t j = 0; j < pointcloud->width; j++) {
          const pcl::PointXYZ &pcl_pt = pointcloud->at(j, i);
          double Z = pcl_pt.z;
          if (Z > 1e-2) {
            double x = pcl_pt.x / Z;
            double y = pcl_pt.y / Z;

            vpImagePoint imPt;
            vpMeterPixelConversion::convertPoint(cam_projection, x, y, imPt);
            unsigned int u =
              std::min<unsigned int>(static_cast<unsigned int>(width - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_u())));
            unsigned int v =
              std::min<unsigned int>(static_cast<unsigned int>(height - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_v())));
            unsigned char depth_viz = getDepthColor(histogram, pcl_pt.z, depth_scale);
            I_pcl[v][u] = vpRGBa(depth_viz, depth_viz, depth_viz);
          }
        }
      }
    }

    I_pcl2 = vpRGBa(0, 0, 0);
    createDepthHist(histogram2, vp_pointcloud, depth_scale);
    for (size_t i = 0; i < vp_pointcloud.size(); i++) {
      const vpColVector &pt = vp_pointcloud[i];
      double Z = pt[2];
      if (Z > 1e-2) {
        double x = pt[0] / Z;
        double y = pt[1] / Z;

        vpImagePoint imPt;
        vpMeterPixelConversion::convertPoint(cam_projection, x, y, imPt);
        unsigned int u =
          std::min<unsigned int>(static_cast<unsigned int>(width - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_u())));
        unsigned int v =
          std::min<unsigned int>(static_cast<unsigned int>(height - 1), static_cast<unsigned int>(std::max<double>(0.0, imPt.get_v())));
        unsigned char depth_viz = getDepthColor(histogram2, Z, depth_scale);
        I_pcl2[v][u] = vpRGBa(depth_viz, depth_viz, depth_viz);
      }
    }

    I_display.insert(I_color, vpImagePoint(0, 0));
    I_display.insert(I_depth, vpImagePoint(I_color.getHeight(), 0));

    I_display2.insert(I_color, vpImagePoint(0, 0));
    I_display2.insert(I_pcl, vpImagePoint(I_color.getHeight(), 0));

    I_display3.insert(I_color, vpImagePoint(0, 0));
    I_display3.insert(I_pcl2, vpImagePoint(I_color.getHeight(), 0));

    vpDisplay::display(I_display);
    vpDisplay::display(I_display2);
    vpDisplay::display(I_display3);

    const int nb_lines = 5;
    for (int i = 1; i < nb_lines; i++) {
      const int col_idx = i * (width / nb_lines);
      vpDisplay::displayLine(I_display, 0, col_idx, I_display.getRows() - 1, col_idx, vpColor::green, 2);
      vpDisplay::displayLine(I_display2, 0, col_idx, I_display.getRows() - 1, col_idx, vpColor::green, 2);
      vpDisplay::displayLine(I_display3, 0, col_idx, I_display.getRows() - 1, col_idx, vpColor::green, 2);
    }

    vpDisplay::flush(I_display);
    vpDisplay::flush(I_display2);
    vpDisplay::flush(I_display3);

    if (vpDisplay::getClick(I_display, false) || vpDisplay::getClick(I_display2, false) ||
        vpDisplay::getClick(I_display3, false)) {
      break;
    }
  }

  return EXIT_SUCCESS;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
