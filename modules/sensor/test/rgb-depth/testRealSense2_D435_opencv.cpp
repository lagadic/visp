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
 * Test Intel RealSense acquisition with librealsense2 (OpenCV demo).
 */
/*!
  \example testRealSense2_D435_opencv.cpp
  Test Intel RealSense D435 acquisition with librealsense2 (OpenCV demo).
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (VISP_HAVE_OPENCV_VERSION >= 0x030000)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/sensor/vpRealSense2.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace
{
struct float3
{
  float x, y, z;
  float3() : x(0), y(0), z(0) { }
  float3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { }
};

void getPointcloud(const rs2::depth_frame &depth_frame, std::vector<float3> &pointcloud)
{
  auto vf = depth_frame.as<rs2::video_frame>();
  const int width = vf.get_width();
  const int height = vf.get_height();
  pointcloud.resize((size_t)(width * height));

  rs2::pointcloud pc;
  rs2::points points = pc.calculate(depth_frame);
  auto vertices = points.get_vertices();
  for (size_t i = 0; i < points.size(); i++) {
    float3 pcl;
    if (vertices[i].z > std::numeric_limits<float>::epsilon()) {
      pcl.x = vertices[i].x;
      pcl.y = vertices[i].y;
      pcl.z = vertices[i].z;
    }

    pointcloud[i] = pcl;
  }
}

void createDepthHist(std::vector<uint32_t> &histogram, const std::vector<float3> &pointcloud, float depth_scale)
{
  std::fill(histogram.begin(), histogram.end(), 0);

  for (size_t i = 0; i < pointcloud.size(); i++) {
    const float3 &pt = pointcloud[i];
    ++histogram[static_cast<uint32_t>(pt.z * depth_scale)];
  }

  for (int i = 2; i < 0x10000; i++)
    histogram[i] += histogram[i - 1]; // Build a cumulative histogram for
                                      // the indices in [1,0xFFFF]
}

unsigned char getDepthColor(const std::vector<uint32_t> &histogram, float z, float depth_scale)
{
  // 0-255 based on histogram location
  return static_cast<unsigned char>(histogram[static_cast<uint32_t>(z * depth_scale)] * 255 / histogram[0xFFFF]);
}

void getNativeFrame(const rs2::frame &frame, unsigned char *const data)
{
  auto vf = frame.as<rs2::video_frame>();
  int size = vf.get_width() * vf.get_height();

  switch (frame.get_profile().format()) {
  case RS2_FORMAT_RGB8:
  case RS2_FORMAT_BGR8:
    memcpy(data, frame.get_data(), size * 3);
    break;

  case RS2_FORMAT_RGBA8:
  case RS2_FORMAT_BGRA8:
    memcpy(data, frame.get_data(), size * 4);
    break;

  case RS2_FORMAT_Y16:
  case RS2_FORMAT_Z16:
    memcpy(data, frame.get_data(), size * 2);
    break;

  case RS2_FORMAT_Y8:
    memcpy(data, frame.get_data(), size);
    break;

  default:
    break;
  }
}

void frame_to_mat(const rs2::frame &f, cv::Mat &img)
{
  auto vf = f.as<rs2::video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();
  const int size = w * h;

  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    memcpy(static_cast<void *>(img.ptr<cv::Vec3b>()), f.get_data(), size * 3);
  }
  else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    cv::Mat tmp(h, w, CV_8UC3, const_cast<void *>(f.get_data()), cv::Mat::AUTO_STEP);
    cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
  }
  else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    memcpy(img.ptr<uchar>(), f.get_data(), size);
  }
}
} // namespace

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  const int width = 640, height = 480, fps = 60;
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
  config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
  config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
  rs.open(config);

  rs2::pipeline_profile &profile = rs.getPipelineProfile();
  rs2::pipeline &pipe = rs.getPipeline();
  float depth_scale = 1 / rs.getDepthScale();

  // initialize the image sizes
  // width and height can also be used instead
  auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  cv::Mat mat_color(color_profile.height(), color_profile.width(), CV_8UC3);

  auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  cv::Mat mat_depth(depth_profile.height(), depth_profile.width(), CV_8UC3);
  rs2::colorizer color_map;

  auto infrared_profile = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
  cv::Mat mat_infrared1(infrared_profile.height(), infrared_profile.width(), CV_8UC1);
  cv::Mat mat_infrared2(infrared_profile.height(), infrared_profile.width(), CV_8UC1);

  std::vector<float3> pointcloud;
  cv::Mat mat_pointcloud(depth_profile.height(), depth_profile.width(), CV_8UC1);
  std::vector<uint32_t> histogram(0x10000);

  vpCameraParameters cam_projection = rs.getCameraParameters(RS2_STREAM_DEPTH);

  std::vector<double> time_vector;
  vpChrono chrono;
  while (true) {
    chrono.start();

    auto data = pipe.wait_for_frames();
    frame_to_mat(data.get_color_frame(), mat_color);
#if (RS2_API_VERSION >= ((2 * 10000) + (16 * 100) + 0))
    frame_to_mat(data.get_depth_frame().apply_filter(color_map), mat_depth);
#else
    frame_to_mat(color_map(data.get_depth_frame()), mat_depth);
#endif

    cv::imshow("OpenCV color", mat_color);
    cv::imshow("OpenCV depth", mat_depth);

#if (RS2_API_VERSION >= ((2 * 10000) + (10 * 100) + 0))
    // rs2::frameset::get_infrared_frame() introduced in librealsense 2.10.0
    frame_to_mat(data.get_infrared_frame(1), mat_infrared1);
    frame_to_mat(data.get_infrared_frame(2), mat_infrared2);

    cv::imshow("OpenCV infrared left", mat_infrared1);
    cv::imshow("OpenCV infrared right", mat_infrared2);
#endif

    getPointcloud(data.get_depth_frame(), pointcloud);
    createDepthHist(histogram, pointcloud, depth_scale);

    mat_pointcloud = 0;
    for (size_t i = 0; i < pointcloud.size(); i++) {
      const float3 &pt = pointcloud[i];
      float Z = pt.z;
      if (Z > 1e-2) {
        double x = pt.x / Z;
        double y = pt.y / Z;

        vpImagePoint imPt;
        vpMeterPixelConversion::convertPoint(cam_projection, x, y, imPt);
        int u = std::min<int>(static_cast<int>(width - 1), static_cast<int>(std::max<double>(0.0, imPt.get_u())));
        int v = std::min<int>(static_cast<int>(height - 1), static_cast<int>(std::max<double>(0.0, imPt.get_v())));
        unsigned char depth_viz = getDepthColor(histogram, Z, depth_scale);
        mat_pointcloud.at<uchar>(v, u) = depth_viz;
      }
    }
    cv::imshow("OpenCV projected pointcloud", mat_pointcloud);

    chrono.stop();
    time_vector.push_back(chrono.getDurationMs());
    if (cv::waitKey(5) == 27 || cv::waitKey(5) == 113) { // Esc or q
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
#if !(VISP_HAVE_OPENCV_VERSION >= 0x030000)
  std::cout << "Install OpenCV version >= 3 to make this test work." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
