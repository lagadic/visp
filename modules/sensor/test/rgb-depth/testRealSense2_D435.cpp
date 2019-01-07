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
 * Test Intel RealSense acquisition with librealsense2.
 *
 *****************************************************************************/
/*!
  \example testRealSense2_D435.cpp
  Test Intel RealSense D435 acquisition with librealsense2.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_CPP11_COMPATIBILITY) &&                                         \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

#ifdef VISP_HAVE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#endif

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace
{
#ifdef VISP_HAVE_PCL
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

void getPointcloud(const rs2::depth_frame &depth_frame, std::vector<vpColVector> &pointcloud)
{
  auto vf = depth_frame.as<rs2::video_frame>();
  const int width = vf.get_width();
  const int height = vf.get_height();
  pointcloud.resize((size_t)(width * height));

  rs2::pointcloud pc;
  rs2::points points = pc.calculate(depth_frame);
  auto vertices = points.get_vertices();
  vpColVector v(4);
  for (size_t i = 0; i < points.size(); i++) {
    if (vertices[i].z) {
      v[0] = vertices[i].x;
      v[1] = vertices[i].y;
      v[2] = vertices[i].z;
      v[3] = 1.0;
    } else {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      v[3] = 1.0;
    }

    pointcloud[i] = v;
  }
}
#endif

void getNativeFrame(const rs2::frame &frame, unsigned char *const data)
{
  auto vf = frame.as<rs2::video_frame>();
  int size = vf.get_width() * vf.get_height();

  switch (frame.get_profile().format()) {
  case RS2_FORMAT_RGB8:
  case RS2_FORMAT_BGR8:
    memcpy(data, (void *)frame.get_data(), size * 3);
    break;

  case RS2_FORMAT_RGBA8:
  case RS2_FORMAT_BGRA8:
    memcpy(data, (void *)frame.get_data(), size * 4);
    break;

  case RS2_FORMAT_Y16:
  case RS2_FORMAT_Z16:
    memcpy(data, (unsigned char *)frame.get_data(), size * 2);
    break;

  case RS2_FORMAT_Y8:
    memcpy(data, (unsigned char *)frame.get_data(), size);
    break;

  default:
    break;
  }
}

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
void frame_to_mat(const rs2::frame &f, cv::Mat &img)
{
  auto vf = f.as<rs2::video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();
  const int size = w * h;

  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    memcpy(static_cast<void*>(img.ptr<cv::Vec3b>()), f.get_data(), size * 3);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    cv::Mat tmp(h, w, CV_8UC3, (void *)f.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    memcpy(img.ptr<uchar>(), f.get_data(), size);
  }
}
#endif
}

int main(int argc, char *argv[])
{
#ifdef VISP_HAVE_PCL
  bool pcl_color = false;
#endif
  bool show_info = false;

  for (int i = 1; i < argc; i++) {
#ifdef VISP_HAVE_PCL
    if (std::string(argv[i]) == "--pcl_color") {
      pcl_color = true;
    } else
#endif
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

  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
  rs.open(config);

  rs2::pipeline_profile &profile = rs.getPipelineProfile();
  auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  vpImage<vpRGBa> color((unsigned int)color_profile.height(), (unsigned int)color_profile.width());

  auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  vpImage<vpRGBa> depth_color((unsigned int)depth_profile.height(), (unsigned int)depth_profile.width());
  vpImage<uint16_t> depth_raw((unsigned int)depth_profile.height(), (unsigned int)depth_profile.width());

  auto infrared_profile = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
  vpImage<unsigned char> infrared1((unsigned int)infrared_profile.height(), (unsigned int)infrared_profile.width());
  vpImage<unsigned char> infrared2((unsigned int)infrared_profile.height(), (unsigned int)infrared_profile.width());

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
#ifdef VISP_HAVE_PCL
  std::mutex mutex;
  ViewerWorker viewer_colvector(false, mutex);
  std::thread viewer_colvector_thread(&ViewerWorker::run, &viewer_colvector);
#endif

  rs2::pipeline &pipe = rs.getPipeline();

  std::cout << "Color intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Color intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion) << std::endl;

  std::cout << "Depth intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Depth intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithDistortion) << std::endl;

  std::cout << "Infrared intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_INFRARED, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Infrared intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_INFRARED, vpCameraParameters::perspectiveProjWithDistortion)
            << std::endl;

  std::cout << "depth_M_color:\n" << rs.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH) << std::endl;
  std::cout << "color_M_infrared:\n" << rs.getTransformation(RS2_STREAM_INFRARED, RS2_STREAM_COLOR) << std::endl;

  std::vector<double> time_vector;
  double t_begin = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - t_begin < 10000) {
    double t = vpTime::measureTimeMs();

    auto data = pipe.wait_for_frames();
    auto color_frame = data.get_color_frame();
    getNativeFrame(color_frame, (unsigned char *)color.bitmap);

    auto depth_frame = data.get_depth_frame();
    getNativeFrame(depth_frame, (unsigned char *)depth_raw.bitmap);

#if (RS2_API_VERSION >= ((2 * 10000) + (10 * 100) + 0))
    // rs2::frameset::get_infrared_frame() introduced in librealsense 2.10.0
    auto infrared1_frame = data.get_infrared_frame(1);
    getNativeFrame(infrared1_frame, (unsigned char *)infrared1.bitmap);

    auto infrared2_frame = data.get_infrared_frame(2);
    getNativeFrame(infrared2_frame, (unsigned char *)infrared2.bitmap);
#endif

#ifdef VISP_HAVE_PCL
    getPointcloud(depth_frame, pointcloud_colvector);

    {
      std::lock_guard<std::mutex> lock(mutex);

      pointcloud->width = depth_profile.width();
      pointcloud->height = depth_profile.height();
      pointcloud->points.resize(pointcloud_colvector.size());
      for (size_t i = 0; i < pointcloud_colvector.size(); i++) {
        pointcloud->points[(size_t)i].x = pointcloud_colvector[i][0];
        pointcloud->points[(size_t)i].y = pointcloud_colvector[i][1];
        pointcloud->points[(size_t)i].z = pointcloud_colvector[i][2];
      }

      update_pointcloud = true;
    }
#endif

    vpImageConvert::createDepthHistogram(depth_raw, depth_color);

    vpDisplay::display(color);
    vpDisplay::display(depth_color);
    vpDisplay::display(infrared1);
    vpDisplay::display(infrared2);

    vpDisplay::displayText(color, 20, 20, "Click to quit.", vpColor::red);

    vpDisplay::flush(color);
    vpDisplay::flush(depth_color);
    vpDisplay::flush(infrared1);
    vpDisplay::flush(infrared2);

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (vpDisplay::getClick(color, false))
      break;
  }

  rs.close();
  d1.close(color);
  d2.close(depth_color);
  d3.close(infrared1);
  d4.close(infrared2);

#ifdef VISP_HAVE_PCL
  {
    std::lock_guard<std::mutex> lock(mutex);
    cancelled = true;
  }

  viewer_colvector_thread.join();
#endif
  std::cout << "Acquisition1 - Mean time: " << vpMath::getMean(time_vector)
            << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

  config.disable_all_streams();
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 60);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
  config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 60);
  rs.open(config);

  color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  color.init((unsigned int)color_profile.height(), (unsigned int)color_profile.width());

  depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  depth_color.init((unsigned int)depth_profile.height(), (unsigned int)depth_profile.width());
  depth_raw.init((unsigned int)depth_profile.height(), (unsigned int)depth_profile.width());

  infrared_profile = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
  infrared1.init((unsigned int)infrared_profile.height(), (unsigned int)infrared_profile.width());

  d1.init(color, 0, 0, "Color");
  d2.init(depth_color, color.getWidth(), 0, "Depth");
  d3.init(infrared1, 0, color.getHeight() + 100, "Infrared");

#ifdef VISP_HAVE_PCL
  cancelled = false;
  ViewerWorker viewer(pcl_color, mutex);
  std::thread viewer_thread(&ViewerWorker::run, &viewer);
#endif

  std::cout << "\n" << std::endl;
  std::cout << "Color intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Color intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion) << std::endl;

  std::cout << "Depth intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Depth intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithDistortion) << std::endl;

  std::cout << "Infrared intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_INFRARED, vpCameraParameters::perspectiveProjWithoutDistortion)
            << std::endl;
  std::cout << "Infrared intrinsics:\n"
            << rs.getCameraParameters(RS2_STREAM_INFRARED, vpCameraParameters::perspectiveProjWithDistortion)
            << std::endl;

  std::cout << "depth_M_color:\n" << rs.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH) << std::endl;
  std::cout << "color_M_infrared:\n" << rs.getTransformation(RS2_STREAM_INFRARED, RS2_STREAM_COLOR) << std::endl;

  time_vector.clear();
  t_begin = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - t_begin < 10000) {
    double t = vpTime::measureTimeMs();

#ifdef VISP_HAVE_PCL
    {
      std::lock_guard<std::mutex> lock(mutex);

      if (pcl_color) {
        rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth_raw.bitmap, NULL, pointcloud_color,
                   (unsigned char *)infrared1.bitmap);
      } else {
        rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth_raw.bitmap, NULL, pointcloud,
                   (unsigned char *)infrared1.bitmap);
      }

      update_pointcloud = true;
    }
#else
    rs.acquire((unsigned char *)color.bitmap, (unsigned char *)depth_raw.bitmap, NULL,
               (unsigned char *)infrared1.bitmap);
#endif

    vpImageConvert::createDepthHistogram(depth_raw, depth_color);

    vpDisplay::display(color);
    vpDisplay::display(depth_color);
    vpDisplay::display(infrared1);

    vpDisplay::displayText(color, 20, 20, "Click to quit.", vpColor::red);

    vpDisplay::flush(color);
    vpDisplay::flush(depth_color);
    vpDisplay::flush(infrared1);

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (vpDisplay::getClick(color, false))
      break;
  }

#ifdef VISP_HAVE_PCL
  {
    std::lock_guard<std::mutex> lock(mutex);
    cancelled = true;
  }

  viewer_thread.join();
#endif

  d1.close(color);
  d2.close(depth_color);
  d3.close(infrared1);
  std::cout << "Acquisition2 - Mean time: " << vpMath::getMean(time_vector)
            << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
  rs.close();
  config.disable_all_streams();
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
  config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
  config.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);
  rs.open(config);

  color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  cv::Mat mat_color(color_profile.height(), color_profile.width(), CV_8UC3);

  depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  cv::Mat mat_depth(depth_profile.height(), depth_profile.width(), CV_8UC3);
  rs2::colorizer color_map;

  infrared_profile = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
  cv::Mat mat_infrared1(infrared_profile.height(), infrared_profile.width(), CV_8U);
  cv::Mat mat_infrared2(infrared_profile.height(), infrared_profile.width(), CV_8U);

  time_vector.clear();
  t_begin = vpTime::measureTimeMs();
  while (vpTime::measureTimeMs() - t_begin < 10000) {
    double t = vpTime::measureTimeMs();

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

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (cv::waitKey(10) == 27)
      break;
  }

  std::cout << "Acquisition3 - Mean time: " << vpMath::getMean(time_vector)
            << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;
#endif

  return EXIT_SUCCESS;
}

#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense2 to make this test work." << std::endl;
#endif
#if !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "Build ViSP with C++11 compiler flag (cmake -DUSE_CPP11=ON) "
               "to make this test work"
            << std::endl;
#endif
#if !defined(VISP_HAVE_X11) && !defined(VISP_HAVE_GDI)
  std::cout << "X11 or GDI are needed." << std::endl;
#endif
  return 0;
}
#endif
