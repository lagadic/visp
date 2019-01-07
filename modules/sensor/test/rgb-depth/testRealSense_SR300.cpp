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
 * Test Intel RealSense acquisition.
 *
 *****************************************************************************/

/*!
  \example testRealSense_SR300.cpp
  Test Intel RealSense SR300 acquisition.
*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense.h>

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY) &&                                          \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
#include <mutex>
#include <thread>

#ifdef VISP_HAVE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
#endif //#ifdef VISP_HAVE_PCL

void test_SR300(vpRealSense &rs, const std::map<rs::stream, bool> &enables,
                const std::map<rs::stream, vpRealSense::vpRsStreamParams> &params, const std::string &title,
                const bool depth_color_visualization = false, const rs::stream &color_stream = rs::stream::color,
                const rs::stream &depth_stream = rs::stream::depth, bool display_pcl = false, bool pcl_color = false)
{
  std::cout << std::endl;

  std::map<rs::stream, bool>::const_iterator it_enable;
  std::map<rs::stream, vpRealSense::vpRsStreamParams>::const_iterator it_param;

  for (it_enable = enables.begin(), it_param = params.begin(); it_enable != enables.end(); ++it_enable) {
    rs.setEnableStream(it_enable->first, it_enable->second);

    if (it_enable->second) {
      it_param = params.find(it_enable->first);

      if (it_param != params.end()) {
        rs.setStreamSettings(it_param->first, it_param->second);
      }
    }
  }

  rs.open();

  vpImage<uint16_t> depth;
  vpImage<unsigned char> I_depth;
  vpImage<vpRGBa> I_depth_color;

  vpImage<vpRGBa> I_color;
  vpImage<uint16_t> infrared;
  vpImage<unsigned char> I_infrared;

  bool direct_infrared_conversion = false;
  for (it_enable = enables.begin(); it_enable != enables.end(); ++it_enable) {
    if (it_enable->second) {
      switch (it_enable->first) {
      case rs::stream::color:
        I_color.init((unsigned int)rs.getIntrinsics(it_enable->first).height,
                     (unsigned int)rs.getIntrinsics(it_enable->first).width);
        break;

      case rs::stream::depth:
        depth.init((unsigned int)rs.getIntrinsics(it_enable->first).height,
                   (unsigned int)rs.getIntrinsics(it_enable->first).width);
        I_depth.init(depth.getHeight(), depth.getWidth());
        I_depth_color.init(depth.getHeight(), depth.getWidth());
        break;

      case rs::stream::infrared:
        infrared.init((unsigned int)rs.getIntrinsics(it_enable->first).height,
                      (unsigned int)rs.getIntrinsics(it_enable->first).width);
        I_infrared.init(infrared.getHeight(), infrared.getWidth());

        it_param = params.find(it_enable->first);
        if (it_param != params.end()) {
          direct_infrared_conversion = it_param->second.m_streamFormat == rs::format::y8;
        }
        break;

      default:
        break;
      }
    }
  }

#if defined(VISP_HAVE_X11)
  vpDisplayX dc, dd, di;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc, dd, di;
#endif

  for (it_enable = enables.begin(); it_enable != enables.end(); ++it_enable) {
    if (it_enable->second) {
      switch (it_enable->first) {
      case rs::stream::color:
        dc.init(I_color, 0, 0, "Color frame");
        break;

      case rs::stream::depth:
        if (depth_color_visualization) {
          dd.init(I_depth_color, (int)I_color.getWidth() + 80, 0, "Depth frame");
        } else {
          dd.init(I_depth, (int)I_color.getWidth() + 80, 0, "Depth frame");
        }
        break;

      case rs::stream::infrared:
        di.init(I_infrared, 0, (int)(std::max)(I_color.getHeight(), I_depth.getHeight()) + 30, "Infrared frame");
        break;

      default:
        break;
      }
    }
  }

  if (rs.getHandler()->is_stream_enabled(rs::stream::infrared)) {
    std::cout << "direct_infrared_conversion=" << direct_infrared_conversion << std::endl;
  }

#ifdef VISP_HAVE_PCL
  std::mutex mutex;
  ViewerWorker viewer(pcl_color, mutex);
  std::thread viewer_thread;

  if (display_pcl) {
    viewer_thread = std::thread(&ViewerWorker::run, &viewer);
  }
#else
  display_pcl = false;
#endif

  // Test stream acquisition during 10 s
  std::vector<double> time_vector;
  double t_begin = vpTime::measureTimeMs();
  while (true) {
    double t = vpTime::measureTimeMs();

    if (display_pcl) {
#ifdef VISP_HAVE_PCL
      std::lock_guard<std::mutex> lock(mutex);

      if (direct_infrared_conversion) {
        if (pcl_color) {
          rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud_color,
                     (unsigned char *)I_infrared.bitmap, NULL, color_stream, depth_stream);
        } else {
          rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud,
                     (unsigned char *)I_infrared.bitmap, NULL, color_stream, depth_stream);
        }
      } else {
        if (pcl_color) {
          rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud_color,
                     (unsigned char *)infrared.bitmap, NULL, color_stream, depth_stream);
        } else {
          rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL, pointcloud,
                     (unsigned char *)infrared.bitmap, NULL, color_stream, depth_stream);
        }

        vpImageConvert::convert(infrared, I_infrared);
      }

      update_pointcloud = true;
#endif
    } else {
      if (direct_infrared_conversion) {
        rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL,
                   (unsigned char *)I_infrared.bitmap, NULL, color_stream, depth_stream);
      } else {
        rs.acquire((unsigned char *)I_color.bitmap, (unsigned char *)depth.bitmap, NULL,
                   (unsigned char *)infrared.bitmap, NULL, color_stream, depth_stream);
        vpImageConvert::convert(infrared, I_infrared);
      }
    }

    if (depth_color_visualization) {
      vpImageConvert::createDepthHistogram(depth, I_depth_color);
    } else {
      vpImageConvert::createDepthHistogram(depth, I_depth);
    }

    vpDisplay::display(I_color);
    if (depth_color_visualization) {
      vpDisplay::display(I_depth_color);
    } else {
      vpDisplay::display(I_depth);
    }
    vpDisplay::display(I_infrared);

    vpDisplay::flush(I_color);
    if (depth_color_visualization) {
      vpDisplay::flush(I_depth_color);
    } else {
      vpDisplay::flush(I_depth);
    }
    vpDisplay::flush(I_infrared);

    if (vpDisplay::getClick(I_color, false) ||
        (depth_color_visualization ? vpDisplay::getClick(I_depth_color, false) : vpDisplay::getClick(I_depth, false)) ||
        vpDisplay::getClick(I_infrared, false)) {
      break;
    }

    time_vector.push_back(vpTime::measureTimeMs() - t);
    if (vpTime::measureTimeMs() - t_begin >= 10000) {
      break;
    }
  }

  if (display_pcl) {
#ifdef VISP_HAVE_PCL
    {
      std::lock_guard<std::mutex> lock(mutex);
      cancelled = true;
    }

    viewer_thread.join();
#endif
  }

  std::cout << title << " - Mean time: " << vpMath::getMean(time_vector)
            << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

  rs.close();
}

} // namespace

int main(int argc, char *argv[])
{
  try {
    vpRealSense rs;

    rs.setEnableStream(rs::stream::color, false);
    rs.open();
    if (rs_get_device_name((const rs_device *)rs.getHandler(), nullptr) != std::string("Intel RealSense SR300")) {
      std::cout << "This test file is used to test the Intel RealSense SR300 only." << std::endl;
      return EXIT_SUCCESS;
    }

    std::cout << "API version: " << rs_get_api_version(nullptr) << std::endl;
    std::cout << "Firmware: " << rs_get_device_firmware_version((const rs_device *)rs.getHandler(), nullptr)
              << std::endl;
    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

    rs.close();

    std::map<rs::stream, bool> enables;
    std::map<rs::stream, vpRealSense::vpRsStreamParams> params;

    enables[rs::stream::color] = false;
    enables[rs::stream::depth] = true;
    enables[rs::stream::infrared] = false;

    params[rs::stream::depth] = vpRealSense::vpRsStreamParams(640, 240, rs::format::z16, 110);

    test_SR300(rs, enables, params, "SR300_DEPTH_Z16_640x240_110FPS");

    enables[rs::stream::color] = true;
    enables[rs::stream::depth] = false;
    enables[rs::stream::infrared] = false;

    params[rs::stream::color] = vpRealSense::vpRsStreamParams(1920, 1080, rs::format::rgba8, 30);

    test_SR300(rs, enables, params, "SR300_COLOR_RGBA8_1920x1080_30FPS");

    enables[rs::stream::color] = false;
    enables[rs::stream::depth] = false;
    enables[rs::stream::infrared] = true;

    params[rs::stream::infrared] = vpRealSense::vpRsStreamParams(640, 480, rs::format::y16, 200);

    test_SR300(rs, enables, params, "SR300_INFRARED_Y16_640x480_200FPS");

    enables[rs::stream::color] = false;
    enables[rs::stream::depth] = true;
    enables[rs::stream::infrared] = false;

    params[rs::stream::depth] = vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60);

    test_SR300(rs, enables, params, "SR300_DEPTH_Z16_640x480_60FPS", true);

    enables[rs::stream::color] = false;
    enables[rs::stream::depth] = true;
    enables[rs::stream::infrared] = true;

    params[rs::stream::depth] = vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60);
    params[rs::stream::infrared] = vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 60);

    test_SR300(rs, enables, params, "SR300_DEPTH_Z16_640x480_60FPS + SR300_INFRARED_Y8_640x480_60FPS", true);

#if (!defined(__APPLE__) && !defined(__MACH__)) // Not OSX, since viewer->spinOnce (10); produces a
                                                // segfault on OSX

    enables[rs::stream::color] = true;
    enables[rs::stream::depth] = true;
    enables[rs::stream::infrared] = true;

    params[rs::stream::color] = vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 60);
    params[rs::stream::depth] = vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60);
    params[rs::stream::infrared] = vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 60);

    // depth == 0 ; color == 1 ; rectified_color == 6 ; color_aligned_to_depth
    // == 7 ; depth_aligned_to_color == 9 ; depth_aligned_to_rectified_color
    // == 10  argv[1] <==> color stream
    rs::stream color_stream = argc > 1 ? (rs::stream)atoi(argv[1]) : rs::stream::color;
    std::cout << "\ncolor_stream: " << color_stream << std::endl;
    // argv[2] <==> depth stream
    rs::stream depth_stream = argc > 2 ? (rs::stream)atoi(argv[2]) : rs::stream::depth;
    std::cout << "depth_stream: " << depth_stream << std::endl;

    test_SR300(rs, enables, params,
               "SR300_COLOR_RGBA8_640x480_60FPS + "
               "SR300_DEPTH_Z16_640x480_60FPS + "
               "SR300_INFRARED_Y8_640x480_60FPS",
               true, color_stream, depth_stream, true, true);
#endif

    // Color stream aligned to depth
    enables[rs::stream::color] = true;
    enables[rs::stream::depth] = true;
    enables[rs::stream::infrared] = false;

    params[rs::stream::color] = vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30);
    params[rs::stream::depth] = vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 30);

    test_SR300(rs, enables, params,
               "SR300_COLOR_ALIGNED_TO_DEPTH_RGBA8_640x480_30FPS + "
               "SR300_DEPTH_Z16_640x480_30FPS",
               true, rs::stream::color_aligned_to_depth);

    // Depth stream aligned to color
    test_SR300(rs, enables, params,
               "SR300_COLOR_RGBA8_640x480_30FPS + "
               "SR300_DEPTH_ALIGNED_TO_COLOR_Z16_640x480_30FPS",
               true, rs::stream::color, rs::stream::depth_aligned_to_color);

#if VISP_HAVE_OPENCV_VERSION >= 0x020409
    rs.setEnableStream(rs::stream::color, true);
    rs.setEnableStream(rs::stream::depth, false);
    rs.setEnableStream(rs::stream::infrared, true);
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::bgr8, 60));
    rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 200));
    rs.open();

    cv::Mat color_mat(480, 640, CV_8UC3);
    cv::Mat infrared_mat(480, 640, CV_8U);

    double t_begin = vpTime::measureTimeMs();
    while (true) {
      rs.acquire(color_mat.data, NULL, NULL, infrared_mat.data);

      cv::imshow("Color mat", color_mat);
      cv::imshow("Infrared mat", infrared_mat);
      char c = cv::waitKey(1);
      if (c == 27) {
        break;
      }

      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
#endif

  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const rs::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args()
              << "): " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}

#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE)
  std::cout << "Install librealsense to make this test work." << std::endl;
#endif
#if !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "Build ViSP with C++11 compiler flag (cmake -DUSE_CPP11=ON) "
               "to make this test work."
            << std::endl;
#endif
#if !defined(VISP_HAVE_X11) && !defined(VISP_HAVE_GDI)
  std::cout << "X11 or GDI are needed." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
