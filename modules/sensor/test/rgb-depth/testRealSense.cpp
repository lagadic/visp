/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
  \example testRealSense.cpp
  Test SR300 Intel RealSense acquisition.
*/

#include <iostream>

#include <visp3/sensor/vpRealSense.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)
#  include <librealsense/rs.h>

// Using a thread to display the pointcloud with PCL produces a segfault on OSX
#if( ! defined(__APPLE__) && ! defined(__MACH__) ) // Not OSX
#  if (defined(VISP_HAVE_PTHREAD) || defined(_WIN32)) // Threading available
#    define USE_THREAD
#  endif
#endif

#ifdef VISP_HAVE_PCL
#  include <pcl/visualization/cloud_viewer.h>
#  include <pcl/visualization/pcl_visualizer.h>
#endif

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
vpMutex s_mutex_capture;


vpThread::Return displayPointcloudFunction(vpThread::Args args)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ = *((pcl::PointCloud<pcl::PointXYZRGB>::Ptr *) args);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud_);
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  viewer->setPosition(2*640+80, 480+80);
  viewer->setCameraPosition(0,0,-0.5, 0,-1,0);
  viewer->setSize(640, 480);

  t_CaptureState capture_state_;

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    if (capture_state_ == capture_started) {
      static bool update = false;
      if (! update) {
        viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud_, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        update = true;
      }
      else {
        viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud_, rgb, "sample cloud");
      }

      viewer->spinOnce (10);
    }
  } while(capture_state_ != capture_stopped);

  std::cout << "End of point cloud display thread" << std::endl;
  return 0;
}
#endif
#endif

#endif


int main(
#if defined(VISP_HAVE_PCL)
    int argc, char *argv[]
#endif
    )
{
#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY) && ( defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) )
  try {
    vpRealSense rs;

    rs.setEnableStream(rs::stream::color, false);
    rs.open();
    if ( rs_get_device_name((const rs_device *) rs.getHandler(), nullptr) != std::string("Intel RealSense SR300") ) {
      std::cout << "This test file is used to test the Intel RealSense SR300 only." << std::endl;
      return EXIT_SUCCESS;
    }
    rs.close();


    rs.setEnableStream(rs::stream::color, false);
    rs.setEnableStream(rs::stream::depth, true);
    rs.setEnableStream(rs::stream::infrared, false);

    rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 240, rs::format::z16, 110));

    rs.open();
    std::cout << "API version: " << rs_get_api_version(nullptr) << std::endl;
    std::cout << "Firmware: " << rs_get_device_firmware_version((const rs_device *) rs.getHandler(), nullptr) << std::endl;
    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

    vpImage<uint16_t> depth((unsigned int) rs.getIntrinsics(rs::stream::depth).height, (unsigned int) rs.getIntrinsics(rs::stream::depth).width);
    vpImage<vpRGBa> I_display_depth(depth.getHeight(), depth.getWidth());

    std::vector<double> time_vector;

#if defined(VISP_HAVE_X11)
    vpDisplayX dd(I_display_depth, 0, 0, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dd(I_display_depth, 0, 0, "Depth image");
#endif

    //Test depth stream during 10 s
    double t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( NULL, (unsigned char *) depth.bitmap, NULL, NULL );
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

      vpDisplay::display(I_display_depth);
      vpDisplay::flush(I_display_depth);

      if (vpDisplay::getClick(I_display_depth, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_DEPTH_Z16_640x240_110FPS - Mean time: " << vpMath::getMean(time_vector) << " ms ; Median time: "
              << vpMath::getMedian(time_vector) << " ms" << std::endl;

    dd.close(I_display_depth);


    rs.setEnableStream(rs::stream::color, true);
    rs.setEnableStream(rs::stream::depth, false);
    rs.setEnableStream(rs::stream::infrared, false);
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(1920, 1080, rs::format::rgba8, 30));
    rs.open();

    vpImage<vpRGBa> I_color((unsigned int) rs.getIntrinsics(rs::stream::color).height, (unsigned int) rs.getIntrinsics(rs::stream::color).width);

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I_color, 0, 0, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I_color, 0, 0, "Color image");
#endif

    time_vector.clear();
    //Test color stream during 10 s
    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( (unsigned char *) I_color.bitmap, NULL, NULL, NULL );

      vpDisplay::display(I_color);
      vpDisplay::flush(I_color);

      if (vpDisplay::getClick(I_color, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_COLOR_RGBA8_1920x1080_30FPS - Mean time: " << vpMath::getMean(time_vector) << " ms ; Median time: "
              << vpMath::getMedian(time_vector) << " ms" << std::endl;

    dc.close(I_color);


    rs.setEnableStream(rs::stream::color, false);
    rs.setEnableStream(rs::stream::depth, false);
    rs.setEnableStream(rs::stream::infrared, true);
    rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y16, 200));
    rs.open();

    vpImage<uint16_t> infrared((unsigned int) rs.getIntrinsics(rs::stream::infrared).height, (unsigned int) rs.getIntrinsics(rs::stream::infrared).width);
    vpImage<unsigned char> I_display_infrared(infrared.getHeight(), infrared.getWidth());

#if defined(VISP_HAVE_X11)
    vpDisplayX di(I_display_infrared, 0, 0, "Infrared image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI di(I_display_infrared, 0, 0, "Infrared image");
#endif

    time_vector.clear();
    //Test infrared stream during 10 s
    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( NULL, NULL, NULL, (unsigned char *) infrared.bitmap );
      vpImageConvert::convert(infrared, I_display_infrared);

      vpDisplay::display(I_display_infrared);
      vpDisplay::flush(I_display_infrared);

      if (vpDisplay::getClick(I_display_infrared, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_INFRARED_Y16_640x480_200FPS - Mean time: " << vpMath::getMean(time_vector) << " ms ; Median time: "
              << vpMath::getMedian(time_vector) << " ms" << std::endl;

    di.close(I_display_infrared);


    rs.setEnableStream(rs::stream::color, false);
    rs.setEnableStream(rs::stream::depth, true);
    rs.setEnableStream(rs::stream::infrared, false);
    rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60));
    rs.open();

    depth.resize( (unsigned int) rs.getIntrinsics(rs::stream::depth).height, (unsigned int) rs.getIntrinsics(rs::stream::depth).width );
    I_display_depth.resize( depth.getHeight(), depth.getWidth() );

#if defined(VISP_HAVE_X11)
    dd.init(I_display_depth, 0, 0, "Depth image");
#elif defined(VISP_HAVE_GDI)
    dd.init(I_display_depth, 0, 0, "Depth image");
#endif

    time_vector.clear();
    //Test depth stream during 10 s
    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( NULL, (unsigned char *) depth.bitmap, NULL, NULL );
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

      vpDisplay::display(I_display_depth);
      vpDisplay::flush(I_display_depth);

      if (vpDisplay::getClick(I_display_depth, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_DEPTH_Z16_640x480_60FPS - Mean time: " << vpMath::getMean(time_vector) << " ms ; Median time: "
              << vpMath::getMedian(time_vector) << " ms" << std::endl;

    dd.close(I_display_depth);


    rs.setEnableStream(rs::stream::color, false);
    rs.setEnableStream(rs::stream::depth, true);
    rs.setEnableStream(rs::stream::infrared, true);
    rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60));
    rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 60));
    rs.open();

#if defined(VISP_HAVE_X11)
    dd.init(I_display_depth, 0, 0, "Depth image");
    di.init(I_display_infrared, (int) I_display_depth.getWidth(), 0, "Infrared image");
#elif defined(VISP_HAVE_GDI)
    dd.init(I_display_depth, 0, 0, "Depth image");
    di.init(I_display_infrared, I_display_depth.getWidth(), 0, "Infrared image");
#endif

    time_vector.clear();
    //Test depth stream during 10 s
    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( NULL, (unsigned char *) depth.bitmap, NULL, (unsigned char *) I_display_infrared.bitmap );
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

      vpDisplay::display(I_display_depth);
      vpDisplay::display(I_display_infrared);
      vpDisplay::flush(I_display_depth);
      vpDisplay::flush(I_display_infrared);

      if (vpDisplay::getClick(I_display_depth, false) || vpDisplay::getClick(I_display_infrared, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_DEPTH_Z16_640x480_60FPS + SR300_INFRARED_Y8_640x480_60FPS - Mean time: " << vpMath::getMean(time_vector)
              << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;


    dd.close(I_display_depth);
    di.close(I_display_infrared);



#ifdef VISP_HAVE_PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

#ifdef USE_THREAD
    vpThread thread_display_pointcloud(displayPointcloudFunction, (vpThread::Args)&pointcloud);
#else
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setPosition(2*640+80, 480+80);
    viewer->setCameraPosition(0,0,-0.5, 0,-1,0);
    viewer->setSize(640, 480);
#endif

    rs.setEnableStream(rs::stream::color, true);
    rs.setEnableStream(rs::stream::depth, true);
    rs.setEnableStream(rs::stream::infrared, true);
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 60));
    rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 60));
    rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 60));
    rs.open();

    I_color.resize(480, 640);

#if defined(VISP_HAVE_X11)
    dc.init(I_color, 0, 0, "Color image");
    dd.init(I_display_depth, 0, (int) I_color.getHeight()+80, "Depth image");
    di.init(I_display_infrared, (int) I_display_depth.getWidth(), 0, "Infrared image");
#elif defined(VISP_HAVE_GDI)
    dc.init(I_color, 0, 0, "Color image");
    dd.init(I_display_depth, 0, (int) I_color.getHeight()+80, "Depth image");
    di.init(I_display_infrared, (int) I_display_depth.getWidth(), 0, "Infrared image");
#endif

    //depth == 0 ; color == 1 ; rectified_color == 6 ; color_aligned_to_depth == 7 ; depth_aligned_to_color == 9 ; depth_aligned_to_rectified_color == 10
    //argv[1] <==> color stream
    rs::stream color_stream = argc > 1 ? (rs::stream) atoi(argv[1]) : rs::stream::color;
    std::cout << "color_stream: " << color_stream << std::endl;
    //argv[2] <==> depth stream
    rs::stream depth_stream = argc > 2 ? (rs::stream) atoi(argv[2]) : rs::stream::depth;
    std::cout << "depth_stream: " << depth_stream << std::endl;

    time_vector.clear();
    //Test depth stream during 10 s
    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( (unsigned char *) I_color.bitmap, (unsigned char *) depth.bitmap, NULL, pointcloud, (unsigned char *) I_display_infrared.bitmap,
                  NULL, color_stream, depth_stream);
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        s_capture_state = capture_started;
      }
#else
      static bool update = false;
        if (! update) {
          viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
          viewer->setPosition( (int) I_color.getWidth()+80, (int) I_color.getHeight()+80) ;
          update = true;
        }
        else {
          viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
        }

        viewer->spinOnce (10);
#endif
#endif

      vpDisplay::display(I_color);
      vpDisplay::display(I_display_depth);
      vpDisplay::display(I_display_infrared);
      vpDisplay::flush(I_color);
      vpDisplay::flush(I_display_depth);
      vpDisplay::flush(I_display_infrared);

      if (vpDisplay::getClick(I_color, false) || vpDisplay::getClick(I_display_depth, false) || vpDisplay::getClick(I_display_infrared, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
    std::cout << "SR300_COLOR_RGBA8_640x480_60FPS + SR300_DEPTH_Z16_640x480_60FPS + SR300_INFRARED_Y8_640x480_60FPS - Mean time: " << vpMath::getMean(time_vector)
              << " ms ; Median time: " << vpMath::getMedian(time_vector) << " ms" << std::endl;

#ifdef VISP_HAVE_PCL
#ifdef USE_THREAD
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      s_capture_state = capture_stopped;
    }
#endif
#endif

    dc.close(I_color);
    dd.close(I_display_depth);
    di.close(I_display_infrared);

#endif


    //Color stream aligned to depth
    rs.setEnableStream(rs::stream::color, true);
    rs.setEnableStream(rs::stream::depth, true);
    rs.setEnableStream(rs::stream::infrared, false);
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30));
    rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 30));
    rs.open();

    dc.init(I_color, 0, 0, "Color aligned to depth");
    di.init(I_display_depth, (int) I_color.getWidth(), 0, "Depth image");

    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( (unsigned char *) I_color.bitmap, (unsigned char *) depth.bitmap, NULL, NULL, NULL, rs::stream::color_aligned_to_depth );
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

      vpDisplay::display(I_color);
      vpDisplay::display(I_display_depth);
      vpDisplay::flush(I_color);
      vpDisplay::flush(I_display_depth);

      if (vpDisplay::getClick(I_color, false) || vpDisplay::getClick(I_display_depth, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }

    dc.close(I_color);
    dd.close(I_display_depth);


    //Depth stream aligned to color
    dc.init(I_color, 0, 0, "Color image");
    di.init(I_display_depth, (int) I_color.getWidth(), 0, "Depth aligned to color");

    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( (unsigned char *) I_color.bitmap, (unsigned char *) depth.bitmap, NULL, NULL, NULL, rs::stream::color, rs::stream::depth_aligned_to_color );
      vpImageConvert::createDepthHistogram(depth, I_display_depth);

      vpDisplay::display(I_color);
      vpDisplay::display(I_display_depth);
      vpDisplay::flush(I_color);
      vpDisplay::flush(I_display_depth);

      if (vpDisplay::getClick(I_color, false) || vpDisplay::getClick(I_display_depth, false)) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }

    dc.close(I_color);
    dd.close(I_display_depth);



#if VISP_HAVE_OPENCV_VERSION >= 0x020409
    rs.setEnableStream(rs::stream::color, true);
    rs.setEnableStream(rs::stream::depth, false);
    rs.setEnableStream(rs::stream::infrared, true);
    rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::bgr8, 60));
    rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 200));
    rs.open();

    cv::Mat color_mat(480, 640, CV_8UC3);
    cv::Mat infrared_mat(480, 640, CV_8U);

    t_begin = vpTime::measureTimeMs();
    while (true) {
      double t = vpTime::measureTimeMs();
      rs.acquire( color_mat.data, NULL, NULL, infrared_mat.data );

      cv::imshow("Color mat", color_mat);
      cv::imshow("Infrared mat", infrared_mat);
      char c = cv::waitKey(10);
      if (c == 27) {
        break;
      }

      time_vector.push_back(vpTime::measureTimeMs() - t);
      if (vpTime::measureTimeMs() - t_begin >= 10000) {
        break;
      }
    }
#endif

  } catch(const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch(const rs::error & e)  {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
  } catch(const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

#elif !defined(VISP_HAVE_REALSENSE)
  std::cout << "Install RealSense SDK to make this test working. X11 or GDI are needed also." << std::endl;
#elif !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "Build ViSP with c++11 compiler flag (cmake -DUSE_CPP11=ON) to make this test working" << std::endl;
#endif
  return EXIT_SUCCESS;
}
