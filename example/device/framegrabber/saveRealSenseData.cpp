/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) &&  \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#if defined(VISP_HAVE_PCL)
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#endif

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

// Priority to libRealSense2
#if defined(VISP_HAVE_REALSENSE2)
#define USE_REALSENSE2
#endif

#define GETOPTARGS "so:acdpiCf:bh"

namespace
{
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
            Save RealSense data.\n\
            \n\
            %s\
            OPTIONS:                                               \n\
            -s                                                     \n\
            Save data.\n\
            \n\
            -o <directory>                                         \n\
            Output directory.\n\
            \n\
            -a                                                     \n\
            Use aligned streams.\n\
            \n\
            -c                                                     \n\
            Save color stream.\n\
            \n\
            -d                                                     \n\
            Save depth stream.\n\
            \n\
            -p                                                     \n\
            Save pointcloud.\n\
            \n\
            -i                                                     \n\
            Save infrared stream.\n\
            \n\
            -C                                                     \n\
            Click to save.\n\
            \n\
            -f <fps>                                               \n\
            Set FPS.\n\
            \n\
            -b                                                     \n\
            Save point cloud in binary format.\n\
            \n\
            -h \n\
            Print the help.\n\n",
          name);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, char **argv, bool &save, std::string &output_directory, bool &use_aligned_stream,
                bool &save_color, bool &save_depth, bool &save_pointcloud, bool &save_infrared, bool &click_to_save,
                int &stream_fps, bool &save_pointcloud_binary_format)
{
  const char *optarg;
  const char **argv1 = (const char **)argv;
  int c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 's':
      save = true;
      break;
    case 'o':
      output_directory = optarg;
      break;
    case 'a':
      use_aligned_stream = true;
      break;
    case 'c':
      save_color = true;
      break;
    case 'd':
      save_depth = true;
      break;
    case 'p':
      save_pointcloud = true;
      break;
    case 'i':
      save_infrared = true;
      break;
    case 'C':
      click_to_save = true;
      break;
    case 'f':
      stream_fps = atoi(optarg);
      break;
    case 'b':
      save_pointcloud_binary_format = true;
      break;

    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

class FrameQueue
{
public:
  struct cancelled {
  };

  FrameQueue()
    : m_cancelled(false), m_cond(), m_queueColor(), m_queueDepth(), m_queuePointCloud(), m_queueInfrared(),
      m_maxQueueSize(1024 * 8), m_mutex()
  {
  }

  void cancel()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancelled = true;
    m_cond.notify_all();
  }

  // Push data to save in the queue (FIFO)
  void push(const vpImage<vpRGBa> &colorImg, const vpImage<uint16_t> &depthImg,
#ifdef VISP_HAVE_PCL
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
#else
            const std::vector<vpColVector> &pointCloud,
#endif
            const vpImage<unsigned char> &infraredImg)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_queueColor.push(colorImg);
    m_queueDepth.push(depthImg);
    m_queuePointCloud.push(pointCloud);
    m_queueInfrared.push(infraredImg);

    // Pop extra data in the queue
    while (m_queueColor.size() > m_maxQueueSize) {
      m_queueColor.pop();
    }

    // Pop extra data in the queue
    while (m_queueDepth.size() > m_maxQueueSize) {
      m_queueDepth.pop();
    }

    // Pop extra data in the queue
    while (m_queuePointCloud.size() > m_maxQueueSize) {
      m_queuePointCloud.pop();
    }

    // Pop extra data in the queue
    while (m_queueInfrared.size() > m_maxQueueSize) {
      m_queueInfrared.pop();
    }

    m_cond.notify_one();
  }

  // Pop the image to save from the queue (FIFO)
  void pop(vpImage<vpRGBa> &colorImg, vpImage<uint16_t> &depthImg,
#ifdef VISP_HAVE_PCL
           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
#else
           std::vector<vpColVector> &pointCloud,
#endif
           vpImage<unsigned char> &infraredImg)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    while (m_queueColor.empty() || m_queueDepth.empty() || m_queuePointCloud.empty() || m_queueInfrared.empty()) {
      if (m_cancelled) {
        throw cancelled();
      }

      m_cond.wait(lock);

      if (m_cancelled) {
        throw cancelled();
      }
    }

    colorImg = m_queueColor.front();
    depthImg = m_queueDepth.front();
    pointCloud = m_queuePointCloud.front();
    infraredImg = m_queueInfrared.front();

    m_queueColor.pop();
    m_queueDepth.pop();
    m_queuePointCloud.pop();
    m_queueInfrared.pop();
  }

  void setMaxQueueSize(const size_t max_queue_size) { m_maxQueueSize = max_queue_size; }

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  std::queue<vpImage<vpRGBa> > m_queueColor;
  std::queue<vpImage<uint16_t> > m_queueDepth;
#ifdef VISP_HAVE_PCL
  std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_queuePointCloud;
#else
  std::queue<std::vector<vpColVector> > m_queuePointCloud;
#endif
  std::queue<vpImage<unsigned char> > m_queueInfrared;
  size_t m_maxQueueSize;
  std::mutex m_mutex;
};

class StorageWorker
{
public:
  StorageWorker(FrameQueue &queue, const std::string &directory, bool save_color, bool save_depth, bool save_pointcloud,
                bool save_infrared, bool save_pointcloud_binary_format,
                int
#ifndef VISP_HAVE_PCL
                    width
#endif
                ,
                int
#ifndef VISP_HAVE_PCL
                    height
#endif
                )
    : m_queue(queue), m_directory(directory), m_cpt(0), m_save_color(save_color), m_save_depth(save_depth),
      m_save_pointcloud(save_pointcloud), m_save_infrared(save_infrared),
      m_save_pointcloud_binary_format(save_pointcloud_binary_format)
#ifndef VISP_HAVE_PCL
      ,
      m_size_height(height), m_size_width(width)
#endif
  {
  }

  // Thread main loop
  void run()
  {
    try {
      vpImage<vpRGBa> colorImg;
      vpImage<uint16_t> depthImg;
#ifdef VISP_HAVE_PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
#else
      std::vector<vpColVector> pointCloud;
#endif
      vpImage<unsigned char> infraredImg;

      char buffer[FILENAME_MAX];
      for (;;) {
        m_queue.pop(colorImg, depthImg, pointCloud, infraredImg);

        if (!m_directory.empty()) {
          std::stringstream ss;

          if (m_save_color) {
            ss << m_directory << "/color_image_%04d.jpg";
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);

            std::string filename_color = buffer;
            vpImageIo::write(colorImg, filename_color);
          }

          if (m_save_depth) {
            ss.str("");
            ss << m_directory << "/depth_image_%04d.bin";
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);
            std::string filename_depth = buffer;

            std::ofstream file_depth(filename_depth.c_str(), std::ios::out | std::ios::binary);
            if (file_depth.is_open()) {
              unsigned int height = depthImg.getHeight(), width = depthImg.getWidth();
              vpIoTools::writeBinaryValueLE(file_depth, height);
              vpIoTools::writeBinaryValueLE(file_depth, width);

              uint16_t value;
              for (unsigned int i = 0; i < height; i++) {
                for (unsigned int j = 0; j < width; j++) {
                  value = depthImg[i][j];
                  vpIoTools::writeBinaryValueLE(file_depth, value);
                }
              }
            }
          }

          if (m_save_pointcloud) {
            ss.str("");
            ss << m_directory << "/point_cloud_%04d" << (m_save_pointcloud_binary_format ? ".bin" : ".pcd");
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);
            std::string filename_point_cloud = buffer;

            if (m_save_pointcloud_binary_format) {
              std::ofstream file_pointcloud(filename_point_cloud.c_str(), std::ios::out | std::ios::binary);

              if (file_pointcloud.is_open()) {
#ifdef VISP_HAVE_PCL
                uint32_t width = pointCloud->width;
                uint32_t height = pointCloud->height;
                // true if pointcloud does not contain NaN or Inf, not handled currently
                char is_dense = pointCloud->is_dense;

                vpIoTools::writeBinaryValueLE(file_pointcloud, height);
                vpIoTools::writeBinaryValueLE(file_pointcloud, width);
                file_pointcloud.write((char *)(&is_dense), sizeof(is_dense));

                for (uint32_t i = 0; i < height; i++) {
                  for (uint32_t j = 0; j < width; j++) {
                    pcl::PointXYZ pt = (*pointCloud)(j, i);

                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.x);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.y);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.z);
                  }
                }
#else
                uint32_t width = m_size_width;
                uint32_t height = m_size_height;
                // to be consistent with PCL version
                char is_dense = 1;

                vpIoTools::writeBinaryValueLE(file_pointcloud, height);
                vpIoTools::writeBinaryValueLE(file_pointcloud, width);
                file_pointcloud.write((char *)(&is_dense), sizeof(is_dense));

                for (uint32_t i = 0; i < height; i++) {
                  for (uint32_t j = 0; j < width; j++) {
                    float x = (float)pointCloud[i * width + j][0];
                    float y = (float)pointCloud[i * width + j][1];
                    float z = (float)pointCloud[i * width + j][2];

                    vpIoTools::writeBinaryValueLE(file_pointcloud, x);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, y);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, z);
                  }
                }
#endif
              }
            } else {
#ifdef VISP_HAVE_PCL
              pcl::io::savePCDFileBinary(filename_point_cloud, *pointCloud);
#endif
            }
          }

          if (m_save_infrared) {
            ss.str("");
            ss << m_directory << "/infrared_image_%04d.jpg";
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);

            std::string filename_infrared = buffer;
            vpImageIo::write(infraredImg, filename_infrared);
          }

          m_cpt++;
        }
      }
    } catch (const FrameQueue::cancelled &) {
      std::cout << "Receive cancel FrameQueue." << std::endl;
    }
  }

private:
  FrameQueue &m_queue;
  std::string m_directory;
  unsigned int m_cpt;
  bool m_save_color;
  bool m_save_depth;
  bool m_save_pointcloud;
  bool m_save_infrared;
  bool m_save_pointcloud_binary_format;
#ifndef VISP_HAVE_PCL
  int m_size_height;
  int m_size_width;
#endif
};
} // Namespace

int main(int argc, char *argv[])
{
  bool save = false;
  std::string output_directory = vpTime::getDateTime("%Y_%m_%d_%H.%M.%S");
  std::string output_directory_custom = "";
  bool use_aligned_stream = false;
  bool save_color = false;
  bool save_depth = false;
  bool save_pointcloud = false;
  bool save_infrared = false;
  bool click_to_save = false;
  int stream_fps = 30;
  bool save_pointcloud_binary_format = false;

  // Read the command line options
  if (!getOptions(argc, argv, save, output_directory_custom, use_aligned_stream, save_color, save_depth,
                  save_pointcloud, save_infrared, click_to_save, stream_fps, save_pointcloud_binary_format)) {
    return EXIT_FAILURE;
  }

  if (!output_directory_custom.empty())
    output_directory = output_directory_custom + "/" + output_directory;

#ifndef VISP_HAVE_PCL
  save_pointcloud_binary_format = true;
#endif

  std::cout << "save: " << save << std::endl;
  std::cout << "output_directory: " << output_directory << std::endl;
  std::cout << "use_aligned_stream: " << use_aligned_stream << std::endl;
  std::cout << "save_color: " << save_color << std::endl;
  std::cout << "save_depth: " << save_depth << std::endl;
  std::cout << "save_pointcloud: " << save_pointcloud << std::endl;
  std::cout << "save_infrared: " << save_infrared << std::endl;
  std::cout << "stream_fps: " << stream_fps << std::endl;
  std::cout << "save_pointcloud_binary_format: " << save_pointcloud_binary_format << std::endl;
  std::cout << "click_to_save: " << click_to_save << std::endl;

  int width = 640, height = 480;
#ifdef USE_REALSENSE2
  vpRealSense2 realsense;

  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, stream_fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, stream_fps);
  config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, stream_fps);
  realsense.open(config);
#else
  vpRealSense realsense;
  realsense.setStreamSettings(rs::stream::color,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::rgba8, stream_fps));
  realsense.setStreamSettings(rs::stream::depth,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::z16, stream_fps));
  realsense.setStreamSettings(rs::stream::infrared,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::y8, stream_fps));
  realsense.setStreamSettings(rs::stream::infrared2,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::y8, stream_fps));

  realsense.open();
#endif

  vpImage<vpRGBa> I_color(height, width);
  vpImage<unsigned char> I_gray(height, width);
  vpImage<unsigned char> I_depth(height, width);
  vpImage<uint16_t> I_depth_raw(height, width);
  vpImage<unsigned char> I_infrared(height, width);

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2, d3;
#else
  vpDisplayGDI d1, d2, d3;
#endif
  d1.init(I_gray, 0, 0, "RealSense color stream");
  d2.init(I_depth, I_gray.getWidth() + 80, 0, "RealSense depth stream");
  d3.init(I_infrared, I_gray.getWidth() + 80, I_gray.getHeight() + 10, "RealSense infrared stream");

  while (true) {
    realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, NULL, NULL);
    vpImageConvert::convert(I_color, I_gray);
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    vpDisplay::display(I_gray);
    vpDisplay::display(I_depth);
    vpDisplay::displayText(I_gray, 20, 20, "Click when ready.", vpColor::red);
    vpDisplay::flush(I_gray);
    vpDisplay::flush(I_depth);

    if (vpDisplay::getClick(I_gray, false)) {
      break;
    }
  }

  if (save) {
    // Create output directory
    vpIoTools::makeDirectory(output_directory);

    // Save intrinsics
#ifdef USE_REALSENSE2
    vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_directory + "/camera.xml", "color_camera", width, height);

    vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH);
    xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);

    vpCameraParameters cam_infrared = realsense.getCameraParameters(RS2_STREAM_INFRARED);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
#else
    vpCameraParameters cam_color = realsense.getCameraParameters(rs::stream::color);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_directory + "/camera.xml", "color_camera", width, height);

    vpCameraParameters cam_color_rectified = realsense.getCameraParameters(rs::stream::rectified_color);
    xml_camera.save(cam_color_rectified, output_directory + "/camera.xml", "color_camera_rectified", width, height);

    vpCameraParameters cam_depth = realsense.getCameraParameters(rs::stream::depth);
    xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);

    vpCameraParameters cam_depth_aligned_to_rectified_color =
        realsense.getCameraParameters(rs::stream::depth_aligned_to_rectified_color);
    xml_camera.save(cam_depth_aligned_to_rectified_color, output_directory + "/camera.xml",
                    "depth_camera_aligned_to_rectified_color", width, height);

    vpCameraParameters cam_infrared = realsense.getCameraParameters(rs::stream::infrared);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color = realsense.getTransformation(rs::stream::color, rs::stream::depth);
#endif
    std::ofstream file(std::string(output_directory + "/depth_M_color.txt"));
    depth_M_color.save(file);
    file.close();
  }

  FrameQueue save_queue;
  StorageWorker storage(std::ref(save_queue), std::cref(output_directory), save_color, save_depth, save_pointcloud,
                        save_infrared, save_pointcloud_binary_format, width, height);
  std::thread storage_thread(&StorageWorker::run, &storage);

#ifdef USE_REALSENSE2
  rs2::align align_to(RS2_STREAM_COLOR);
  if (use_aligned_stream && save_infrared) {
    std::cerr << "Cannot use aligned streams with infrared acquisition currently."
                 "\nInfrared stream acquisition is disabled!"
              << std::endl;
  }
#endif

  int nb_saves = 0;
  bool quit = false;
#ifdef VISP_HAVE_PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
#else
  std::vector<vpColVector> pointCloud;
#endif
  while (!quit) {
    if (use_aligned_stream) {
#ifdef USE_REALSENSE2
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, NULL, pointCloud, NULL,
                        &align_to);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, NULL,
                        &align_to);
#endif
#else
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, NULL, pointCloud,
                        (unsigned char *)I_infrared.bitmap, NULL, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap, NULL, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#endif
#endif
    } else {
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, NULL, pointCloud,
                        (unsigned char *)I_infrared.bitmap, NULL);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap);
#endif
    }

    vpImageConvert::convert(I_color, I_gray);
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    vpDisplay::display(I_gray);
    vpDisplay::display(I_depth);
    vpDisplay::display(I_infrared);

    if (!click_to_save) {
      vpDisplay::displayText(I_gray, 20, 20, "Click to quit.", vpColor::red);
    } else {
      std::stringstream ss;
      ss << "Images saved:" << nb_saves;
      vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
    }

    vpDisplay::flush(I_gray);
    vpDisplay::flush(I_depth);
    vpDisplay::flush(I_infrared);

    if (save && !click_to_save) {
#ifdef VISP_HAVE_PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy = pointCloud->makeShared();
      save_queue.push(I_color, I_depth_raw, pointCloud_copy, I_infrared);
#else
      save_queue.push(I_color, I_depth_raw, pointCloud, I_infrared);
#endif
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_gray, button, false)) {
      if (!click_to_save) {
        save_queue.cancel();
        quit = true;
      } else {
        switch (button) {
        case vpMouseButton::button1:
          if (save) {
            nb_saves++;
#ifdef VISP_HAVE_PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy = pointCloud->makeShared();
            save_queue.push(I_color, I_depth_raw, pointCloud_copy, I_infrared);
#else
            save_queue.push(I_color, I_depth_raw, pointCloud, I_infrared);
#endif
          }
          break;

        case vpMouseButton::button2:
        case vpMouseButton::button3:
        default:
          save_queue.cancel();
          quit = true;
          break;
        }
      }
    }
  }

  storage_thread.join();

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "Need libRealSense or libRealSense2 and C++11 and displayX or displayGDI!" << std::endl;
  return EXIT_SUCCESS;
}
#endif
