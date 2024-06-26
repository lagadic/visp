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
 *****************************************************************************/

 /*!
   \example saveRealSenseData.cpp

   \brief Example that shows how to save realsense data that can be replayed with readRealSenseData.cpp
 */

#include <iostream>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PUGIXML)

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#if defined(VISP_HAVE_PCL)
#include <pcl/pcl_config.h>
#if defined(VISP_HAVE_PCL_COMMON)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif
#if defined(VISP_HAVE_PCL_IO)
#include <pcl/io/pcd_io.h>
#endif
#endif

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoException.h>
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

#ifdef VISP_HAVE_MINIZ
#define GETOPTARGS "se:o:acdpzijCf:bvh"
#else
#define GETOPTARGS "se:o:acdpijCf:bvh"
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void usage(const char *name, const char *badparam, int fps)
{
  std::cout << "\nSYNOPSIS " << std::endl
    << "  " << name
    << " [-s]"
    << " [-e <filename pattern (e.g. %06d)>]"
    << " [-a]"
    << " [-c]"
    << " [-d]"
    << " [-p]"
    << " [-b]"
#ifdef VISP_HAVE_MINIZ
    << " [-z]"
#endif
    << " [-i]"
    << " [-j]"
    << " [-C]"
    << " [-f <fps>]"
    << " [-v]"
    << " [-o <directory>]"
    << " [--help,-h]"
    << std::endl;
  std::cout << "\nOPTIONS " << std::endl
    << "  -s" << std::endl
    << "    Flag to enable data saving." << std::endl
    << std::endl
    << "  -e <pattern>" << std::endl
    << "    Filename pattern when saving data." << std::endl
    << std::endl
    << "  -a" << std::endl
    << "    Color and depth are aligned." << std::endl
    << std::endl
    << "  -c" << std::endl
    << "    Add color stream to saved data when -s option is enable." << std::endl
    << std::endl
    << "  -d" << std::endl
    << "    Add depth stream to saved data when -s option is enable." << std::endl
    << std::endl
    << "  -p" << std::endl
    << "    Add point cloud stream to saved data when -s option is enabled." << std::endl
    << "    By default (if available), the point cloud is saved in Point Cloud Data file format (.PCD extension file)."
    << std::endl
    << "    You can also use the -z option to save the point cloud in .npz (NumPy)." << std::endl
    << std::endl
    << "  -b" << std::endl
    << "    Force depth and pointcloud to be saved in (little-endian) binary format." << std::endl
    << std::endl
#ifdef VISP_HAVE_MINIZ
    << "  -z" << std::endl
    << "    Pointcloud is saved in NPZ format." << std::endl
    << std::endl
#endif
    << "  -i" << std::endl
    << "    Add infrared stream to saved data when -s option is enabled." << std::endl
    << std::endl
    << "  -j" << std::endl
    << "    Save image data using JPEG format (otherwise PNG is used)." << std::endl
    << std::endl
    << "  -C" << std::endl
    << "    Trigger one shot data saver after each user click." << std::endl
    << std::endl
    << "  -f <fps>" << std::endl
    << "    Set camera framerate." << std::endl
    << "    Default: " << fps << std::endl
    << std::endl
    << "  -v" << std::endl
    << "    Display depth using a cumulative histogram." << std::endl
    << "    Warning: this operation is time consuming" << std::endl
    << std::endl
    << "  -o <directory>" << std::endl
    << "    Output directory that will host saved data." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Display this helper message." << std::endl
    << std::endl;
  std::cout << "\nEXAMPLE " << std::endl
    << "- Save aligned color + depth + point cloud in data folder" << std::endl
    << "  " << name << " -s -a -c -d -p -o data" << std::endl
    << "- Save color + IR + depth + point cloud in NPZ format in data folder" << std::endl
    << "  " << name << " -s -c -d -i -p -z -o data" << std::endl
    << std::endl;

  if (badparam) {
    std::cout << "\nERROR: Bad parameter " << badparam << std::endl;
  }
}

bool getOptions(int argc, const char *argv[], bool &save, std::string &pattern, std::string &output_directory,
                bool &use_aligned_stream, bool &save_color, bool &save_depth, bool &save_pointcloud,
                bool &save_infrared, bool &click_to_save, int &stream_fps, bool &save_pcl_npz_format,
                bool &save_force_binary_format, bool &save_jpeg, bool &depth_hist_visu)
{
  const char *optarg;
  const char **argv1 = (const char **)argv;
  int c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 's':
      save = true;
      break;
    case 'e':
      pattern = optarg;
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
    case 'j':
      save_jpeg = true;
      break;
    case 'C':
      click_to_save = true;
      break;
    case 'f':
      stream_fps = atoi(optarg);
      break;
    case 'v':
      depth_hist_visu = true;
      break;
#ifdef VISP_HAVE_MINIZ
    case 'z':
      save_pcl_npz_format = true;
      break;
#endif
    case 'b':
      save_force_binary_format = true;
      break;

    case 'h':
      usage(argv[0], nullptr, stream_fps);
      return false;
      break;

    default:
      usage(argv[0], optarg, stream_fps);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, stream_fps);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl
      << std::endl;
    return false;
  }

  return true;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// Code adapted from: https://stackoverflow.com/a/37146523
class vpFrameQueue
{
public:
  struct vpCancelled_t
  { };

  vpFrameQueue()
    : m_cancelled(false), m_cond(), m_queueColor(), m_queueDepth(), m_queuePointCloud(), m_queueInfrared(),
    m_maxQueueSize(1024 * 8), m_mutex()
  { }

  void cancel()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancelled = true;
    m_cond.notify_all();
  }

  // Push data to save in the queue (FIFO)
  void push(const std::unique_ptr<vpImage<vpRGBa>> &ptr_colorImg,
            const std::unique_ptr<vpImage<uint16_t>> &ptr_depthImg,
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
#else
            const std::unique_ptr<std::vector<vpColVector>> &ptr_pointCloud,
#endif
            const std::unique_ptr<vpImage<unsigned char>> &ptr_infraredImg)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (ptr_colorImg) {
      m_queueColor.push(*ptr_colorImg);
    }
    if (ptr_depthImg) {
      m_queueDepth.push(*ptr_depthImg);
    }
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    if (pointCloud) {
      m_queuePointCloud.push(pointCloud);
    }
#else
    if (ptr_pointCloud) {
      m_queuePointCloud.push(*ptr_pointCloud);
    }
#endif
    if (ptr_infraredImg) {
      m_queueInfrared.push(*ptr_infraredImg);
    }

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
  void pop(std::unique_ptr<vpImage<vpRGBa>> &ptr_colorImg,
           std::unique_ptr<vpImage<uint16_t>> &ptr_depthImg,
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
#else
           std::unique_ptr<std::vector<vpColVector>> &ptr_pointCloud,
#endif
           std::unique_ptr<vpImage<unsigned char>> &ptr_infraredImg)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    // Since we push all 4 data at a time, there should be no situation where a queue size is different from the others
    while (m_queueColor.empty() && m_queueDepth.empty() && m_queuePointCloud.empty() && m_queueInfrared.empty()) {
      if (m_cancelled) {
        throw vpCancelled_t();
      }

      m_cond.wait(lock);

      if (m_cancelled) {
        throw vpCancelled_t();
      }
    }

    if (!m_queueColor.empty()) {
      ptr_colorImg = std::make_unique<vpImage<vpRGBa>>(m_queueColor.front());
      m_queueColor.pop();
    }
    if (!m_queueDepth.empty()) {
      ptr_depthImg = std::make_unique<vpImage<uint16_t>>(m_queueDepth.front());
      m_queueDepth.pop();
    }
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    if (!m_queuePointCloud.empty()) {
      pointCloud = m_queuePointCloud.front();
      m_queuePointCloud.pop();
    }
#else
    if (!m_queuePointCloud.empty()) {
      ptr_pointCloud = std::make_unique<std::vector<vpColVector>>(m_queuePointCloud.front());
      m_queuePointCloud.pop();
    }
#endif
    if (!m_queueInfrared.empty()) {
      ptr_infraredImg = std::make_unique<vpImage<unsigned char>>(m_queueInfrared.front());
      m_queueInfrared.pop();
    }
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queueColor.empty() && m_queueDepth.empty() && m_queuePointCloud.empty() && m_queueInfrared.empty();
  }

  void setMaxQueueSize(const size_t max_queue_size) { m_maxQueueSize = max_queue_size; }

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  std::queue<vpImage<vpRGBa>> m_queueColor;
  std::queue<vpImage<uint16_t>> m_queueDepth;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_queuePointCloud;
#else
  std::queue<std::vector<vpColVector>> m_queuePointCloud;
#endif
  std::queue<vpImage<unsigned char>> m_queueInfrared;
  size_t m_maxQueueSize;
  std::mutex m_mutex;
};

class vpStorageWorker
{
public:
  vpStorageWorker(vpFrameQueue &queue, const std::string &save_pattern, const std::string &directory, bool save_color,
                  bool save_depth, bool save_pointcloud, bool save_infrared, bool save_pcl_npz_format,
                  bool save_force_binary_format, bool save_jpeg,
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
    : m_queue(queue), m_save_pattern(save_pattern), m_directory(directory), m_cpt(0), m_save_color(save_color), m_save_depth(save_depth),
    m_save_pointcloud(save_pointcloud), m_save_infrared(save_infrared), m_save_pcl_npz_format(save_pcl_npz_format),
    m_save_force_binary_format(save_force_binary_format), m_save_jpeg(save_jpeg)
#ifndef VISP_HAVE_PCL
    ,
    m_size_height(height), m_size_width(width)
#endif
  { }

  // Thread main loop
  void run()
  {
    try {
      std::unique_ptr<vpImage<vpRGBa>> ptr_colorImg;
      std::unique_ptr<vpImage<uint16_t>> ptr_depthImg;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pointCloud;
#else
      std::unique_ptr<std::vector<vpColVector>> ptr_pointCloud;
#endif
      std::unique_ptr<vpImage<unsigned char>> ptr_infraredImg;

      std::vector<float> vec_pcl;

      char buffer[FILENAME_MAX];
      std::string image_filename_ext = m_save_jpeg ? ".jpg" : ".png";
      for (;;) {
        m_queue.pop(ptr_colorImg, ptr_depthImg, ptr_pointCloud, ptr_infraredImg);

        if (!m_directory.empty()) {
          std::string current_time = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
          std::stringstream ss;

          if (m_save_color && ptr_colorImg) {
            ss << m_directory << "/color_image_" << m_save_pattern << image_filename_ext;
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);

            std::string filename_color = buffer;
            vpImageIo::write(*ptr_colorImg, filename_color);
          }

          if (m_save_depth && ptr_depthImg) {
            ss.str("");

            if (m_save_force_binary_format) {
              ss << m_directory << "/depth_image_" << m_save_pattern << ".bin";
              snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);
              std::string filename_depth = buffer;

              std::ofstream file_depth(filename_depth.c_str(), std::ios::out | std::ios::binary);
              if (file_depth.is_open()) {
                unsigned int height = ptr_depthImg->getHeight(), width = ptr_depthImg->getWidth();
                vpIoTools::writeBinaryValueLE(file_depth, height);
                vpIoTools::writeBinaryValueLE(file_depth, width);

                uint16_t value;
                for (unsigned int i = 0; i < height; i++) {
                  for (unsigned int j = 0; j < width; j++) {
                    value = (*ptr_depthImg)[i][j];
                    vpIoTools::writeBinaryValueLE(file_depth, value);
                  }
                }
              }
            }
#ifdef VISP_HAVE_MINIZ
            else {
              ss << m_directory << "/depth_image_" << m_save_pattern << ".npz";
              snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);
              std::string filename_depth = buffer;

              // Write Npz headers
              std::vector<char> vec_filename(filename_depth.begin(), filename_depth.end());
              // Null-terminated character is handled at reading
              // For null-terminated character handling, see:
              // https://stackoverflow.com/a/8247804
              // https://stackoverflow.com/a/45491652
              visp::cnpy::npz_save(filename_depth, "filename", &vec_filename[0], { vec_filename.size() }, "w");

              std::vector<char> vec_current_time(current_time.begin(), current_time.end());
              visp::cnpy::npz_save(filename_depth, "timestamp", &vec_current_time, { vec_current_time.size() }, "a");

              unsigned int height = ptr_depthImg->getHeight();
              unsigned int width = ptr_depthImg->getWidth();
              unsigned int channel = 1;
              visp::cnpy::npz_save(filename_depth, "height", &height, { 1 }, "a");
              visp::cnpy::npz_save(filename_depth, "width", &width, { 1 }, "a");
              visp::cnpy::npz_save(filename_depth, "channel", &channel, { 1 }, "a");

              // Write data
              std::vector<uint16_t> I_depth_raw_vec(ptr_depthImg->bitmap, ptr_depthImg->bitmap + ptr_depthImg->getSize());
              visp::cnpy::npz_save(filename_depth, "data", I_depth_raw_vec.data(), { height, width }, "a");
            }
#else
            else {
              throw(vpIoException(vpIoException::ioError, "Cannot manage non-binary files when npz I/O functions are disabled."));
            }
#endif
          }

          if (m_save_pointcloud && ptr_pointCloud) {
            ss.str("");
            std::string pcl_extension = m_save_force_binary_format ? ".bin" : (m_save_pcl_npz_format ? ".npz" : ".pcd");
            ss << m_directory << "/point_cloud_" << m_save_pattern << pcl_extension;
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);
            std::string filename_point_cloud = buffer;

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
            uint32_t width = ptr_pointCloud->width;
            uint32_t height = ptr_pointCloud->height;
#else
            uint32_t width = m_size_width;
            uint32_t height = m_size_height;
#endif

            if (m_save_force_binary_format) {
              std::ofstream file_pointcloud(filename_point_cloud.c_str(), std::ios::out | std::ios::binary);

              if (file_pointcloud.is_open()) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
                // true if ptr_pointCloud does not contain NaN or Inf, not handled currently
                char is_dense = ptr_pointCloud->is_dense;

                vpIoTools::writeBinaryValueLE(file_pointcloud, height);
                vpIoTools::writeBinaryValueLE(file_pointcloud, width);
                file_pointcloud.write((char *)(&is_dense), sizeof(is_dense));

                for (uint32_t i = 0; i < height; i++) {
                  for (uint32_t j = 0; j < width; j++) {
                    pcl::PointXYZ pt = (*ptr_pointCloud)(j, i);

                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.x);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.y);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, pt.z);
                  }
                }
#else
                // to be consistent with PCL version
                const char is_dense = 1;

                vpIoTools::writeBinaryValueLE(file_pointcloud, height);
                vpIoTools::writeBinaryValueLE(file_pointcloud, width);
                file_pointcloud.write((char *)(&is_dense), sizeof(is_dense));

                for (uint32_t i = 0; i < height; i++) {
                  for (uint32_t j = 0; j < width; j++) {
                    float x = (float)(*ptr_pointCloud)[i * width + j][0];
                    float y = (float)(*ptr_pointCloud)[i * width + j][1];
                    float z = (float)(*ptr_pointCloud)[i * width + j][2];

                    vpIoTools::writeBinaryValueLE(file_pointcloud, x);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, y);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, z);
                  }
                }
#endif
              }
            }
            else if (m_save_pcl_npz_format) {
#ifdef VISP_HAVE_MINIZ
              // Write Npz headers
              std::vector<char> vec_filename(filename_point_cloud.begin(), filename_point_cloud.end());
              // Null-terminated character is handled at reading
              // For null-terminated character handling, see:
              // https://stackoverflow.com/a/8247804
              // https://stackoverflow.com/a/45491652
              visp::cnpy::npz_save(filename_point_cloud, "filename", &vec_filename[0], { vec_filename.size() }, "w");

              std::vector<char> vec_current_time(current_time.begin(), current_time.end());
              visp::cnpy::npz_save(filename_point_cloud, "timestamp", &vec_current_time, { vec_current_time.size() }, "a");

              const uint32_t channels = 3;
              visp::cnpy::npz_save(filename_point_cloud, "height", &height, { 1 }, "a");
              visp::cnpy::npz_save(filename_point_cloud, "width", &width, { 1 }, "a");
              visp::cnpy::npz_save(filename_point_cloud, "channel", &channels, { 1 }, "a");

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
              // can probably be optimized by assuming channel=1 and use data of type XYZ
              // but this should probably not work with the Python script for display
              for (uint32_t i = 0; i < height; i++) {
                for (uint32_t j = 0; j < width; j++) {
                  pcl::PointXYZ pt = (*ptr_pointCloud)(j, i);
                  vec_pcl[channels * (i*width + j) + 0] = pt.x;
                  vec_pcl[channels * (i*width + j) + 1] = pt.y;
                  vec_pcl[channels * (i*width + j) + 2] = pt.z;
                }
              }
#else
              vec_pcl.resize(height * width * channels);
              for (uint32_t i = 0; i < height; i++) {
                for (uint32_t j = 0; j < width; j++) {
                  vec_pcl[channels * (i*width + j) + 0] = (*ptr_pointCloud)[i*width + j][0];
                  vec_pcl[channels * (i*width + j) + 1] = (*ptr_pointCloud)[i*width + j][1];
                  vec_pcl[channels * (i*width + j) + 2] = (*ptr_pointCloud)[i*width + j][2];
                }
              }
#endif
              // Write data
              visp::cnpy::npz_save(filename_point_cloud, "data", vec_pcl.data(), { height, width, channels }, "a");
#else
              throw(vpIoException(vpIoException::fatalError, "Cannot save in npz format when npz I/O functions are disabled."));
#endif
            }
            else {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
              pcl::io::savePCDFileBinary(filename_point_cloud, *ptr_pointCloud);
#endif
            }
          }

          if (m_save_infrared && ptr_infraredImg) {
            ss.str("");
            ss << m_directory << "/infrared_image_" << m_save_pattern << image_filename_ext;
            snprintf(buffer, FILENAME_MAX, ss.str().c_str(), m_cpt);

            std::string filename_infrared = buffer;
            vpImageIo::write(*ptr_infraredImg, filename_infrared);
          }

          m_cpt++;
        }
      }
    }
    catch (const vpFrameQueue::vpCancelled_t &) {
      std::cout << "Receive cancel vpFrameQueue." << std::endl;
    }
  }

private:
  vpFrameQueue &m_queue;
  std::string m_save_pattern;
  std::string m_directory;
  unsigned int m_cpt;
  bool m_save_color;
  bool m_save_depth;
  bool m_save_pointcloud;
  bool m_save_infrared;
  bool m_save_pcl_npz_format;
  bool m_save_force_binary_format;
  bool m_save_jpeg;
#ifndef VISP_HAVE_PCL
  int m_size_height;
  int m_size_width;
#endif
};
} // Namespace

#endif // DOXYGEN_SHOULD_SKIP_THIS

int main(int argc, const char *argv[])
{
  bool save = false;
  std::string save_pattern = "%04d";
  std::string output_directory = vpTime::getDateTime("%Y_%m_%d_%H.%M.%S");
  std::string output_directory_custom = "";
  bool use_aligned_stream = false;
  bool save_color = false;
  bool save_depth = false;
  bool save_pointcloud = false;
  bool save_infrared = false;
  bool click_to_save = false;
  int stream_fps = 30;
  bool save_pcl_npz_format = false;
  bool save_force_binary_format = false;
  bool save_jpeg = false;
  bool depth_hist_visu = false;

  // Read the command line options
  if (!getOptions(argc, argv, save, save_pattern, output_directory_custom, use_aligned_stream, save_color, save_depth,
                  save_pointcloud, save_infrared, click_to_save, stream_fps, save_pcl_npz_format,
                  save_force_binary_format, save_jpeg, depth_hist_visu)) {
    return EXIT_FAILURE;
  }

  if (!output_directory_custom.empty())
    output_directory = output_directory_custom + "/" + output_directory;

#ifndef VISP_HAVE_PCL
  save_pcl_npz_format = !save_force_binary_format ? true : false;
#endif

  std::cout << "save: " << save << std::endl;
  std::cout << "save_pattern: " << save_pattern << std::endl;
  std::cout << "output_directory: " << output_directory << std::endl;
  std::cout << "use_aligned_stream: " << use_aligned_stream << std::endl;
  std::cout << "save_color: " << save_color << std::endl;
  std::cout << "save_depth: " << save_depth << std::endl;
  std::cout << "save_pointcloud: " << save_pointcloud << std::endl;
  std::cout << "save_infrared: " << save_infrared << std::endl;
  std::cout << "save_jpeg: " << save_jpeg << std::endl;
  std::cout << "stream_fps: " << stream_fps << std::endl;
  std::cout << "depth_hist_visu: " << depth_hist_visu << std::endl;
#ifdef VISP_HAVE_MINIZ
  std::cout << "save_pcl_npz_format: " << save_pcl_npz_format << std::endl;
#endif
  std::cout << "save_force_binary_format: " << save_force_binary_format << std::endl;
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
  vpImage<unsigned char> I_depth(height, width);
  vpImage<uint16_t> I_depth_raw(height, width);
  vpImage<unsigned char> I_infrared(height, width);

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2, d3;
#else
  vpDisplayGDI d1, d2, d3;
#endif
  d1.init(I_color, 0, 0, "RealSense color stream");
  d2.init(I_depth, I_color.getWidth() + 80, 0, "RealSense depth stream");
  d3.init(I_infrared, I_color.getWidth() + 80, I_color.getHeight() + 70, "RealSense infrared stream");

  while (true) {
    double start = vpTime::measureTimeMs();
    realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, nullptr);
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    vpDisplay::display(I_color);
    vpDisplay::display(I_depth);

    double delta_time = vpTime::measureTimeMs() - start;
    std::ostringstream oss_time;
    oss_time << delta_time << " ms ; fps=" << 1000/delta_time;
    vpDisplay::displayText(I_color, 40, 20, oss_time.str(), vpColor::red);
    vpDisplay::displayText(I_color, 20, 20, "Click when ready.", vpColor::red);

    vpDisplay::flush(I_color);
    vpDisplay::flush(I_depth);

    if (vpDisplay::getClick(I_color, false)) {
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

    if (use_aligned_stream) {
      xml_camera.save(cam_color, output_directory + "/camera.xml", "depth_camera", width, height);
    }
    else {
      vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH);
      xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);
    }

    vpCameraParameters cam_infrared = realsense.getCameraParameters(RS2_STREAM_INFRARED);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color;
    if (!use_aligned_stream) {
      depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
    }
#else
    vpCameraParameters cam_color = realsense.getCameraParameters(rs::stream::color);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_directory + "/camera.xml", "color_camera", width, height);

    vpCameraParameters cam_color_rectified = realsense.getCameraParameters(rs::stream::rectified_color);
    xml_camera.save(cam_color_rectified, output_directory + "/camera.xml", "color_camera_rectified", width, height);

    if (use_aligned_stream) {
      vpCameraParameters cam_depth = realsense.getCameraParameters(rs::stream::depth);
      xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);
    }
    else {
      xml_camera.save(cam_color, output_directory + "/camera.xml", "depth_camera", width, height);
    }

    vpCameraParameters cam_depth_aligned_to_rectified_color =
      realsense.getCameraParameters(rs::stream::depth_aligned_to_rectified_color);
    xml_camera.save(cam_depth_aligned_to_rectified_color, output_directory + "/camera.xml",
                    "depth_camera_aligned_to_rectified_color", width, height);

    vpCameraParameters cam_infrared = realsense.getCameraParameters(rs::stream::infrared);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color;
    if (!use_aligned_stream) {
      depth_M_color = realsense.getTransformation(rs::stream::color, rs::stream::depth);
    }
#endif
    std::ofstream file(std::string(output_directory + "/depth_M_color.txt"));
    depth_M_color.save(file);
    file.close();
  }

  vpFrameQueue save_queue;
  vpStorageWorker storage(std::ref(save_queue), save_pattern, std::cref(output_directory), save_color, save_depth,
    save_pointcloud, save_infrared, save_pcl_npz_format, save_force_binary_format, save_jpeg, width, height);
  std::thread storage_thread(&vpStorageWorker::run, &storage);

#ifdef USE_REALSENSE2
  rs2::align align_to(RS2_STREAM_COLOR);
  if (use_aligned_stream && save_infrared) {
    std::cerr << "Cannot use aligned streams with infrared acquisition currently."
      << "\nInfrared stream acquisition is disabled!"
      << std::endl;
  }
#endif

  int nb_saves = 0;
  bool quit = false;
  // If PCL is available, always use PCL datatype even if we will save in NPZ or BIN file format
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
#else
  std::vector<vpColVector> pointCloud;
#endif

  std::unique_ptr<vpImage<vpRGBa>> ptr_colorImg;
  std::unique_ptr<vpImage<uint16_t>> ptr_depthImg;
  std::unique_ptr<std::vector<vpColVector>> ptr_pointCloud;
  std::unique_ptr<vpImage<unsigned char>> ptr_infraredImg;

  std::vector<double> vec_delta_time;
  while (!quit) {
    double start = vpTime::measureTimeMs();
    if (use_aligned_stream) {
#ifdef USE_REALSENSE2
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud, nullptr,
                        &align_to);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, nullptr,
                        &align_to);
#endif
#else
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#endif
#endif
    }
    else {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap);
#endif
    }

    if (depth_hist_visu) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
    }
    else {
      // Seems like sometimes using createDepthHistogram takes lots of time?
      // so we simply perform bit shift from uint16_t to uint8_t
      for (unsigned int i = 0; i < I_depth_raw.getRows(); i++) {
        for (unsigned int j = 0; j < I_depth_raw.getCols(); j++) {
          I_depth[i][j] = I_depth_raw[i][j] >> 8;
        }
      }
    }

    vpDisplay::display(I_color);
    vpDisplay::display(I_depth);
    vpDisplay::display(I_infrared);

    if (!click_to_save) {
      vpDisplay::displayText(I_color, 20, 20, "Click to quit.", vpColor::red);
    }
    else {
      std::stringstream ss;
      ss << "Images saved: " << nb_saves;
      vpDisplay::displayText(I_color, 20, 20, ss.str(), vpColor::red);
    }

    if (save && !click_to_save) {
      if (save_color) {
        ptr_colorImg = std::make_unique<vpImage<vpRGBa>>(I_color);
      }
      if (save_depth) {
        ptr_depthImg = std::make_unique<vpImage<uint16_t>>(I_depth_raw);
      }
      if (save_infrared) {
        ptr_infraredImg = std::make_unique<vpImage<unsigned char>>(I_infrared);
      }

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy;
      if (save_pointcloud) {
        pointCloud_copy = pointCloud->makeShared();
      }
      save_queue.push(ptr_colorImg, ptr_depthImg, pointCloud_copy, ptr_infraredImg);
#else
      if (save_pointcloud) {
        ptr_pointCloud = std::make_unique<std::vector<vpColVector>>(pointCloud);
      }
      save_queue.push(ptr_colorImg, ptr_depthImg, ptr_pointCloud, ptr_infraredImg);
#endif
    }

    double delta_time = vpTime::measureTimeMs() - start;
    vec_delta_time.push_back(delta_time);
    std::ostringstream oss_time;
    oss_time << delta_time << " ms ; fps=" << 1000/delta_time;
    vpDisplay::displayText(I_color, 40, 20, oss_time.str(), vpColor::red);

    vpDisplay::flush(I_color);
    vpDisplay::flush(I_depth);
    vpDisplay::flush(I_infrared);

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_color, button, false)) {
      if (!click_to_save) {
        save = false;
        quit = true;
        save_queue.cancel();
      }
      else {
        switch (button) {
        case vpMouseButton::button1:
          if (save) {
            nb_saves++;

            if (save_color) {
              ptr_colorImg = std::make_unique<vpImage<vpRGBa>>(I_color);
            }
            if (save_depth) {
              ptr_depthImg = std::make_unique<vpImage<uint16_t>>(I_depth_raw);
            }
            if (save_infrared) {
              ptr_infraredImg = std::make_unique<vpImage<unsigned char>>(I_infrared);
            }

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy;
            if (save_pointcloud) {
              pointCloud_copy = pointCloud->makeShared();
            }
            save_queue.push(ptr_colorImg, ptr_depthImg, pointCloud_copy, ptr_infraredImg);
#else
            if (save_pointcloud) {
              ptr_pointCloud = std::make_unique<std::vector<vpColVector>>(pointCloud);
            }
            save_queue.push(ptr_colorImg, ptr_depthImg, ptr_pointCloud, ptr_infraredImg);
#endif
          }
          break;

        case vpMouseButton::button2:
        case vpMouseButton::button3:
        default:
          save = false;
          quit = true;
          save_queue.cancel();
          break;
        }
      }
    }
  }

  double mean_vec_delta_time = vpMath::getMean(vec_delta_time);
  double median_vec_delta_time = vpMath::getMedian(vec_delta_time);
  double std_vec_delta_time = vpMath::getStdev(vec_delta_time);
  std::cout << "Acquisition time, mean=" << mean_vec_delta_time << " ms ; median="
    << median_vec_delta_time << " ms ; std=" << std_vec_delta_time << " ms" << std::endl;
  std::cout << "FPS, mean=" << 1000/mean_vec_delta_time << " fps ; median="
    << 1000/median_vec_delta_time << " fps" << std::endl;

  storage_thread.join();

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "Need libRealSense or libRealSense2 and C++11 and displayX or displayGDI!" << std::endl;

#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
