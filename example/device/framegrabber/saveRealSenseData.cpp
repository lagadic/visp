/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */

 /*!
   \example saveRealSenseData.cpp

   \brief App that allows to record data (color, depth, infrared, point cloud) from a Realsense device.
   Recorded data can be replayed with readRealSenseData app.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)) && defined(VISP_HAVE_THREADS) \
  && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML) \
  && ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))

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
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void usage(const char *argv[], int error, int stream_width, int stream_height, int stream_fps)
{
  std::cout << "\nNAME " << std::endl
    << "  " << vpIoTools::getName(argv[0])
    << " - Record data (color, depth, infrared, point cloud) with a Realsense device." << std::endl;

  std::cout << "\nDESCRIPTION " << std::endl
    << "  This app allows to record a dataset (color, depth, infrared, point cloud) acquired" << std::endl
    << "  with a Realsense device. Once acquired, the dataset can be visualized using readRealSenseData app." << std::endl;

  std::cout << "\nSYNOPSIS " << std::endl
    << "  " << vpIoTools::getName(argv[0])
    << " [--save,-s]"
    << " [--output-folder,-o <output folder>]"
    << " [--pattern,-e <filename numbering pattern (e.g. %06d)>]"
    << " [--step-by-step,-s]"
    << " [--aligned,-a]"
    << " [--save-color,-c]"
    << " [--save-depth,-d]"
    << " [--save-infrared,-i]"
    << " [--save-pcl,-p]"
    << " [--img-in-jpeg-fmt,-j]"
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    << " [--depth-in-npz-fmt,-depth-npz]"
    << " [--pcl-in-npz-fmt,-pcl-npz]"
#endif
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
    << " [--pcl-in-pcd-fmt,-pcl-pcd]"
#endif
    << " [--stream-width,-sw <stream width>]"
    << " [--stream-height,-sh <stream height>]"
    << " [--stream-fps,-f <framerate>]"
    << " [--display-colored-depth,-colored-depth]"
    << " [--help,-h]"
    << std::endl;

  std::cout << "\nOPTIONS " << std::endl
    << "  --save, -s" << std::endl
    << "    Flag to enable data saving in output folder." << std::endl
    << "    When this flag is not used, we only display data." << std::endl
    << std::endl
    << "  --output-folder,-o <custom output folder>" << std::endl
    << "    Custom output folder that will host saved data." << std::endl
    << "    By default, for each data acquisition, a new output folder is" << std::endl
    << "    created with the current date in the following format" << std::endl
    << "    YYYY_MM_DD_HH:MM:SS, for example 2025_11_07_15:44:34." << std::endl
    << std::endl
    << "  --step-by-step,-s" << std::endl
    << "    Trigger one shot data saver after each user click." << std::endl
    << std::endl
    << "  --pattern,-e <filename numbering pattern (e.g. %06d)>" << std::endl
    << "    Filename numbering pattern to use when saving data." << std::endl
    << std::endl
    << "  --aligned,-a" << std::endl
    << "    Color and depth are aligned." << std::endl
    << std::endl
    << "  --save-color,-c" << std::endl
    << "    Add color stream to saved data when --save option is used." << std::endl
    << "    By default, images are saved in png format. To save in jpeg format" << std::endl
    << "    use --img-in-jpeg-fmt flag." << std::endl
    << std::endl
    << "  --save-depth,-d" << std::endl
    << "    Add depth stream to saved data when --save option is enabled." << std::endl
    << "    By default, depth images are saved in binary format." << std::endl
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    << "    To save in NumPy format use --depth-in-npz-fmt flag." << std::endl
#endif
    << std::endl
    << "  --save-infrared" << std::endl
    << "    Add infrared stream to saved data when --save option is enabled." << std::endl
    << "    By default, images are saved in png format. To save in jpeg format" << std::endl
    << "    use --img-in-jpeg-fmt flag." << std::endl
    << std::endl
    << "  --save-pcl,-p" << std::endl
    << "    Add point cloud stream to saved data when --save option is enabled." << std::endl
    << "    By default, the point cloud is saved in little-endian binary format" << std::endl
    << "    (.bin extension file)." << std::endl
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    << "    There is also the possibility to save the point cloud in NumPy" << std::endl
    << "    format (.npy extension file) enabling --pcl-in-npz-fmt option." << std::endl
#endif
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
    << "    Since PCL library is available, the point cloud could also be saved in" << std::endl
    << "    Point Cloud Data file format (.pcd extension file) enabling" << std::endl
    << "    --pcl-in-pcd-fmt option." << std::endl
#endif
    << std::endl
    << "  --img-in-jpeg-fmt,-j" << std::endl
    << "    Save image data using jpeg format (otherwise PNG is used)." << std::endl
    << std::endl
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    << "  --depth-in-npz-fmt,-depth-npz" << std::endl
    << "    Depth is saved in npz (NumPy) format." << std::endl
    << std::endl
    << "  --pcl-in-npz-fmt,-pcl-npz" << std::endl
    << "    Point cloud is saved in npz (NumPy) format." << std::endl
    << std::endl
#endif
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
    << "  --pcl-in-pcd-fmt,-pcl-pcd" << std::endl
    << "    Point cloud is saved in pcd format using PCL 3rd-party library." << std::endl
    << std::endl
#endif
    << "  --stream-width,-sw <stream width>" << std::endl
    << "    Set camera image width resolution." << std::endl
    << "    Default: " << stream_width << std::endl
    << std::endl
    << "  --stream-height,-sh <stream height>" << std::endl
    << "    Set camera image height resolution." << std::endl
    << "    Default: " << stream_height << std::endl
    << std::endl
    << "  --stream-fps,-f <fps>" << std::endl
    << "    Set camera framerate in Hz." << std::endl
    << "    Default: " << stream_fps << std::endl
    << std::endl
    << "  --display-colored-depth,-colored-depth" << std::endl
    << "    Display depth using a cumulative histogram." << std::endl
    << "    Warning: this operation is time consuming" << std::endl
    << std::endl
    << "  --help,-h" << std::endl
    << "    Display this helper message." << std::endl
    << std::endl;
  std::cout << "EXAMPLE " << std::endl
    << "- Save aligned color + depth + point cloud in data folder" << std::endl
    << "  " << argv[0] << " --save --save-color --save-depth --save-pcl --display-colored-depth --output-folder data --aligned" << std::endl
    << "- Save color + infrared + depth + point cloud in NPZ format in data folder" << std::endl
    << "  " << argv[0] << " --save --save-color --save-depth --save-infrared --save-pcl --display-colored-depth --depth-in-npz-fmt --pcl-in-npz-fmt --output-folder data" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

bool getOptions(int argc, const char *argv[], bool &save, std::string &numbering_pattern, std::string &output_folder,
                bool &use_aligned_stream, bool &save_color, bool &save_depth, bool &save_pcl,
                bool &save_infrared, bool &step_by_step, int &stream_width, int &stream_height, int &stream_fps,
                bool &depth_npz_fmt, bool &pcl_npz_fmt, bool &pcl_pcd_fmt, bool &img_jpeg_fmt,
                bool &display_colored_depth)
{
  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--save") || (std::string(argv[i]) == "-s")) {
      save = true;
    }
    else if (((std::string(argv[i]) == "--pattern") || (std::string(argv[i]) == "-e")) && (i + 1 < argc)) {
      numbering_pattern = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--aligned") || (std::string(argv[i]) == "-a")) {
      use_aligned_stream = true;
    }
    else if (((std::string(argv[i]) == "--output-folder") || (std::string(argv[i]) == "-o")) && (i + 1 < argc)) {
      output_folder = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--save-color") || (std::string(argv[i]) == "-c")) {
      save_color = true;
    }
    else if ((std::string(argv[i]) == "--save-depth") || (std::string(argv[i]) == "-d")) {
      save_depth = true;
    }
    else if ((std::string(argv[i]) == "--save-infrared") || (std::string(argv[i]) == "-i")) {
      save_infrared = true;
    }
    else if ((std::string(argv[i]) == "--save-pcl") || (std::string(argv[i]) == "-p")) {
      save_pcl = true;
    }
    else if ((std::string(argv[i]) == "--img-in-jpeg-fmt") || (std::string(argv[i]) == "-j")) {
      img_jpeg_fmt = true;
    }
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    else if ((std::string(argv[i]) == "--depth-in-npz-fmt") || (std::string(argv[i]) == "-depth-npz")) {
      depth_npz_fmt = true;
    }
    else if ((std::string(argv[i]) == "--pcl-in-npz-fmt") || (std::string(argv[i]) == "-pcl-npz")) {
      pcl_npz_fmt = true;
    }
#endif
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
    else if ((std::string(argv[i]) == "--pcl-in-pcd-fmt") || (std::string(argv[i]) == "-pcl-pcd")) {
      pcl_pcd_fmt = true;
    }
#endif
    else if ((std::string(argv[i]) == "--click") || (std::string(argv[i]) == "-c")) {
      step_by_step = true;
    }
    else if (((std::string(argv[i]) == "--stream-width") || (std::string(argv[i]) == "-sw")) && (i + 1 < argc)) {
      stream_width = std::atoi(argv[++i]);
    }
    else if (((std::string(argv[i]) == "--stream-height") || (std::string(argv[i]) == "-sh")) && (i + 1 < argc)) {
      stream_height = std::atoi(argv[++i]);
    }
    else if (((std::string(argv[i]) == "--stream-fps") || (std::string(argv[i]) == "-f")) && (i + 1 < argc)) {
      stream_fps = std::atoi(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--display-colored-depth") || (std::string(argv[i]) == "--colored-depth")) {
      display_colored_depth = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0, stream_width, stream_height, stream_fps);
      return false;
    }
    else {
      usage(argv, i, stream_width, stream_height, stream_fps);
      return false;
    }
  }

#if !(defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX))
  (void)depth_npz_fmt;
  (void)pcl_npz_fmt;
#endif
#if !(defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON))
  (void)pcl_pcd_fmt;
#endif

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
  vpStorageWorker(vpFrameQueue &queue, const std::string &output_pattern, const std::string &directory, bool save_color,
                  bool save_depth, bool save_pcl, bool save_infrared, bool depth_npz_fmt,
                  bool pcl_npz_fmt, bool pcl_pcd_fmt, bool img_jpeg_fmt,
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
    : m_queue(queue), m_output_pattern(output_pattern), m_directory(directory), m_cpt(0), m_save_color(save_color), m_save_depth(save_depth),
    m_save_pcl(save_pcl), m_save_infrared(save_infrared), m_depth_npz_fmt(depth_npz_fmt),
    m_pcl_npz_fmt(pcl_npz_fmt), m_pcl_pcd_fmt(pcl_pcd_fmt),
    m_img_jpeg_fmt(img_jpeg_fmt)
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

      std::string image_filename_ext = m_img_jpeg_fmt ? ".jpg" : ".png";
      for (;;) {
        m_queue.pop(ptr_colorImg, ptr_depthImg, ptr_pointCloud, ptr_infraredImg);

        if (!m_directory.empty()) {
          std::string current_time = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");

          if (m_save_color && ptr_colorImg) {
            std::string filename_color = vpIoTools::formatString(m_directory + "/color_image_" + m_output_pattern + image_filename_ext, m_cpt);
            vpImageIo::write(*ptr_colorImg, filename_color);
          }

          if (m_save_depth && ptr_depthImg) {
            if (!m_depth_npz_fmt) { // Use binary format

              std::string filename_depth = vpIoTools::formatString(m_directory + "/depth_image_" + m_output_pattern + ".bin", m_cpt);

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
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
            else {
              std::string filename_depth = vpIoTools::formatString(m_directory + "/depth_image_" + m_output_pattern + ".npz", m_cpt);

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

          if (m_save_pcl && ptr_pointCloud) {

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
            uint32_t width = ptr_pointCloud->width;
            uint32_t height = ptr_pointCloud->height;
#else
            uint32_t width = m_size_width;
            uint32_t height = m_size_height;
#endif

            if (m_pcl_npz_fmt) {
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
              std::string filename_pcl = vpIoTools::formatString(m_directory + "/point_cloud_" + m_output_pattern + ".npz", m_cpt);

              // Write Npz headers
              std::vector<char> vec_filename(filename_pcl.begin(), filename_pcl.end());
              // Null-terminated character is handled at reading
              // For null-terminated character handling, see:
              // https://stackoverflow.com/a/8247804
              // https://stackoverflow.com/a/45491652
              visp::cnpy::npz_save(filename_pcl, "filename", &vec_filename[0], { vec_filename.size() }, "w");

              std::vector<char> vec_current_time(current_time.begin(), current_time.end());
              visp::cnpy::npz_save(filename_pcl, "timestamp", &vec_current_time, { vec_current_time.size() }, "a");

              const uint32_t channels = 3;
              visp::cnpy::npz_save(filename_pcl, "height", &height, { 1 }, "a");
              visp::cnpy::npz_save(filename_pcl, "width", &width, { 1 }, "a");
              visp::cnpy::npz_save(filename_pcl, "channel", &channels, { 1 }, "a");

              vec_pcl.resize(height * width * channels);
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
              for (uint32_t i = 0; i < height; i++) {
                for (uint32_t j = 0; j < width; j++) {
                  vec_pcl[channels * (i*width + j) + 0] = static_cast<float>((*ptr_pointCloud)[i*width + j][0]);
                  vec_pcl[channels * (i*width + j) + 1] = static_cast<float>((*ptr_pointCloud)[i*width + j][1]);
                  vec_pcl[channels * (i*width + j) + 2] = static_cast<float>((*ptr_pointCloud)[i*width + j][2]);
                }
              }
#endif
              // Write data
              visp::cnpy::npz_save(filename_pcl, "data", vec_pcl.data(), { height, width, channels }, "a");
#else
              throw(vpIoException(vpIoException::fatalError, "Cannot save in npz format when npz I/O functions are disabled."));
#endif
            }
            else if (m_pcl_pcd_fmt) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
              std::string filename_pcl = vpIoTools::formatString(m_directory + "/point_cloud_" + m_output_pattern + ".pcd", m_cpt);
              pcl::io::savePCDFileBinary(filename_pcl, *ptr_pointCloud);
#else
              std::cout << "Error: unable to save point cloud in pcd format. PCL io module not available." << std::endl;
#endif
            }
            else {
              std::string filename_pcl = vpIoTools::formatString(m_directory + "/point_cloud_" + m_output_pattern + ".bin", m_cpt);

              std::ofstream file_pointcloud(filename_pcl.c_str(), std::ios::out | std::ios::binary);

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
                    float x = static_cast<float>((*ptr_pointCloud)[i * width + j][0]);
                    float y = static_cast<float>((*ptr_pointCloud)[i * width + j][1]);
                    float z = static_cast<float>((*ptr_pointCloud)[i * width + j][2]);

                    vpIoTools::writeBinaryValueLE(file_pointcloud, x);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, y);
                    vpIoTools::writeBinaryValueLE(file_pointcloud, z);
                  }
                }
#endif
              }
            }
          }

          if (m_save_infrared && ptr_infraredImg) {
            std::string filename_infrared = vpIoTools::formatString(m_directory + "/infrared_image_" + m_output_pattern + image_filename_ext, m_cpt);

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
  std::string m_output_pattern;
  std::string m_directory;
  unsigned int m_cpt;
  bool m_save_color;
  bool m_save_depth;
  bool m_save_pcl;
  bool m_save_infrared;
  bool m_depth_npz_fmt;
  bool m_pcl_npz_fmt;
  bool m_pcl_pcd_fmt;
  bool m_img_jpeg_fmt;
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
  std::string output_pattern = "%04d";
  std::string output_folder = vpTime::getDateTime("%Y_%m_%d_%H.%M.%S");
  std::string output_folder_custom = "";
  bool use_aligned_stream = false;
  bool save_color = false;
  bool save_depth = false;
  bool save_pcl = false;
  bool save_infrared = false;
  bool step_by_step = false;
  int  stream_width = 640;
  int  stream_height = 480;
  int  stream_fps = 30;
  bool depth_npz_fmt = false;
  bool pcl_npz_fmt = false;
  bool pcl_pcd_fmt = false;
  bool img_jpeg_fmt = false;
  bool display_colored_depth = false;

  // Read the command line options
  if (!getOptions(argc, argv, save, output_pattern, output_folder_custom, use_aligned_stream, save_color, save_depth,
                  save_pcl, save_infrared, step_by_step, stream_width, stream_height, stream_fps, depth_npz_fmt, pcl_npz_fmt,
                  pcl_pcd_fmt, img_jpeg_fmt, display_colored_depth)) {
    return EXIT_FAILURE;
  }

  if (!output_folder_custom.empty()) {
    output_folder = output_folder_custom + "/" + output_folder;
  }


  std::cout << "Options summary" << std::endl;
  std::cout << "  Realsense device           " << std::endl;
  std::cout << "    Stream width           : " << stream_width << std::endl;
  std::cout << "    Stream height          : " << stream_height << std::endl;
  std::cout << "    Stream framerate       : " << stream_fps << std::endl;
  std::cout << "  Considered data            " << std::endl;
  std::cout << "    Color stream           : " << (save_color ? "yes" : "no") << std::endl;
  std::cout << "    Depth stream           : " << (save_depth ? "yes" : "no") << std::endl;
  std::cout << "    Infrared stream        : " << (save_infrared ? "yes" : "no") << std::endl;
  std::cout << "    Point cloud            : " << (save_pcl ? "yes" : "no") << std::endl;
  std::cout << "    Color and depth aligned: " << (use_aligned_stream ? "yes" : "no") << std::endl;
  std::cout << "  Data visualization         " << std::endl;
  std::cout << "    Colored depth          : " << (display_colored_depth ? "yes" : "no") << std::endl;
  std::cout << "  Save dataset               " << (save ? "yes" : "no") << std::endl;
  if (save) {
    std::cout << "    Output folder          : " << output_folder << std::endl;
    std::cout << "    File numbering pattern : " << output_pattern << std::endl;
    std::cout << "    Color images format    : " << (save_color ? (img_jpeg_fmt ? "jpeg" : "png") : "N/A") << std::endl;
    std::cout << "    Depth images format    : " << (save_depth ? (depth_npz_fmt ? "npy" : "bin") : "N/A") << std::endl;
    std::cout << "    Infrared images format : " << (save_infrared ? (img_jpeg_fmt ? "jpeg" : "png") : "N/A") << std::endl;
    std::cout << "    Point cloud format     : " << (save_pcl ? (pcl_npz_fmt ? "npy" : (pcl_pcd_fmt ? "pcd" : "bin")) : "N/A") << std::endl;
    std::cout << "    Save after each click  : " << (step_by_step ? "yes" : "no") << std::endl << std::endl;
  }

  vpRealSense2 realsense;

  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGBA8, stream_fps);
  config.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
  config.enable_stream(RS2_STREAM_INFRARED, stream_width, stream_height, RS2_FORMAT_Y8, stream_fps);
  realsense.open(config);

  vpImage<vpRGBa> I_color(stream_height, stream_width);
  vpImage<unsigned char> I_depth_gray(stream_height, stream_width);
  vpImage<vpRGBa> I_depth_color(stream_height, stream_width);
  vpImage<uint16_t> I_depth_raw(stream_height, stream_width);
  vpImage<unsigned char> I_infrared(stream_height, stream_width);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d1 = vpDisplayFactory::createDisplay();
  std::shared_ptr<vpDisplay> d2 = vpDisplayFactory::createDisplay();
  std::shared_ptr<vpDisplay> d3 = vpDisplayFactory::createDisplay();
#else
  vpDisplay *d1 = vpDisplayFactory::allocateDisplay();
  vpDisplay *d2 = vpDisplayFactory::allocateDisplay();
  vpDisplay *d3 = vpDisplayFactory::allocateDisplay();
#endif
  d1->init(I_color, 0, 0, "RealSense color stream");
  if (display_colored_depth) {
    d2->init(I_depth_color, I_color.getWidth() + 80, 0, "RealSense depth stream");
  }
  else {
    d2->init(I_depth_gray, I_color.getWidth() + 80, 0, "RealSense depth stream");
  }

  rs2::align align_to(RS2_STREAM_COLOR);
  if (use_aligned_stream && save_infrared) {
    std::cerr << "Cannot use aligned streams with infrared acquisition currently."
      << "\nInfrared stream acquisition is disabled!"
      << std::endl;
    save_infrared = false;
  }
  if (save_infrared) {
    d3->init(I_infrared, I_color.getWidth() + 80, I_color.getHeight() + 70, "RealSense infrared stream");
  }

  // If PCL is available, always use PCL datatype even if we will save in NPZ or BIN file format
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
#else
  std::vector<vpColVector> pointCloud;
#endif

  bool quit = false;
  while (!quit) {
    double start = vpTime::measureTimeMs();
    if (use_aligned_stream) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud, nullptr,
                        &align_to);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, nullptr,
                        &align_to);
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

    vpDisplay::display(I_color);

    if (display_colored_depth) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_color);
      vpDisplay::display(I_depth_color);
    }
    else {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_gray);
      vpDisplay::display(I_depth_gray);
    }
    if (save_infrared) {
      vpDisplay::display(I_infrared);
    }

    double delta_time = vpTime::measureTimeMs() - start;
    std::ostringstream oss_time;
    oss_time << delta_time << " ms ; fps=" << 1000/delta_time;
    vpDisplay::displayText(I_color, 40, 20, oss_time.str(), vpColor::red);
    if (save) {
      vpDisplay::displayText(I_color, 20, 20, "Click to start recording dataset", vpColor::red);
    }
    else {
      vpDisplay::displayText(I_color, 20, 20, "Click to stop recording", vpColor::red);
    }

    vpDisplay::flush(I_color);
    if (display_colored_depth) {
      vpDisplay::flush(I_depth_color);
    }
    else {
      vpDisplay::flush(I_depth_gray);
    }
    if (save_infrared) {
      vpDisplay::flush(I_infrared);
    }

    if (vpDisplay::getClick(I_color, false)) {
      quit = true;
    }
  }

  if (save) {
    // Create output directory
    vpIoTools::makeDirectory(output_folder);

    // Save intrinsics
    vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_folder + "/camera.xml", "color_camera", stream_width, stream_height);

    if (use_aligned_stream) {
      xml_camera.save(cam_color, output_folder + "/camera.xml", "depth_camera", stream_width, stream_height);
    }
    else {
      vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH);
      xml_camera.save(cam_depth, output_folder + "/camera.xml", "depth_camera", stream_width, stream_height);
    }

    vpCameraParameters cam_infrared = realsense.getCameraParameters(RS2_STREAM_INFRARED);
    xml_camera.save(cam_infrared, output_folder + "/camera.xml", "infrared_camera", stream_width, stream_height);
    if (save_depth) {
      vpHomogeneousMatrix depth_M_color;
      if (!use_aligned_stream) {
        depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
      }
      std::ofstream file(std::string(output_folder + "/depth_M_color.txt"));
      depth_M_color.save(file);
      file.close();
    }
    if (save_infrared) {
      vpHomogeneousMatrix infrared_M_color;
      infrared_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_INFRARED);
      std::ofstream file(std::string(output_folder + "/infrared_M_color.txt"));
      infrared_M_color.save(file);
      file.close();
    }

    vpFrameQueue save_queue;
    vpStorageWorker storage(std::ref(save_queue), output_pattern, std::cref(output_folder), save_color, save_depth,
      save_pcl, save_infrared, depth_npz_fmt, pcl_npz_fmt, pcl_pcd_fmt, img_jpeg_fmt, stream_width, stream_height);
    std::thread storage_thread(&vpStorageWorker::run, &storage);

    int nb_saves = 0;
    quit = false;

    std::unique_ptr<vpImage<vpRGBa>> ptr_colorImg;
    std::unique_ptr<vpImage<uint16_t>> ptr_depthImg;
    std::unique_ptr<std::vector<vpColVector>> ptr_pointCloud;
    std::unique_ptr<vpImage<unsigned char>> ptr_infraredImg;

    std::vector<double> vec_delta_time;
    while (!quit) {
      double start = vpTime::measureTimeMs();
      if (use_aligned_stream) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
        realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud, nullptr,
                          &align_to);
#else
        realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, nullptr,
                          &align_to);
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

      if (display_colored_depth) {
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_color);
      }
      else {
        // Seems like sometimes using createDepthHistogram() takes lots of time?
        // so we simply perform bit shift from uint16_t to uint8_t
        for (unsigned int i = 0; i < I_depth_raw.getRows(); i++) {
          for (unsigned int j = 0; j < I_depth_raw.getCols(); j++) {
            I_depth_gray[i][j] = I_depth_raw[i][j] >> 8;
          }
        }
      }

      vpDisplay::display(I_color);
      if (display_colored_depth) {
        vpDisplay::display(I_depth_color);
      }
      else {
        vpDisplay::display(I_depth_gray);
      }
      vpDisplay::display(I_infrared);

      if (!step_by_step) {
        vpDisplay::displayText(I_color, 20, 20, "Click to quit.", vpColor::red);
      }
      else {
        std::stringstream ss;
        ss << "Images saved: " << nb_saves;
        vpDisplay::displayText(I_color, 20, 20, ss.str(), vpColor::red);
      }

      if (save && !step_by_step) {
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
        if (save_pcl) {
          pointCloud_copy = pointCloud->makeShared();
        }
        save_queue.push(ptr_colorImg, ptr_depthImg, pointCloud_copy, ptr_infraredImg);
#else
        if (save_pcl) {
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
      if (display_colored_depth) {
        vpDisplay::flush(I_depth_color);
      }
      else {
        vpDisplay::flush(I_depth_gray);
      }
      vpDisplay::flush(I_infrared);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I_color, button, false)) {
        if (!step_by_step) {
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
              if (save_pcl) {
                pointCloud_copy = pointCloud->makeShared();
              }
              save_queue.push(ptr_colorImg, ptr_depthImg, pointCloud_copy, ptr_infraredImg);
#else
              if (save_pcl) {
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

    std::cout << "\nSaving dataset in " << output_folder << " folder in progress...\n" << std::endl;
    double mean_vec_delta_time = vpMath::getMean(vec_delta_time);
    double median_vec_delta_time = vpMath::getMedian(vec_delta_time);
    double std_vec_delta_time = vpMath::getStdev(vec_delta_time);
    std::cout << "Acquisition time, mean=" << mean_vec_delta_time << " ms ; median="
      << median_vec_delta_time << " ms ; std=" << std_vec_delta_time << " ms" << std::endl;
    std::cout << "FPS, mean=" << 1000/mean_vec_delta_time << " fps ; median="
      << 1000/median_vec_delta_time << " fps" << std::endl;

    storage_thread.join();

    std::cout << "\nDataset was successfully saved in " << output_folder << " folder\n" << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (d1 != nullptr) {
    delete d1;
  }
  if (d2 != nullptr) {
    delete d2;
  }
  if (d3 != nullptr) {
    delete d3;
  }
#endif

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "Need libRealSense or libRealSense2 and C++11 and displayX or displayGDI!" << std::endl;

#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested." << std::endl;
#elif !((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
  std::cout << "At least c++14 standard is requested." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
