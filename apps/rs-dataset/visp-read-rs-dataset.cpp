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
  \example visp-read-rs-dataset.cpp

  \brief App that allows to replay realsense data saved with visp-save-rs-dataset.cpp
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_THREADS) && defined(VISP_HAVE_DISPLAY)
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoException.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpDisplayPCL.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpVideoWriter.h>

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

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void usage(const char *argv[], int error)
{
  std::cout << "\nNAME " << std::endl
    << "  " << vpIoTools::getName(argv[0])
    << " - Replay data (color, depth, infrared, point cloud) acquired" << std::endl
    << "  with a Realsense device using visp-save-rs-dataset app." << std::endl;

  std::cout << "\nDESCRIPTION " << std::endl
    << "  This app allows to replay a dataset (color, depth, infrared, point cloud)" << std::endl
    << "  acquired with a Realsense device using visp-save-rs-dataset app." << std::endl;

  std::cout << "\nSYNOPSIS " << std::endl
    << "  " << vpIoTools::getName(argv[0])
    << " [--input-folder,-i <input folder>]"
    << " [--pattern,-e <filename numbering pattern (e.g. %06d)>]"
    << " [--step-by-step,-s]"
    << " [--save-video,-o]"
    << " [--display-colored-depth,-colored-depth]"
    << " [--loop,-l]"
    << " [--fps,-f <framerate>]"
    << " [--help,-h]"
    << std::endl;

  std::cout << "\nOPTIONS " << std::endl
    << "  --input-folder,-i <input folder>" << std::endl
    << "    Input folder that contains the data to read." << std::endl
    << std::endl
    << "  --pattern,-e <filename numbering pattern (e.g. %06d)>" << std::endl
    << "    Filename numbering pattern used when saving data." << std::endl
    << std::endl
    << "  --step-by-step,-s" << std::endl
    << "    Flag to display data in step by step mode triggered by a user click." << std::endl
    << std::endl
    << "  --save-video,-o" << std::endl
    << "    Creates and saves a new image with the color image on the left and" << std::endl
    << "    the depth image on the right. Images are saved in a new folder" << std::endl
    << "    that corresponds to the date of the day with the following format" << std::endl
    << "    YYYY-MM-DD_HH.MM.SS, for example 2025-11-10_17.06.57."
    << std::endl
    << "  --display-colored-depth,-colored-depth" << std::endl
    << "    Display depth using a cumulative histogram." << std::endl
    << "    Warning: this operation is time consuming" << std::endl
    << std::endl
    << "  --loop,-l" << std::endl
    << "    When this option is enabled, once the sequence has reached the end," << std::endl
    << "    playback will restart from the first frame." << std::endl
    << std::endl
    << "  --fps,-f <framerate>" << std::endl
    << "    Framerate in Hz used to playback the dataset." << std::endl
    << "    Default: 30 fps" << std::endl
    << std::endl
    << "  --help,-h" << std::endl
    << "    Display this helper message." << std::endl
    << std::endl;

  std::cout << "EXAMPLE " << std::endl
    << "- Read dataset in data folder" << std::endl
    << "  " << argv[0] << " --input-folder data --display-colored-depth" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

bool getOptions(int argc, const char *argv[], std::string &input_folder, std::string &numbering_pattern,
                bool &step_by_step, bool &save_video, bool &display_colored_depth, bool &loop, double &fps)
{
  for (int i = 1; i < argc; i++) {
    if (((std::string(argv[i]) == "--input-folder") || (std::string(argv[i]) == "-i")) && (i + 1 < argc)) {
      input_folder = std::string(argv[++i]);
    }
    else if (((std::string(argv[i]) == "--pattern") || (std::string(argv[i]) == "-e")) && (i + 1 < argc)) {
      numbering_pattern = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--step-by-step") || (std::string(argv[i]) == "-s")) {
      step_by_step = true;
    }
    else if ((std::string(argv[i]) == "--save-video") || (std::string(argv[i]) == "-o")) {
      save_video = true;
    }
    else if ((std::string(argv[i]) == "--display-colored-depth") || (std::string(argv[i]) == "-colored-depth")) {
      display_colored_depth = true;
    }
    else if ((std::string(argv[i]) == "--loop") || (std::string(argv[i]) == "-l")) {
      loop = true;
    }
    else if (((std::string(argv[i]) == "--fps") || (std::string(argv[i]) == "-f")) && (i + 1 < argc)) {
      fps = std::atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return false;
    }
    else {
      usage(argv, i);
      return false;
    }
  }

  return true;
}

void checkData(unsigned int cpt_frame, const std::string &input_folder, const std::string &input_pattern,
               bool &color_found, std::string &color_ext,
               bool &depth_found, std::string &depth_ext,
               bool &infra_found, std::string &infra_ext,
               bool &pcl_found, std::string &pcl_ext,
               unsigned int &frame_first, unsigned int &frame_last)
{
  // Check if color present
  {
    std::vector<std::string> ext;
    ext.push_back(".jpg");
    ext.push_back(".png");
    for (size_t i = 0; i < ext.size(); ++i) {
      std::string f = vpIoTools::formatString(input_folder + "/color_image_" + input_pattern + ext[i], cpt_frame);
      if (vpIoTools::checkFilename(f)) {
        color_ext = ext[i];
        color_found = true;
        break;
      }
    }
  }

  // Check if depth present
  {
    std::vector<std::string> ext;
    ext.push_back(".bin");
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    ext.push_back(".npz");
#endif
    for (size_t i = 0; i < ext.size(); ++i) {
      std::string f = vpIoTools::formatString(input_folder + "/depth_image_" + input_pattern + ext[i], cpt_frame);
      if (vpIoTools::checkFilename(f)) {
        depth_ext = ext[i];
        depth_found = true;
        break;
      }
    }
  }

  // Check if infrared present
  {
    std::vector<std::string> ext;
    ext.push_back(".jpg");
    ext.push_back(".png");
    for (size_t i = 0; i < ext.size(); ++i) {
      std::string f = vpIoTools::formatString(input_folder + "/infrared_image_" + input_pattern + ext[i], cpt_frame);
      if (vpIoTools::checkFilename(f)) {
        infra_ext = ext[i];
        infra_found = true;
        break;
      }
    }
  }
  // Check if point cloud present
  {
    std::vector<std::string> ext;
    ext.push_back(".bin");
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
    ext.push_back(".npz");
#endif
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_PCL_COMMON)
    ext.push_back(".pcd");
#endif
    for (size_t i = 0; i < ext.size(); ++i) {
      std::string f = vpIoTools::formatString(input_folder + "/point_cloud_" + input_pattern + ext[i], cpt_frame);
      if (vpIoTools::checkFilename(f)) {
        pcl_ext = ext[i];
        pcl_found = true;
        break;
      }
    }
  }

  // Get first and last frame index
  {
    vpVideoReader g;
    if (color_found) {
      g.setFileName(input_folder + "/color_image_" + input_pattern + color_ext);
    }
    else if (infra_found) {
      g.setFileName(input_folder + "/infrared_image_" + input_pattern + infra_ext);
    }
    else {
      std::cout << "WARNING: Unable to find first and last frame index images" << std::endl;
      return;
    }
    vpImage<unsigned char> I;
    g.open(I);
    frame_first = g.getFirstFrameIndex();
    frame_last = g.getLastFrameIndex();
    g.close();
  }
}

/*!
 * Read color, depth and infrared data from dataset.
 * Point cloud is only read when PCL 3rd party is available. This 3rd party is then used to visualize
 * the point cloud in the main() function.
 */
bool readData(int cpt, const std::string &input_folder, const std::string &input_pattern,
              bool color_found, std::string color_ext,
              bool depth_found, std::string depth_ext,
              bool infra_found, std::string infra_ext,
              vpImage<vpRGBa> &I_color, vpImage<uint16_t> &I_depth_raw, vpImage<unsigned char> &I_infra,
              std::string &filename_color, std::string &filename_depth, std::string &filename_infra
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
              , bool pcl_found, std::string pcl_ext, std::string &filename_pcl
              , pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud
#endif
)
{
  filename_color = vpIoTools::formatString(input_folder + "/color_image_" + input_pattern + color_ext, cpt);
  filename_depth = vpIoTools::formatString(input_folder + "/depth_image_" + input_pattern + depth_ext, cpt);
  filename_infra = vpIoTools::formatString(input_folder + "/infrared_image_" + input_pattern + infra_ext, cpt);
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  filename_pcl = vpIoTools::formatString(input_folder + "/point_cloud_" + input_pattern + pcl_ext, cpt);
#endif

  if (!vpIoTools::checkFilename(filename_color) && !vpIoTools::checkFilename(filename_depth) &&
      !vpIoTools::checkFilename(filename_infra)) {
    std::cerr << "End of sequence." << std::endl;
    return false;
  }

  // Read color
  if (color_found) {
    if (vpIoTools::checkFilename(filename_color)) {
      vpImageIo::read(I_color, filename_color);
    }
  }

  // Read raw depth
  if (depth_found) {
    if (vpIoTools::checkFilename(filename_depth)) {
      if (depth_ext == ".bin") { // Use binary format
        std::ifstream file_depth(filename_depth.c_str(), std::ios::in | std::ios::binary);
        if (file_depth.is_open()) {
          unsigned int height = 0, width = 0;
          vpIoTools::readBinaryValueLE(file_depth, height);
          vpIoTools::readBinaryValueLE(file_depth, width);
          I_depth_raw.resize(height, width);

          uint16_t depth_value = 0;
          for (unsigned int i = 0; i < height; i++) {
            for (unsigned int j = 0; j < width; j++) {
              vpIoTools::readBinaryValueLE(file_depth, depth_value);
              I_depth_raw[i][j] = depth_value;
            }
          }
        }
      }
      else if (depth_ext == ".npz") {
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
        visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(filename_depth);

        // Load depth data
        visp::cnpy::NpyArray arr_depth_data = npz_data["data"];
        if (arr_depth_data.data_holder == nullptr) {
          throw vpIoException(vpIoException::ioError, "Loaded NPZ data is null.");
        }

        uint16_t *depth_data_ptr = arr_depth_data.data<uint16_t>();
        assert(arr_depth_data.shape.size() == 3); // H x W x C
        assert(arr_depth_data.shape[2] == 1); // Single channel

        unsigned int height = static_cast<unsigned int>(arr_depth_data.shape[0]);
        unsigned int width = static_cast<unsigned int>(arr_depth_data.shape[1]);
        const bool copyData = true;
        I_depth_raw = vpImage<uint16_t>(depth_data_ptr, height, width, copyData);

#else
        throw(vpIoException(vpIoException::ioError, "Cannot read depth image in npz format."));
#endif
      }
    }
  }

  // Read infrared
  if (infra_found) {
    if (vpIoTools::checkFilename(filename_infra)) {
      vpImageIo::read(I_infra, filename_infra);
    }
  }

  // Read pointcloud
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  if (pcl_found) {
    if (vpIoTools::checkFilename(filename_pcl)) {
      if (pcl_ext == ".npz") {
#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
        visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(filename_pcl);

        // Load pointcloud data
        visp::cnpy::NpyArray arr_pcl_data = npz_data["data"];
        if (arr_pcl_data.data_holder == nullptr) {
          throw vpIoException(vpIoException::ioError, "Loaded NPZ data is null.");
        }

        float *pcl_data_ptr = arr_pcl_data.data<float>();
        assert(arr_pcl_data.shape.size() == 3); // H x W x C
        assert(arr_pcl_data.shape[2] == 3); // 3-channels: X, Y, Z

        uint32_t height = arr_pcl_data.shape[0], width = arr_pcl_data.shape[1];
        const char is_dense = 1;

        point_cloud->width = width;
        point_cloud->height = height;
        point_cloud->is_dense = (is_dense != 0);
        point_cloud->resize(static_cast<size_t>(width * height));

        for (uint32_t i = 0; i < height; i++) {
          for (uint32_t j = 0; j < width; j++) {
            point_cloud->points[static_cast<size_t>(i * width + j)].x = pcl_data_ptr[static_cast<size_t>(i * width + j)*3 + 0];
            point_cloud->points[static_cast<size_t>(i * width + j)].y = pcl_data_ptr[static_cast<size_t>(i * width + j)*3 + 1];
            point_cloud->points[static_cast<size_t>(i * width + j)].z = pcl_data_ptr[static_cast<size_t>(i * width + j)*3 + 2];
          }
        }
#endif
      }
      else if (pcl_ext == ".pcd") {
#if defined(VISP_HAVE_PCL_IO)
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcl, *point_cloud) == -1) {
          std::cerr << "Cannot read PCD: " << filename_pcl << std::endl;
        }
#else
        throw(vpIoException(vpIoException::ioError, "Cannot read pcd file without PCL io module"));
#endif
      }
      else if (pcl_ext == ".bin") {
        std::ifstream file_pointcloud(filename_pcl.c_str(), std::ios::in | std::ios::binary);
        if (!file_pointcloud.is_open()) {
          std::cerr << "Cannot read pointcloud file: " << filename_pcl << std::endl;
        }

        uint32_t height = 0, width = 0;
        const char is_dense = 1;
        vpIoTools::readBinaryValueLE(file_pointcloud, height);
        vpIoTools::readBinaryValueLE(file_pointcloud, width);
        file_pointcloud.read((char *)(&is_dense), sizeof(is_dense));

        point_cloud->width = width;
        point_cloud->height = height;
        point_cloud->is_dense = (is_dense != 0);
        point_cloud->resize(static_cast<size_t>(width * height));

        float x = 0.0f, y = 0.0f, z = 0.0f;
        for (uint32_t i = 0; i < height; i++) {
          for (uint32_t j = 0; j < width; j++) {
            vpIoTools::readBinaryValueLE(file_pointcloud, x);
            vpIoTools::readBinaryValueLE(file_pointcloud, y);
            vpIoTools::readBinaryValueLE(file_pointcloud, z);

            point_cloud->points[static_cast<size_t>(i * width + j)].x = x;
            point_cloud->points[static_cast<size_t>(i * width + j)].y = y;
            point_cloud->points[static_cast<size_t>(i * width + j)].z = z;
          }
        }
      }
    }
  }
#endif

  return true;
}
} // Namespace

int main(int argc, const char *argv[])
{
  std::string opt_input_folder = "";
  std::string opt_input_pattern = "%04d";
  bool opt_step_by_step = false;
  bool opt_save_video = false;
  bool opt_display_colored_depth = false;
  bool opt_loop = false;
  double opt_fps = 30.0; // Hz

  // Read the command line options
  if (!getOptions(argc, argv, opt_input_folder, opt_input_pattern, opt_step_by_step,
                  opt_save_video, opt_display_colored_depth, opt_loop, opt_fps)) {
    return EXIT_FAILURE;
  }

  vpImage<vpRGBa> I_color, I_depth_color;
  vpImage<uint16_t> I_depth_raw;
  vpImage<unsigned char> I_depth_gray, I_infra;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d1 = vpDisplayFactory::createDisplay(); // color
  std::shared_ptr<vpDisplay> d2 = vpDisplayFactory::createDisplay(); // depth
  std::shared_ptr<vpDisplay> d3 = vpDisplayFactory::createDisplay(); // infrared
#else
  vpDisplay *d1 = vpDisplayFactory::allocateDisplay(); // color
  vpDisplay *d2 = vpDisplayFactory::allocateDisplay(); // depth
  vpDisplay *d3 = vpDisplayFactory::allocateDisplay(); // infrared
#endif

  bool init_display = false;

#if defined(VISP_HAVE_PCL)
  std::mutex mutex;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
#if defined(VISP_HAVE_PCL_VISUALIZATION)
  vpDisplayPCL pcl_viewer;
#endif
#endif

  vpVideoWriter writer;
  vpImage<vpRGBa> O;
  std::string output_folder = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  if (opt_save_video) {
    vpIoTools::makeDirectory(output_folder);
    writer.setFileName(output_folder + "/" + opt_input_pattern + ".png");
  }

  unsigned int cpt_frame = 0;
  bool color_found = false;
  bool depth_found = false;
  bool infra_found = false;
  bool pcl_found = false;
  std::string color_ext, depth_ext, infra_ext, pcl_ext;
  unsigned int frame_first = 0, frame_last = 0;

  checkData(cpt_frame, opt_input_folder, opt_input_pattern,
            color_found, color_ext,
            depth_found, depth_ext,
            infra_found, infra_ext,
            pcl_found, pcl_ext,
            frame_first, frame_last);

  std::cout << "Dataset in " << opt_input_folder << " contains" << std::endl;
  std::cout << "  - Color images     : " << (color_found ? "yes" : "no") << std::endl;
  std::cout << "  - Depth images     : " << (depth_found ? "yes" : "no") << std::endl;
  std::cout << "  - Infrared images  : " << (infra_found ? "yes" : "no") << std::endl;
  std::cout << "  - Point cloud      : " << (pcl_found ? "yes" : "no") << std::endl;
  std::cout << "Dataset" << std::endl;
  std::cout << "  - First frame index: " << frame_first << std::endl;
  std::cout << "  - Last  frame index: " << frame_last << std::endl;


  std::cout << "Options summary" << std::endl;
  std::cout << "  Data visualization   " << std::endl;
  std::cout << "    Colored depth    : " << (opt_display_colored_depth ? "yes" : "no") << std::endl;
  std::cout << "    Frame per seconds: " << opt_fps << std::endl;
  std::cout << "  Save dataset       : " << (opt_save_video ? "yes" : "no") << std::endl;
  if (opt_save_video) {
    std::cout << "    Output folder    : " << output_folder << std::endl;
  }

  if (!color_found && !depth_found && !infra_found && !pcl_found) {
    std::cout << "\nError: No data found in " << opt_input_folder << " folder" << std::endl;
    return EXIT_FAILURE;
  }
  bool quit = false;
  std::string filename_color, filename_depth, filename_infra, filename_pcl;

  while (!quit) {
    double t = vpTime::measureTimeMs();
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    {
      std::lock_guard<std::mutex> lock(mutex);
      quit = !readData(cpt_frame, opt_input_folder, opt_input_pattern,
                       color_found, color_ext,
                       depth_found, depth_ext,
                       infra_found, infra_ext,
                       I_color, I_depth_raw, I_infra,
                       filename_color, filename_depth, filename_infra,
                       pcl_found, pcl_ext, filename_pcl, pointcloud);
    }
#else
    quit = !readData(cpt_frame, opt_input_folder, opt_input_pattern,
                     color_found, color_ext,
                     depth_found, depth_ext,
                     infra_found, infra_ext,
                     I_color, I_depth_raw, I_infra,
                     filename_color, filename_depth, filename_infra);
#endif

    if (opt_display_colored_depth) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_color);
    }
    else {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_gray);
    }

    if (!init_display) {
      init_display = true;
      if (color_found) {
        d1->init(I_color, 0, 0, "Color image");
      }
      if (depth_found) {
        if (opt_display_colored_depth) {
          d2->init(I_depth_color, I_color.getWidth() + 80, 0, "Depth image");
        }
        else {
          d2->init(I_depth_gray, I_color.getWidth() + 80, 0, "Depth image");
        }
      }
      if (infra_found) {
        d3->init(I_infra, I_color.getWidth() + 80, I_color.getHeight() + 70, "Infrared image");
      }
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION)
      if (pcl_found) {
        if (pointcloud->size() > 0) {
          pcl_viewer.setPosition(0, I_color.getHeight() + 70);
          pcl_viewer.setWindowName("3D point cloud");
          pcl_viewer.startThread(std::ref(mutex), pointcloud);
        }
      }
#endif
    }

    vpDisplay::display(I_color);
    vpDisplay::setTitle(I_color, "Color image: " + vpIoTools::getName(filename_color));
    if (opt_step_by_step) {
      vpDisplay::displayText(I_color, 15, 15, "Left click to view next data", vpColor::red);
      vpDisplay::displayText(I_color, 30, 15, "Right click to switch to continuous mode", vpColor::red);
    }
    else {
      vpDisplay::displayText(I_color, 15, 15, "Left click to quit", vpColor::red);
      vpDisplay::displayText(I_color, 30, 15, "Right click to switch to step-by-step mode", vpColor::red);
    }
    if (opt_display_colored_depth) {
      vpDisplay::display(I_depth_color);
      vpDisplay::setTitle(I_depth_color, "Colored depth image: " + vpIoTools::getName(filename_depth));
    }
    else {
      vpDisplay::display(I_depth_gray);
      vpDisplay::setTitle(I_depth_color, "Depth image: " + vpIoTools::getName(filename_depth));
    }
    vpDisplay::display(I_infra);
    vpDisplay::setTitle(I_infra, "Infrared image: " + vpIoTools::getName(filename_infra));

    vpDisplay::flush(I_color);
    if (opt_display_colored_depth) {
      vpDisplay::flush(I_depth_color);
    }
    else {
      vpDisplay::flush(I_depth_gray);
    }
    vpDisplay::flush(I_infra);

    if (opt_save_video) {
      if (O.getSize() == 0) {
        O.resize(I_color.getHeight(), I_color.getWidth() + I_depth_color.getWidth());
        writer.open(O);
      }

      O.insert(I_color, vpImagePoint());
      if (!opt_display_colored_depth) {
        vpImageConvert::convert(I_depth_gray, I_depth_color);
      }
      O.insert(I_depth_color, vpImagePoint(0, I_color.getWidth()));
      writer.saveFrame(O);
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_color, button, opt_step_by_step)) {
      switch (button) {
      case vpMouseButton::button1:
        if (!quit)
          quit = !opt_step_by_step;
        break;

      case vpMouseButton::button3:
        opt_step_by_step = !opt_step_by_step;
        break;

      default:
        break;
      }
    }

    vpTime::wait(t, 1000. / opt_fps);
    cpt_frame++;
    if (opt_loop) {
      if (cpt_frame == frame_last) {
        std::cout << "End of sequence reached" << std::endl;
        cpt_frame = frame_first;
      }
    }
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
  std::cerr << "Enable C++11 or higher (cmake -DUSE_CXX_STANDARD=11) and install X11 or GDI!" << std::endl;
  return EXIT_SUCCESS;
}
#endif
