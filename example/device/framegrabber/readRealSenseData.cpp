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
  \example readRealSenseData.cpp

  \brief Example that shows how to replay realsense data saved with saveRealSenseData.cpp
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_THREADS) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoException.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayPCL.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
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

#ifdef VISP_HAVE_MINIZ
#define GETOPTARGS "ci:e:jbzodh"
#else
#define GETOPTARGS "ci:e:jbodh"
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{

void usage(const char *name, const char *badparam)
{
  std::cout << "\nNAME " << std::endl
    << "  " << vpIoTools::getName(name)
    << " - Read data acquired with a Realsense device." << std::endl
    << std::endl
    << "SYNOPSIS " << std::endl
    << "  " << name
    << " [-i <directory>]"
    << " [-e <filename pattern (e.g. %04d)>]"
    << " [-c]"
    << " [-j]"
    << " [-b]"
#ifdef VISP_HAVE_MINIZ
    << " [-z]"
#endif
    << " [-o]"
    << " [-d]"
    << " [--help,-h]"
    << std::endl;
  std::cout << "\nOPTIONS " << std::endl
    << "  -i <directory>" << std::endl
    << "    Input folder that contains the data to read." << std::endl
    << std::endl
    << "  -e <pattern>" << std::endl
    << "    Filename pattern, e.g. %04d." << std::endl
    << std::endl
    << "  -c" << std::endl
    << "    Flag to display data in step by step mode triggered by a user click." << std::endl
    << std::endl
    << "  -j" << std::endl
    << "    Image data are saved in JPEG format, otherwise in PNG." << std::endl
    << std::endl
    << "  -b" << std::endl
    << "    Depth and Pointcloud streams are saved in binary format." << std::endl
    << std::endl
#ifdef VISP_HAVE_MINIZ
    << "  -z" << std::endl
    << "    Pointcloud stream is saved in NPZ format." << std::endl
    << std::endl
#endif
    << "  -o" << std::endl
    << "    Save color images in PNG format (lossless) in a new folder." << std::endl
    << std::endl
    << "  -d" << std::endl
    << "    Display depth in color." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Display this helper message." << std::endl
    << std::endl;

  if (badparam) {
    std::cout << "\nERROR: Bad parameter " << badparam << std::endl;
  }
}

bool getOptions(int argc, const char *argv[], std::string &input_directory, std::string &pattern, bool &click,
                bool &force_binary_format, bool &read_jpeg, bool &read_npz, bool &save_video, bool &color_depth)
{
  const char *optarg;
  const char **argv1 = (const char **)argv;
  int c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'i':
      input_directory = optarg;
      break;
    case 'e':
      pattern = optarg;
      break;
    case 'c':
      click = true;
      break;
    case 'j':
      read_jpeg = true;
      break;
    case 'b':
      force_binary_format = true;
      break;
#ifdef VISP_HAVE_MINIZ
    case 'z':
      read_npz = true;
      break;
#endif
    case 'o':
      save_video = true;
      break;
    case 'd':
      color_depth = true;
      break;

    case 'h':
      usage(argv[0], nullptr);
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
    usage(argv[0], nullptr);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

bool readData(int cpt, const std::string &input_directory, const std::string &pattern, vpImage<vpRGBa> &I_color,
              vpImage<uint16_t> &I_depth_raw, bool force_binary_format, bool read_jpeg, bool read_npz
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
              , pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud
#endif
)
{
  std::string image_filename_ext = read_jpeg ? ".jpg" : ".png";
  std::string depth_filename_ext = force_binary_format ? ".bin" : ".npz";
  char buffer[FILENAME_MAX];
  std::stringstream ss;
  ss << input_directory << "/color_image_" << pattern << image_filename_ext;
  snprintf(buffer, FILENAME_MAX, ss.str().c_str(), cpt);
  std::string filename_color = buffer;

  ss.str("");
  ss << input_directory << "/depth_image_" << pattern << depth_filename_ext;
  snprintf(buffer, FILENAME_MAX, ss.str().c_str(), cpt);
  std::string filename_depth = buffer;

  ss.str("");
  ss << input_directory << "/point_cloud_" << pattern << (force_binary_format ? ".bin" :
    (read_npz ? ".npz" : ".pcd"));
  snprintf(buffer, FILENAME_MAX, ss.str().c_str(), cpt);
  std::string filename_pointcloud = buffer;

  if (!vpIoTools::checkFilename(filename_color) && !vpIoTools::checkFilename(filename_depth) &&
      !vpIoTools::checkFilename(filename_pointcloud)) {
    std::cerr << "End of sequence." << std::endl;
    return false;
  }

  // Read color
  if (vpIoTools::checkFilename(filename_color)) {
    vpImageIo::read(I_color, filename_color);
  }

  // Read raw depth
  if (vpIoTools::checkFilename(filename_depth)) {
    if (force_binary_format) {
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
#ifdef VISP_HAVE_MINIZ
    else {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(filename_depth);

      // Load depth data
      visp::cnpy::NpyArray arr_depth_data = npz_data["data"];
      if (arr_depth_data.data_holder == nullptr) {
        throw vpIoException(vpIoException::ioError, "Loaded NPZ data is null.");
      }

      uint16_t *depth_data_ptr = arr_depth_data.data<uint16_t>();
      assert(arr_depth_data.shape.size() == 3); // H x W x C
      assert(arr_depth_data.shape[2] == 1); // Single channel

      unsigned int height = arr_depth_data.shape[0], width = arr_depth_data.shape[1];
      const bool copyData = true;
      I_depth_raw = vpImage<uint16_t>(depth_data_ptr, height, width, copyData);
    }
#else
    else {
      throw(vpIoException(vpIoException::ioError, "Cannot open non-binary depth file when npz I/O functions are disabled."));
    }
#endif
  }

  // Read pointcloud
  if (vpIoTools::checkFilename(filename_pointcloud)) {
#if defined(VISP_HAVE_PCL)
    // TODO: not tested
    if (force_binary_format) {
      std::ifstream file_pointcloud(filename_pointcloud.c_str(), std::ios::in | std::ios::binary);
      if (!file_pointcloud.is_open()) {
        std::cerr << "Cannot read pointcloud file: " << filename_pointcloud << std::endl;
      }

      uint32_t height = 0, width = 0;
      const char is_dense = 1;
      vpIoTools::readBinaryValueLE(file_pointcloud, height);
      vpIoTools::readBinaryValueLE(file_pointcloud, width);
      file_pointcloud.read((char *)(&is_dense), sizeof(is_dense));

      point_cloud->width = width;
      point_cloud->height = height;
      point_cloud->is_dense = (is_dense != 0);
      point_cloud->resize((size_t)width * height);

      float x = 0.0f, y = 0.0f, z = 0.0f;
      for (uint32_t i = 0; i < height; i++) {
        for (uint32_t j = 0; j < width; j++) {
          vpIoTools::readBinaryValueLE(file_pointcloud, x);
          vpIoTools::readBinaryValueLE(file_pointcloud, y);
          vpIoTools::readBinaryValueLE(file_pointcloud, z);

          point_cloud->points[(size_t)(i * width + j)].x = x;
          point_cloud->points[(size_t)(i * width + j)].y = y;
          point_cloud->points[(size_t)(i * width + j)].z = z;
        }
      }
    }
#ifdef VISP_HAVE_MINIZ
    else if (read_npz) {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(filename_pointcloud);

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
      point_cloud->resize((size_t)width * height);

      for (uint32_t i = 0; i < height; i++) {
        for (uint32_t j = 0; j < width; j++) {
          point_cloud->points[(size_t)(i * width + j)].x = pcl_data_ptr[(size_t)(i * width + j)*3 + 0];
          point_cloud->points[(size_t)(i * width + j)].y = pcl_data_ptr[(size_t)(i * width + j)*3 + 1];
          point_cloud->points[(size_t)(i * width + j)].z = pcl_data_ptr[(size_t)(i * width + j)*3 + 2];
        }
      }
    }
#endif
    else {
#if defined(VISP_HAVE_PCL_IO)
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pointcloud, *point_cloud) == -1) {
        std::cerr << "Cannot read PCD: " << filename_pointcloud << std::endl;
      }
#else
      throw(vpIoException(vpIoException::ioError, "Cannot read pcd file without PCL io module"));
#endif
    }
#endif
  }

  return true;
}
} // Namespace

int main(int argc, const char *argv[])
{
  std::string input_directory = "";
  std::string pattern = "%04d";
  bool click = false;
  bool force_binary_format = false;
  bool save_video = false;
  bool color_depth = false;
  bool read_jpeg = false;
  bool read_npz = false;

  // Read the command line options
  if (!getOptions(argc, argv, input_directory, pattern, click, force_binary_format, read_jpeg, read_npz,
                  save_video, color_depth)) {
    return EXIT_FAILURE;
  }

  vpImage<vpRGBa> I_color(480, 640), I_depth_color(480, 640);
  vpImage<uint16_t> I_depth_raw(480, 640);
  vpImage<unsigned char> I_depth(480, 640);

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2;
#else
  vpDisplayGDI d1, d2;
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
  if (save_video) {
    std::string output_directory = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
    vpIoTools::makeDirectory(output_directory);
    writer.setFileName(output_directory + "/" + pattern + ".png");
  }

  int cpt_frame = 0;
  bool quit = false;
  while (!quit) {
    double t = vpTime::measureTimeMs();

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    {
      std::lock_guard<std::mutex> lock(mutex);
      quit = !readData(cpt_frame, input_directory, pattern, I_color, I_depth_raw, force_binary_format, read_jpeg,
        read_npz, pointcloud);
    }
#else
    quit = !readData(cpt_frame, input_directory, pattern, I_color, I_depth_raw, force_binary_format, read_jpeg,
      read_npz);
#endif

    if (color_depth)
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_color);
    else
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    if (!init_display) {
      init_display = true;
      d1.init(I_color, 0, 0, "Color image");
      if (color_depth) {
        d2.init(I_depth_color, I_color.getWidth() + 10, 0, "Depth image");
      }
      else {
        d2.init(I_depth, I_color.getWidth() + 10, 0, "Depth image");
      }
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION)
      pcl_viewer.setPosition(I_color.getWidth() + 10, I_color.getHeight() + 70);
      pcl_viewer.setWindowName("3D point cloud");
      pcl_viewer.startThread(std::ref(mutex), pointcloud);
#endif
    }

    vpDisplay::display(I_color);
    if (color_depth)
      vpDisplay::display(I_depth_color);
    else
      vpDisplay::display(I_depth);

    std::stringstream ss;
    ss << "Frame: " << cpt_frame;
    vpDisplay::displayText(I_color, 20, 20, ss.str(), vpColor::red);
    if (color_depth)
      vpDisplay::displayText(I_depth_color, 20, 20, ss.str(), vpColor::red);
    else
      vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);

    vpDisplay::flush(I_color);
    if (color_depth)
      vpDisplay::flush(I_depth_color);
    else
      vpDisplay::flush(I_depth);

    if (save_video) {
      if (O.getSize() == 0) {
        O.resize(I_color.getHeight(), I_color.getWidth() + I_depth_color.getWidth());
        writer.open(O);
      }

      O.insert(I_color, vpImagePoint());
      if (!color_depth)
        vpImageConvert::convert(I_depth, I_depth_color);
      O.insert(I_depth_color, vpImagePoint(0, I_color.getWidth()));
      writer.saveFrame(O);
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_color, button, click)) {
      switch (button) {
      case vpMouseButton::button1:
        if (!quit)
          quit = !click;
        break;

      case vpMouseButton::button3:
        click = !click;
        break;

      default:
        break;
      }
    }

    vpTime::wait(t, 30);
    cpt_frame++;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "Enable C++11 or higher (cmake -DUSE_CXX_STANDARD=11) and install X11 or GDI!" << std::endl;
  return EXIT_SUCCESS;
}
#endif
