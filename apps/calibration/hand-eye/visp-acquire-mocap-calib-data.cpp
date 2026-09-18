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
 *
 * Description:
 * Mocap using Qualisys or Vicon system data acquisition to prepare hand-eye calibration.
 */

//! \example visp-acquire-mocap-calib-data.cpp

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpMocapVicon.h>
#include <visp3/sensor/vpMocapQualisys.h>

#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_REALSENSE2) && \
    defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML) && \
    defined(VISP_HAVE_MODULE_GUI) && defined(VISP_HAVE_MODULE_SENSOR)

namespace
{
// Shared state between the mocap polling thread and the main thread
std::mutex g_mocap_mutex;
vpHomogeneousMatrix g_mocap_pose;
bool g_mocap_pose_valid = false;
std::atomic<bool> g_mocap_stop_thread(false);

void mocapThreadFunction(vpMocap *mocap, const std::string &object_name)
{
  const std::chrono::milliseconds period(5); // 200 Hz

  while (!g_mocap_stop_thread) {
    auto start_time = std::chrono::steady_clock::now();

    vpHomogeneousMatrix world_M_body;
    bool success = mocap->getSpecificBodyPose(object_name, world_M_body);

    if (success) {
      std::lock_guard<std::mutex> lock(g_mocap_mutex);
      g_mocap_pose = world_M_body;
      g_mocap_pose_valid = true;
    }

    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time < period) {
      std::this_thread::sleep_for(period - elapsed_time);
    }
  }
}
} // namespace

int main(int argc, char **argv)
{
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  vpDisplay *pdisp = nullptr;
#endif

  try {
    bool opt_without_mocap = false;
    bool opt_with_vicon = false;
    bool opt_with_qualisys = false;
    std::string opt_vicon_host = "10.135.2.39";
    std::string opt_qualisys_host = "10.135.2.40";
    std::string opt_body_name = "mkQuadro5";
    std::string opt_output_folder = "./data/";
    bool opt_qualisys_big_endian = false;
    int opt_img_width = 640;
    int opt_img_height = 480;

    for (int i = 1; i < argc; ++i) {
      if ((std::string(argv[i]) == "--img-width") && (i + 1 < argc)) {
        opt_img_width = std::atoi(argv[++i]);
      }
      else if ((std::string(argv[i]) == "--img-height") && (i + 1 < argc)) {
        opt_img_height = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--without-mocap") {
        opt_without_mocap = true;
      }
      else if ((std::string(argv[i]) == "--vicon-ip") && (i + 1 < argc)) {
        opt_vicon_host = std::string(argv[++i]);
        opt_with_vicon = true;
      }
      else if ((std::string(argv[i]) == "--qualisys-ip") && (i + 1 < argc)) {
        opt_qualisys_host = std::string(argv[++i]);
        opt_with_qualisys = true;
      }
      else if (std::string(argv[i]) == "--qualisys-big-endian") {
        opt_qualisys_big_endian = true;
      }
      else if ((std::string(argv[i]) == "--body-name") && (i + 1 < argc)) {
        opt_body_name = std::string(argv[++i]);
      }
      else if ((std::string(argv[i]) == "--output-folder") && (i + 1 < argc)) {
        opt_output_folder = std::string(argv[++i]);
      }
      else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
        std::cout << "\nUsage: " << std::endl
          << argv[0]
          << " [--img-width <width>]"
          << " [--img-height <height>]"
          << " [--without-mocap]"
          << " [--vicon-ip <address:port>]"
          << " [--qualisys-ip <address:port>]"
          << " [--body-name <name>]"
          << " [--help] [-h]\n"
          << std::endl;
        std::cout << "Options: " << std::endl
          << "--img-width <width>" << std::endl
          << "\tImage width. Default: " << opt_img_width << std::endl
          << std::endl
          << "--img-height <height>" << std::endl
          << "\tImage height. Default: " << opt_img_height << std::endl
          << std::endl
          << "--without-mocap" << std::endl
          << "\tDoesn't connect to Vicon or Qualisys to get object pose." << std::endl
          << std::endl
          << "--vicon-ip <address:port>" << std::endl
          << "\tVicon router IP address and port like: " << opt_vicon_host << std::endl
          << std::endl
          << "--qualisys-ip <address:port>" << std::endl
          << "\tQualisys router IP address and port like: " << opt_qualisys_host << std::endl
          << std::endl
          << "--body-name <name>" << std::endl
          << "\tName of the tracked object by the mocap system. Default: " << opt_qualisys_host << std::endl
          << std::endl
          << "--output-folder <name>" << std::endl
          << "\tName of the folder that will contain acquired data. Default: " << opt_output_folder << std::endl
          << std::endl
          << "--help, -h" << std::endl
          << "\tPrint this helper message" << std::endl
          << std::endl;
        return EXIT_SUCCESS;
      }
      else {
        std::cout << "Error: unknown command line option" << std::endl;
        std::cout << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
    }

    if (opt_with_vicon && opt_with_qualisys) {
      std::cout << "Cannot dial simultaneously with Vicon and Qualisys mocap systems" << std::endl;
      return EXIT_FAILURE;
    }
#if !defined(VISP_HAVE_VICON)
    if (opt_with_vicon) {
      std::cout << "Cannot dial with Vicon MoCap. Build ViSP with Vicon 3rd party support" << std::endl;
      return EXIT_FAILURE;
    }
#endif
#if !defined(VISP_HAVE_QUALISYS)
    if (opt_with_qualisys) {
      std::cout << "Cannot dial with Qualisys MoCap. Build ViSP with Qualisys 3rd party support" << std::endl;
      return EXIT_FAILURE;
    }
#endif

    // Create output folder if necessary
    if (!vpIoTools::checkDirectory(opt_output_folder)) {
      std::cout << "Create output directory: " << opt_output_folder << std::endl;
      vpIoTools::makeDirectory(opt_output_folder);
    }

    std::cout << "Grab data from: " << std::endl;
    std::cout << "- Realsense camera" << std::endl;
    std::cout << "- Image size      : " << opt_img_width << " x " << opt_img_height << std::endl;
    if (opt_with_vicon) {
      std::cout << "-  Mocap system   : Vicon" << std::endl;
      std::cout << "-  Considered body: " << opt_body_name << std::endl;
    }
    else if (opt_with_qualisys) {
      std::cout << "-  Mocap system   : Qualisys" << std::endl;
      std::cout << "-  Considered body: " << opt_body_name << std::endl;
    }

    vpImage<unsigned char> I;
    vpMocap *mocap = nullptr;
    std::thread mocap_thread;

    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, opt_img_width, opt_img_height, RS2_FORMAT_RGBA8, 30);
    g.open(config);
    g.acquire(I);

    unsigned int width = I.getWidth();
    unsigned int height = I.getHeight();

    std::cout << "Acquired image size: " << width << " x " << height << std::endl;

    // Save intrinsics
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam, vpIoTools::createFilePath(opt_output_folder, "/camera.xml"), "Camera", width, height);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> pdisp = vpDisplayFactory::createDisplay(I, 10, 10, "Color image");
#else
    pdisp = vpDisplayFactory::allocateDisplay(I, 10, 10, "Color image");
#endif

    bool end = false;
    unsigned cpt = 0;

    if (!opt_without_mocap) {
      if (opt_with_vicon) {
#if defined(VISP_HAVE_VICON)
        mocap = new vpMocapVicon;
        mocap->setServerAddress(opt_vicon_host);
#endif
      }
      else if (opt_with_qualisys) {
#if defined(VISP_HAVE_QUALISYS)
        mocap = new vpMocapQualisys;
        mocap->setServerAddress(opt_qualisys_host);
        reinterpret_cast<vpMocapQualisys *>(mocap)->setBigEndian(opt_qualisys_big_endian);
#endif
      }
      else {
        std::cout << "Unable to establish connection with Mocap" << std::endl;
        return EXIT_FAILURE;
      }

      if (opt_with_vicon || opt_with_qualisys) {
#if defined(VISP_HAVE_VICON) || defined(VISP_HAVE_QUALISYS)
        mocap->setVerbose(true);
        mocap->connect();
        std::cout << "Mocap connection established" << std::endl;

        // Start the polling thread at 200 Hz
        g_mocap_stop_thread = false;
        mocap_thread = std::thread(mocapThreadFunction, mocap, opt_body_name);
#endif

      }
    }

    while (!end) {
      vpHomogeneousMatrix fMe;
      bool fMe_valid = false;
      g.acquire(I);

      if (!opt_without_mocap) {
        if (opt_with_vicon || opt_with_qualisys) {
#if defined(VISP_HAVE_VICON)|| defined(VISP_HAVE_QUALISYS)
          std::lock_guard<std::mutex> lock(g_mocap_mutex);
          fMe = g_mocap_pose;
          fMe_valid = g_mocap_pose_valid;
#endif
        }
      }

      vpDisplay::display(I);
      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt++;

          std::stringstream ss_pose;
          // Save pose from Vicon or Qualisys
          if (!opt_without_mocap) {
            if (opt_with_vicon || opt_with_qualisys) {
              if (fMe_valid) {
                ss_pose << vpIoTools::createFilePath(opt_output_folder, "/mocap_pose_world_P_body") << cpt << ".yaml";
                vpPoseVector::saveYAML(ss_pose.str(), vpPoseVector(fMe));
              }
              else {
                std::cout << "No valid pose available yet, skipping pose save" << std::endl;
              }
            }
            else {
              std::cout << "Unable to get object pose from Vicon/Qualisys" << std::endl;
              end = true;
            }
          }
          // Save images
          std::stringstream ss_img;
          ss_img << vpIoTools::createFilePath(opt_output_folder, "/chessboard_image-") << cpt << ".png";
          if (opt_with_vicon || opt_with_qualisys) {
            std::cout << "Save: " << ss_img.str() << " and " << ss_pose.str() << std::endl;
          }
          else {
            std::cout << "Save: " << ss_img.str() << std::endl;
          }
          vpImageIo::write(I, ss_img.str());
        }
        else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);
    }

#if defined(VISP_HAVE_VICON) || defined(VISP_HAVE_QUALISYS)
    if (mocap_thread.joinable()) {
      g_mocap_stop_thread = true;
      mocap_thread.join();
    }
#endif

    std::cout << "Data is saved in : " << opt_output_folder << std::endl;
  }
  catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (pdisp != nullptr) {
    delete pdisp;
  }
#endif
  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_MODULE_GUI)
  std::cout << "visp_gui module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_MODULE_SENSOR)
  std::cout << "visp_sensor module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x." << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "Enable pugyxml built-in usage." << std::endl;
#endif
  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake "
    << "and build ViSP again." << std::endl;
  return EXIT_SUCCESS;
}
#endif
