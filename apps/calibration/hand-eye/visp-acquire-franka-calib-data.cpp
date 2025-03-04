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
 * Franka-Emika Panda robot data acquisition to prepare hand-eye calibration.
 */

//! \example visp-acquire-franka-calib-data.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && \
    defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_FRANKA) && defined(VISP_HAVE_PUGIXML) && \
    defined(VISP_HAVE_MODULE_GUI) && defined(VISP_HAVE_MODULE_ROBOT) && defined(VISP_HAVE_MODULE_SENSOR) // optional

void usage(const char **argv, int error, const std::string &robot_ip)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--ip <address>] [--output-folder <name>] [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  --ip <address>" << std::endl
    << "    Ethernet address to dial with the Panda robot." << std::endl
    << "    Default: " << robot_ip << std::endl
    << std::endl
    << "  --output-folder <name>" << std::endl
    << "    Acquired data output folder." << std::endl
    << "    Default: ./" << std::endl
    << std::endl
    << "  --help, -h  Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
{
#if defined(ENABLE_VISP_NAMESPACE)
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  vpDisplay *pdisp = nullptr;
#endif
  try {
    std::string opt_robot_ip = "192.168.1.1";
    std::string opt_output_folder = "./";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        opt_robot_ip = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--output-folder" && i + 1 < argc) {
        opt_output_folder = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0, opt_robot_ip);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i, opt_robot_ip);
        return EXIT_FAILURE;
      }
    }

    // Create output folder if necessary
    if (!vpIoTools::checkDirectory(opt_output_folder)) {
      std::cout << "Create output directory: " << opt_output_folder << std::endl;
      vpIoTools::makeDirectory(opt_output_folder);
    }

    vpImage<unsigned char> I;

    vpRobotFranka robot;
    robot.connect(opt_robot_ip);

    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);
    g.acquire(I);

    unsigned int width = I.getWidth();
    unsigned int height = I.getHeight();

    std::cout << "Image size: " << width << " x " << height << std::endl;
    // Save intrinsics
    vpCameraParameters cam;


    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam, opt_output_folder + "/franka_camera.xml", "Camera", width, height);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> pdisp = vpDisplayFactory::createDisplay(I, 10, 10, "Color image");
#else
    pdisp = vpDisplayFactory::allocateDisplay(I, 10, 10, "Color image");
#endif

    bool end = false;
    unsigned cpt = 0;
    while (!end) {
      g.acquire(I);

      vpDisplay::display(I);

      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt++;

          vpPoseVector rPe;
          robot.getPosition(vpRobot::END_EFFECTOR_FRAME, rPe);

          std::stringstream ss_img, ss_pos;

          ss_img << opt_output_folder + "/franka_image-" << cpt << ".png";
          ss_pos << opt_output_folder + "/franka_pose_rPe_" << cpt << ".yaml";
          std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
          vpImageIo::write(I, ss_img.str());
          rPe.saveYAML(ss_pos.str(), rPe);
        }
        else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cerr << "ViSP exception " << e.what() << std::endl;
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
#if !defined(VISP_HAVE_MODULE_ROBOT)
  std::cout << "visp_robot module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_MODULE_SENSOR)
  std::cout << "visp_sensor module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x." << std::endl;
#endif
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka." << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "Enable pugyxml built-in usage." << std::endl;
#endif

  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake "
    << "and build ViSP again." << std::endl;
  return EXIT_SUCCESS;
}
#endif
