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
 * Compute apriltag poses to prepare hand-eye calibration.
 */

//! \example visp-compute-apriltag-poses.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML) and defined(VISP_HAVE_APRILTAG)

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoReader.h>

#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--tag-size <size>]"
    << " [--tag-z-aligned]"
    << " [--input <images filename>]"
    << " [--intrinsic <xml file>]"
    << " [--camera-name <name>]"
    << " [--output <poses filename>]"
#if defined(VISP_HAVE_MODULE_GUI)
    << " [--no-interactive]"
#endif
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute the pose of a 36h11 Apriltag in a sequence of images." << std::endl
    << "  Consider that only one tag is present in each image." << std::endl
    << std::endl
    << "  --tag-size <size>" << std::endl
    << "    Apriltag size in [m]." << std::endl
    << "    Default: 0.03" << std::endl
    << std::endl
    << "  --tag-z-aligned" << std::endl
    << "    When enabled, tag z-axis and camera z-axis are aligned." << std::endl
    << "    Default: false" << std::endl
    << std::endl
    << "  --input <images filename>" << std::endl
    << "    Generic name of the images to process." << std::endl
    << "    Default: empty" << std::endl
    << "    Example: \"image-%d.jpg\"" << std::endl
    << std::endl
    << "  --intrinsic <xml file>" << std::endl
    << "    XML file that contains camera intrinsic parameters. " << std::endl
    << "    Default: \"camera.xml\"" << std::endl
    << std::endl
    << "  --camera-name <name>" << std::endl
    << "    Camera name in the XML file that contains camera intrinsic parameters." << std::endl
    << "    Default: \"Camera\"" << std::endl
    << std::endl
    << "  --output <poses filename>" << std::endl
    << "    Generic name of the yaml files that contains the resulting tag poses." << std::endl
    << "    Default: \"pose_cPo_%d.yaml\"" << std::endl
    << std::endl
#if defined(VISP_HAVE_MODULE_GUI)
    << "  --no-interactive" << std::endl
    << "    To compute the tag poses without interactive validation by the user." << std::endl
    << std::endl
#endif
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;

  std::cout << "Example" << std::endl
    << "  "
    << argv[0]
    << " --input image-%d.jpg" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
{
  double opt_tag_size = 0.048;
  std::string opt_input_img_files = "";
  std::string opt_intrinsic_file = "camera.xml";
  std::string opt_camera_name = "Camera";
  std::string opt_output_pose_files = "pose_cPo_%d.yaml";
  bool opt_interactive = true;

  vpDetectorAprilTag::vpAprilTagFamily tag_family = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

  float quad_decimate = 1.0;
  vpDetectorAprilTag detector(tag_family);
  bool display_tag = true;
  bool opt_tag_z_aligned = false;

  unsigned int thickness = 2;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-z-aligned") {
      opt_tag_z_aligned = true;
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_input_img_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      opt_output_pose_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[++i]);
    }
#if defined(VISP_HAVE_MODULE_GUI)
    else if (std::string(argv[i]) == "--no-interactive") {
      opt_interactive = false;
    }
#endif
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  if (!vpIoTools::checkFilename(opt_intrinsic_file)) {
    std::cout << "Camera parameters file " << opt_intrinsic_file << " doesn't exist." << std::endl;
    std::cout << "Use --help option to see how to set its location..." << std::endl;
    return EXIT_FAILURE;
  }

  if (opt_input_img_files.empty()) {
    std::cout << "Input images location empty." << std::endl;
    std::cout << "Use --help option to see how to set input image location..." << std::endl;
    return EXIT_FAILURE;
  }

  // Create output folder if necessary
  std::string output_parent = vpIoTools::getParent(opt_output_pose_files);
  if (!vpIoTools::checkDirectory(output_parent)) {
    std::cout << "Create output directory: " << output_parent << std::endl;
    vpIoTools::makeDirectory(output_parent);
  }

  try {
    vpVideoReader reader;
    reader.setFileName(opt_input_img_files);

    vpImage<vpRGBa> I;
    vpImage<unsigned char> I_gray;
    reader.open(I);

    std::cout << "Parameters:" << std::endl;
    std::cout << "  Apriltag                  " << std::endl;
    std::cout << "    Size [m]              : " << opt_tag_size << std::endl;
    std::cout << "    Z aligned             : " << (opt_tag_z_aligned ? "true" : "false") << std::endl;
    std::cout << "  Input images location   : " << opt_input_img_files << std::endl;
    std::cout << "    First frame           : " << reader.getFirstFrameIndex() << std::endl;
    std::cout << "    Last  frame           : " << reader.getLastFrameIndex() << std::endl;
    std::cout << "  Camera intrinsics         " << std::endl;
    std::cout << "    Param file name [.xml]: " << opt_intrinsic_file << std::endl;
    std::cout << "    Camera name           : " << opt_camera_name << std::endl;
    std::cout << "  Output camera poses     : " << opt_output_pose_files << std::endl;
    std::cout << "  Interactive mode        : " << (opt_interactive ? "yes" : "no") << std::endl << std::endl;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> display;
#else
    vpDisplay *display = nullptr;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif

    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(pose_estimation_method);
    detector.setDisplayTag(display_tag, vpColor::none, thickness);
    detector.setZAlignedWithCameraAxis(opt_tag_z_aligned);

    vpCameraParameters cam;
    vpXmlParserCamera parser;
    if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
      if (parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) !=
        vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to parse parameters with distortion for camera \"" << opt_camera_name << "\" from "
          << opt_intrinsic_file << " file" << std::endl;
        std::cout << "Attempt to find parameters without distortion" << std::endl;

        if (parser.parse(cam, opt_intrinsic_file, opt_camera_name,
                         vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
          std::cout << "Unable to parse parameters without distortion for camera \"" << opt_camera_name << "\" from "
            << opt_intrinsic_file << " file" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
    std::cout << "Camera parameters used to compute the pose:\n" << cam << std::endl;

    bool quit = false;
    do {
      reader.acquire(I);
      std::cout << "Process image: " << reader.getFrameName() << std::endl;

      vpImageConvert::convert(I, I_gray);

#if defined(VISP_HAVE_MODULE_GUI)
      vpDisplay::display(I);
#endif

      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I_gray, opt_tag_size, cam, cMo_vec);

      bool found = (detector.getNbObjects() == 1) ? true : false;
      vpHomogeneousMatrix cMo;
      if (found) {
        cMo = cMo_vec[0];
        vpPoseVector pose_vec(cMo);
        std::string  filename = vpIoTools::formatString(opt_output_pose_files, reader.getFrameIndex());
        pose_vec.saveYAML(filename, pose_vec);
      }

#if defined(VISP_HAVE_MODULE_GUI)
      if (opt_interactive) {
        vpDisplay::setTitle(I, reader.getFrameName());
        vpDisplay::displayText(I, 20, 20, "Left click for the next image, right click to quit.", vpColor::red);
        if (found) {
          vpDisplay::displayFrame(I, cMo, cam, opt_tag_size / 2, vpColor::none, 3);
        }
        vpDisplay::flush(I);

        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(I, button, true)) {
          if (button == vpMouseButton::button3) {
            quit = true;
          }
        }
      }
#endif
    } while (!quit && !reader.end());

#if defined(VISP_HAVE_MODULE_GUI)
    if (opt_interactive) {
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
      if (display) {
        delete display;
      }
#endif
    }
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested to compute tag poses." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
