/*
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
 * Description:
 * Compute chessboard poses to prepare hand-eye calibration.
 */

//! \example visp-compute-chessboard-poses.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_CALIB3D) && defined(VISP_HAVE_PUGIXML)

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpXmlParserCamera.h>
#if defined(VISP_HAVE_MODULE_GUI)
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#endif
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpPose.h>

#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void calcChessboardCorners(int width, int height, double squareSize, std::vector<vpPoint> &corners)
{
  corners.resize(0);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      vpPoint pt;
      pt.set_oX(j * squareSize);
      pt.set_oY(i * squareSize);
      pt.set_oZ(0.0);
      corners.push_back(pt);
    }
  }
}

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0] << " [-w <chessboard width>] [-h <chessboard height>]"
    << " [--square-size <square size in meter>]"
    << " [--input <input images path>]"
    << " [--intrinsic <Camera intrinsic parameters xml file>]"
    << " [--camera-name <Camera name in the xml intrinsic file>]"
    << " [--output <camera pose files>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  -w <chessboard width>  Chessboard width." << std::endl
    << "    Default: 9" << std::endl
    << std::endl
    << "  -h <chessboard height>  Chessboard height." << std::endl
    << "    Default: 6" << std::endl
    << std::endl
    << "  --square-size <square size in meter>  Chessboard square size in [m]." << std::endl
    << "    Default: 0.03" << std::endl
    << std::endl
    << "  --input <input images path>  Generic name of the images to process." << std::endl
    << "    Default: empty" << std::endl
    << "    Example: \"image-%d.png\"" << std::endl
    << std::endl
    << "  --intrinsic <Camera intrinsic parameters xml file>  XML file that contains" << std::endl
    << "    camera parameters. " << std::endl
    << "    Default: \"camera.xml\"" << std::endl
    << std::endl
    << "  --camera-name <Camera name in the xml intrinsic file>  Camera name in the XML file." << std::endl
    << "    Default: \"Camera\"" << std::endl
    << std::endl
    << "  --output <camera pose files>  Generic name of the yaml files that contains the camera poses."
    << std::endl
    << "    Default: \"pose_cPo_%d.yaml\"" << std::endl
    << std::endl
#if defined(VISP_HAVE_MODULE_GUI)
    << "  --no-interactive To compute the chessboard poses without interactive validation by the user." << std::endl
    << std::endl
#endif
    << "  --help, -h  Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}
} // namespace

int main(int argc, const char **argv)
{
  int opt_chessboard_width = 9, opt_chessboard_height = 6;
  double opt_chessboard_square_size = 0.03;
  std::string opt_input_img_files = "";
  std::string opt_intrinsic_file = "camera.xml";
  std::string opt_camera_name = "Camera";
  std::string opt_output_pose_files = "pose_cPo_%d.yaml";
  bool opt_interactive = true;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-w" && i + 1 < argc) {
      opt_chessboard_width = atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "-h" && i + 1 < argc) {
      opt_chessboard_height = atoi(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--square-size" && i + 1 < argc) {
      opt_chessboard_square_size = atof(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_input_img_files = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_file = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      opt_output_pose_files = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[i + 1]);
      i++;
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

  try {
    vpVideoReader reader;
    reader.setFileName(opt_input_img_files);

    vpImage<vpRGBa> I;
    reader.open(I);

    std::cout << "Parameters:" << std::endl;
    std::cout << "  Chessboard width             : " << opt_chessboard_width << std::endl;
    std::cout << "  Chessboard height            : " << opt_chessboard_height << std::endl;
    std::cout << "  Chessboard square size [m]   : " << opt_chessboard_square_size << std::endl;
    std::cout << "  Input images location        : " << opt_input_img_files << std::endl;
    std::cout << "    First frame                : " << reader.getFirstFrameIndex() << std::endl;
    std::cout << "    Last  frame                : " << reader.getLastFrameIndex() << std::endl;
    std::cout << "  Camera param file name [.xml]: " << opt_intrinsic_file << std::endl;
    std::cout << "  Camera name                  : " << opt_camera_name << std::endl;
    std::cout << "  Output camera poses          : " << opt_output_pose_files << std::endl;
    std::cout << "  Interactive mode             : " << (opt_interactive ? "yes" : "no") << std::endl << std::endl;


#if defined(VISP_HAVE_MODULE_GUI)
    vpDisplay *display = nullptr;
    if (opt_interactive) {
#if defined(VISP_HAVE_X11)
      display = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      display = new vpDisplayGDI(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
      display = new vpDisplayOpenCV(I);
#elif defined(VISP_HAVE_D3D9)
      display = new vpDisplayD3D(I);
#elif defined(VISP_HAVE_GTK)
      display = new vpDisplayGTK(I);
#endif
  }
#endif

    std::vector<vpPoint> corners_pts;
    calcChessboardCorners(opt_chessboard_width, opt_chessboard_height, opt_chessboard_square_size, corners_pts);

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

      cv::Mat matImg;
      vpImageConvert::convert(I, matImg);

      cv::Size chessboardSize(opt_chessboard_width, opt_chessboard_height);
      std::vector<cv::Point2f> corners2D;
      bool found = cv::findChessboardCorners(matImg, chessboardSize, corners2D,
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
        cv::CALIB_CB_NORMALIZE_IMAGE);
#else
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
        CV_CALIB_CB_NORMALIZE_IMAGE);
#endif

      vpHomogeneousMatrix cMo;
      if (found) {
        cv::Mat matImg_gray;
        cv::cvtColor(matImg, matImg_gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(matImg_gray, corners2D, cv::Size(11, 11), cv::Size(-1, -1),
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
#else
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
#endif

        for (size_t i = 0; i < corners_pts.size(); i++) {
          vpImagePoint imPt(corners2D[i].y, corners2D[i].x);
          double x = 0.0, y = 0.0;
          vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
          corners_pts[i].set_x(x);
          corners_pts[i].set_y(y);
        }

        vpPose pose;
        pose.addPoints(corners_pts);
        if (!pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo)) {
          std::cerr << "Problem when computing final pose using VVS" << std::endl;
          return EXIT_FAILURE;
        }

        cv::drawChessboardCorners(matImg, chessboardSize, corners2D, found);
        vpImageConvert::convert(matImg, I);
      }

      if (found) {
        vpPoseVector pose_vec(cMo);
        char name[FILENAME_MAX];
        snprintf(name, FILENAME_MAX, opt_output_pose_files.c_str(), reader.getFrameIndex());
        std::string s = name;
        std::cout << "Save " << s << std::endl;
        pose_vec.saveYAML(s, pose_vec);
      }

#if defined(VISP_HAVE_MODULE_GUI)
      if (opt_interactive) {
        vpDisplay::setTitle(I, reader.getFrameName());
        vpDisplay::displayText(I, 20, 20, "Right click to quit.", vpColor::red);
        vpDisplay::display(I);
        vpDisplay::displayText(I, 20, 20, "Left click for the next image, right click to quit.", vpColor::red);
        if (found) {
          vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
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
      if (display) {
        delete display;
      }
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
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_CALIB3D))
  std::cerr << "OpenCV calib3d module is requested to run the calibration." << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested to run the calibration." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
