/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Description:
 * Regression test for depth MBT.
 *
 *****************************************************************************/

/*!
  \example testGenericTrackerDepth.cpp

  \brief Regression test for depth MBT.
*/

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_MBT) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_XML2)

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#define GETOPTARGS "i:dcle:h"

namespace
{
  void usage(const char *name, const char *badparam)
  {
    fprintf(stdout, "\n\
    Regression test for vpGenericTracker and depth.\n\
    \n\
    SYNOPSIS\n\
      %s [-i <test image path>] [-c] [-d] [-h] [-l] \n\
     [-e <last frame index>]\n", name);

    fprintf(stdout, "\n\
    OPTIONS:                                               \n\
      -i <input image path>                                \n\
         Set image input path.\n\
         These images come from ViSP-images-x.y.z.tar.gz available \n\
         on the ViSP website.\n\
         Setting the VISP_INPUT_IMAGE_PATH environment\n\
         variable produces the same behavior than using\n\
         this option.\n\
    \n\
      -d \n\
         Turn off the display.\n\
    \n\
      -c\n\
         Disable the mouse click. Useful to automate the \n\
         execution of this program without human intervention.\n\
    \n\
      -l\n\
         Use the scanline for visibility tests.\n\
    \n\
      -e <last frame index>\n\
         Specify the index of the last frame. Once reached, the tracking is stopped.\n\
    \n\
      -h \n\
         Print the help.\n\n");

    if (badparam)
      fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
  }

  bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display,
                  bool &useScanline, int &lastFrame)
  {
    const char *optarg_;
    int c;
    while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

      switch (c) {
      case 'i':
        ipath = optarg_;
        break;
      case 'c':
        click_allowed = false;
        break;
      case 'd':
        display = false;
        break;
      case 'l':
        useScanline = true;
        break;
      case 'e':
        lastFrame = atoi(optarg_);
        break;
      case 'h':
        usage(argv[0], NULL);
        return false;
        break;

      default:
        usage(argv[0], optarg_);
        return false;
        break;
      }
    }

    if ((c == 1) || (c == -1)) {
      // standalone param or error
      usage(argv[0], NULL);
      std::cerr << "ERROR: " << std::endl;
      std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
      return false;
    }

    return true;
  }

  bool read_data(const std::string &input_directory, const int cpt, const vpCameraParameters &cam_depth,
                 vpImage<unsigned char> &I, vpImage<uint16_t> &I_depth,
                 std::vector<vpColVector> &pointcloud, vpHomogeneousMatrix &cMo)
  {
    char buffer[256];
    sprintf(buffer, std::string(input_directory + "/Images/Image_%04d.pgm").c_str(), cpt);
    std::string image_filename = buffer;

    sprintf(buffer, std::string(input_directory + "/Depth/Depth_%04d.bin").c_str(), cpt);
    std::string depth_filename = buffer;

    sprintf(buffer, std::string(input_directory + "/CameraPose/Camera_%03d.txt").c_str(), cpt);
    std::string pose_filename = buffer;

    if (!vpIoTools::checkFilename(image_filename) || !vpIoTools::checkFilename(depth_filename)
        || !vpIoTools::checkFilename(pose_filename))
      return false;

    vpImageIo::read(I, image_filename);

    unsigned int depth_width = 0, depth_height = 0;
    std::ifstream file_depth(depth_filename, std::ios::in | std::ios::binary);
    if (!file_depth.is_open())
      return false;

    vpIoTools::readBinaryValueLE(file_depth, depth_height);
    vpIoTools::readBinaryValueLE(file_depth, depth_width);
    I_depth.resize(depth_height, depth_width);
    pointcloud.resize(depth_height*depth_width);

    const float depth_scale = 0.000030518f;
    for (unsigned int i = 0; i < I_depth.getHeight(); i++) {
      for (unsigned int j = 0; j < I_depth.getWidth(); j++) {
        vpIoTools::readBinaryValueLE(file_depth, I_depth[i][j]);
        double x = 0.0, y = 0.0, Z = I_depth[i][j] * depth_scale;
        vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
        vpColVector pt3d(4, 1.0);
        pt3d[0] = x*Z;
        pt3d[1] = y*Z;
        pt3d[2] = Z;
        pointcloud[i*I_depth.getWidth()+j] = pt3d;
      }
    }

    std::ifstream file_pose(pose_filename);
    if (!file_pose.is_open()) {
      return false;
    }

    for (unsigned int i = 0; i < 4; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        file_pose >> cMo[i][j];
      }
    }

    return true;
  }
}

int main(int argc, const char *argv[])
{
  try {
    std::string env_ipath = "";
    std::string opt_ipath = "";
    bool opt_click_allowed = true;
    bool opt_display = true;
    bool useScanline = false;
#if defined(__mips__) || defined(__mips) || defined(mips) || defined(__MIPS__)
    // To avoid Debian test timeout
    int opt_lastFrame = 5;
#else
    int opt_lastFrame = -1;
#endif

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display,
                    useScanline, opt_lastFrame)) {
      return EXIT_FAILURE;
    }

    std::cout << "useScanline: " << useScanline << std::endl;

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;

      return EXIT_FAILURE;
    }

    std::string input_directory = vpIoTools::createFilePath(!opt_ipath.empty() ? opt_ipath : env_ipath, "mbt-depth/Castle-simu");
    if (!vpIoTools::checkDirectory(input_directory)) {
      std::cerr << "ViSP-images does not contain the folder: " << input_directory << "!" << std::endl;
      return EXIT_SUCCESS;
    }

    // Initialise a  display
#if defined VISP_HAVE_X11
    vpDisplayX display1, display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display1, display2;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display1, display2;
#elif defined VISP_HAVE_D3D9
    vpDisplayD3D display1, display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display1, display2;
#else
    opt_display = false;
#endif

    std::vector<int> tracker_type = {vpMbGenericTracker::DEPTH_DENSE_TRACKER};
    vpMbGenericTracker tracker(tracker_type);
    tracker.loadConfigFile(input_directory + "/Config/chateau_depth.xml");
    tracker.loadModel(input_directory + "/Models/chateau.cao");
    vpHomogeneousMatrix T;
    T[0][0] = -1;
    T[0][3] = -0.2;
    T[1][1] = 0;
    T[1][2] = 1;
    T[1][3] = 0.12;
    T[2][1] = 1;
    T[2][2] = 0;
    T[2][3] = -0.15;
    tracker.loadModel(input_directory + "/Models/cube.cao", false, T);
    vpCameraParameters cam_depth;
    tracker.getCameraParameters(cam_depth);
    tracker.setDisplayFeatures(true);
    tracker.setScanLineVisibilityTest(useScanline);

    vpImage<unsigned char> I;
    vpImage<uint16_t> I_depth_raw;
    vpImage<vpRGBa> I_depth;
    vpHomogeneousMatrix cMo_truth;
    std::vector<vpColVector> pointcloud;
    int cpt_frame = 1;
    if (!read_data(input_directory, cpt_frame, cam_depth, I, I_depth_raw, pointcloud, cMo_truth)) {
      std::cerr << "Cannot read first frame!" << std::endl;
      return EXIT_FAILURE;
    }

    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
    if (opt_display) {
      display1.init(I, 0, 0, "Image");
      display2.init(I_depth, I.getWidth(), 0, "Depth");
    }

    vpHomogeneousMatrix depth_M_color;
    depth_M_color[0][3] = -0.05;
    tracker.initFromPose(I, depth_M_color*cMo_truth);

    bool click = false, quit = false;
    while (read_data(input_directory, cpt_frame, cam_depth, I, I_depth_raw, pointcloud, cMo_truth) && !quit
           && (opt_lastFrame > 0 ? (int)cpt_frame <= opt_lastFrame : true)) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      if (opt_display) {
        vpDisplay::display(I);
        vpDisplay::display(I_depth);
      }

      std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
      std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
      mapOfPointclouds["Camera"] = &pointcloud;
      std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
      mapOfWidths["Camera"] = I_depth.getWidth();
      mapOfHeights["Camera"] = I_depth.getHeight();
      tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);

      vpHomogeneousMatrix cMo = tracker.getPose();
      if (opt_display) {
        tracker.display(I_depth, cMo, cam_depth, vpColor::red, 3);
        vpDisplay::displayFrame(I_depth, cMo, cam_depth, 0.05, vpColor::none, 3);

        std::stringstream ss;
        ss << "Frame: " << cpt_frame;
        vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
        ss.str("");
        ss << "Nb features: " << tracker.getError().getRows();
        vpDisplay::displayText(I_depth, 40, 20, ss.str(), vpColor::red);
      }

      vpPoseVector pose_est(cMo);
      vpPoseVector pose_truth(depth_M_color*cMo_truth);
      vpColVector t_est(3), t_truth(3);
      vpColVector tu_est(3), tu_truth(3);
      for (int i = 0; i < 3; i++) {
        t_est[i] = pose_est[i];
        t_truth[i] = pose_truth[i];
        tu_est[i] = pose_est[i+3];
        tu_truth[i] = pose_truth[i+3];
      }

      vpColVector t_err = t_truth-t_est, tu_err = tu_truth-tu_est;
      const double t_thresh = 0.003, tu_thresh = 0.5;
      if ( sqrt(t_err.sumSquare()) > t_thresh || vpMath::deg(sqrt(tu_err.sumSquare())) > tu_thresh ) {
        std::cerr << "Pose estimated exceeds the threshold (t_thresh = 0.003, tu_thresh = 0.5)!" << std::endl;
        std::cout << "t_err: " << sqrt(t_err.sumSquare()) << " ; tu_err: " << vpMath::deg(sqrt(tu_err.sumSquare())) << std::endl;
        return EXIT_FAILURE;
      }

      if (opt_display) {
        vpDisplay::flush(I);
        vpDisplay::flush(I_depth);
      }

      if (opt_click_allowed) {
        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(I, button, click)) {
          switch (button) {
            case vpMouseButton::button1:
              quit = !click;
              break;

            case vpMouseButton::button3:
              click = !click;
              break;

            default:
              break;
          }
        }
      }

      cpt_frame++;
    }

  // Cleanup memory allocated by xml library used to parse the xml config
  // file in vpMbGenericTracker::loadConfigFile()
  vpXmlParser::cleanup();

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main() {
  std::cout << "Missing VISP_HAVE_MODULE_MBT or VISP_HAVE_DISPLAY or VISP_HAVE_XML2" << std::endl;
  return 0;
}
#endif
