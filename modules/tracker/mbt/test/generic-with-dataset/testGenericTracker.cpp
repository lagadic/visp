/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Regression test for MBT.
 */

/*!
  \example testGenericTracker.cpp

  \brief Regression test for MBT.
*/

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_MBT) &&                                                                                   \
    (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <type_traits>
#endif

#include <visp3/core/vpFont.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#define GETOPTARGS "i:dsclt:e:DmCh"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
    Regression test for vpGenericTracker.\n\
    \n\
    SYNOPSIS\n\
      %s [-i <test image path>] [-c] [-d] [-s] [-h] [-l] \n\
     [-t <tracker type>] [-e <last frame index>] [-D] [-m] [-C]\n",
    name);

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
      -s \n\
         If display is turn off, tracking results are saved in a video folder.\n\
    \n\
      -c\n\
         Disable the mouse click. Useful to automate the \n\
         execution of this program without human intervention.\n\
    \n\
      -t <tracker type>\n\
         Set tracker type (<1 (Edge)>, <2 (KLT)>, <3 (both)>) for color sensor.\n\
    \n\
      -l\n\
         Use the scanline for visibility tests.\n\
    \n\
      -e <last frame index>\n\
         Specify the index of the last frame. Once reached, the tracking is stopped.\n\
    \n\
      -D \n\
         Use depth.\n\
    \n\
      -m \n\
         Set a tracking mask.\n\
    \n\
      -C \n\
         Use color images.\n\
    \n\
      -h \n\
         Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display, bool &save,
  bool &useScanline, int &trackerType, int &lastFrame, bool &use_depth, bool &use_mask,
  bool &use_color_image)
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
    case 's':
      save = true;
      break;
    case 'l':
      useScanline = true;
      break;
    case 't':
      trackerType = atoi(optarg_);
      break;
    case 'e':
      lastFrame = atoi(optarg_);
      break;
    case 'D':
      use_depth = true;
      break;
    case 'm':
      use_mask = true;
      break;
    case 'C':
      use_color_image = true;
      break;
    case 'h':
      usage(argv[0], nullptr);
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
    usage(argv[0], nullptr);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

template <typename Type>
bool read_data(const std::string &input_directory, int cpt, const vpCameraParameters &cam_depth, vpImage<Type> &I,
  vpImage<uint16_t> &I_depth, std::vector<vpColVector> &pointcloud, vpHomogeneousMatrix &cMo)
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  static_assert(std::is_same<Type, unsigned char>::value || std::is_same<Type, vpRGBa>::value,
    "Template function supports only unsigned char and vpRGBa images!");
#endif
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
  char buffer[FILENAME_MAX];
  snprintf(buffer, FILENAME_MAX, std::string(input_directory + "/Images/Image_%04d." + ext).c_str(), cpt);
  std::string image_filename = buffer;

  snprintf(buffer, FILENAME_MAX, std::string(input_directory + "/Depth/Depth_%04d.bin").c_str(), cpt);
  std::string depth_filename = buffer;

  snprintf(buffer, FILENAME_MAX, std::string(input_directory + "/CameraPose/Camera_%03d.txt").c_str(), cpt);
  std::string pose_filename = buffer;

  if (!vpIoTools::checkFilename(image_filename) || !vpIoTools::checkFilename(depth_filename) ||
    !vpIoTools::checkFilename(pose_filename))
    return false;

  vpImageIo::read(I, image_filename);

  unsigned int depth_width = 0, depth_height = 0;
  std::ifstream file_depth(depth_filename.c_str(), std::ios::in | std::ios::binary);
  if (!file_depth.is_open())
    return false;

  vpIoTools::readBinaryValueLE(file_depth, depth_height);
  vpIoTools::readBinaryValueLE(file_depth, depth_width);
  I_depth.resize(depth_height, depth_width);
  pointcloud.resize(depth_height * depth_width);

  const float depth_scale = 0.000030518f;
  for (unsigned int i = 0; i < I_depth.getHeight(); i++) {
    for (unsigned int j = 0; j < I_depth.getWidth(); j++) {
      vpIoTools::readBinaryValueLE(file_depth, I_depth[i][j]);
      double x = 0.0, y = 0.0, Z = I_depth[i][j] * depth_scale;
      vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
      vpColVector pt3d(4, 1.0);
      pt3d[0] = x * Z;
      pt3d[1] = y * Z;
      pt3d[2] = Z;
      pointcloud[i * I_depth.getWidth() + j] = pt3d;
    }
  }

  std::ifstream file_pose(pose_filename.c_str());
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

void convert(const vpImage<vpRGBa> &src, vpImage<vpRGBa> &dst) { dst = src; }

void convert(const vpImage<unsigned char> &src, vpImage<vpRGBa> &dst) { vpImageConvert::convert(src, dst); }

template <typename Type>
bool run(const std::string &input_directory, bool opt_click_allowed, bool opt_display, bool useScanline,
  int trackerType_image, int opt_lastFrame, bool use_depth, bool use_mask, bool save)
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  static_assert(std::is_same<Type, unsigned char>::value || std::is_same<Type, vpRGBa>::value,
    "Template function supports only unsigned char and vpRGBa images!");
#endif
  // Initialise a  display
#if defined(VISP_HAVE_X11)
  vpDisplayX display1, display2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display1, display2;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV display1, display2;
#elif defined(VISP_HAVE_D3D9)
  vpDisplayD3D display1, display2;
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK display1, display2;
#else
  opt_display = false;
#endif

  std::vector<int> tracker_type(2);
  tracker_type[0] = trackerType_image;
  tracker_type[1] = vpMbGenericTracker::DEPTH_DENSE_TRACKER;
  vpMbGenericTracker tracker(tracker_type);

#if defined(VISP_HAVE_PUGIXML)
  std::string configFileCam1 = input_directory + std::string("/Config/chateau.xml");
  std::string configFileCam2 = input_directory + std::string("/Config/chateau_depth.xml");
  std::cout << "Load config file for camera 1: " << configFileCam1 << std::endl;
  std::cout << "Load config file for camera 2: " << configFileCam2 << std::endl;
  tracker.loadConfigFile(configFileCam1, configFileCam2);
#else
  // Corresponding parameters manually set to have an example code
  {
    vpCameraParameters cam_color, cam_depth;
    cam_color.initPersProjWithoutDistortion(700.0, 700.0, 320.0, 240.0);
    cam_depth.initPersProjWithoutDistortion(700.0, 700.0, 320.0, 240.0);
    tracker.setCameraParameters(cam_color, cam_depth);
  }

  // Edge
  vpMe me;
  me.setMaskSize(5);
  me.setMaskNumber(180);
  me.setRange(8);
  me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
  me.setThreshold(5);
  me.setMu1(0.5);
  me.setMu2(0.5);
  me.setSampleStep(5);
  tracker.setMovingEdge(me);

  // Klt
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  vpKltOpencv klt;
  tracker.setKltMaskBorder(5);
  klt.setMaxFeatures(10000);
  klt.setWindowSize(5);
  klt.setQuality(0.01);
  klt.setMinDistance(5);
  klt.setHarrisFreeParameter(0.02);
  klt.setBlockSize(3);
  klt.setPyramidLevels(3);

  tracker.setKltOpencv(klt);
#endif

  // Depth
  tracker.setDepthNormalFeatureEstimationMethod(vpMbtFaceDepthNormal::ROBUST_FEATURE_ESTIMATION);
  tracker.setDepthNormalPclPlaneEstimationMethod(2);
  tracker.setDepthNormalPclPlaneEstimationRansacMaxIter(200);
  tracker.setDepthNormalPclPlaneEstimationRansacThreshold(0.001);
  tracker.setDepthNormalSamplingStep(2, 2);

  tracker.setDepthDenseSamplingStep(4, 4);

  tracker.setAngleAppear(vpMath::rad(85.0));
  tracker.setAngleDisappear(vpMath::rad(89.0));
  tracker.setNearClippingDistance(0.01);
  tracker.setFarClippingDistance(2.0);
  tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
#endif

#ifdef VISP_HAVE_COIN3D
  tracker.loadModel(input_directory + "/Models/chateau.wrl", input_directory + "/Models/chateau.cao");
#else
  tracker.loadModel(input_directory + "/Models/chateau.cao", input_directory + "/Models/chateau.cao");
#endif
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
  vpCameraParameters cam_color, cam_depth;
  tracker.getCameraParameters(cam_color, cam_depth);
  tracker.setDisplayFeatures(true);
  tracker.setScanLineVisibilityTest(useScanline);

  std::map<int, std::pair<double, double> > map_thresh;
  // Take the highest thresholds between all CI machines
#ifdef VISP_HAVE_COIN3D
  map_thresh[vpMbGenericTracker::EDGE_TRACKER] =
    useScanline ? std::pair<double, double>(0.005, 6.) : std::pair<double, double>(0.007, 3.9);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  map_thresh[vpMbGenericTracker::KLT_TRACKER] =
    useScanline ? std::pair<double, double>(0.007, 1.9) : std::pair<double, double>(0.007, 1.8);
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER] =
    useScanline ? std::pair<double, double>(0.005, 3.7) : std::pair<double, double>(0.006, 3.4);
#endif
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    useScanline ? std::pair<double, double>(0.003, 1.7) : std::pair<double, double>(0.002, 0.8);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  map_thresh[vpMbGenericTracker::KLT_TRACKER | vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    std::pair<double, double>(0.002, 0.3);
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER |
    vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    useScanline ? std::pair<double, double>(0.002, 1.8) : std::pair<double, double>(0.002, 0.7);
#endif
#else
  map_thresh[vpMbGenericTracker::EDGE_TRACKER] =
    useScanline ? std::pair<double, double>(0.015, 3.0) : std::pair<double, double>(0.009, 4.0);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  map_thresh[vpMbGenericTracker::KLT_TRACKER] =
    useScanline ? std::pair<double, double>(0.006, 1.7) : std::pair<double, double>(0.005, 1.4);
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER] =
    useScanline ? std::pair<double, double>(0.004, 1.2) : std::pair<double, double>(0.004, 1.2);
#endif
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    useScanline ? std::pair<double, double>(0.002, 0.7) : std::pair<double, double>(0.001, 0.4);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  map_thresh[vpMbGenericTracker::KLT_TRACKER | vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    std::pair<double, double>(0.002, 0.3);
  map_thresh[vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER |
    vpMbGenericTracker::DEPTH_DENSE_TRACKER] =
    useScanline ? std::pair<double, double>(0.001, 0.5) : std::pair<double, double>(0.001, 0.4);
#endif
#endif

  vpImage<Type> I, I_depth;
  vpImage<uint16_t> I_depth_raw;
  vpHomogeneousMatrix cMo_truth;
  std::vector<vpColVector> pointcloud;
  int cpt_frame = 1;
  if (!read_data(input_directory, cpt_frame, cam_depth, I, I_depth_raw, pointcloud, cMo_truth)) {
    std::cerr << "Cannot read first frame!" << std::endl;
    return EXIT_FAILURE;
  }

  vpImage<bool> mask(I.getHeight(), I.getWidth());
  const double roi_step = 7.0;
  const double roi_step2 = 6.0;
  if (use_mask) {
    mask = false;
    for (unsigned int i = (unsigned int)(I.getRows() / roi_step);
      i < (unsigned int)(I.getRows() * roi_step2 / roi_step); i++) {
      for (unsigned int j = (unsigned int)(I.getCols() / roi_step);
        j < (unsigned int)(I.getCols() * roi_step2 / roi_step); j++) {
        mask[i][j] = true;
      }
    }
    tracker.setMask(mask);
  }

  vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

  vpImage<vpRGBa> results(I.getHeight(), I.getWidth() + I_depth.getWidth());
  vpImage<vpRGBa> resultsColor(I.getHeight(), I.getWidth());
  vpImage<vpRGBa> resultsDepth(I_depth.getHeight(), I_depth.getWidth());
  if (save) {
    vpIoTools::makeDirectory("results");
  }
  if (opt_display) {
#ifdef VISP_HAVE_DISPLAY
    display1.init(I, 0, 0, "Image");
    display2.init(I_depth, (int)I.getWidth(), 0, "Depth");
#endif
  }

  vpHomogeneousMatrix depth_M_color;
  depth_M_color[0][3] = -0.05;
  tracker.setCameraTransformationMatrix("Camera2", depth_M_color);
  tracker.initFromPose(I, cMo_truth);

  vpFont font(24);
  bool click = false, quit = false, correct_accuracy = true;
  std::vector<double> vec_err_t, vec_err_tu;
  std::vector<double> time_vec;
  while (read_data(input_directory, cpt_frame, cam_depth, I, I_depth_raw, pointcloud, cMo_truth) && !quit &&
    (opt_lastFrame > 0 ? (int)cpt_frame <= opt_lastFrame : true)) {
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    if (opt_display) {
      vpDisplay::display(I);
      vpDisplay::display(I_depth);
    }
    else if (save) {
      convert(I, resultsColor);
      convert(I_depth, resultsDepth);
    }

    double t = vpTime::measureTimeMs();
    std::map<std::string, const vpImage<Type> *> mapOfImages;
    mapOfImages["Camera1"] = &I;
    std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
    mapOfPointclouds["Camera2"] = &pointcloud;
    std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
    if (!use_depth) {
      mapOfWidths["Camera2"] = 0;
      mapOfHeights["Camera2"] = 0;
    }
    else {
      mapOfWidths["Camera2"] = I_depth.getWidth();
      mapOfHeights["Camera2"] = I_depth.getHeight();
    }

    tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
    vpHomogeneousMatrix cMo = tracker.getPose();
    t = vpTime::measureTimeMs() - t;
    time_vec.push_back(t);

    if (opt_display) {
      tracker.display(I, I_depth, cMo, depth_M_color * cMo, cam_color, cam_depth, vpColor::red, 3);
      vpDisplay::displayFrame(I, cMo, cam_depth, 0.05, vpColor::none, 3);
      vpDisplay::displayFrame(I_depth, depth_M_color * cMo, cam_depth, 0.05, vpColor::none, 3);

      std::stringstream ss;
      ss << "Frame: " << cpt_frame;
      vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
      ss.str("");
      ss << "Nb features: " << tracker.getError().getRows();
      vpDisplay::displayText(I_depth, 40, 20, ss.str(), vpColor::red);
    }
    else if (save) {
      //! [Draw CAD model]
      std::map<std::string, std::vector<std::vector<double> > > mapOfModels;
      std::map<std::string, unsigned int> mapOfW;
      mapOfW["Camera1"] = I.getWidth();
      mapOfW["Camera2"] = I_depth.getWidth();
      std::map<std::string, unsigned int> mapOfH;
      mapOfH["Camera1"] = I.getHeight();
      mapOfH["Camera2"] = I_depth.getHeight();
      std::map<std::string, vpHomogeneousMatrix> mapOfcMos;
      mapOfcMos["Camera1"] = cMo;
      mapOfcMos["Camera2"] = depth_M_color * cMo;
      std::map<std::string, vpCameraParameters> mapOfCams;
      mapOfCams["Camera1"] = cam_color;
      mapOfCams["Camera2"] = cam_depth;
      tracker.getModelForDisplay(mapOfModels, mapOfW, mapOfH, mapOfcMos, mapOfCams);
      for (std::map<std::string, std::vector<std::vector<double> > >::const_iterator it = mapOfModels.begin();
        it != mapOfModels.end(); ++it) {
        for (size_t i = 0; i < it->second.size(); i++) {
          // test if it->second[i][0] = 0
          if (std::fabs(it->second[i][0]) <= std::numeric_limits<double>::epsilon()) {
            vpImageDraw::drawLine(it->first == "Camera1" ? resultsColor : resultsDepth,
              vpImagePoint(it->second[i][1], it->second[i][2]),
              vpImagePoint(it->second[i][3], it->second[i][4]), vpColor::red, 3);
          }
        }
      }
      //! [Draw CAD model]

      //! [Draw features]
      std::map<std::string, std::vector<std::vector<double> > > mapOfFeatures;
      tracker.getFeaturesForDisplay(mapOfFeatures);
      for (std::map<std::string, std::vector<std::vector<double> > >::const_iterator it = mapOfFeatures.begin();
        it != mapOfFeatures.end(); ++it) {
        for (size_t i = 0; i < it->second.size(); i++) {
          if (std::fabs(it->second[i][0]) <=
            std::numeric_limits<double>::epsilon()) { // test it->second[i][0] = 0 for ME
            vpColor color = vpColor::yellow;
            if (std::fabs(it->second[i][3]) <= std::numeric_limits<double>::epsilon()) { // test it->second[i][3] = 0
              color = vpColor::green;
            }
            else if (std::fabs(it->second[i][3] - 1) <=
              std::numeric_limits<double>::epsilon()) { // test it->second[i][3] = 1
              color = vpColor::blue;
            }
            else if (std::fabs(it->second[i][3] - 2) <=
              std::numeric_limits<double>::epsilon()) { // test it->second[i][3] = 2
              color = vpColor::purple;
            }
            else if (std::fabs(it->second[i][3] - 3) <=
              std::numeric_limits<double>::epsilon()) { // test it->second[i][3] = 3
              color = vpColor::red;
            }
            else if (std::fabs(it->second[i][3] - 4) <=
              std::numeric_limits<double>::epsilon()) { // test it->second[i][3] = 4
              color = vpColor::cyan;
            }
            vpImageDraw::drawCross(it->first == "Camera1" ? resultsColor : resultsDepth,
              vpImagePoint(it->second[i][1], it->second[i][2]), 3, color, 1);
          }
          else if (std::fabs(it->second[i][0] - 1) <=
            std::numeric_limits<double>::epsilon()) { // test it->second[i][0] = 1 for KLT
            vpImageDraw::drawCross(it->first == "Camera1" ? resultsColor : resultsDepth,
              vpImagePoint(it->second[i][1], it->second[i][2]), 10, vpColor::red, 1);
          }
        }
      }
      //! [Draw features]

      // Computation time
      std::ostringstream oss;
      oss << "Tracking time: " << t << " ms";
      font.drawText(resultsColor, oss.str(), vpImagePoint(20, 20), vpColor::red);
    }

    vpPoseVector pose_est(cMo);
    vpPoseVector pose_truth(cMo_truth);
    vpColVector t_est(3), t_truth(3);
    vpColVector tu_est(3), tu_truth(3);
    for (unsigned int i = 0; i < 3; i++) {
      t_est[i] = pose_est[i];
      t_truth[i] = pose_truth[i];
      tu_est[i] = pose_est[i + 3];
      tu_truth[i] = pose_truth[i + 3];
    }

    vpColVector t_err = t_truth - t_est, tu_err = tu_truth - tu_est;
    const double t_thresh =
      map_thresh[!use_depth ? trackerType_image : trackerType_image | vpMbGenericTracker::DEPTH_DENSE_TRACKER].first;
    const double tu_thresh =
      map_thresh[!use_depth ? trackerType_image : trackerType_image | vpMbGenericTracker::DEPTH_DENSE_TRACKER].second;
    double t_err2 = sqrt(t_err.sumSquare()), tu_err2 = vpMath::deg(sqrt(tu_err.sumSquare()));
    vec_err_t.push_back(t_err2);
    vec_err_tu.push_back(tu_err2);
    if (!use_mask && (t_err2 > t_thresh || tu_err2 > tu_thresh)) { // no accuracy test with mask
      std::cerr << "Pose estimated exceeds the threshold (t_thresh = " << t_thresh << " ; tu_thresh = " << tu_thresh
        << ")!" << std::endl;
      std::cout << "t_err: " << t_err2 << " ; tu_err: " << tu_err2 << std::endl;
      correct_accuracy = false;
    }

    if (opt_display) {
      if (use_mask) {
        vpRect roi(vpImagePoint(I.getRows() / roi_step, I.getCols() / roi_step),
          vpImagePoint(I.getRows() * roi_step2 / roi_step, I.getCols() * roi_step2 / roi_step));
        vpDisplay::displayRectangle(I, roi, vpColor::yellow);
        vpDisplay::displayRectangle(I_depth, roi, vpColor::yellow);
      }

      vpDisplay::flush(I);
      vpDisplay::flush(I_depth);
    }
    else if (save) {
      //! [Save drawings]
      char buffer[FILENAME_MAX];
      std::ostringstream oss;
      oss << "results/image_%04d.png";
      snprintf(buffer, FILENAME_MAX, oss.str().c_str(), cpt_frame);

      results.insert(resultsColor, vpImagePoint());
      results.insert(resultsDepth, vpImagePoint(0, resultsColor.getWidth()));

      vpImageIo::write(results, buffer);
      //! [Save drawings]
    }

    if (opt_display && opt_click_allowed) {
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

  if (!time_vec.empty())
    std::cout << "Computation time, Mean: " << vpMath::getMean(time_vec)
    << " ms ; Median: " << vpMath::getMedian(time_vec) << " ms ; Std: " << vpMath::getStdev(time_vec) << " ms"
    << std::endl;

  if (!vec_err_t.empty())
    std::cout << "Max translation error: " << *std::max_element(vec_err_t.begin(), vec_err_t.end()) << std::endl;

  if (!vec_err_tu.empty())
    std::cout << "Max thetau error: " << *std::max_element(vec_err_tu.begin(), vec_err_tu.end()) << std::endl;

  std::cout << "Test result: " << (correct_accuracy ? "success" : "failure") << std::endl;
  return correct_accuracy ? EXIT_SUCCESS : EXIT_FAILURE;
}
} // namespace

int main(int argc, const char *argv[])
{
  try {
    std::string env_ipath;
    std::string opt_ipath = "";
    bool opt_click_allowed = true;
    bool opt_display = true;
    bool opt_save = false;
    bool useScanline = false;
    int trackerType_image = vpMbGenericTracker::EDGE_TRACKER;
#if defined(__mips__) || defined(__mips) || defined(mips) || defined(__MIPS__)
    // To avoid Debian test timeout
    int opt_lastFrame = 5;
#else
    int opt_lastFrame = -1;
#endif
    bool use_depth = false;
    bool use_mask = false;
    bool use_color_image = false;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_save, useScanline, trackerType_image,
                    opt_lastFrame, use_depth, use_mask, use_color_image)) {
      return EXIT_FAILURE;
    }

#if ! (defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO))
    if (trackerType_image == 2 || trackerType_image == 3) {
      std::cout << "Using klt tracker is not possible without OpenCV imgproc and video modules." << std::endl;
      std::cout << "Use rather command line option -t 1 to use edges." << std::endl;
      return EXIT_SUCCESS;
    }
#endif
    std::cout << "trackerType_image: " << trackerType_image << std::endl;
    std::cout << "useScanline: " << useScanline << std::endl;
    std::cout << "use_depth: " << use_depth << std::endl;
    std::cout << "use_mask: " << use_mask << std::endl;
    std::cout << "use_color_image: " << use_color_image << std::endl;
#ifdef VISP_HAVE_COIN3D
    std::cout << "COIN3D available." << std::endl;
#endif

#if !defined(VISP_HAVE_MODULE_KLT) || (!defined(VISP_HAVE_OPENCV) || (VISP_HAVE_OPENCV_VERSION < 0x020100))
    if (trackerType_image & 2) {
      std::cout << "KLT features cannot be used: ViSP is not built with "
        "KLT module or OpenCV is not available.\nTest is not run."
        << std::endl;
      return EXIT_SUCCESS;
    }
#endif

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], nullptr);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << std::endl;

      return EXIT_FAILURE;
    }

    std::string input_directory =
      vpIoTools::createFilePath(!opt_ipath.empty() ? opt_ipath : env_ipath, "mbt-depth/Castle-simu");
    if (!vpIoTools::checkDirectory(input_directory)) {
      std::cerr << "ViSP-images does not contain the folder: " << input_directory << "!" << std::endl;
      return EXIT_SUCCESS;
    }

    if (use_color_image) {
      return run<vpRGBa>(input_directory, opt_click_allowed, opt_display, useScanline, trackerType_image, opt_lastFrame,
        use_depth, use_mask, opt_save);
    }
    else {
      return run<unsigned char>(input_directory, opt_click_allowed, opt_display, useScanline, trackerType_image,
        opt_lastFrame, use_depth, use_mask, opt_save);
    }

    std::cout << "Test succeed" << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#elif !(defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
int main()
{
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "Enable MBT module (VISP_HAVE_MODULE_MBT) to launch this test." << std::endl;
  return EXIT_SUCCESS;
}
#endif
