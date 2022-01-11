/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Benchmark generic tracker.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>

//#define DEBUG_DISPLAY // uncomment to check that the tracking is correct
#ifdef DEBUG_DISPLAY
#include <visp3/gui/vpDisplayX.h>
#endif

namespace
{
bool runBenchmark = false;

template <typename Type>
bool read_data(const std::string &input_directory, int cpt, const vpCameraParameters &cam_depth,
               vpImage<Type> &I, vpImage<uint16_t> &I_depth,
               std::vector<vpColVector> &pointcloud, vpHomogeneousMatrix &cMo)
{
  static_assert(std::is_same<Type, unsigned char>::value || std::is_same<Type, vpRGBa>::value,
                "Template function supports only unsigned char and vpRGBa images!");
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
  std::ifstream file_depth(depth_filename.c_str(), std::ios::in | std::ios::binary);
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
} //anonymous namespace

TEST_CASE("Benchmark generic tracker", "[benchmark]") {
  if (runBenchmark) {
    std::vector<int> tracker_type(2);
    tracker_type[0] = vpMbGenericTracker::EDGE_TRACKER;
    tracker_type[1] = vpMbGenericTracker::DEPTH_DENSE_TRACKER;
    vpMbGenericTracker tracker(tracker_type);

    const std::string input_directory = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "mbt-depth/Castle-simu");
    const std::string configFileCam1 = input_directory + std::string("/Config/chateau.xml");
    const std::string configFileCam2 = input_directory + std::string("/Config/chateau_depth.xml");
    REQUIRE(vpIoTools::checkFilename(configFileCam1));
    REQUIRE(vpIoTools::checkFilename(configFileCam2));
    tracker.loadConfigFile(configFileCam1, configFileCam2);
    REQUIRE(vpIoTools::checkFilename(input_directory + "/Models/chateau.cao"));
    tracker.loadModel(input_directory + "/Models/chateau.cao", input_directory + "/Models/chateau.cao");

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

    vpImage<unsigned char> I;
    vpImage<uint16_t> I_depth_raw;
    vpHomogeneousMatrix cMo_truth;
    std::vector<vpColVector> pointcloud;

    vpCameraParameters cam_color, cam_depth;
    tracker.getCameraParameters(cam_color, cam_depth);

    vpHomogeneousMatrix depth_M_color;
    depth_M_color[0][3] = -0.05;
    tracker.setCameraTransformationMatrix("Camera2", depth_M_color);

    // load all the data in memory to not take into account I/O from disk
    std::vector<vpImage<unsigned char>> images;
    std::vector<vpImage<uint16_t>> depth_raws;
    std::vector<std::vector<vpColVector>> pointclouds;
    std::vector<vpHomogeneousMatrix> cMo_truth_all;
    // forward
    for (int i = 1; i <= 40; i++) {
      if (read_data(input_directory, i, cam_depth, I, I_depth_raw, pointcloud, cMo_truth)) {
        images.push_back(I);
        depth_raws.push_back(I_depth_raw);
        pointclouds.push_back(pointcloud);
        cMo_truth_all.push_back(cMo_truth);
      }
    }
    // backward
    for (int i = 40; i >= 1; i--) {
      if (read_data(input_directory, i, cam_depth, I, I_depth_raw, pointcloud, cMo_truth)) {
        images.push_back(I);
        depth_raws.push_back(I_depth_raw);
        pointclouds.push_back(pointcloud);
        cMo_truth_all.push_back(cMo_truth);
      }
    }

    // Stereo MBT
    {
      std::vector<std::map<std::string, int>> mapOfTrackerTypes;
      mapOfTrackerTypes.push_back({{"Camera1", vpMbGenericTracker::EDGE_TRACKER}, {"Camera2", vpMbGenericTracker::DEPTH_DENSE_TRACKER}});
      mapOfTrackerTypes.push_back({{"Camera1", vpMbGenericTracker::EDGE_TRACKER}, {"Camera2", vpMbGenericTracker::DEPTH_DENSE_TRACKER}});
  #if defined(VISP_HAVE_OPENCV)
      mapOfTrackerTypes.push_back({{"Camera1", vpMbGenericTracker::KLT_TRACKER}, {"Camera2", vpMbGenericTracker::DEPTH_DENSE_TRACKER}});
      mapOfTrackerTypes.push_back({{"Camera1", vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER}, {"Camera2", vpMbGenericTracker::DEPTH_DENSE_TRACKER}});
      mapOfTrackerTypes.push_back({{"Camera1", vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER}, {"Camera2", vpMbGenericTracker::DEPTH_DENSE_TRACKER}});
  #endif

      std::vector<std::string> benchmarkNames = {
        "Edge MBT",
        "Edge + Depth dense MBT",
  #if defined(VISP_HAVE_OPENCV)
        "KLT MBT",
        "KLT + depth dense MBT",
        "Edge + KLT + depth dense MBT"
  #endif
      };

      std::vector<bool> monoculars = {
        true,
        false,
  #if defined(VISP_HAVE_OPENCV)
        true,
        false,
        false
  #endif
      };

      for (size_t idx = 0; idx < mapOfTrackerTypes.size(); idx++) {
        tracker.resetTracker();
        tracker.setTrackerType(mapOfTrackerTypes[idx]);

        tracker.loadConfigFile(configFileCam1, configFileCam2);
        tracker.loadModel(input_directory + "/Models/chateau.cao", input_directory + "/Models/chateau.cao");
        tracker.loadModel(input_directory + "/Models/cube.cao", false, T);
        tracker.initFromPose(images.front(), cMo_truth_all.front());

        std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
        mapOfWidths["Camera2"] = monoculars[idx] ? 0 : I_depth_raw.getWidth();
        mapOfHeights["Camera2"] = monoculars[idx] ? 0 : I_depth_raw.getHeight();

        vpHomogeneousMatrix cMo;
    #ifndef DEBUG_DISPLAY
        BENCHMARK(benchmarkNames[idx].c_str())
    #else
        vpImage<unsigned char> I_depth;
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

        vpDisplayX d_color(I, 0, 0, "Color image");
        vpDisplayX d_depth(I_depth, I.getWidth(), 0, "Depth image");
        tracker.setDisplayFeatures(true);
    #endif
        {
          tracker.initFromPose(images.front(), cMo_truth_all.front());

          for (size_t i = 0; i < images.size(); i++) {
            const vpImage<unsigned char>& I_current = images[i];
            const std::vector<vpColVector>& pointcloud_current = pointclouds[i];

      #ifdef DEBUG_DISPLAY
            vpImageConvert::createDepthHistogram(depth_raws[i], I_depth);
            I = I_current;
            vpDisplay::display(I);
            vpDisplay::display(I_depth);
      #endif

            std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
            mapOfImages["Camera1"] = &I_current;

            std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
            mapOfPointclouds["Camera2"] = &pointcloud_current;

            tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
            cMo = tracker.getPose();

      #ifdef DEBUG_DISPLAY
            tracker.display(I, I_depth, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::red, 3);
            vpDisplay::displayFrame(I, cMo, cam_color, 0.05, vpColor::none, 3);
            vpDisplay::displayFrame(I_depth, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
            vpDisplay::displayText(I, 20, 20, benchmarkNames[idx], vpColor::red);
            vpDisplay::displayText(I, 40, 20, std::string("Nb features: " + std::to_string(tracker.getError().getRows())), vpColor::red);

            vpDisplay::flush(I);
            vpDisplay::flush(I_depth);
            vpTime::wait(33);
      #endif
          }

    #ifndef DEBUG_DISPLAY
          return cMo;
        };
    #else
        }
    #endif

        vpPoseVector pose_est(cMo);
        vpPoseVector pose_truth(cMo_truth);
        vpColVector t_err(3), tu_err(3);
        for (unsigned int i = 0; i < 3; i++) {
          t_err[i] = pose_est[i] - pose_truth[i];
          tu_err[i] = pose_est[i+3] - pose_truth[i+3];
        }

        const double max_translation_error = 0.005;
        const double max_rotation_error = 0.03;
        CHECK(sqrt(t_err.sumSquare()) < max_translation_error);
        CHECK(sqrt(tu_err.sumSquare()) < max_rotation_error);
      }
    }
  } //if (runBenchmark)
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()   // Get Catch's composite command line parser
      | Opt(runBenchmark)    // bind variable to a new option, with a hint string
      ["--benchmark"]        // the option names it will respond to
      ("run benchmark comparing naive code with ViSP implementation");     // description string for the help output

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main()
{
  return 0;
}
#endif
