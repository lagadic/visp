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
 * Check that MBT is deterministic.
 */

/*!
  \example testGenericTrackerDeterminist.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_THREADS)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <future>
#include <thread>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>

// #define DEBUG_DISPLAY // uncomment to check that the tracking is correct
#ifdef DEBUG_DISPLAY
#include <visp3/gui/vpDisplayX.h>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
bool read_data(int cpt, vpImage<unsigned char> &I)
{
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
  const std::string env_ipath = vpIoTools::getViSPImagesDataPath();
  const std::string ipath = vpIoTools::createFilePath(env_ipath, "mbt/cube/image%04d." + ext);

  char buffer[FILENAME_MAX];
  snprintf(buffer, FILENAME_MAX, ipath.c_str(), cpt);
  std::string image_filename = buffer;

  if (!vpIoTools::checkFilename(image_filename)) {
    return false;
  }

  vpImageIo::read(I, image_filename);
  return true;
}

void checkPoses(const vpHomogeneousMatrix &cMo1, const vpHomogeneousMatrix &cMo2)
{
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      CHECK(cMo1[i][j] == Approx(cMo2[i][j]).epsilon(std::numeric_limits<double>::epsilon()));
    }
  }
}

void configureTracker(vpMbGenericTracker &tracker, vpCameraParameters &cam)
{
  const std::string env_ipath = vpIoTools::getViSPImagesDataPath();
  const std::string configFile = vpIoTools::createFilePath(env_ipath, "mbt/cube.xml");
  const std::string modelFile = vpIoTools::createFilePath(env_ipath, "mbt/cube_and_cylinder.cao");
#if defined(VISP_HAVE_PUGIXML)
  const bool verbose = false;
  tracker.loadConfigFile(configFile, verbose);
#else
  // Corresponding parameters manually set to have an example code
  // By setting the parameters:
  cam.initPersProjWithoutDistortion(547, 542, 338, 234);

  vpMe me;
  me.setMaskSize(5);
  me.setMaskNumber(180);
  me.setRange(7);
  me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
  me.setThreshold(5);
  me.setMu1(0.5);
  me.setMu2(0.5);
  me.setSampleStep(4);

  vpKltOpencv klt;
  klt.setMaxFeatures(300);
  klt.setWindowSize(5);
  klt.setQuality(0.01);
  klt.setMinDistance(5);
  klt.setHarrisFreeParameter(0.01);
  klt.setBlockSize(3);
  klt.setPyramidLevels(3);

  tracker.setCameraParameters(cam);
  tracker.setMovingEdge(me);
  tracker.setKltOpencv(klt);
  tracker.setKltMaskBorder(5);
  tracker.setAngleAppear(vpMath::rad(65));
  tracker.setAngleDisappear(vpMath::rad(75));

  // Specify the clipping to
  tracker.setNearClippingDistance(0.01);
  tracker.setFarClippingDistance(0.90);
  tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
  //   tracker.setClipping(tracker.getClipping() | vpMbtPolygon::LEFT_CLIPPING |
  //   vpMbtPolygon::RIGHT_CLIPPING | vpMbtPolygon::UP_CLIPPING |
  //   vpMbtPolygon::DOWN_CLIPPING); // Equivalent to FOV_CLIPPING
#endif
  tracker.getCameraParameters(cam);
  tracker.loadModel(modelFile);
  tracker.setDisplayFeatures(true);

  const vpPoseVector initPose(0.02231950571, 0.1071368004, 0.5071128378, 2.100485509, 1.146812236, -0.4560126437);
  vpImage<unsigned char> I;
  read_data(0, I);
  tracker.initFromPose(I, vpHomogeneousMatrix(initPose));
}
} // anonymous namespace

TEST_CASE("Check MBT determinism sequential", "[MBT_determinism]")
{
  // First tracker
  vpMbGenericTracker tracker1;
  vpCameraParameters cam;
  configureTracker(tracker1, cam);

  vpImage<unsigned char> I;
  read_data(0, I);
#ifdef DEBUG_DISPLAY
  vpDisplayX d(I);
#endif

  vpHomogeneousMatrix cMo1;
  for (int cpt = 0; read_data(cpt, I); cpt++) {
    tracker1.track(I);
    tracker1.getPose(cMo1);

#ifdef DEBUG_DISPLAY
    vpDisplay::display(I);
    tracker1.display(I, cMo1, cam, vpColor::red, 3);
    vpDisplay::displayFrame(I, cMo1, cam, 0.05, vpColor::none, 3);
    vpDisplay::flush(I);
#endif
  }
  std::cout << "First tracker, final cMo:\n" << cMo1 << std::endl;

  // Second tracker
  vpMbGenericTracker tracker2;
  configureTracker(tracker2, cam);
  vpHomogeneousMatrix cMo2;
  for (int cpt = 0; read_data(cpt, I); cpt++) {
    tracker2.track(I);
    tracker2.getPose(cMo2);

#ifdef DEBUG_DISPLAY
    vpDisplay::display(I);
    tracker2.display(I, cMo2, cam, vpColor::red, 3);
    vpDisplay::displayFrame(I, cMo2, cam, 0.05, vpColor::none, 3);
    vpDisplay::flush(I);
#endif
  }
  std::cout << "Second tracker, final cMo:\n" << cMo2 << std::endl;

  // Check that both poses are identical
  checkPoses(cMo1, cMo2);
}

TEST_CASE("Check MBT determinism parallel", "[MBT_determinism]")
{
  // First tracker
  std::future<vpHomogeneousMatrix> res_cMo1 = std::async(std::launch::async, []() {
    vpMbGenericTracker tracker1;
    vpCameraParameters cam;
    configureTracker(tracker1, cam);

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo1;
    for (int cpt = 0; read_data(cpt, I); cpt++) {
      tracker1.track(I);
      tracker1.getPose(cMo1);
    }
    return cMo1;
  });

  // Second tracker
  std::future<vpHomogeneousMatrix> res_cMo2 = std::async(std::launch::async, []() {
    vpMbGenericTracker tracker2;
    vpCameraParameters cam;
    configureTracker(tracker2, cam);

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo2;
    for (int cpt = 0; read_data(cpt, I); cpt++) {
      tracker2.track(I);
      tracker2.getPose(cMo2);
    }
    return cMo2;
  });

  vpHomogeneousMatrix cMo1 = res_cMo1.get();
  vpHomogeneousMatrix cMo2 = res_cMo2.get();
  std::cout << "Run both trackers in separate threads" << std::endl;
  std::cout << "First tracker, final cMo:\n" << cMo1 << std::endl;
  std::cout << "Second tracker, final cMo:\n" << cMo2 << std::endl;

  // Check that both poses are identical
  checkPoses(cMo1, cMo2);
}

TEST_CASE("Check Stereo MBT determinism parallel", "[MBT_determinism]")
{
  // First tracker
  std::future<vpHomogeneousMatrix> res_cMo1 = std::async(std::launch::async, []() {
    vpMbGenericTracker tracker1(2);
    vpCameraParameters cam;
    configureTracker(tracker1, cam);

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo1;
    for (int cpt = 0; read_data(cpt, I); cpt++) {
      tracker1.track(I, I);
      tracker1.getPose(cMo1);
    }
    return cMo1;
  });

  // Second tracker
  std::future<vpHomogeneousMatrix> res_cMo2 = std::async(std::launch::async, []() {
    vpMbGenericTracker tracker2(2);
    vpCameraParameters cam;
    configureTracker(tracker2, cam);

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo2;
    for (int cpt = 0; read_data(cpt, I); cpt++) {
      tracker2.track(I, I);
      tracker2.getPose(cMo2);
    }
    return cMo2;
  });

  vpHomogeneousMatrix cMo1 = res_cMo1.get();
  vpHomogeneousMatrix cMo2 = res_cMo2.get();
  std::cout << "Run both stereo trackers in separate threads" << std::endl;
  std::cout << "First tracker, final cMo:\n" << cMo1 << std::endl;
  std::cout << "Second tracker, final cMo:\n" << cMo2 << std::endl;

  // Check that both poses are identical
  checkPoses(cMo1, cMo2);
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

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

int main() { return EXIT_SUCCESS; }
#endif
