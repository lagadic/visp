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
 * Test vpMbGenericTracker JSON parse / save.
 */

/*!
  \example catchRBTDataset.cpp

  Verify on dataset that RBT works
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/rbt/vpRBTracker.h>

#include <visp3/rbt/vpRBSilhouetteMeTracker.h>
#include <visp3/rbt/vpRBSilhouetteCCDTracker.h>
#include <visp3/rbt/vpRBKltTracker.h>
#include <visp3/rbt/vpRBDenseDepthTracker.h>
#include <visp3/ar/vpPanda3DFrameworkManager.h>

#include "test_utils.h"

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

#define CATCH_CONFIG_RUNNER
#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_DATASET)
bool opt_no_display = false; // If true, disable display or tests requiring display

struct SequenceFrame
{
  vpImage<unsigned char> I;
  vpImage<vpRGBa> IRGB;
  vpImage<float> depth;
};

class Sequence
{
public:

  Sequence(const std::string &path)
  {
    m_path = path;
    loadCameraSettings();
    loadFrames();
    loadGroundTruth();
  }

  unsigned int getImageHeight() const { return m_h; }
  unsigned int getImageWidth() const { return m_w; }
  vpCameraParameters cam() const { return m_cam; }

  void loadGroundTruth()
  {
    const std::string &groundTruthPath = vpIoTools::createFilePath(m_path, "apriltag_data.json");
    std::ifstream f(groundTruthPath);
    if (!f.good()) {
      throw vpException(vpException::ioError, "Could not open ground truth file %s", groundTruthPath.c_str());
    }
    nlohmann::json j = nlohmann::json::parse(f);
    m_cMg = j.at("grid");
    // std::cout << "Loaded ground truth data from apriltag grid: " << std::endl;
    // for (const vpHomogeneousMatrix &cMg: m_cMg) {
    //   std::cout << vpPoseVector(cMg).t() << std::endl;
    // }
    f.close();
  }

  void loadCameraSettings()
  {
    const std::string camFile = vpIoTools::createFilePath(m_path, "cam.json");
    if (!vpIoTools::checkFilename(camFile)) {
      throw vpException(vpException::ioError, "Camera file %s does not exist", camFile.c_str());
    }
    std::ifstream cf(camFile);

    if (!cf.good()) {
      throw vpException(vpException::ioError, "Problem opening %s", camFile.c_str());
    }
    const nlohmann::json j = nlohmann::json::parse(cf);
    m_cam.initPersProjWithoutDistortion(j.at("px"), j.at("py"), j.at("u0"), j.at("v0"));
    m_h = j.at("h"), m_w = j.at("w");
    m_depthScale = j.at("depthScale");
  }

  const std::string getColorFrame(unsigned int index)
  {
    std::stringstream colorName;
    colorName << "color_image_" << std::setfill('0') << std::setw(4) << index << ".png";
    return vpIoTools::createFilePath(m_path, colorName.str());
  }

  const std::string getDepthFrame(unsigned int index)
  {
    std::stringstream colorName;
    colorName << "depth_image_" << std::setfill('0') << std::setw(4) << index << ".npz";
    return vpIoTools::createFilePath(m_path, colorName.str());
  }

  void initTracker(vpRBTracker &tracker)
  {
    tracker.setCameraParameters(m_cam, m_h, m_w);
  }

  void loadFrames()
  {
    unsigned int frameIndex = 0;
    while (true) {
      SequenceFrame frame;
      const std::string colorFramePath = getColorFrame(frameIndex);
      if (!vpIoTools::checkFilename(colorFramePath)) {
        break;
      }
      vpImageIo::read(frame.IRGB, colorFramePath);
      vpImageConvert::convert(frame.IRGB, frame.I);

      const std::string depthFramePath = getDepthFrame(frameIndex);
      if (vpIoTools::checkFilename(depthFramePath)) {
        visp::cnpy::NpyArray depth_data = visp::cnpy::npz_load(depthFramePath).find("data")->second;
        vpImage<uint16_t> depthRaw(depth_data.data<uint16_t>(), m_h, m_w, true);
        frame.depth.resize(m_h, m_w);
        for (unsigned int i = 0; i < m_h * m_w; ++i) {
          frame.depth.bitmap[i] = static_cast<float>(depthRaw.bitmap[i]) * m_depthScale;
        }
      }

      m_frames.push_back(frame);
      ++frameIndex;
    }
  }

  unsigned int numFrames() const
  {
    return m_frames.size();
  }
  SequenceFrame getFrame(unsigned int index) const
  {
    return m_frames[index];
  }

  std::map<std::string, vpHomogeneousMatrix> getInitialPoses(vpRBTracker &tracker, const std::string &initsFolder, const std::string &objectsFolder, const std::vector<std::string> &objectNames)
  {
    std::map<std::string, vpHomogeneousMatrix> result;
    for (const std::string &objectName: objectNames) {
      const std::string objectInitFile = vpIoTools::createFilePath(initsFolder, objectName + ".json");
      const std::string objectFolder = vpIoTools::createFilePath(objectsFolder, objectName);
      const std::string objectClickInitFile = vpIoTools::createFilePath(objectFolder, objectName + ".init");

      if (vpIoTools::checkFilename(objectInitFile)) {
        std::ifstream f(objectInitFile);
        if (f.good()) {
          vpHomogeneousMatrix cMo = nlohmann::json::parse(f);
          result[objectName] = cMo;
        }
        else {
          throw vpException(vpException::ioError, "There was an issue opening init file %s", objectInitFile.c_str());
        }
        f.close();
      }
      else {
        tracker.setModelPath(vpIoTools::createFilePath(objectFolder, objectName + ".obj"));
        tracker.startTracking();
        tracker.initClick(m_frames[0].IRGB, objectClickInitFile, true);
        vpHomogeneousMatrix cMo;
        tracker.getPose(cMo);
        std::ofstream f(objectInitFile);
        if (f.good()) {
          nlohmann::json j = cMo;
          f << j.dump(2);
        }
      }
    }
    return result;
  }

  vpHomogeneousMatrix getGroundTruthGridPose(unsigned int index) const
  {
    return m_cMg[index];
  }

private:
  std::string m_path;
  vpCameraParameters m_cam;
  float m_depthScale;
  unsigned int m_h, m_w;
  std::vector<SequenceFrame> m_frames;
  std::vector<vpHomogeneousMatrix> m_cMg;
};

struct RunData
{
  RunData(const std::string &config, double errorT, double errorR) : configName(config), thresholdErrorT(errorT), thresholdErrorR(errorR) { }
  const std::string configName;
  const double thresholdErrorT;
  const double thresholdErrorR;

};

SCENARIO("Running tracker on sequences with ground truth", "[rbt]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
    const std::string datasetPath = vpIoTools::getViSPImagesDataPath();
    const std::string rbtDatasetPath = vpIoTools::createFilePath(datasetPath, "rbt");
    const std::string sequencePath = vpIoTools::createFilePath(rbtDatasetPath, "sequence");
    const std::string initsFolder = vpIoTools::createFilePath(sequencePath, "init");
    const std::string modelsPath = vpIoTools::createFilePath(rbtDatasetPath, "models");
    const std::string configsPath = vpIoTools::createFilePath(rbtDatasetPath, "configs");

    GIVEN("A sequence")
    {
      Sequence sequence(sequencePath);
      vpRBTracker tracker;
      sequence.initTracker(tracker);
      unsigned int h = sequence.getImageHeight(), w = sequence.getImageWidth();

      vpImage<unsigned char> displayI(h, w), displayDepth(h, w, 255), displayMask(h, w);
      vpImage<vpRGBa> displayRGB(h, w);

      std::vector<std::shared_ptr<vpDisplay>> displays = vpDisplayFactory::makeDisplayGrid(2, 2, 0, 0, 20, 20,
        "Gray", displayI,
        "Color", displayRGB,
        "Depth", displayDepth,
        "Mask", displayMask
      );

      const std::vector<std::string> objectNames = { "dragon", "cube", "stomach", "lower_teeth" };

      std::map<std::string, vpHomogeneousMatrix> init_cMos = sequence.getInitialPoses(tracker, initsFolder, modelsPath, objectNames);
      const double EPSILON_M = 0.002, EPSILON_DEG = 0.2;
      const std::map<std::string, std::vector<RunData>> configMap = {
        { "dragon", {
          RunData("ccd.json", 0.03, 10.0),
          RunData("ccd-temporal-smoothing.json", 0.03, 20.0),
          RunData("depth-ccd-mask.json", 0.025, 8.0),
          RunData("depth-ccd.json", 0.025, 8.0),
#ifdef VP_HAVE_RB_KLT_TRACKER
          RunData("depth-klt-ccd-mask.json", 0.02, 5.0),
          RunData("depth-klt.json", 0.02, 5.0),
          RunData("depth-klt-me-multi.json", 0.02, 5.0),
          RunData("depth-klt-me-single.json", 0.025, 10.0),
#endif
          }
        },
        { "cube", {
            RunData("ccd-temporal-smoothing.json", 0.03, 10.0),
            RunData("ccd.json", 0.03, 10.0),
            RunData("depth-ccd.json", 0.03, 10.0),
            RunData("depth-ccd-mask.json", 0.03, 10.0),
#ifdef VP_HAVE_RB_KLT_TRACKER
            RunData("depth-klt.json", 0.03, 10.0),
            RunData("depth-klt-ccd-mask.json", 0.03, 5.0),
            RunData("depth-klt-me-multi.json", 0.03, 5.0),
            RunData("depth-klt-me-single.json", 0.03, 10.0),
#endif
          }
        },
        { "stomach", {
            RunData("ccd-temporal-smoothing.json", 0.02, 10.0),
            RunData("ccd.json", 0.02, 4.0),
            RunData("depth-ccd.json", 0.025, 5.0),
            RunData("depth-ccd-mask.json", 0.03, 10.0),
#ifdef VP_HAVE_RB_KLT_TRACKER
            RunData("depth-klt-ccd-mask.json", 0.03, 5.0),
            RunData("depth-klt.json", 0.03, 3.0),
            RunData("depth-klt-me-multi.json", 0.03, 5.0),
            RunData("depth-klt-me-single.json", 0.03, 10.0),
#endif

          }
        },
        { "lower_teeth", {  } },
      };

      for (const std::string &objectName: objectNames) {

        const std::string modelFolder = vpIoTools::createFilePath(modelsPath, objectName);
        const std::string modelPath = vpIoTools::createFilePath(modelFolder, objectName + ".obj");
        tracker.setModelPath(modelPath);
        const auto configsIt = configMap.find(objectName);
        if (configsIt == configMap.end()) {
          std::cout << "No config found for " << objectName << std::endl;
          continue;
        }
        const std::vector<RunData> objectConfigs = configsIt->second;

        for (const RunData &runConfig: objectConfigs) {
          const std::string configName = runConfig.configName;
          std::cout << "Running tracker on object " <<  objectName << " with configuration " << configName << std::endl;

          const std::string configPath = vpIoTools::createFilePath(configsPath, configName);
          tracker.loadConfigurationFile(configPath);
          tracker.reset();
          tracker.startTracking();
          tracker.setPose(init_cMos.find(objectName)->second);
          std::vector<vpHomogeneousMatrix> poses;

          for (unsigned int i = 0; i < sequence.numFrames(); ++i) {
            SequenceFrame frame = sequence.getFrame(i);
            vpRBTrackingResult result = tracker.track(frame.I, frame.IRGB, frame.depth);
            vpHomogeneousMatrix cMo;
            tracker.getPose(cMo);
            poses.push_back(cMo);

            displayI = frame.I;
            displayRGB = frame.IRGB;
            for (unsigned int j = 0; j < frame.depth.getSize(); ++j) {
              displayDepth.bitmap[j] = static_cast<unsigned char>(std::min(frame.depth.bitmap[j], 1.f) * 255.f);
            }

            vpDisplay::display(displayI); vpDisplay::display(displayRGB); vpDisplay::display(displayDepth);
            tracker.display(displayI, displayRGB, displayDepth);
            vpDisplay::displayFrame(displayI, cMo, sequence.cam(), 0.05, vpColor::none);
            vpDisplay::displayFrame(displayI, sequence.getGroundTruthGridPose(i), sequence.cam(), 0.05, vpColor::yellow);

            tracker.displayMask(displayMask);
            vpDisplay::flush(displayI); vpDisplay::flush(displayRGB); vpDisplay::flush(displayDepth);
            vpDisplay::flush(displayMask);
          }
          const vpHomogeneousMatrix init_cMo = init_cMos.find(objectName)->second;
          const vpHomogeneousMatrix init_cMg = sequence.getGroundTruthGridPose(0);
          std::vector<vpHomogeneousMatrix> first_gMos;
          for (unsigned int i = 0; i < 10; ++i) {
            first_gMos.push_back(sequence.getGroundTruthGridPose(0).inverse() * poses[i]);
          }
          const vpHomogeneousMatrix gMo = vpHomogeneousMatrix::mean(first_gMos);
          std::vector<double> errorsT, errorsR;
          for (unsigned int i = 0; i < sequence.numFrames(); ++i) {
            const vpHomogeneousMatrix cMo_star = sequence.getGroundTruthGridPose(i) * gMo;
            const vpHomogeneousMatrix cMo = poses[i];
            const vpHomogeneousMatrix error = cMo_star.inverse() * cMo;
            const double errorT = error.getTranslationVector().frobeniusNorm();
            const double errorR = vpMath::deg(error.getThetaUVector().getTheta());
            errorsT.push_back(errorT);
            errorsR.push_back(errorR);
          }
          std::sort(errorsT.begin(), errorsT.end()); std::sort(errorsR.begin(), errorsR.end());
          unsigned int index90p = static_cast<unsigned int>(round(0.9 * errorsT.size()));
          double error90pT = errorsT[index90p], error90pR = errorsR[index90p];

          std::cout << error90pT << "m, " << error90pR << "°" << std::endl;

          if (error90pT > (runConfig.thresholdErrorT + EPSILON_M) || error90pR > (runConfig.thresholdErrorR + EPSILON_DEG)) {
            std::stringstream ss;
            ss << "Using object " << objectName << " with config " << configName << ":" << std::endl;
            ss << "\tMaximum tolerated median error:\t" << runConfig.thresholdErrorT << "m, " << runConfig.thresholdErrorR << "°" << std::endl;
            ss << "\tActual median error:\t\t\t" << error90pT << "m, " << error90pR << "°" << std::endl;

            FAIL(ss.str());
          }
        }

      }
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  auto cli = session.cli()
    | Catch::Clara::Opt(opt_no_display)["--no-display"]("Disable display");
  session.cli(cli);

  const int returnCode = session.applyCommandLine(argc, argv);
  if (returnCode != 0) { // Indicates a command line error
    return returnCode;
  }

  const int numFailed = session.run();
  vpPanda3DFrameworkManager::getInstance().exit();
  return numFailed;
}

#else

int main()
{
  return EXIT_SUCCESS;
}

#endif

#endif
