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

  void loadFrames()
  {
    unsigned int frameIndex = 0;
    while (true) {
      SequenceFrame frame;
      vpImage<vpRGBa> IRGB;
      vpImage<unsigned char> I;
      vpImage<float> depth;

      const std::string colorFramePath = getColorFrame(frameIndex);
      if (!vpIoTools::checkFilename(colorFramePath)) {
        break;
      }
      vpImageIo::read(frame.IRGB, colorFramePath);
      vpImageConvert::convert(frame.IRGB, frame.I);


      const std::string depthFramePath = getDepthFrame(frameIndex);
      if (vpIoTools::checkFilename(colorFramePath)) {
        visp::cnpy::NpyArray depth_data = visp::cnpy::npz_load(depthFramePath).find("data")->second;
        vpImage<uint16_t> depthRaw(depth_data.data<uint16_t>(), m_h, m_w, true);
        frame.depth.resize(m_h, m_w);
        for (unsigned int i = 0; i < m_h * m_w; ++i) {
          frame.depth.bitmap[i] = static_cast<float>(depthRaw.bitmap[i]) * m_depthScale;
        }
      }

      m_frames.push_back(frame);
    }

  }

private:
  std::string m_path;
  vpCameraParameters m_cam;
  float m_depthScale;
  unsigned int m_h, m_w;
  std::vector<SequenceFrame> m_frames;
  std::map<std::string, vpHomogeneousMatrix> m_initcMos;
  std::vector<vpHomogeneousMatrix> m_cMg;
};



SCENARIO("Running tracker on static synthetic sequences", "[rbt]")
{
  if (opt_no_display) {
    std::cout << "Display is disabled for tests, skipping..." << std::endl;
  }
  else {
    const std::string datasetPath = vpIoTools::getViSPImagesDataPath();
    const std::string rbtDatasetPath = vpIoTools::createFilePath(datasetPath, "rbt");

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
