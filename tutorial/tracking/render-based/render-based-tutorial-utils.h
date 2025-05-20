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
 */
#ifndef VP_RB_TRACKER_TUTORIAL_HELPER_H
#define VP_RB_TRACKER_TUTORIAL_HELPER_H

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

#include <fstream>
#include <math.h>
#include <string.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayFactory.h>

#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpJsonArgumentParser.h>
#include <visp3/io/vpVideoWriter.h>

#include <visp3/gui/vpPlot.h>

#include <visp3/rbt/vpRBTracker.h>
#include <visp3/rbt/vpObjectMask.h>
#include <visp3/rbt/vpRBDriftDetector.h>

#include "pStatClient.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace vpRBTrackerTutorial
{

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif


/**
 * \brief Base Argument parsing
 * Relies on vpJsonArgumentParser
*/
struct BaseArguments
{
  BaseArguments() : trackerConfiguration(""), maxDepthDisplay(1.f), verbose(false), display(true), debugDisplay(false), enableRenderProfiling(false) { }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addArgument("--tracker", trackerConfiguration, false, "Path to the JSON file containing the tracker")
      .addArgument("--object", object, false, "Name of the object to track. Used to potentially fetch the init file")
      .addArgument("--init-file", initFile, false, "Path to the JSON file containing the 2D/3D correspondences for initialization by click")
      .addArgument("--pose", inlineInit, false, "Initial pose of the object in the camera frame.")
      .addArgument("--max-depth-display", maxDepthDisplay, false, "Maximum depth value, used to scale the depth display")
      .addFlag("--verbose", verbose, "Log additional information in console")
      .addFlag("--no-display", display, "Disable display windows")
      .addFlag("--debug-display", debugDisplay, "Enable additional displays from the renderer")
      .addFlag("--profile", enableRenderProfiling, "Enable the use of Pstats to profile rendering times");
  }
#endif

  void postProcessArguments()
  {
    if (trackerConfiguration.empty()) {
      throw vpException(vpException::badValue, "No tracker configuration was specified");
    }
    if (object.empty()) {

      object = vpIoTools::getNameWE(trackerConfiguration);
    }
    else {
      modelPath = object;
      object = vpIoTools::getNameWE(modelPath);
    }
    if (initFile.empty()) {
      initFile = vpIoTools::getParent(modelPath.empty() ? trackerConfiguration : modelPath) + vpIoTools::separator + object + ".init";
    }

    if (!display && inlineInit.empty()) {
      throw vpException(vpException::badValue, "Cannot disable displays without specifying the initial pose");
    }
    if (inlineInit.size() > 0) {
      if (inlineInit.size() != 6) {
        throw vpException(vpException::dimensionError, "Inline pose initialization expected to have 6 values (tx, ty, tz, tux, tuy, tuz)");
      }
      for (unsigned int i = 0; i < 6; ++i) {
        std::cout << "inline i = " << inlineInit[i] << std::endl;
      }
      cMoInit = vpHomogeneousMatrix(inlineInit[0], inlineInit[1], inlineInit[2], inlineInit[3], inlineInit[4], inlineInit[5]);
    }
  }

  bool hasInlineInit()
  {
    return !inlineInit.empty();
  }

  std::string trackerConfiguration;
  std::string object;
  std::string modelPath;
  std::string initFile;
  std::vector<double> inlineInit;
  float maxDepthDisplay;
  vpHomogeneousMatrix cMoInit;
  bool verbose;
  bool display;
  bool debugDisplay;
  bool enableRenderProfiling;
};

/**
 * \brief Experiment data logger
 *
 * Relies on nlohmann json for output data logging and OpenCV when saving video
 *
 * Json will store the pose of the object in the camera frame (cMo)
 *
 *
*/
class vpRBExperimentLogger
{
public:
  vpRBExperimentLogger() : enabled(false), videoEnabled(false), framerate(30)
  { }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addFlag("--save", enabled, "Whether to save experiment data")
      .addArgument("--save-path", folder, false, "Where to save the experiment log. The folder should not exist.")
      .addFlag("--save-video", videoEnabled, "Whether to save the video")
      .addArgument("--video-framerate", framerate, false, "Output video framerate");
  }
#endif

  void startLog()
  {
    if (enabled) {
      if (folder.empty()) {
        throw vpException(vpException::badValue, "Experiment logging enabled but folder not specified");
      }
      vpIoTools::makeDirectory(folder);
      if (videoEnabled) {
#ifdef VISP_HAVE_OPENCV
        videoWriter.setFramerate(framerate);
        videoWriter.setCodec(cv::VideoWriter::fourcc('P', 'I', 'M', '1'));
        videoWriter.setFileName(folder + vpIoTools::separator + "video.mp4");
#endif
      }
    }
  }

  void logFrame(const vpRBTracker &
#if defined(VISP_HAVE_NLOHMANN_JSON)
                tracker
#endif
                , unsigned int iter, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB,
                const vpImage<unsigned char> &Idepth, const vpImage<unsigned char> &Imask)
  {
    if (videoEnabled) {
      Iout.resize(IRGB.getHeight() * 2, IRGB.getWidth() * 2);

      vpDisplay::getImage(I, IgrayOverlay);
      vpDisplay::getImage(IRGB, IColOverlay);
      vpDisplay::getImage(Idepth, IdepthOverlay);
      vpDisplay::getImage(Imask, ImaskOverlay);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (unsigned int i = 0; i < IRGB.getHeight(); ++i) {
        memcpy(Iout[i], IgrayOverlay[i], IRGB.getWidth() * sizeof(vpRGBa));
        memcpy(Iout[i] + IRGB.getWidth(), IColOverlay[i], IRGB.getWidth() * sizeof(vpRGBa));
        memcpy(Iout[i + IRGB.getHeight()], IdepthOverlay[i], IRGB.getWidth() * sizeof(vpRGBa));
        memcpy(Iout[i + IRGB.getHeight()] + IRGB.getWidth(), ImaskOverlay[i], IRGB.getWidth() * sizeof(vpRGBa));
      }

      if (iter == 1) {
        videoWriter.open(Iout);
      }
      else {
        videoWriter.saveFrame(Iout);
      }
    }
#if defined(VISP_HAVE_NLOHMANN_JSON)
    nlohmann::json iterLog;
    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);
    iterLog["cMo"] = cMo;

    log.push_back(iterLog);
#endif
  }

  void close()
  {
    if (videoEnabled) {
      videoWriter.close();
    }

#if defined(VISP_HAVE_NLOHMANN_JSON)
    std::ofstream f(folder + vpIoTools::separator + "log.json");
    f << log.dump(2) << std::endl;
    f.close();
#endif
  }

private:
  bool enabled;
  std::string folder;

  vpImage<vpRGBa> IColOverlay;
  vpImage<vpRGBa> IgrayOverlay;
  vpImage<vpRGBa> IdepthOverlay;
  vpImage<vpRGBa> ImaskOverlay;
  vpImage<vpRGBa> Iout;

  bool videoEnabled;
  unsigned int framerate;
  vpVideoWriter videoWriter;

#if defined(VISP_HAVE_NLOHMANN_JSON)
  nlohmann::json log;
#endif
};

/**
 * \brief Helper class to plot experiment data
 *
*/
class vpRBExperimentPlotter
{
public:

  vpRBExperimentPlotter() : enabled(false), plotPose(false), plotPose3d(false), plotDivergenceMetrics(false), plotCovariance(false) { }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addFlag("--plot-pose", plotPose, "Plot the pose of the object in the camera frame")
      .addFlag("--plot-position", plotPose3d, "Plot the position of the object in a 3d figure")
      .addFlag("--plot-divergence", plotDivergenceMetrics, "Plot the metrics associated to the divergence threshold computation")
      .addFlag("--plot-cov", plotCovariance, "Plot the pose covariance trace for each feature");
  }
#endif

  void postProcessArguments(bool displayEnabled)
  {
    enabled = plotPose || plotDivergenceMetrics || plotPose3d || plotCovariance;
    if (enabled && !displayEnabled) {
      throw vpException(vpException::badValue, "Tried to plot data, but display is disabled");
    }
  }

  void init(std::vector<std::shared_ptr<vpDisplay>> &displays)
  {
    if (!enabled) {
      return;
    }
    int ypos = 0, xpos = 0;
    for (std::shared_ptr<vpDisplay> &display : displays) {
      ypos = std::min(ypos, display->getWindowYPosition());
      xpos = std::max(xpos, display->getWindowXPosition() + static_cast<int>(display->getWidth()));
    }

    numPlots = static_cast<int>(plotPose) + static_cast<int>(plotDivergenceMetrics) + static_cast<int>(plotPose3d) + static_cast<int>(plotCovariance);
    plotter.init(numPlots, 600, 800, xpos, ypos, "Plot");
    unsigned int plotIndex = 0;
    if (plotPose) {
      plotter.initGraph(plotIndex, 6);
      plotter.setTitle(plotIndex, "cMo");
      std::vector<std::string> legends = {
        "tx", "ty", "tz", "tux", "tuy", "tuz"
      };
      for (unsigned int i = 0; i < 6; ++i) {
        plotter.setLegend(plotIndex, i, legends[i]);
      }
      plotter.setGraphThickness(plotIndex, 2);
      ++plotIndex;
    }
    if (plotPose3d) {
      plotter.initGraph(plotIndex, 1);
      plotter.setTitle(plotIndex, "3D object position");
      plotter.setGraphThickness(plotIndex, 2);
      ++plotIndex;
    }

    if (plotDivergenceMetrics) {
      plotter.initGraph(plotIndex, 1);
      plotter.initRange(plotIndex, 0.0, 1.0, 0.0, 1.0);
      plotter.setTitle(plotIndex, "Divergence");
      ++plotIndex;
    }
    if (plotCovariance) {
      plotter.initGraph(plotIndex, 2);
      plotter.setLegend(plotIndex, 0, "Translation trace standard deviation (cm)");
      plotter.setLegend(plotIndex, 1, "Rotation trace standard deviation (deg)");

      plotter.setTitle(plotIndex, "Covariance");
      ++plotIndex;
    }
  }

  void plot(const vpRBTracker &tracker, double time)
  {
    if (!enabled) {
      return;
    }
    unsigned int plotIndex = 0;
    if (plotPose) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      plotter.plot(plotIndex, time, vpPoseVector(cMo));
      ++plotIndex;
    }
    if (plotPose3d) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      vpTranslationVector t = cMo.getTranslationVector();
      plotter.plot(plotIndex, 0, t[0], t[1], t[2]);
      ++plotIndex;
    }
    if (plotDivergenceMetrics) {
      const std::shared_ptr<const vpRBDriftDetector> driftDetector = tracker.getDriftDetector();
      double metric = driftDetector ? driftDetector->getScore() : 0.0;
      plotter.plot(plotIndex, 0, time, metric);
      ++plotIndex;
    }
    if (plotCovariance) {
      vpMatrix cov = tracker.getCovariance();
      double traceTranslation = 0.0, traceRotation = 0.0;
      for (unsigned int i = 0; i < 3; ++i) {
        traceTranslation += cov[i][i];
        traceRotation += cov[i + 3][i + 3];
      }
      traceTranslation = sqrt(traceTranslation) * 100;
      traceRotation = vpMath::deg(sqrt(traceRotation));

      plotter.plot(plotIndex, 0, time, traceTranslation);
      plotter.plot(plotIndex, 1, time, traceRotation);

      ++plotIndex;
    }
  }
private:
  bool enabled;
  bool plotPose;
  bool plotPose3d;
  bool plotDivergenceMetrics;
  bool plotCovariance;
  int numPlots;
  vpPlot plotter;
};


/**
 * \brief Create a grid of displays for the RBT
 *
 * \param Id Grayscale image
 * \param Icol Color image
 * \param depthDisplay Displayable depth image
 * \param probaDisplay Displayable grayscale image for the mask
 * \return std::vector<std::shared_ptr<vpDisplay>> a list of displays that should be kept around to display tracking iteration data
*/
std::vector<std::shared_ptr<vpDisplay>> createDisplays(
  vpImage<unsigned char> &Id, vpImage<vpRGBa> &Icol,
  vpImage<unsigned char> &depthDisplay, vpImage<unsigned char> &probaDisplay)
{
  return vpDisplayFactory::makeDisplayGrid(
      2, 2,
      0, 0,
      80, 80,
      "Grayscale", Id,
      "Color", Icol,
      "Depth", depthDisplay,
      "Proba mask", probaDisplay
  );
}

std::vector<std::shared_ptr<vpDisplay>> createDisplays(
  vpImage<unsigned char> &Id, vpImage<vpRGBa> &Icol, vpImage<unsigned char> &probaDisplay)
{
  return vpDisplayFactory::makeDisplayGrid(
      1, 3,
      0, 0,
      80, 80,
      "Grayscale", Id,
      "Color", Icol,
      "Proba mask", probaDisplay
  );
}

/**
 * \brief Helper to profile render times in Panda3D (these timings may be unreliable when gl-finish is false in Panda3D config)
*/
void enableRendererProfiling()
{
  if (PStatClient::is_connected()) {
    PStatClient::disconnect();
  }

  std::string host = ""; // Empty = default config var value
  int port = -1; // -1 = default config var value
  if (!PStatClient::connect(host, port)) {
    std::cout << "Could not connect to PStat server." << std::endl;
  }
}

/**
 * \brief Method to convert a raw normals image into a vpRGBa image, essentially remapping from (-1, 1) to (0, 255)
 *
 * \param normalsImage the raw normal image
 * \param normalDisplayImage the output image, that can be displayed
*/
void displayNormals(const vpImage<vpRGBf> &normalsImage, vpImage<vpRGBa> &normalDisplayImage)
{
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < normalsImage.getSize(); ++i) {
    normalDisplayImage.bitmap[i].R = static_cast<unsigned char>((normalsImage.bitmap[i].R + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].G = static_cast<unsigned char>((normalsImage.bitmap[i].G + 1.0) * 127.5f);
    normalDisplayImage.bitmap[i].B = static_cast<unsigned char>((normalsImage.bitmap[i].B + 1.0) * 127.5f);
  }

  vpDisplay::display(normalDisplayImage);
  vpDisplay::flush(normalDisplayImage);
}

/**
 * \brief Display the depth canny information
 *
 * The arrows represent the 2D normal of the contour
 *
 * \param cannyRawData
 * \param canny
 * \param valid
*/
void displayCanny(const vpImage<vpRGBf> &cannyRawData,
                  vpImage<unsigned char> &canny, const vpImage<unsigned char> &valid)
{
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < cannyRawData.getSize(); ++i) {
    //vpRGBf &px = cannyRawData.bitmap[i];
    canny.bitmap[i] = valid.bitmap[i] * 255;
    //canny.bitmap[i] = static_cast<unsigned char>(127.5f + 127.5f * atan(px.B));
  }

  vpDisplay::display(canny);
  for (unsigned int i = 0; i < canny.getHeight(); i += 4) {
    for (unsigned int j = 0; j < canny.getWidth(); j += 4) {
      if (!valid[i][j]) continue;
      float angle = cannyRawData[i][j].B;
      unsigned x = j + 10 * cos(angle);
      unsigned y = i + 10 * sin(angle);
      vpDisplay::displayArrow(canny, i, j, y, x, vpColor::green);
    }
  }
  vpDisplay::flush(canny);
}
}
#endif
#endif
