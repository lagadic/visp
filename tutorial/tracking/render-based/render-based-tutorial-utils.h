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

struct BaseArguments
{
  BaseArguments() : trackerConfiguration(""), maxDepthDisplay(1.f), display(true), debugDisplay(false), enableRenderProfiling(false) { }

  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addArgument("--tracker", trackerConfiguration, false, "Path to the JSON file containing the tracker")
      .addArgument("--object", object, false, "Name of the object to track. Used to potentially fetch the init file")
      .addArgument("--init-file", initFile, false, "Path to the JSON file containing the 2D/3D correspondences for initialization by click")
      .addArgument("--pose", inlineInit, false, "Initial pose of the object in the camera frame.")
      .addArgument("--max-depth-display", maxDepthDisplay, false, "Maximum depth value, used to scale the depth display")
      .addFlag("--no-display", display, "Disable display windows")
      .addFlag("--debug-display", debugDisplay, "Enable additional displays from the renderer")
      .addFlag("--profile", enableRenderProfiling, "Enable the use of Pstats to profile rendering times");
  }

  void postProcessArguments()
  {
    if (trackerConfiguration.empty()) {
      throw vpException(vpException::badValue, "No tracker configuration was specified");
    }
    if (object.empty()) {
      object = vpIoTools::getName(trackerConfiguration);
      object.erase(object.end() - 5, object.end());
    }
    if (initFile.empty()) {
      initFile = vpIoTools::getParent(trackerConfiguration) + vpIoTools::separator + object + ".init";
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
  std::string initFile;
  std::vector<double> inlineInit;
  float maxDepthDisplay;
  vpHomogeneousMatrix cMoInit;
  bool display;
  bool debugDisplay;
  bool enableRenderProfiling;
};

class vpRBExperimentLogger
{
public:
  vpRBExperimentLogger() : enabled(false), videoEnabled(false), framerate(30)
  { }

  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addFlag("--save", enabled, "Whether to save experiment data")
      .addArgument("--save-path", folder, false, "Where to save the experiment log. The folder should not exist.")
      .addFlag("--save-video", videoEnabled, "Whether to save the video")
      .addArgument("--video-framerate", framerate, false, "Output video framerate");

  }

  void startLog()
  {
    if (enabled) {
      if (folder.empty()) {
        throw vpException(vpException::badValue, "Experiment logging enabled but folder not specified");
      }
      vpIoTools::makeDirectory(folder);
      if (videoEnabled) {
        videoWriter.setFramerate(framerate);
        videoWriter.setCodec(cv::VideoWriter::fourcc('P', 'I', 'M', '1'));
        videoWriter.setFileName(folder + vpIoTools::separator + "video.mp4");
      }
    }
  }

  void logFrame(const vpRBTracker &tracker, unsigned int iter, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &Idepth, const vpImage<unsigned char> &Imask)
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

    nlohmann::json iterLog;
    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);
    iterLog["cMo"] = cMo;

    log.push_back(iterLog);
  }

  void close()
  {
    if (videoEnabled) {
      videoWriter.close();
    }
    std::ofstream f(folder + vpIoTools::separator + "log.json");
    f << log.dump(2) << std::endl;
    f.close();
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

  nlohmann::json log;
};

class vpRBExperimentPlotter
{
public:

  vpRBExperimentPlotter() : enabled(false), plotPose(false), plotPose3d(false), plotDivergenceMetrics(false) { }

  void registerArguments(vpJsonArgumentParser &parser)
  {
    parser
      .addFlag("--plot-pose", plotPose, "Plot the pose of the object in the camera frame")
      .addFlag("--plot-position", plotPose3d, "Plot the position of the object in a 3d figure")
      .addFlag("--plot-divergence", plotDivergenceMetrics, "Plot the metrics associated to the divergence threshold computation");
  }

  void postProcessArguments(bool displayEnabled)
  {
    enabled = plotPose || plotDivergenceMetrics || plotPose3d;
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

    numPlots = static_cast<int>(plotPose) + static_cast<int>(plotDivergenceMetrics) + static_cast<int>(plotPose3d);
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
  }
private:
  bool enabled;
  bool plotPose;
  bool plotPose3d;

  bool plotDivergenceMetrics;
  int numPlots;
  vpPlot plotter;
};

std::vector<std::shared_ptr<vpDisplay>> createDisplays(
  vpImage<unsigned char> &Id, vpImage<vpRGBa> &Icol,
  vpImage<unsigned char> &depthDisplay, vpImage<unsigned char> &probaDisplay)
{
  return vpDisplayFactory::makeDisplayGrid(
      2, 2,
      0, 0,
      20, 40,
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
      20, 40,
      "Grayscale", Id,
      "Color", Icol,
      "Proba mask", probaDisplay
  );
}

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
