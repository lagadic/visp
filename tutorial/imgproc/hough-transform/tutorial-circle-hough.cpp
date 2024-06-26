//! \example tutorial-circle-hough.cpp

#include <iostream>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/imgproc/vpCircleHoughTransform.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>

#include "drawingHelpers.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool run_detection(const vpImage<unsigned char> &I_src, vpCircleHoughTransform &detector, const int &nbCirclesToDetect, const bool &blockingMode, const bool &displayCanny)
{
  double t0 = vpTime::measureTimeMicros();
  //! [Run detection]
  std::vector<vpImageCircle> detectedCircles = detector.detect(I_src, nbCirclesToDetect);
  std::vector<float> probas = detector.getDetectionsProbabilities();
  //! [Run detection]
  double tF = vpTime::measureTimeMicros();
  std::cout << "Process time = " << (tF - t0) * 0.001 << "ms" << std::endl << std::flush;
  vpImage<vpRGBa> I_disp;
  vpImageConvert::convert(I_src, I_disp);

  unsigned int id = 0;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::vector<vpColor> v_colors = { vpColor::red, vpColor::purple, vpColor::orange, vpColor::yellow, vpColor::blue };
#else
  std::vector<vpColor> v_colors;
  v_colors.push_back(vpColor::red);
  v_colors.push_back(vpColor::purple);
  v_colors.push_back(vpColor::orange);
  v_colors.push_back(vpColor::yellow);
  v_colors.push_back(vpColor::blue);
#endif
  unsigned int idColor = 0;
  //! [Iterate detections]
  const unsigned int nbCircle = static_cast<unsigned int>(detectedCircles.size());
  for (unsigned int idCircle = 0; idCircle < nbCircle; ++idCircle) {
    const vpImageCircle &circleCandidate = detectedCircles[idCircle];
    vpImageDraw::drawCircle(I_disp, circleCandidate, v_colors[idColor], 2);
    std::cout << "Circle #" << id << ":" << std::endl;
    std::cout << "\tCenter: (" << circleCandidate.getCenter() << ")" << std::endl;
    std::cout << "\tRadius: (" << circleCandidate.getRadius() << ")" << std::endl;
    std::cout << "\tProba: " << probas[id] << std::endl;
    std::cout << "\tTheoretical arc length: " << circleCandidate.computeArcLengthInRoI(vpRect(0, 0, I_src.getWidth(), I_src.getHeight())) << std::endl;
    id++;
    idColor = (idColor + 1) % v_colors.size();
  }
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  std::optional<vpImage<bool>> opt_mask = std::nullopt;
  std::optional<std::vector<std::vector<std::pair<unsigned int, unsigned int>>>> opt_votingPoints = std::nullopt;
#elif (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpImage<bool> *opt_mask = nullptr;
  std::vector<std::vector<std::pair<unsigned int, unsigned int>>> *opt_votingPoints = nullptr;
  detector.computeVotingMask(I_src, detectedCircles, &opt_mask, &opt_votingPoints); // Get, if available, the voting points
#else
  vpImage<bool> *opt_mask = NULL;
  std::vector<std::vector<std::pair<unsigned int, unsigned int> > > *opt_votingPoints = NULL;
  detector.computeVotingMask(I_src, detectedCircles, &opt_mask, &opt_votingPoints); // Get, if available, the voting points
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  if (opt_votingPoints)
#elif (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  if (opt_votingPoints != nullptr)
#else
  if (opt_votingPoints != NULL)
#endif
  {
    const unsigned int crossSize = 3;
    const unsigned int crossThickness = 1;
    unsigned int nbVotedCircles = static_cast<unsigned int>(opt_votingPoints->size());
    for (unsigned int idCircle = 0; idCircle < nbVotedCircles; ++idCircle) {
      // Get the voting points of a detected circle
      const std::vector<std::pair<unsigned int, unsigned int> > &votingPoints = (*opt_votingPoints)[idCircle];
      unsigned int nbVotingPoints = static_cast<unsigned int>(votingPoints.size());
      for (unsigned int idPoint = 0; idPoint < nbVotingPoints; ++idPoint) {
        // Draw the voting points
        const std::pair<unsigned int, unsigned int> &pt = votingPoints[idPoint];
        vpImageDraw::drawCross(I_disp, vpImagePoint(pt.first, pt.second), crossSize, vpColor::red, crossThickness);
      }
    }
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_17)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  if (opt_mask != nullptr)
#else
  if (opt_mask != NULL)
#endif
  {
    delete opt_mask;
  }
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  if (opt_votingPoints != nullptr)
#else
  if (opt_votingPoints != NULL)
#endif
  {
    delete opt_votingPoints;
  }
#endif
//! [Iterate detections]

  if (displayCanny) {
    vpImage<unsigned char> edgeMap = detector.getEdgeMap();
    drawingHelpers::display(edgeMap, "Edge map", true);
  }
  return drawingHelpers::display(I_disp, "Detection results", blockingMode);
}

int main(int argc, char **argv)
{
  const std::string def_input("coins2.jpg");
  const std::string def_jsonFilePath = std::string("");
  const int def_nbCirclesToDetect = -1;
  const int def_gaussianKernelSize = 5;
  const float def_gaussianSigma = 1.f;
  const int def_sobelKernelSize = 3;
  const float def_lowerCannyThresh = -1.f;
  const float def_upperCannyThresh = -1.f;
  const int def_nbEdgeFilteringIter = 3;
  const std::pair<int, int> def_centerXlimits = std::pair<int, int>(0, 1920);
  const std::pair<int, int> def_centerYlimits = std::pair<int, int>(0, 1080);
  const unsigned int def_minRadius = 34;
  const unsigned int def_maxRadius = 75;
  const int def_dilatationKernelSize = 5;
  const float def_centerThresh = 70.f;
  const float def_circleProbaThresh = 0.725f;
  const float def_circlePerfectness = 0.85f;
  const float def_centerDistanceThresh = 5.f;
  const float def_radiusDifferenceThresh = 5.f;
  const int def_averagingWindowSize = 5;
  const vpImageFilter::vpCannyFilteringAndGradientType def_filteringAndGradientType = vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING;
  const vpImageFilter::vpCannyBackendType def_cannyBackendType = vpImageFilter::CANNY_OPENCV_BACKEND;
  const float def_lowerCannyThreshRatio = 0.6f;
  const float def_upperCannyThreshRatio = 0.9f;
  const int def_expectedNbCenters = -1;
  const bool def_recordVotingPoints = false;
  const float def_visibilityRatioThresh = 0.1f;

  std::string opt_input(def_input);
  std::string opt_jsonFilePath = def_jsonFilePath;
  int opt_nbCirclesToDetect = def_nbCirclesToDetect;
  int opt_gaussianKernelSize = def_gaussianKernelSize;
  float opt_gaussianSigma = def_gaussianSigma;
  int opt_sobelKernelSize = def_sobelKernelSize;
  float opt_lowerCannyThresh = def_lowerCannyThresh;
  float opt_upperCannyThresh = def_upperCannyThresh;
  int opt_nbEdgeFilteringIter = def_nbEdgeFilteringIter;
  std::pair<int, int> opt_centerXlimits = def_centerXlimits;
  std::pair<int, int> opt_centerYlimits = def_centerYlimits;
  unsigned int opt_minRadius = def_minRadius;
  unsigned int opt_maxRadius = def_maxRadius;
  int opt_dilatationKerneSize = def_dilatationKernelSize;
  float opt_centerThresh = def_centerThresh;
  float opt_circleProbaThresh = def_circleProbaThresh;
  float opt_circlePerfectness = def_circlePerfectness;
  float opt_centerDistanceThresh = def_centerDistanceThresh;
  float opt_radiusDifferenceThresh = def_radiusDifferenceThresh;
  int opt_averagingWindowSize = def_averagingWindowSize;
  vpImageFilter::vpCannyFilteringAndGradientType opt_filteringAndGradientType = def_filteringAndGradientType;
  vpImageFilter::vpCannyBackendType opt_cannyBackendType = def_cannyBackendType;
  float opt_lowerCannyThreshRatio = def_lowerCannyThreshRatio;
  float opt_upperCannyThreshRatio = def_upperCannyThreshRatio;
  int opt_expectedNbCenters = def_expectedNbCenters;
  bool opt_recordVotingPoints = def_recordVotingPoints;
  float opt_visibilityRatioThresh = def_visibilityRatioThresh;
  bool opt_displayCanny = false;

  for (int i = 1; i < argc; i++) {
    std::string argName(argv[i]);
    if (argName == "--input" && i + 1 < argc) {
      opt_input = std::string(argv[i + 1]);
      i++;
    }
#ifdef VISP_HAVE_NLOHMANN_JSON
    else if (argName == "--config" && i + 1 < argc) {
      opt_jsonFilePath = std::string(argv[i + 1]);
      i++;
    }
#endif
    else if (argName == "--nb-circles" && i + 1 < argc) {
      opt_nbCirclesToDetect = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--gaussian-kernel" && i + 1 < argc) {
      opt_gaussianKernelSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--gaussian-sigma" && i + 1 < argc) {
      opt_gaussianSigma = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--gradient-kernel" && i + 1 < argc) {
      opt_sobelKernelSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--canny-thresh" && i + 2 < argc) {
      opt_lowerCannyThresh = static_cast<float>(atof(argv[i + 1]));
      opt_upperCannyThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--edge-filter" && i + 1 < argc) {
      opt_nbEdgeFilteringIter = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--dilatation-kernel-size" && i + 1 < argc) {
      opt_dilatationKerneSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--averaging-window-size" && i + 1 < argc) {
      opt_averagingWindowSize = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--radius-limits" && i + 2 < argc) {
      opt_minRadius = atoi(argv[i + 1]);
      opt_maxRadius = atoi(argv[i + 2]);
      i += 2;
    }
    else if (argName == "--center-thresh" && i + 1 < argc) {
      opt_centerThresh = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--center-xlim" && i + 2 < argc) {
      opt_centerXlimits = std::pair<int, int>(atoi(argv[i + 1]), atoi(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--center-ylim" && i + 2 < argc) {
      opt_centerYlimits = std::pair<int, int>(atoi(argv[i + 1]), atoi(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--circle-probability-thresh" && i + 1 < argc) {
      opt_circleProbaThresh = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--circle-perfectness" && i + 1 < argc) {
      opt_circlePerfectness = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--merging-thresh" && i + 2 < argc) {
      opt_centerDistanceThresh = static_cast<float>(atof(argv[i + 1]));
      opt_radiusDifferenceThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if (argName == "--filtering-type" && i + 1 < argc) {
      opt_filteringAndGradientType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(std::string(argv[i+1]));
      i++;
    }
    else if (argName == "--canny-backend" && i + 1 < argc) {
      opt_cannyBackendType = vpImageFilter::vpCannyBackendTypeFromString(std::string(argv[i+1]));
      i++;
    }
    else if (argName == "--lower-canny-ratio" && i + 1 < argc) {
      opt_lowerCannyThreshRatio = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--upper-canny-ratio" && i + 1 < argc) {
      opt_upperCannyThreshRatio = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--expected-nb-centers" && i + 1 < argc) {
      opt_expectedNbCenters = atoi(argv[i + 1]);
      i++;
    }
    else if (argName == "--visibility-ratio-thresh" && i + 1 < argc) {
      opt_visibilityRatioThresh = static_cast<float>(atof(argv[i + 1]));
      i++;
    }
    else if (argName == "--record-voting-points") {
      opt_recordVotingPoints = true;
    }
    else if (argName == "--display-edge-map") {
      opt_displayCanny = true;
    }
    else if (argName == "--help" || argName == "-h") {
      std::cout << "NAME" << std::endl;
      std::cout << "\t" << argv[0] << "  Test program for the home-made Hough Circle Detection algorithm" << std::endl
        << std::endl;
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "\t" << argv[0]
        << "\t [--input <path/to/file>]" << std::endl
#ifdef VISP_HAVE_NLOHMANN_JSON
        << "\t [--config <path/to/json/file>] (default: " << (def_jsonFilePath.empty() ? "unused" : def_jsonFilePath) << ")" << std::endl
#endif
        << "\t [--nb-circles <number-circles-to-detect>] (default: " << def_nbCirclesToDetect << ")" << std::endl
        << "\t [--gaussian-kernel <kernel-size>] (default: " << def_gaussianKernelSize << ")" << std::endl
        << "\t [--gaussian-sigma <stddev>] (default: " << def_gaussianSigma << ")" << std::endl
        << "\t [--gradient-kernel <kernel-size>] (default: " << def_sobelKernelSize << ")" << std::endl
        << "\t [--canny-thresh <lower-canny-thresh upper-canny-thresh>] (default: " << def_lowerCannyThresh << " ; " << def_upperCannyThresh << ")" << std::endl
        << "\t [--edge-filter <nb-iter>] (default: " << def_nbEdgeFilteringIter << ")" << std::endl
        << "\t [--radius-limits <radius-min> <radius-max>] (default: min = " << def_minRadius << ", max = " << def_maxRadius << ")" << std::endl
        << "\t [--dilatation-kernel-size <kernel-size>] (default: " << def_dilatationKernelSize << ")" << std::endl
        << "\t [--averaging-window-size <size>] (default: " << def_averagingWindowSize << ")" << std::endl
        << "\t [--center-thresh <center-detection-threshold>] (default: " << def_centerThresh << ")" << std::endl
        << "\t [--center-xlim <center-horizontal-min center-horizontal-max>] (default: " << def_centerXlimits.first << " , " << def_centerXlimits.second  << ")" << std::endl
        << "\t [--center-ylim <center-vertical-min center-vertical-max>] (default: " << def_centerYlimits.first << " , " << def_centerYlimits.second  << ")" << std::endl
        << "\t [--circle-probability-thresh <probability-threshold>] (default: " << def_circleProbaThresh  << ")" << std::endl
        << "\t [--circle-perfectness <circle-perfectness-threshold>] (default: " << def_circlePerfectness << ")" << std::endl
        << "\t [--merging-thresh <center-distance-thresh> <radius-difference-thresh>] (default: centers distance threshold = " << def_centerDistanceThresh << ", radius difference threshold = " << def_radiusDifferenceThresh << ")" << std::endl
        << "\t [--filtering-type <type-name>]"
        << " (default: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(def_filteringAndGradientType) << ")" << std::endl
        << "\t [--canny-backend <backend-name>]"
        << " (default: " << vpImageFilter::vpCannyBackendTypeToString(def_cannyBackendType) << ")" << std::endl
        << "\t [--lower-canny-ratio <value>]"
        << " (default: " << def_lowerCannyThreshRatio<< ")" << std::endl
        << "\t [--upper-canny-ratio <value>]"
        << " (default: " << def_upperCannyThreshRatio << ")" << std::endl
        << "\t [--expected-nb-centers <number>]"
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
        << " (default: " << (def_expectedNbCenters < 0 ? "no limits" : std::to_string(def_expectedNbCenters)) << ")" << std::endl
#else
        << std::endl
#endif
        << "\t [--visibility-ratio-thresh <ratio ]0; 1[> ]"
        << " (default: " << def_visibilityRatioThresh << ")" << std::endl
        << "\t [--record-voting-points]" << std::endl
        << "\t [--display-edge-map]" << std::endl
        << "\t [--help, -h]" << std::endl
        << std::endl;

      std::cout << "DESCRIPTION" << std::endl
        << "\t--input" << std::endl
        << "\t\tPermit to choose the input of the Hough Circle Algorithm." << std::endl
        << "\t\tIf you want to use a succession of images as video, their name must be in the format ${BASENAME}%d.{jpg, png}." << std::endl
        << "\t\tDefault: " << def_input << std::endl
        << std::endl
#ifdef VISP_HAVE_NLOHMANN_JSON
        << "\t--config" << std::endl
        << "\t\tPermit to configure the Hough Circle Algorithm using a JSON file." << std::endl
        << "\t\tDefault: " << (def_jsonFilePath.empty() ? "unused" : def_jsonFilePath) << std::endl
        << std::endl
#endif
        << "\t--nb-circles" << std::endl
        << "\t\tPermit to choose the number of circles we want to detect in the image" << std::endl
        << "\t\tThe results will be the circles having the greatest number of votes." << std::endl
        << "\t\tDefault: " << def_nbCirclesToDetect << std::endl
        << std::endl
        << "\t--gaussian-kernel" << std::endl
        << "\t\tPermit to set the size of the Gaussian filter used to smooth the input image and compute its gradients." << std::endl
        << "\t\tMust be an odd value." << std::endl
        << "\t\tDefault: " << def_gaussianKernelSize << std::endl
        << std::endl
        << "\t--gaussian-sigma" << std::endl
        << "\t\tPermit to set the standard deviation of the Gaussian filter." << std::endl
        << "\t\tMust be a positive value." << std::endl
        << "\t\tDefault: " << def_gaussianSigma << std::endl
        << std::endl
        << "\t--gradient-kernel" << std::endl
        << "\t\tPermit to set the size of the Gaussian filter used to smooth the input image and compute its gradients." << std::endl
        << "\t\tMust be an odd value." << std::endl
        << "\t\tDefault: " << def_gaussianKernelSize << std::endl
        << std::endl
        << "\t--canny-thresh" << std::endl
        << "\t\tPermit to set the lower and upper thresholds of the Canny edge detector." << std::endl
        << "\t\tIf a value is negative, it will be automatically computed." << std::endl
        << "\t\tDefault: " << def_lowerCannyThresh << " ; " << def_upperCannyThresh << std::endl
        << std::endl
        << "\t--edge-filter" << std::endl
        << "\t\tPermit to set the number of iteration of 8-neighbor filter iterations of the result of the Canny edge detector." << std::endl
        << "\t\tIf negative, no filtering is performed." << std::endl
        << "\t\tDefault: " << def_nbEdgeFilteringIter << std::endl
        << std::endl
        << "\t--radius-limits" << std::endl
        << "\t\tPermit to set the minimum and maximum radii of the circles we are looking for." << std::endl
        << "\t\tDefault: min = " << def_minRadius << ", max = " << def_maxRadius << std::endl
        << std::endl
        << "\t--dilatation-kernel-size" << std::endl
        << "\t\tPermit to set the size of the kernel of the dilatation operation used to detect the maxima of the centers votes." << std::endl
        << "\t\tMinimum tolerated value is 1." << std::endl
        << "\t\tDefault: " << def_dilatationKernelSize << std::endl
        << std::endl
        << "\t--averaging-window-size" << std::endl
        << "\t\tPermit to set the number size of the averaging window used to detect the maxima of the centers votes." << std::endl
        << "\t\tMust be odd." << std::endl
        << "\t\tDefault: " << def_averagingWindowSize << std::endl
        << std::endl
        << "\t--center-thresh" << std::endl
        << "\t\tPermit to set the minimum number of votes a point must reach to be considered as a center candidate." << std::endl
        << "\t\tIf the input is a real image, must be a positive value." << std::endl
        << "\t\tOtherwise, if the input is a synthetic image and the value is negative, a fine-tuned value will be used." << std::endl
        << "\t\tDefault: " << def_centerThresh << std::endl
        << std::endl
        << "\t--center-xlim" << std::endl
        << "\t\tPermit to set the minimum and maximum horizontal position to be considered as a center candidate." << std::endl
        << "\t\tThe search area is limited to [-maxRadius; +image.width + maxRadius]." << std::endl
        << "\t\tDefault: " << def_centerXlimits.first << " , " << def_centerXlimits.second << std::endl
        << std::endl
        << "\t--center-ylim" << std::endl
        << "\t\tPermit to set the minimum and maximum vertical position to be considered as a center candidate." << std::endl
        << "\t\tThe search area is limited to [-maxRadius; +image.height + maxRadius]." << std::endl
        << "\t\tDefault: " << def_centerYlimits.first << " , " << def_centerYlimits.second << std::endl
        << std::endl
        << "\t--circle-probability-thresh" << std::endl
        << "\t\tPermit to to set the minimum probability a circle must reach to be kept." << std::endl
        << "\t\tDefault: " << def_circleProbaThresh << std::endl
        << std::endl
        << "\t--circle-perfectness" << std::endl
        << "\t\tPermit to set the set the circle perfectness threshold." << std::endl
        << "\t\tThis parameter is used during the radius candidates computation." << std::endl
        << "\t\tThe scalar product radius RC_ij . gradient(Ep_j) >=  m_circlePerfectness * || RC_ij || * || gradient(Ep_j) || to add a vote for the radius RC_ij." << std::endl
        << "\t\tDefault: " << def_circlePerfectness << std::endl
        << std::endl
        << "\t--merging-thresh" << std::endl
        << "\t\tPermit to set the thresholds used during the merging stage of the algorithm." << std::endl
        << "\t\tThe center distance threshold indicates the maximum distance the centers can be in order to be merged." << std::endl
        << "\t\tThe radius difference threshold indicates the maximum absolute difference between the two circle candidates in order to be merged." << std::endl
        << "\t\tTwo circle candidates must met these two conditions in order to be merged together." << std::endl
        << "\t\tDefault: centers distance threshold = " << def_centerDistanceThresh << ", radius difference threshold = " << def_radiusDifferenceThresh << std::endl
        << std::endl
        << "\t--filtering-type" << std::endl
        << "\t\tPermit to choose the gradient filters." << std::endl
        << "\t\tDefault: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(def_filteringAndGradientType) << ", available: " << vpImageFilter::vpGetCannyFiltAndGradTypes() << std::endl
        << std::endl
        << "\t--canny-backend" << std::endl
        << "\t\tPermit to choose the backend used to compute the edge map." << std::endl
        << "\t\tDefault: " << vpImageFilter::vpCannyBackendTypeToString(def_cannyBackendType) << ", available: " << vpImageFilter::vpCannyBackendTypeList() << std::endl
        << std::endl
        << "\t--lower-canny-ratio" << std::endl
        << "\t\tPermit to choose the ratio for the lower threshold if automatic thresholding is chosen." << std::endl
        << "\t\tDefault: " << def_lowerCannyThreshRatio << std::endl
        << std::endl
        << "\t--upper-canny-ratio" << std::endl
        << "\t\tPermit to choose the ratio for the upper threshold if automatic thresholding is chosen." << std::endl
        << "\t\tDefault: " << def_upperCannyThreshRatio << std::endl
        << std::endl
        << "\t--expected-nb-centers" << std::endl
        << "\t\tPermit to choose the maximum number of centers having more votes than the threshold that are kept." << std::endl
        << "\t\tA negative value makes that all the centers having more votes than the threshold are kept." << std::endl
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
        << "\t\tDefault: " << (def_expectedNbCenters < 0 ? "no limits" : std::to_string(def_expectedNbCenters)) << std::endl
#else
        << std::endl
#endif
        << std::endl
        << "\t--expected-nb-centers" << std::endl
        << "\t\tPermit to choose the maximum number of centers having more votes than the threshold that are kept." << std::endl
        << "\t\tA negative value makes that all the centers having more votes than the threshold are kept." << std::endl
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
        << "\t\tDefault: " << (def_expectedNbCenters < 0 ? "no limits" : std::to_string(def_expectedNbCenters)) << std::endl
#else
        << std::endl
#endif
        << std::endl
        << "\t--record-voting-points" << std::endl
        << "\t\tPermit to display the edge map used to detect the circles" << std::endl
        << "\t\tDefault: off" << std::endl
        << std::endl
        << "\t--display-edge-map" << std::endl
        << "\t\tPermit to display the edge map used to detect the circles" << std::endl
        << "\t\tDefault: off" << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Algo params]
  vpCircleHoughTransform::vpCircleHoughTransformParams
    algoParams(opt_gaussianKernelSize
      , opt_gaussianSigma
      , opt_sobelKernelSize
      , opt_lowerCannyThresh
      , opt_upperCannyThresh
      , opt_nbEdgeFilteringIter
      , opt_centerXlimits
      , opt_centerYlimits
      , static_cast<float>(opt_minRadius)
      , static_cast<float>(opt_maxRadius)
      , opt_dilatationKerneSize
      , opt_averagingWindowSize
      , opt_centerThresh
      , opt_circleProbaThresh
      , opt_circlePerfectness
      , opt_centerDistanceThresh
      , opt_radiusDifferenceThresh
      , opt_filteringAndGradientType
      , opt_cannyBackendType
      , opt_lowerCannyThreshRatio
      , opt_upperCannyThreshRatio
      , opt_expectedNbCenters
      , opt_recordVotingPoints
      , opt_visibilityRatioThresh
    );
  //! [Algo params]

  //! [Algo init]
  vpCircleHoughTransform detector;
  if (opt_jsonFilePath.empty()) {
    std::cout << "Initializing detector from the program arguments [...]" << std::endl;
    detector.init(algoParams);
  }
  else {
#ifdef VISP_HAVE_NLOHMANN_JSON
    std::cout << "Initializing detector from JSON file \"" << opt_jsonFilePath << "\", some of the program arguments will be ignored [...]" << std::endl;
    detector.initFromJSON(opt_jsonFilePath);
#else
    throw(vpException(vpException::functionNotImplementedError, "You must install nlohmann JSON library to use this feature, see https://visp-doc.inria.fr/doxygen/visp-daily/supported-third-parties.html#soft_tool_json for more information."));
#endif
  }
  //! [Algo init]
  std::cout << detector;

  vpImage<unsigned char> I_src;

  //! [Manage video]
  if (opt_input.find("%") != std::string::npos) {
    // The user wants to read a sequence of images from different files
    bool hasToContinue = true;
    vpVideoReader g;
    g.setFileName(opt_input);
    g.open(I_src);
    while (!g.end() && hasToContinue) {
      g.acquire(I_src);
      hasToContinue = run_detection(I_src, detector, opt_nbCirclesToDetect, false, opt_displayCanny);
      vpTime::wait(40);
    }
  }
  //! [Manage video]
  else {
    //! [Manage single image]
    // Check if opt_input exists
    if (!vpIoTools::checkFilename(opt_input)) {
      throw(vpException(vpException::ioError, "Input file \"" + opt_input + "\" does not exist !"));
    }
    // Read the image and perform detection on it
    vpImageIo::read(I_src, opt_input);
    run_detection(I_src, detector, opt_nbCirclesToDetect, true, opt_displayCanny);
    //! [Manage single image]
  }

  return EXIT_SUCCESS;
}
