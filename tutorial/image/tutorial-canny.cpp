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
 */

//! \example tutorial-canny.cpp

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/io/vpImageIo.h>

#ifdef HAVE_OPENCV_IMGPROC
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "drawingHelpers.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

typedef struct SoftwareArguments
{
  std::string m_img;
  bool m_gradientOutsideClass;
  bool m_useVpImageFilterCanny;
  bool m_saveEdgeList;
  int m_gaussianKernelSize;
  int m_apertureSize;
  vpImageFilter::vpCannyFilteringAndGradientType m_filteringType;
  float m_gaussianStdev;
  float m_lowerThresh;
  float m_upperThresh;
  float m_lowerThreshRatio;
  float m_upperThreshRatio;
  vpImageFilter::vpCannyBackendType m_backend;

  SoftwareArguments()
    : m_img("")
    , m_gradientOutsideClass(false)
    , m_useVpImageFilterCanny(false)
    , m_saveEdgeList(false)
    , m_gaussianKernelSize(3)
    , m_apertureSize(3)
    , m_filteringType(vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
    , m_gaussianStdev(1.)
    , m_lowerThresh(-1.)
    , m_upperThresh(-1.)
    , m_lowerThreshRatio(0.6f)
    , m_upperThreshRatio(0.8f)
#ifdef VISP_HAVE_OPENCV
    , m_backend(vpImageFilter::CANNY_OPENCV_BACKEND)
#else
    , m_backend(vpImageFilter::CANNY_VISP_BACKEND)
#endif
  { }
} SoftwareArguments;

void setGradientOutsideClass(const vpImage<unsigned char> &I, const int &gaussianKernelSize, const float &gaussianStdev,
                             vpCannyEdgeDetection &cannyDetector, const unsigned int apertureSize,
                             const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
                             vpImage<unsigned char> &dIx_uchar, vpImage<unsigned char> &dIy_uchar);
bool sortImagePoints(const vpImagePoint &a, const vpImagePoint &b);
void checkEdgeList(const vpCannyEdgeDetection &cannyDetector, const vpImage<unsigned char> &I_canny_visp);
void usage(const std::string &softName, const SoftwareArguments &options);

template <class T>
void computeMeanMaxStdev(const vpImage<T> &I, float &mean, float &max, float &stdev)
{
  max = std::numeric_limits<float>::epsilon();
  mean = 0.;
  stdev = 0.;
  unsigned int nbRows = I.getRows();
  unsigned int nbCols = I.getCols();
  float scale = 1.f / (static_cast<float>(nbRows) * static_cast<float>(nbCols));
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      mean += I[r][c];
      max = std::max<float>(max, static_cast<float>(I[r][c]));
    }
  }
  mean *= scale;
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      stdev += (I[r][c] - mean) * (I[r][c] - mean);
    }
  }
  stdev *= scale;
  stdev = std::sqrt(stdev);
}

void setGradientOutsideClass(const vpImage<unsigned char> &I, const int &gaussianKernelSize, const float &gaussianStdev,
                             vpCannyEdgeDetection &cannyDetector, const unsigned int apertureSize,
                             const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
                             vpImage<unsigned char> &dIx_uchar, vpImage<unsigned char> &dIy_uchar)
{
  // Computing the gradients
  vpImage<float> dIx, dIy;
  vpImageFilter::computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
      apertureSize, filteringType);

  // Set the gradients of the vpCannyEdgeDetection
  cannyDetector.setGradients(dIx, dIy);

  // Display the gradients
  float mean, max, stdev;
  computeMeanMaxStdev(dIx, mean, max, stdev);

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  std::string title = "Gradient along the horizontal axis. Mean = " + std::to_string(mean)
    + "+/-" + std::to_string(stdev) + " Max = " + std::to_string(max);
#else
  std::string title;
  {
    std::stringstream ss;
    ss << "Gradient along the horizontal axis. Mean = " << mean<< "+/-" << stdev<< " Max = " << max;
    title = ss.str();
  }
#endif
  vpImageConvert::convert(dIx, dIx_uchar);
  drawingHelpers::display(dIx_uchar, title);
  computeMeanMaxStdev(dIy, mean, max, stdev);
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  title = "Gradient along the horizontal axis. Mean = " + std::to_string(mean)
    + "+/-" + std::to_string(stdev) + " Max = " + std::to_string(max);
#else
  {
    std::stringstream ss;
    ss << "Gradient along the horizontal axis. Mean = " << mean<< "+/-" << stdev<< " Max = " << max;
    title = ss.str();
  }
#endif
  vpImageConvert::convert(dIy, dIy_uchar);
  drawingHelpers::display(dIy_uchar, title);
}

bool sortImagePoints(const vpImagePoint &a, const vpImagePoint &b)
{
  if (a.get_i() < b.get_i()) {
    return true;
  }
  else if (vpMath::equal(a.get_i(), b.get_i()) && (a.get_j() < b.get_j())) {
    return true;
  }
  return false;
}

void checkEdgeList(const vpCannyEdgeDetection &cannyDetector, const vpImage<unsigned char> &I_canny_visp)
{
  std::vector<vpImagePoint> listEdgePoints = cannyDetector.getEdgePointsList();
  // Check if the edge points are uniquely present in the edge lists
  std::vector<vpImagePoint> cpyListEdgePoints = listEdgePoints;
  std::sort(cpyListEdgePoints.begin(), cpyListEdgePoints.end(), sortImagePoints);
  std::vector<vpImagePoint>::iterator last = std::unique(cpyListEdgePoints.begin(), cpyListEdgePoints.end());
  static_cast<void>(cpyListEdgePoints.erase(last, cpyListEdgePoints.end()));
  if (listEdgePoints.size() != cpyListEdgePoints.size()) {
    throw(vpException(vpException::fatalError, "There are duplicated points in the edge points list !"));
  }
  // Check if all the edge points in the list have been set in the image
  std::vector<vpImagePoint>::iterator start = listEdgePoints.begin();
  std::vector<vpImagePoint>::iterator stop = listEdgePoints.end();
  for (std::vector<vpImagePoint>::iterator it = start; it != stop; ++it) {
    if (I_canny_visp[static_cast<unsigned int>(it->get_i())][static_cast<unsigned int>(it->get_j())] != 255) {
      throw(vpException(vpException::fatalError, "A point of the edge-point list has not been set in the image !"));
    }
  }
  // Check if all the edge points in the image have been set in the list
  unsigned int nbRows = I_canny_visp.getRows();
  unsigned int nbCols = I_canny_visp.getCols();
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      if (I_canny_visp[i][j] == 255) {
        vpImagePoint searchedPoint(i, j);
        std::vector<vpImagePoint>::iterator idx = std::find(listEdgePoints.begin(), listEdgePoints.end(), searchedPoint);
        if (idx == listEdgePoints.end()) {
          throw(vpException(vpException::fatalError, "A point of the image has not been set in the edge-point list !"));
        }
      }
    }
  }

  std::cout << "All the edge-point list tests went well !" << std::endl;
}

void usage(const std::string &softName, const SoftwareArguments &options)
{
  std::cout << "NAME" << std::endl;
  std::cout << softName << ": software to test the vpCannyEdgeComputation class and vpImageFilter::canny method" << std::endl;
  std::cout << "SYNOPSIS" << std::endl;
  std::cout << "\t" << softName
    << " [-i, --image <pathToImg>]"
    << " [-g, --gradient <kernelSize stdev>]"
    << " [-t, --thresh <lowerThresh upperThresh>]"
    << " [-a, --aperture <apertureSize>]"
    << " [-f, --filter <filterName>]"
    << " [-r, --ratio <lowerThreshRatio upperThreshRatio>]"
    << " [-b, --backend <backendName>]"
    << " [-e, --edge-list]" << std::endl
    << " [-h, --help]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl;
  std::cout << "\t-i, --image <pathToImg>" << std::endl
    << "\t\tPermits to load an image on which will be tested the vpCanny class." << std::endl
    << "\t\tWhen empty uses a simulated image." << std::endl
    << std::endl;
  std::cout << "\t-g, --gradient <kernelSize stdev>" << std::endl
    << "\t\tPermits to compute the gradients of the image outside the vpCanny class." << std::endl
    << "\t\tFirst parameter is the size of the Gaussian kernel used to compute the gradients." << std::endl
    << "\t\tSecond parameter is the standard deviation of the Gaussian kernel used to compute the gradients." << std::endl
    << "\t\tDefault: " << options.m_gaussianKernelSize << " " << options.m_gaussianStdev << std::endl
    << std::endl;
  std::cout << "\t-t, --thresh <lowerThresh upperThresh>" << std::endl
    << "\t\tPermits to set the lower and upper thresholds of the vpCanny class." << std::endl
    << "\t\tFirst parameter is the lower threshold." << std::endl
    << "\t\tSecond parameter is the upper threshold." << std::endl
    << "\t\tWhen set to -1 thresholds are computed automatically." << std::endl
    << "\t\tDefault: " << options.m_lowerThresh << " " << options.m_upperThresh << std::endl
    << std::endl;
  std::cout << "\t-a, --aperture <apertureSize>" << std::endl
    << "\t\tPermits to set the size of the gradient filter kernel." << std::endl
    << "\t\tParameter must be odd and positive." << std::endl
    << "\t\tDefault: " << options.m_apertureSize << std::endl
    << std::endl;
  std::cout << "\t-f, --filter <filterName>" << std::endl
    << "\t\tPermits to choose the type of filter to apply to compute the gradient." << std::endl
    << "\t\tAvailable values: " << vpImageFilter::vpGetCannyFiltAndGradTypes("<", " | ", ">") << std::endl
    << "\t\tDefault: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(options.m_filteringType) << std::endl
    << std::endl;
  std::cout << "\t-r, --ratio <lowerThreshRatio upperThreshRatio>" << std::endl
    << "\t\tPermits to set the lower and upper thresholds ratio of the vpCanny class." << std::endl
    << "\t\tFirst parameter is the lower threshold ratio." << std::endl
    << "\t\tSecond parameter is the upper threshold ratio." << std::endl
    << "\t\tDefault: " << options.m_lowerThreshRatio << " " << options.m_upperThreshRatio << std::endl
    << std::endl;
  std::cout << "\t-b, --backend <backendName>" << std::endl
    << "\t\tPermits to use the vpImageFilter::canny method for comparison." << std::endl
    << "\t\tAvailable values: " << vpImageFilter::vpCannyBackendTypeList("<", " | ", ">") << std::endl
    << "\t\tDefault: " << vpImageFilter::vpCannyBackendTypeToString(options.m_backend) << std::endl
    << std::endl;
  std::cout << "\t-e, --edge-list" << std::endl
    << "\t\tPermits to save the edge list." << std::endl
    << "\t\tDefault: OFF" << std::endl
    << std::endl;
  std::cout << "\t-h, --help" << std::endl
    << "\t\tPermits to display the different arguments this software handles." << std::endl
    << std::endl;
}

int main(int argc, const char *argv[])
{
  SoftwareArguments options;
  for (int i = 1; i < argc; i++) {
    std::string argv_str = std::string(argv[i]);
    if ((argv_str == "-i" || argv_str == "--image") && i + 1 < argc) {
      options.m_img = std::string(argv[i + 1]);
      i++;
    }
    else if ((argv_str == "-g" || argv_str == "--gradient") && i + 2 < argc) {
      options.m_gradientOutsideClass = true;
      options.m_gaussianKernelSize = atoi(argv[i + 1]);
      options.m_gaussianStdev = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-t" || argv_str == "--thresh") && i + 2 < argc) {
      options.m_lowerThresh = static_cast<float>(atof(argv[i + 1]));
      options.m_upperThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-a" || argv_str == "--aperture") && i + 1 < argc) {
      options.m_apertureSize = std::atoi(argv[i + 1]);
      i++;
    }
    else if ((argv_str == "-f" || argv_str == "--filter") && i + 1 < argc) {
      options.m_filteringType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(std::string(argv[i + 1]));
      i++;
    }
    else if ((argv_str == "-r" || argv_str == "--ratio") && i + 2 < argc) {
      options.m_lowerThreshRatio = static_cast<float>(std::atof(argv[i + 1]));
      options.m_upperThreshRatio = static_cast<float>(std::atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-b" || argv_str == "--backend") && i + 1 < argc) {
      options.m_useVpImageFilterCanny = true;
      options.m_backend = vpImageFilter::vpCannyBackendTypeFromString(std::string(argv[i+1]));
      i++;
    }
    else if ((argv_str == "-e") || (argv_str == "--edge-list")) {
      options.m_saveEdgeList = true;
    }
    else if (argv_str == "-h" || argv_str == "--help") {
      usage(std::string(argv[0]), options);
      return EXIT_SUCCESS;
    }
    else {
      std::cerr << "Argument \"" << argv_str << "\" is unknown." << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::string configAsTxt("Canny Configuration:\n");
  configAsTxt += "\tFiltering + gradient operators = " + vpImageFilter::vpCannyFiltAndGradTypeToStr(options.m_filteringType) + "\n";
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  configAsTxt += "\tGaussian filter kernel size = " + std::to_string(options.m_gaussianKernelSize) + "\n";
  configAsTxt += "\tGaussian filter standard deviation = " + std::to_string(options.m_gaussianStdev) + "\n";
  configAsTxt += "\tGradient filter kernel size = " + std::to_string(options.m_apertureSize) + "\n";
  configAsTxt += "\tCanny edge filter thresholds = [" + std::to_string(options.m_lowerThresh) + " ; " + std::to_string(options.m_upperThresh) + "]\n";
  configAsTxt += "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" + std::to_string(options.m_lowerThreshRatio) + " ; " + std::to_string(options.m_upperThreshRatio) + "]\n";
#else
  {
    std::stringstream ss;
    ss << "\tGaussian filter kernel size = " << options.m_gaussianKernelSize << "\n";
    ss << "\tGaussian filter standard deviation = " << options.m_gaussianStdev << "\n";
    ss << "\tGradient filter kernel size = " << options.m_apertureSize << "\n";
    ss << "\tCanny edge filter thresholds = [" << options.m_lowerThresh << " ; " << options.m_upperThresh << "]\n";
    ss << "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" << options.m_lowerThreshRatio << " ; " << options.m_upperThreshRatio << "]\n";
    configAsTxt += ss.str();
  }
#endif
  std::cout << configAsTxt << std::endl;

  vpCannyEdgeDetection cannyDetector(options.m_gaussianKernelSize, options.m_gaussianStdev, options.m_apertureSize,
                                     options.m_lowerThresh, options.m_upperThresh, options.m_lowerThreshRatio, options.m_upperThreshRatio,
                                     options.m_filteringType, options.m_saveEdgeList);
  vpImage<unsigned char> I_canny_input, I_canny_visp, dIx_uchar, dIy_uchar, I_canny_imgFilter;
  if (!options.m_img.empty()) {
    // Detection on the user image
    vpImageIo::read(I_canny_input, options.m_img);
  }
  else {
    // Detection on a fake image of a square
    I_canny_input.resize(500, 500, 0);
    for (unsigned int r = 150; r < 350; r++) {
      for (unsigned int c = 150; c < 350; c++) {
        I_canny_input[r][c] = 125;
      }
    }
  }

  // Initialization of the displays
  I_canny_visp = I_canny_imgFilter = dIx_uchar = dIy_uchar = I_canny_input;
  vpImage<unsigned char> *p_dIx = nullptr, *p_dIy = nullptr, *p_IcannyImgFilter = nullptr;

  if (options.m_gradientOutsideClass) {
    p_dIx = &dIx_uchar;
    p_dIy = &dIy_uchar;
  }

  if (options.m_useVpImageFilterCanny) {
    p_IcannyImgFilter = &I_canny_imgFilter;
  }
  drawingHelpers::init(I_canny_input, I_canny_visp, p_dIx, p_dIy, p_IcannyImgFilter);

  // Computing the gradient outside the vpCannyEdgeDetection class if asked
  if (options.m_gradientOutsideClass) {
    setGradientOutsideClass(I_canny_input, options.m_gaussianKernelSize, options.m_gaussianStdev, cannyDetector, options.m_apertureSize,
                            options.m_filteringType, dIx_uchar, dIy_uchar);
  }
  I_canny_visp = cannyDetector.detect(I_canny_input);
  float mean, max, stdev;
  computeMeanMaxStdev(I_canny_input, mean, max, stdev);
  if (options.m_saveEdgeList) {
    checkEdgeList(cannyDetector, I_canny_visp);
  }
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  std::string title("Input of the Canny edge detector. Mean = " + std::to_string(mean) + "+/-" + std::to_string(stdev) + " Max = " + std::to_string(max));
#else
  std::string title;
  {
    std::stringstream ss;
    ss << "Input of the Canny edge detector. Mean = " << mean << "+/-"  << stdev << " Max = " << max;
    title = ss.str();
  }
#endif
  drawingHelpers::display(I_canny_input, title);
  drawingHelpers::display(I_canny_visp, "Canny results on image " + options.m_img);

  if (options.m_useVpImageFilterCanny) {
    float cannyThresh = options.m_upperThresh;
    float lowerThresh(options.m_lowerThresh);
    vpImageFilter::canny(I_canny_input, I_canny_imgFilter, options.m_gaussianKernelSize, lowerThresh, cannyThresh,
                         options.m_apertureSize, options.m_gaussianStdev, options.m_lowerThreshRatio, options.m_upperThreshRatio, true,
                         options.m_backend, options.m_filteringType);
    drawingHelpers::display(I_canny_imgFilter, "Canny results with \"" + vpImageFilter::vpCannyBackendTypeToString(options.m_backend) + "\" backend");
  }

  drawingHelpers::waitForClick(I_canny_input, true);
  return EXIT_SUCCESS;
}
