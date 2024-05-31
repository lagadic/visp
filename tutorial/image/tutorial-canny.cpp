/****************************************************************************
 *
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
 *
*****************************************************************************/

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
                             vpImage<unsigned char> &dIx_uchar, vpImage<unsigned char> &dIy_uchar
)
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

void usage(const std::string &softName, int gaussianKernelSize, float gaussianStdev, float lowerThresh, float upperThresh,
           int apertureSize, vpImageFilter::vpCannyFilteringAndGradientType filteringType,
           float lowerThreshRatio, float upperThreshRatio, vpImageFilter::vpCannyBackendType backend)
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
    << "\t\tDefault: " << gaussianKernelSize << " " << gaussianStdev << std::endl
    << std::endl;
  std::cout << "\t-t, --thresh <lowerThresh upperThresh>" << std::endl
    << "\t\tPermits to set the lower and upper thresholds of the vpCanny class." << std::endl
    << "\t\tFirst parameter is the lower threshold." << std::endl
    << "\t\tSecond parameter is the upper threshold." << std::endl
    << "\t\tWhen set to -1 thresholds are computed automatically." << std::endl
    << "\t\tDefault: " << lowerThresh << " " << upperThresh << std::endl
    << std::endl;
  std::cout << "\t-a, --aperture <apertureSize>" << std::endl
    << "\t\tPermits to set the size of the gradient filter kernel." << std::endl
    << "\t\tParameter must be odd and positive." << std::endl
    << "\t\tDefault: " << apertureSize << std::endl
    << std::endl;
  std::cout << "\t-f, --filter <filterName>" << std::endl
    << "\t\tPermits to choose the type of filter to apply to compute the gradient." << std::endl
    << "\t\tAvailable values: " << vpImageFilter::vpGetCannyFiltAndGradTypes("<", " | ", ">") << std::endl
    << "\t\tDefault: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(filteringType) << std::endl
    << std::endl;
  std::cout << "\t-r, --ratio <lowerThreshRatio upperThreshRatio>" << std::endl
    << "\t\tPermits to set the lower and upper thresholds ratio of the vpCanny class." << std::endl
    << "\t\tFirst parameter is the lower threshold ratio." << std::endl
    << "\t\tSecond parameter is the upper threshold ratio." << std::endl
    << "\t\tDefault: " << lowerThreshRatio << " " << upperThreshRatio << std::endl
    << std::endl;
  std::cout << "\t-b, --backend <backendName>" << std::endl
    << "\t\tPermits to use the vpImageFilter::canny method for comparison." << std::endl
    << "\t\tAvailable values: " << vpImageFilter::vpCannyBackendTypeList("<", " | ", ">") << std::endl
    << "\t\tDefault: " << vpImageFilter::vpCannyBackendTypeToString(backend) << std::endl
    << std::endl;
  std::cout << "\t-h, --help" << std::endl
    << "\t\tPermits to display the different arguments this software handles." << std::endl
    << std::endl;
}

int main(int argc, const char *argv[])
{
  std::string opt_img;
  bool opt_gradientOutsideClass = false;
  bool opt_useVpImageFilterCanny = false;
  int opt_gaussianKernelSize = 3;
  int opt_apertureSize = 3;
  vpImageFilter::vpCannyFilteringAndGradientType opt_filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING;
  float opt_gaussianStdev = 1.;
  float opt_lowerThresh = -1.;
  float opt_upperThresh = -1.;
  float opt_lowerThreshRatio = 0.6f;
  float opt_upperThreshRatio = 0.8f;
  vpImageFilter::vpCannyBackendType opt_backend = vpImageFilter::CANNY_VISP_BACKEND;
  for (int i = 1; i < argc; i++) {
    std::string argv_str = std::string(argv[i]);
    if ((argv_str == "-i" || argv_str == "--image") && i + 1 < argc) {
      opt_img = std::string(argv[i + 1]);
      i++;
    }
    else if ((argv_str == "-g" || argv_str == "--gradient") && i + 2 < argc) {
      opt_gradientOutsideClass = true;
      opt_gaussianKernelSize = atoi(argv[i + 1]);
      opt_gaussianStdev = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-t" || argv_str == "--thresh") && i + 2 < argc) {
      opt_lowerThresh = static_cast<float>(atof(argv[i + 1]));
      opt_upperThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-a" || argv_str == "--aperture") && i + 1 < argc) {
      opt_apertureSize = std::atoi(argv[i + 1]);
      i++;
    }
    else if ((argv_str == "-f" || argv_str == "--filter") && i + 1 < argc) {
      opt_filteringType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(std::string(argv[i + 1]));
      i++;
    }
    else if ((argv_str == "-r" || argv_str == "--ratio") && i + 2 < argc) {
      opt_lowerThreshRatio = static_cast<float>(std::atof(argv[i + 1]));
      opt_upperThreshRatio = static_cast<float>(std::atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-b" || argv_str == "--backend") && i + 1 < argc) {
      opt_useVpImageFilterCanny = true;
      opt_backend = vpImageFilter::vpCannyBackendTypeFromString(std::string(argv[i+1]));
      i++;
    }
    else if (argv_str == "-h" || argv_str == "--help") {
      usage(std::string(argv[0]), opt_gaussianKernelSize, opt_gaussianStdev, opt_lowerThresh, opt_upperThresh,
           opt_apertureSize, opt_filteringType, opt_lowerThreshRatio, opt_upperThreshRatio, opt_backend);
      return EXIT_SUCCESS;
    }
    else {
      std::cerr << "Argument \"" << argv_str << "\" is unknown." << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::string configAsTxt("Canny Configuration:\n");
  configAsTxt += "\tFiltering + gradient operators = " + vpImageFilter::vpCannyFiltAndGradTypeToStr(opt_filteringType) + "\n";
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  configAsTxt += "\tGaussian filter kernel size = " + std::to_string(opt_gaussianKernelSize) + "\n";
  configAsTxt += "\tGaussian filter standard deviation = " + std::to_string(opt_gaussianStdev) + "\n";
  configAsTxt += "\tGradient filter kernel size = " + std::to_string(opt_apertureSize) + "\n";
  configAsTxt += "\tCanny edge filter thresholds = [" + std::to_string(opt_lowerThresh) + " ; " + std::to_string(opt_upperThresh) + "]\n";
  configAsTxt += "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" + std::to_string(opt_lowerThreshRatio) + " ; " + std::to_string(opt_upperThreshRatio) + "]\n";
#else
  {
    std::stringstream ss;
    ss << "\tGaussian filter kernel size = " << opt_gaussianKernelSize << "\n";
    ss << "\tGaussian filter standard deviation = " << opt_gaussianStdev << "\n";
    ss << "\tGradient filter kernel size = " << opt_apertureSize << "\n";
    ss << "\tCanny edge filter thresholds = [" << opt_lowerThresh << " ; " << opt_upperThresh << "]\n";
    ss << "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" << opt_lowerThreshRatio << " ; " << opt_upperThreshRatio << "]\n";
    configAsTxt += ss.str();
  }
#endif
  std::cout << configAsTxt << std::endl;

  vpCannyEdgeDetection cannyDetector(opt_gaussianKernelSize, opt_gaussianStdev, opt_apertureSize,
                                     opt_lowerThresh, opt_upperThresh, opt_lowerThreshRatio, opt_upperThreshRatio,
                                     opt_filteringType);
  vpImage<unsigned char> I_canny_input, I_canny_visp, dIx_uchar, dIy_uchar, I_canny_imgFilter;
  if (!opt_img.empty()) {
    // Detection on the user image
    vpImageIo::read(I_canny_input, opt_img);
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

  if (opt_gradientOutsideClass) {
    p_dIx = &dIx_uchar;
    p_dIy = &dIy_uchar;
  }

  if (opt_useVpImageFilterCanny) {
    p_IcannyImgFilter = &I_canny_imgFilter;
  }
  drawingHelpers::init(I_canny_input, I_canny_visp, p_dIx, p_dIy, p_IcannyImgFilter);

  // Computing the gradient outside the vpCannyEdgeDetection class if asked
  if (opt_gradientOutsideClass) {
    setGradientOutsideClass(I_canny_input, opt_gaussianKernelSize, opt_gaussianStdev, cannyDetector, opt_apertureSize,
                            opt_filteringType, dIx_uchar, dIy_uchar);
  }
  I_canny_visp = cannyDetector.detect(I_canny_input);
  float mean, max, stdev;
  computeMeanMaxStdev(I_canny_input, mean, max, stdev);
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
  drawingHelpers::display(I_canny_visp, "Canny results on image " + opt_img);

  if (opt_useVpImageFilterCanny) {
    float cannyThresh = opt_upperThresh;
    float lowerThresh(opt_lowerThresh);
    vpImageFilter::canny(I_canny_input, I_canny_imgFilter, opt_gaussianKernelSize, lowerThresh, cannyThresh,
                         opt_apertureSize, opt_gaussianStdev, opt_lowerThreshRatio, opt_upperThreshRatio, true,
                         opt_backend, opt_filteringType);
    drawingHelpers::display(I_canny_imgFilter, "Canny results with \"" + vpImageFilter::vpCannyBackendTypeToString(opt_backend) + "\" backend");
  }

  drawingHelpers::waitForClick(I_canny_input, true);
  return EXIT_SUCCESS;
}
