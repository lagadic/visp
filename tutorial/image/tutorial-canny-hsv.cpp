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
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// #define BUILD_REFERENCE_METHOD

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)

vpImage<unsigned char> IdebugX;
vpImage<unsigned char> IdebugY;

template <typename FilterType>
void computeAbsoluteGradient(const vpImage<FilterType> &GIx, const vpImage<FilterType> &GIy, vpImage<FilterType> &GI, FilterType &min, FilterType &max)
{
  const unsigned int h = GIx.getHeight(), w = GIx.getWidth();
  GI.resize(h, w);
  max = -1.;
  min = std::numeric_limits<FilterType>::max();
  for (unsigned int r = 0; r < h; ++r) {
    for (unsigned int c = 0; c < w; ++c) {
      GI[r][c] = std::abs(GIx[r][c]) + std::abs(GIy[r][c]);
      max = std::max(max, GI[r][c]);
      min = std::min(min, GI[r][c]);
    }
  }
}

template<typename FilterType>
vpImage<unsigned char> convertToDisplay(const vpImage<FilterType> &GI, const FilterType &min, const FilterType &max)
{
  const unsigned int h = GI.getHeight(), w = GI.getWidth();
  const FilterType range = max - min;
  const FilterType step = range / 256.;
  vpImage<unsigned char> Idisp(h, w);
  for (unsigned int r = 0; r < h; ++r) {
    for (unsigned int c = 0; c < w; ++c) {
      Idisp[r][c] = std::floor((GI[r][c] - min) / step);
    }
  }
  return Idisp;
}

bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c)
{
  bool computeVal = true;
  if (p_mask != nullptr) {
    computeVal = (*p_mask)[r][c];
  }
  return computeVal;
}

template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterX(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  std::vector<FilterType> filter(3);
  FilterType scale;
  std::string name;
  switch (type) {
  case vpImageFilter::CANNY_COUNT_FILTERING:
    // Prewitt case
    filter = { 1., 1., 1. };
    scale = 6.;
    name = "Prewitt";
    break;
  case vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    name = "Sobel";
    break;
  case vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    name = "Scharr";
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }
  // std::cout << "Using " << name << " filter" << std::endl;
  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

  auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c, const unsigned int &h, const unsigned int &w)
    {
      if (!p_mask) {
        return true;
      }
      bool hasToCompute = (*p_mask)[r][c];

      if (c < w - 1) { // We do not compute gradient on the last column
        hasToCompute |= (*p_mask)[r][c + 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c + 1];
        }
      }

      if (r < h - 1) { // We do not compute gradient on the last row
        hasToCompute |= (*p_mask)[r + 1][c];
      }

      if (r > 1) { // We do not compute gradient on the first row
        hasToCompute |= (*p_mask)[r - 1][c];
        if (c < w - 1) { // We do not compute gradient on the last column
          hasToCompute |= (*p_mask)[r - 1][c + 1];
        }
      }
      return hasToCompute;
    };

  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> Isign(nbRows, nbCols), IabsDiff(nbRows, nbCols);
  // Computation for I[0][0]
  if (vpColVector::dotProd((I[0][1] - I[0][0]), I[0][0].toColVector()) < 0.) {
    Isign[0][0] = -1.;
  }
  else {
    Isign[0][0] = 1.;
  }

  // Computation for the rest of the first row
  for (unsigned int c = 1; c < cStop; ++c) {
    if (vpColVector::dotProd((I[0][c + 1] - I[0][c]), (I[0][c] - I[0][c - 1])) < 0.) {
#ifdef BUILD_REFERENCE_METHOD
// Inverting sign when cosine distance is negative
      Isign[0][c] = -1. * Isign[0][c - 1];
#else
      Isign[0][c] = -1.;
#endif
    }
    else {
#ifdef BUILD_REFERENCE_METHOD
      Isign[0][c] = Isign[0][c - 1];
#else
      Isign[0][c] = 1.;
#endif
    }
  }

  // Computation of the rest of the image
  for (unsigned int r = 1; r < rStop; ++r) {
    // Computation for I[r][0]
    if (vpColVector::dotProd((I[r][1] - I[r][0]), I[r][0].toColVector()) < 0.) {
      Isign[r][0] = -1.;
    }
    else {
      Isign[r][0] = 1.;
    }
    if (checkBooleanPatch(p_mask, r, 0, nbRows, nbCols)) {
#ifdef BUILD_REFERENCE_METHOD
      IabsDiff[r][0] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][0], I[r][1]);
#else
      // IabsDiff[r][0] = I[r][1].V - I[r][0].V;
      IabsDiff[r][0] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][0], I[r][1]);
#endif
    }

      // Computation for all the other columns
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanPatch(p_mask, r, c, nbRows, nbCols)) {
        // Of the absolute value of the distance
#ifdef BUILD_REFERENCE_METHOD
        IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][c], I[r][c + 1]);
#else
        // IabsDiff[r][c] = I[r][c + 1].V - I[r][c].V;
        IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][c], I[r][c + 1]);
#endif

      }
      // Of the sign
      if (vpColVector::dotProd((I[r][c + 1] - I[r][c]), (I[r][c] - I[r][c - 1])) < 0.) {
        // Inverting sign when cosine distance is negative
#ifdef BUILD_REFERENCE_METHOD
        Isign[r][c] = -1. * Isign[r][c - 1];
#else
        Isign[r][c] = -1.;
#endif
      }
      else {
#ifdef BUILD_REFERENCE_METHOD
        Isign[r][c] = Isign[r][c - 1];
#else
        Isign[r][c] = 1.;
#endif
      }
    }
  }

  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanMask(p_mask, r, c)) {
        GIx[r][c] = 0.;
        for (int dr = -1; dr <= 1; ++dr) {
#ifdef BUILD_REFERENCE_METHOD
          GIx[r][c] += filter[dr + 1] * (Isign[r + dr][c - 1] *  IabsDiff[r + dr][c - 1] +  Isign[r + dr][c] *  IabsDiff[r + dr][c]);
#else
          GIx[r][c] += filter[dr + 1] * (Isign[r + dr][c - 1] *  IabsDiff[r + dr][c - 1] +  Isign[r + dr][c] *  IabsDiff[r + dr][c]);
#endif
        }
      }
    }
  }

  IdebugX.resize(nbRows, nbCols, 0);
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      if (Isign[r][c] > 0.) {
        IdebugX[r][c] = 255;
      }
    }
  }
}

template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterY(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIy, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  std::vector<FilterType> filter(3);
  FilterType scale;
  switch (type) {
  case vpImageFilter::CANNY_COUNT_FILTERING:
    // Prewitt case
    filter = { 1., 1., 1. };
    scale = 6.;
    break;
  case vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    break;
  case vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }
  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> Isign(nbRows, nbCols), IabsDiff(nbRows, nbCols);

  auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c, const unsigned int &h, const unsigned int &w)
    {
      if (!p_mask) {
        return true;
      }

      bool hasToCompute = (*p_mask)[r][c];
      if (c < w - 1) { // We do not compute gradient on the last column
        hasToCompute |= (*p_mask)[r][c + 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c + 1];
        }
      }

      if (r < h - 1) { // We do not compute gradient on the last row
        hasToCompute |= (*p_mask)[r + 1][c];
      }

      if (c > 1) { // We do not compute gradient on the first column
        hasToCompute |= (*p_mask)[r][c - 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c - 1];
        }
      }
      return hasToCompute;
    };

  // Computation for the first row
  for (unsigned int c = 0; c < nbCols; ++c) {
    if (checkBooleanPatch(p_mask, 0, c, nbRows, nbCols)) {
#ifdef BUILD_REFERENCE_METHOD
      IabsDiff[0][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[0][c], I[1][c]);
#else
      // IabsDiff[0][c] = I[1][c].V - I[0][c].V;
      IabsDiff[0][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[0][c], I[1][c]);
#endif
    }
    if (vpColVector::dotProd((I[1][c] - I[0][c]), I[0][c].toColVector()) < 0.) {
      // Inverting sign when cosine distance is negative
      Isign[0][c] = -1.;
    }
    else {
      Isign[0][c] = 1.;
    }
  }

  // Computation for the rest of the image of d and sign
  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      // Of the absolute value of the distance
      if (checkBooleanPatch(p_mask, r, c, nbRows, nbCols)) {
#ifdef BUILD_REFERENCE_METHOD
        IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][c], I[r + 1][c]);
#else
        // IabsDiff[r][c] = I[r + 1][c].V - I[r][c].V;
        IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template mahalanobisDistance<double>(I[r][c], I[r + 1][c]);
#endif
      }
      // Of the sign
      if (vpColVector::dotProd((I[r +1][c] - I[r][c]), (I[r][c] - I[r - 1][c])) < 0.) {
        // Inverting sign when cosine distance is negative
#ifdef BUILD_REFERENCE_METHOD
        Isign[r][c] = -1. * Isign[r - 1][c];
#else
        Isign[r][c] = -1.;
#endif
      }
      else {
#ifdef BUILD_REFERENCE_METHOD
        Isign[r][c] = Isign[r - 1][c];
#else
        Isign[r][c] = 1.;
#endif
      }
    }
  }

  // Computation of the gradient
  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanMask(p_mask, r, c)) {
        GIy[r][c] = 0.;
        for (int dc = -1; dc <= 1; ++dc) {
#ifdef BUILD_REFERENCE_METHOD
          GIy[r][c] += filter[dc + 1] * (Isign[r - 1][c + dc] * IabsDiff[r - 1][c + dc] + Isign[r][c + dc] * IabsDiff[r][c + dc]);
#else
          GIy[r][c] += filter[dc + 1] * (Isign[r - 1][c + dc] * IabsDiff[r - 1][c + dc] + Isign[r][c + dc] * IabsDiff[r][c + dc]);
#endif
        }
      }
    }
  }

  IdebugY.resize(nbRows, nbCols, 0);
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      if (Isign[r][c] > 0.) {
        IdebugY[r][c] = 255;
      }
    }
  }
}

template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilter(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, vpImage<FilterType> &GIy, const int &nbThread, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  (void)nbThread;
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  gradientFilterX(I, GIx, p_mask, type);
  gradientFilterY(I, GIy, p_mask, type);
}

typedef struct SoftwareArguments
{
  std::string m_img;
  int m_gaussianKernelSize;
  float m_gaussianStdev;
  float m_lowerThresh;
  float m_upperThresh;
  float m_lowerThreshRatio;
  float m_upperThreshRatio;
  vpImageFilter::vpCannyFilteringAndGradientType m_filteringType;
  bool m_saveImages;
  bool m_useDisplay; //!< If true, activate the plot and the renderer if VISP_HAVE_DISPLAY is defined.
  int m_nbThread;

  SoftwareArguments()
    : m_img("")
    , m_gaussianKernelSize(3)
    , m_gaussianStdev(1.)
    , m_lowerThresh(-1.)
    , m_upperThresh(-1.)
    , m_lowerThreshRatio(0.6f)
    , m_upperThreshRatio(0.8f)
    , m_filteringType(vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING)
    , m_saveImages(false)
#ifdef VISP_HAVE_DISPLAY
    , m_useDisplay(true)
#else
    , m_useDisplay(false)
#endif
    , m_nbThread(-1)
  { }
}SoftwareArguments;

void usage(const std::string &softName, const SoftwareArguments &options)
{
  std::cout << "NAME" << std::endl;
  std::cout << softName << ": software to test the vpCannyEdgeComputation class and vpImageFilter::canny method" << std::endl;
  std::cout << "SYNOPSIS" << std::endl;
  std::cout << "\t" << softName
    << " [-i, --image <pathToImg>]"
    << " [-g, --gradient <kernelSize stdev>]"
    << " [-t, --thresh <lowerThresh upperThresh>]"
    << " [-f, --filter " << vpImageFilter::vpGetCannyFiltAndGradTypes("<", " | ", ">") << "]"
    << " [-r, --ratio <lowerThreshRatio upperThreshRatio>]"
    << " [-n, --nb-threads <number of threads>]"
    << " [-s, --save]" << std::endl
    << " [-d, --no-display]" << std::endl
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
  std::cout << "\t-r, --ratio <lowerThreshRatio upperThreshRatio>" << std::endl
    << "\t\tPermits to set the lower and upper thresholds ratio of the vpCanny class." << std::endl
    << "\t\tFirst parameter is the lower threshold ratio." << std::endl
    << "\t\tSecond parameter is the upper threshold ratio." << std::endl
    << "\t\tDefault: " << options.m_lowerThreshRatio << " " << options.m_upperThreshRatio << std::endl
    << std::endl;
  std::cout << "\t-f, --filter <filterName>" << std::endl
    << "\t\tPermits to choose the type of filter to apply to compute the gradient." << std::endl
    << "\t\tAvailable values: " << vpImageFilter::vpGetCannyFiltAndGradTypes("<", " | ", ">") << std::endl
    << "\t\tDefault: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(options.m_filteringType) << std::endl
    << std::endl;
  std::cout << "\t-n, --nb-threads <number of threads>" << std::endl
    << "\t\tPermits to choose the number of threads to use for the Canny." << std::endl
    << "\t\tUse -1 to automatically choose the highest possible number of threads." << std::endl
    << "\t\tDefault: " << options.m_nbThread << std::endl
    << std::endl;
  std::cout << "\t-s, --save" << std::endl
    << "\t\tPermits to save the different images." << std::endl
    << std::endl;
  std::cout << "  -d, --no-display" << std::endl
    << "    Deactivate display." << std::endl
    << "    Default: display is "
#ifdef VISP_HAVE_DISPLAY
    << "ON" << std::endl
#else
    << "OFF" << std::endl
#endif
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
      options.m_gaussianKernelSize = atoi(argv[i + 1]);
      options.m_gaussianStdev = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-t" || argv_str == "--thresh") && i + 2 < argc) {
      options.m_lowerThresh = static_cast<float>(atof(argv[i + 1]));
      options.m_upperThresh = static_cast<float>(atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-r" || argv_str == "--ratio") && i + 2 < argc) {
      options.m_lowerThreshRatio = static_cast<float>(std::atof(argv[i + 1]));
      options.m_upperThreshRatio = static_cast<float>(std::atof(argv[i + 2]));
      i += 2;
    }
    else if ((argv_str == "-f" || argv_str == "--filter") && i + 1 < argc) {
      options.m_filteringType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(std::string(argv[i + 1]));
      i++;
    }
    else if ((argv_str == "-n" || argv_str == "--nb-threads") && i + 1 < argc) {
      options.m_nbThread = std::atoi(argv[i + 1]);
      i++;
    }
    else if (argv_str == "-s" || argv_str == "--save") {
      options.m_saveImages = true;
    }
    else if (argv_str == "-d" || argv_str == "--no-display") {
      options.m_useDisplay = false;
    }
    else if (argv_str == "-h" || argv_str == "--help") {
      usage(std::string(argv[0]), SoftwareArguments());
      return EXIT_SUCCESS;
    }
    else {
      std::cerr << "Argument \"" << argv_str << "\" is unknown." << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::string configAsTxt("Canny Configuration:\n");
  configAsTxt += "\tFiltering + gradient operators = " + vpImageFilter::vpCannyFiltAndGradTypeToStr(options.m_filteringType) + "\n";
  configAsTxt += "\tGaussian filter kernel size = " + std::to_string(options.m_gaussianKernelSize) + "\n";
  configAsTxt += "\tGaussian filter standard deviation = " + std::to_string(options.m_gaussianStdev) + "\n";
  configAsTxt += "\tCanny edge filter thresholds = [" + std::to_string(options.m_lowerThresh) + " ; " + std::to_string(options.m_upperThresh) + "]\n";
  configAsTxt += "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" + std::to_string(options.m_lowerThreshRatio) + " ; " + std::to_string(options.m_upperThreshRatio) + "]\n";
  configAsTxt += "\tCanny edge filter nb threads = " + (options.m_nbThread > 0 ? std::to_string(options.m_nbThread) : std::string("auto")) + "\n";
  std::cout << configAsTxt << std::endl;

  unsigned int uselessAperture = 3;
  vpCannyEdgeDetection cannyDetector(options.m_gaussianKernelSize, options.m_gaussianStdev, uselessAperture,
                                     options.m_lowerThresh, options.m_upperThresh, options.m_lowerThreshRatio, options.m_upperThreshRatio, options.m_filteringType);
  vpImage<vpRGBa> Iload;
  vpImage<vpHSV<unsigned char, true>> Iin_hsvuc;
  vpImage<vpHSV<double>> Iin_hsvd;
  if (!options.m_img.empty()) {
    // Detection on the user image
    vpImageIo::read(Iload, options.m_img);
  }
  else {
    std::cout << "This example only works on a real image. Please use the -i option." << std::endl;
    return EXIT_SUCCESS;
  }

  // vpImage<bool> mask(Iload.getRows(), Iload.getCols(), false);
  // for (int r = 0; r < Iload.getRows()/4; ++r) {
  //   for (int c = 0; c < Iload.getCols()/4; ++c) {
  //     mask[Iload.getRows()/2 - r][Iload.getCols()/2 - c] = true;
  //     mask[Iload.getRows()/2 + r][Iload.getCols()/2 - c] = true;
  //     mask[Iload.getRows()/2 - r][Iload.getCols()/2 + c] = true;
  //     mask[Iload.getRows()/2 + r][Iload.getCols()/2 + c] = true;
  //   }
  // }

  vpImage<bool> *p_mask = nullptr;
  cannyDetector.setMask(p_mask);
  cannyDetector.setNbThread(options.m_nbThread);

  double tStartHSVuc = vpTime::measureTimeMicros();
  vpImageConvert::convert(Iload, Iin_hsvuc);
  vpImage<unsigned char> I_canny_hsvuc = cannyDetector.detect(Iin_hsvuc);
  double tEndHSVuc = vpTime::measureTimeMicros();
  std::cout << "Time to convert RGBa into HSV uchar + compute the edge-map: " << (tEndHSVuc - tStartHSVuc) / 1000. << " ms" << std::endl;

  double tStartHSVd = vpTime::measureTimeMicros();
  vpImageConvert::convert(Iload, Iin_hsvd);
  vpImage<unsigned char> I_canny_hsvd = cannyDetector.detect(Iin_hsvd);
  double tEndHSVd = vpTime::measureTimeMicros();
  std::cout << "Time to convert RGBa into HSV double + compute the edge-map: " << (tEndHSVd - tStartHSVd) / 1000. << " ms" << std::endl;

  vpCannyEdgeDetection cannyDetectorUC(options.m_gaussianKernelSize, options.m_gaussianStdev, uselessAperture,
    options.m_lowerThresh, options.m_upperThresh, options.m_lowerThreshRatio, options.m_upperThreshRatio);
  cannyDetectorUC.setMask(p_mask);
  cannyDetectorUC.setNbThread(options.m_nbThread);
  vpImage<unsigned char> Iin_convert;
  double tStartChar = vpTime::measureTimeMicros();
  vpImageConvert::convert(Iload, Iin_convert);
  vpImage<unsigned char> I_canny_uc = cannyDetectorUC.detect(Iin_convert);
  double tEndChar = vpTime::measureTimeMicros();
  std::cout << "Time to convert RGBa into uchar + compute the edge-map for RGBa: " << (tEndChar - tStartChar) / 1000. << " ms" << std::endl;

  // Initialization of the displays
#ifdef VISP_HAVE_DISPLAY
  using FilterType = float;
  vpImage<FilterType> GIx, GIy, GI;
  vpImage<vpHSV<unsigned char, true>> Iblur_hsvuc;
  double tStartBlurHSVUC = vpTime::measureTimeMicros();
  vpImageFilter::gaussianBlur(Iin_hsvuc, Iblur_hsvuc, options.m_gaussianKernelSize, options.m_gaussianStdev, true, p_mask);
  double tEndBlurHSVUC = vpTime::measureTimeMicros();

  double tStartGradientHSVUC = vpTime::measureTimeMicros();
  vpImageFilter::gradientFilter(Iblur_hsvuc, GIx, GIy, options.m_nbThread, p_mask, options.m_filteringType);
  double tEndGradientHSVUC = vpTime::measureTimeMicros();
  FilterType min = 0., max = 0.;
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvuc_imgfilter = convertToDisplay(GI, min, max);

  double tStartGradientHSVUCRef = vpTime::measureTimeMicros();
  gradientFilter(Iblur_hsvuc, GIx, GIy, options.m_nbThread, p_mask, options.m_filteringType);
  double tEndGradientHSVUCRef = vpTime::measureTimeMicros();
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvuc_vonly = convertToDisplay(GI, min, max);
  cannyDetector.setGradients(GIx, GIy);
  I_canny_hsvuc = cannyDetector.detect(Iin_hsvuc);


  vpImage<vpHSV<unsigned char, true>> Iblur_hsvd;
  double tStartBlurHSVd = vpTime::measureTimeMicros();
  vpImageFilter::gaussianBlur(Iin_hsvd, Iblur_hsvd, options.m_gaussianKernelSize, options.m_gaussianStdev, true, p_mask);
  double tEndBlurHSVd = vpTime::measureTimeMicros();
  double tStartGradientHSVd = vpTime::measureTimeMicros();
  vpImageFilter::gradientFilter(Iblur_hsvd, GIx, GIy, options.m_nbThread, p_mask, options.m_filteringType);
  double tEndGradientHSVd = vpTime::measureTimeMicros();
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvd_imgfilter = convertToDisplay(GI, min, max);

  // gradientFilter(Iblur_hsvd, GIx, GIy, 1, p_mask, options.m_filteringType);
  // computeAbsoluteGradient(GIx, GIy, GI, min, max);
  // vpImage<unsigned char> GIdisp_hsvd_vonly = convertToDisplay(GI, min, max);

  vpImage<FilterType> IblurUC, GIx_uc, GIy_uc, GI_uc;
  double tStartBlurUC = vpTime::measureTimeMicros();
  vpImageFilter::gaussianBlur(Iin_convert, IblurUC, options.m_gaussianKernelSize, (FilterType)options.m_gaussianStdev, true, p_mask);
  double tEndBlurUC = vpTime::measureTimeMicros();
  vpArray2D<FilterType> derFilterX(3, 3), derFilterY(3, 3);
  FilterType scaleX, scaleY;
  switch (options.m_filteringType) {
  case vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING:
    scaleX = vpImageFilter::getScharrKernelX(derFilterX.data, 1);
    scaleY = vpImageFilter::getScharrKernelY(derFilterY.data, 1);
    break;
  case vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING:
    scaleX = vpImageFilter::getSobelKernelX(derFilterX.data, 1);
    scaleY = vpImageFilter::getSobelKernelY(derFilterY.data, 1);
    break;
  default:
    throw vpException(vpException::notImplementedError, "Other type of filter not handled for uchar");
  }
  auto scaleFilter = [](vpArray2D<FilterType> &array, const FilterType &scale) {
    for (unsigned int r = 0; r < array.getRows(); ++r) {
      for (unsigned int c = 0; c < array.getCols(); ++c) {
        array[r][c] = array[r][c] / scale;
      }
    }
    };
  scaleFilter(derFilterX, scaleX);
  scaleFilter(derFilterY, scaleY);
  // Computing the gradients
  double tStartGradientUC = vpTime::measureTimeMicros();
  vpImageFilter::filter(IblurUC, GIx_uc, derFilterX, true, p_mask);
  vpImageFilter::filter(IblurUC, GIy_uc, derFilterY, true, p_mask);
  double tEndGradientUC = vpTime::measureTimeMicros();
  computeAbsoluteGradient(GIx_uc, GIy_uc, GI_uc, min, max);
  vpImage<unsigned char> GIdisp_uc = convertToDisplay(GI_uc, min, max);

  std::cout << "[vpHSV<uchar>]" << std::endl;
  std::cout <<"\tgblur = " << (tEndBlurHSVUC - tStartBlurHSVUC) / 1000. << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientHSVUC - tStartGradientHSVUC) / 1000. << std::endl;
  std::cout << std::endl;

  std::cout << "[vpHSV<uchar> ref]" << std::endl;
  std::cout <<"\tgblur = " << "N/A" << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientHSVUCRef - tStartGradientHSVUCRef) / 1000. << std::endl;
  std::cout << std::endl;

  std::cout << "[vpHSV<double>]" << std::endl;
  std::cout <<"\tgblur = " << (tEndBlurHSVd - tStartBlurHSVd) / 1000. << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientHSVd - tStartGradientHSVd) / 1000. << std::endl;
  std::cout << std::endl;

  std::cout << "[uchar]" << std::endl;
  std::cout <<"\tgblur = " << (tEndBlurUC - tStartBlurUC) / 1000. << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientUC - tStartGradientUC) / 1000. << std::endl;
  std::cout << std::endl;

  if (options.m_useDisplay) {
    std::shared_ptr<vpDisplay> disp_input = vpDisplayFactory::createDisplay(Iload, -1, -1, "Input color image", vpDisplay::SCALE_AUTO);
    int posX = disp_input->getWidth() + 20;
    int posY = disp_input->getHeight() + 20;
    std::shared_ptr<vpDisplay> disp_canny = vpDisplayFactory::createDisplay(I_canny_hsvuc, posX, -1, "HSV UC Canny", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_input_uc = vpDisplayFactory::createDisplay(Iin_convert, -1, posY, "Input converted image", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_canny_uc = vpDisplayFactory::createDisplay(I_canny_uc, posX, posY, "UC Canny", vpDisplay::SCALE_AUTO);

#ifdef BUILD_REFERENCE_METHOD
    std::shared_ptr<vpDisplay> disp_GI_hsvuc_vonly = vpDisplayFactory::createDisplay(GIdisp_hsvuc_vonly, posX, -1, "Gradient reference method", vpDisplay::SCALE_AUTO);
#else
    std::shared_ptr<vpDisplay> disp_GI_hsvuc_vonly = vpDisplayFactory::createDisplay(GIdisp_hsvuc_vonly, 2 * posX, -1, "Gradient V only", vpDisplay::SCALE_AUTO);
#endif
    vpDisplay::display(GIdisp_hsvuc_vonly);
    vpDisplay::flush(GIdisp_hsvuc_vonly);

    std::shared_ptr<vpDisplay> disp_GI_hsvuc_imgfilter = vpDisplayFactory::createDisplay(GIdisp_hsvuc_imgfilter, 3 * posX, -1, "Gradient vpImgFilter");
    vpDisplay::display(GIdisp_hsvuc_imgfilter);
    vpDisplay::flush(GIdisp_hsvuc_imgfilter);

    // std::shared_ptr<vpDisplay> disp_GI_hsvd = vpDisplayFactory::createDisplay(GIdisp_hsvd, 3 * posX, -1, "Gradient");
    // vpDisplay::display(GIdisp_hsvd);
    // vpDisplay::flush(GIdisp_hsvd);

    std::shared_ptr<vpDisplay> disp_GI_uc = vpDisplayFactory::createDisplay(GIdisp_uc, 2 * posX, posY, "Gradient (unsigned char)");
    vpDisplay::display(GIdisp_uc);
    vpDisplay::flush(GIdisp_uc);

    vpDisplay::display(Iload);
    vpDisplay::flush(Iload);
    vpDisplay::display(Iin_convert);
    vpDisplay::flush(Iin_convert);
    vpDisplay::display(I_canny_uc);
    vpDisplay::flush(I_canny_uc);
    vpDisplay::display(I_canny_hsvuc);
    // vpDisplay::displayText(I_canny_hsvuc, vpImagePoint(20, 20), "Click to leave.", vpColor::red);
    vpDisplay::flush(I_canny_hsvuc);
    // vpDisplay::getClick(I_canny_hsvuc);

    auto dispSignX = vpDisplayFactory::createDisplay(IdebugX, -1, -1, "Sign for GIx");
    vpDisplay::display(IdebugX);
    vpDisplay::flush(IdebugX);

    auto dispSignY = vpDisplayFactory::createDisplay(IdebugY, -1, -1, "Sign for GIy");
    vpDisplay::display(IdebugY);
    vpDisplay::flush(IdebugY);

    vpDisplay::displayText(Iload, vpImagePoint(20, 20), "Click to leave.", vpColor::red);
    vpDisplay::flush(Iload);
    vpDisplay::getClick(Iload);
  }
#else
  options.m_saveImages = true;
#endif
  if (options.m_saveImages) {
    std::string basename = vpIoTools::getNameWE(options.m_img);
    vpImageIo::write(I_canny_hsvuc, "Canny_HSVUC_" + basename + ".jpg");
    vpImageIo::write(I_canny_hsvd, "Canny_HSVD_" + basename + ".jpg");
    vpImageIo::write(I_canny_uc, "Canny_UC_" + basename + ".jpg");
    vpImageIo::write(GIdisp_hsvuc_imgfilter, "Gradient_HSVUC_" + basename + ".jpg");
    vpImageIo::write(GIdisp_hsvd_imgfilter, "Gradient_HSVD_" + basename + ".jpg");
    vpImageIo::write(GIdisp_uc, "Gradient_UC_" + basename + ".jpg");
  }
  return EXIT_SUCCESS;
}
#else
void main()
{
  std::cout << "C++11 is needed to work with vpHSV." << std::endl;
}
#endif
