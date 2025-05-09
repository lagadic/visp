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

//! \example tutorial-canny-hsv.cpp

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

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)
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
  bool m_useMask; //!< If true, use a predifined boolean mask that determines which pixels should be considered and which should be ignored.
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
    , m_useMask(false)
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
    << " [-m, --use-mask]" << std::endl
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
  std::cout << "  -m, --use-mask" << std::endl
    << "    If true, use a predifined boolean mask that determines which pixels should be considered and which should be ignored" << std::endl
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
    else if (argv_str == "-m" || argv_str == "--use-mask") {
      options.m_useMask = true;
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

  vpImage<bool> mask(Iload.getRows(), Iload.getCols(), false);
  int height = Iload.getRows()/4;
  int width = Iload.getCols()/4;
  int midHeight = Iload.getRows()/2;
  int midWidth = Iload.getCols()/2;
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      mask[midHeight - r][midWidth - c] = true;
      mask[midHeight + r][midWidth - c] = true;
      mask[midHeight - r][midWidth + c] = true;
      mask[midHeight + r][midWidth + c] = true;
    }
  }

  vpImage<bool> *p_mask = (options.m_useMask ? &mask : nullptr);
  cannyDetector.setMask(p_mask);
  cannyDetector.setNbThread(options.m_nbThread);

  double tStartHSVuc = vpTime::measureTimeMicros();
  vpImageConvert::convert(Iload, Iin_hsvuc);
  vpImage<unsigned char> I_canny_hsvuc = cannyDetector.detect(Iin_hsvuc);
  double tEndHSVuc = vpTime::measureTimeMicros();
  std::cout << "Time to convert RGBa into HSV uchar + compute the edge-map: " << (tEndHSVuc - tStartHSVuc) / 1000. << " ms" << std::endl;

  double tStartHSVd = vpTime::measureTimeMicros();
  vpImageConvert::convert(Iload, Iin_hsvd);
  cannyDetector.reinit();
  cannyDetector.setMask(p_mask);
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

  vpImage<vpHSV<unsigned char, true>> Iblur_hsvd;
  double tStartBlurHSVd = vpTime::measureTimeMicros();
  vpImageFilter::gaussianBlur(Iin_hsvd, Iblur_hsvd, options.m_gaussianKernelSize, options.m_gaussianStdev, true, p_mask);
  double tEndBlurHSVd = vpTime::measureTimeMicros();
  double tStartGradientHSVd = vpTime::measureTimeMicros();
  vpImageFilter::gradientFilter(Iblur_hsvd, GIx, GIy, options.m_nbThread, p_mask, options.m_filteringType);
  double tEndGradientHSVd = vpTime::measureTimeMicros();
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvd_imgfilter = convertToDisplay(GI, min, max);

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

  std::cout << "[vpHSV<double>]" << std::endl;
  std::cout <<"\tgblur = " << (tEndBlurHSVd - tStartBlurHSVd) / 1000. << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientHSVd - tStartGradientHSVd) / 1000. << std::endl;
  std::cout << std::endl;

  std::cout << "[uchar]" << std::endl;
  std::cout <<"\tgblur = " << (tEndBlurUC - tStartBlurUC) / 1000. << std::endl;
  std::cout <<"\tgrad = " << (tEndGradientUC - tStartGradientUC) / 1000. << std::endl;
  std::cout << std::endl;

#ifdef VISP_HAVE_DISPLAY
  if (options.m_useDisplay) {
    std::shared_ptr<vpDisplay> disp_input = vpDisplayFactory::createDisplay(Iload, -1, -1, "Input color image", vpDisplay::SCALE_AUTO);
    int posX = disp_input->getWidth() + 20;
    int posY = disp_input->getHeight() + 20;
    std::shared_ptr<vpDisplay> disp_canny = vpDisplayFactory::createDisplay(I_canny_hsvuc, posX, -1, "HSV UC Canny", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_input_uc = vpDisplayFactory::createDisplay(Iin_convert, -1, posY, "Input converted image", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_canny_uc = vpDisplayFactory::createDisplay(I_canny_uc, posX, posY, "UC Canny", vpDisplay::SCALE_AUTO);

    std::shared_ptr<vpDisplay> disp_GI_hsvuc_imgfilter = vpDisplayFactory::createDisplay(GIdisp_hsvuc_imgfilter, 2 * posX, -1, "Gradient HSV");
    vpDisplay::display(GIdisp_hsvuc_imgfilter);
    vpDisplay::flush(GIdisp_hsvuc_imgfilter);

    std::shared_ptr<vpDisplay> disp_GI_uc = vpDisplayFactory::createDisplay(GIdisp_uc, 2 * posX, posY, "Gradient (unsigned char)");
    vpDisplay::display(GIdisp_uc);
    vpDisplay::flush(GIdisp_uc);

    vpDisplay::display(Iin_convert);
    vpDisplay::flush(Iin_convert);
    vpDisplay::display(I_canny_uc);
    vpDisplay::flush(I_canny_uc);
    vpDisplay::display(I_canny_hsvuc);
    vpDisplay::flush(I_canny_hsvuc);

    vpDisplay::display(Iload);
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
int main()
{
  std::cout << "C++11 is needed to work with vpHSV." << std::endl;
  return EXIT_SUCCESS;
}
#endif
