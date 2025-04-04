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

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)

void computeAbsoluteGradient(const vpImage<double> &GIx, const vpImage<double> &GIy, vpImage<double> &GI, double &min, double &max)
{
  const unsigned int h = GIx.getHeight(), w = GIx.getWidth();
  GI.resize(h, w);
  max = -1.;
  min = std::numeric_limits<double>::max();
  for (unsigned int r = 0; r < h; ++r) {
    for (unsigned int c = 0; c < w; ++c) {
      GI[r][c] = std::abs(GIx[r][c]) + std::abs(GIy[r][c]);
      max = std::max(max, GI[r][c]);
      min = std::min(min, GI[r][c]);
    }
  }
}

vpImage<unsigned char> convertToDisplay(const vpImage<double> &GI, const double &min, const double &max)
{
  const unsigned int h = GI.getHeight(), w = GI.getWidth();
  const double range = max - min;
  const double step = range / 256.;
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
    else if (argv_str == "-s" || argv_str == "--save") {
      options.m_saveImages = true;
    }
    else if (argv_str == "-d" || argv_str == "--no-display") {
      options.m_useDisplay = false;
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
  configAsTxt += "\tGaussian filter kernel size = " + std::to_string(options.m_gaussianKernelSize) + "\n";
  configAsTxt += "\tGaussian filter standard deviation = " + std::to_string(options.m_gaussianStdev) + "\n";
  configAsTxt += "\tCanny edge filter thresholds = [" + std::to_string(options.m_lowerThresh) + " ; " + std::to_string(options.m_upperThresh) + "]\n";
  configAsTxt += "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" + std::to_string(options.m_lowerThreshRatio) + " ; " + std::to_string(options.m_upperThreshRatio) + "]\n";
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

  vpImage<bool> *p_mask = nullptr;
  cannyDetector.setMask(p_mask);

  vpImageConvert::convert(Iload, Iin_hsvuc);
  vpImage<unsigned char> I_canny_hsvuc = cannyDetector.detect(Iin_hsvuc);

  vpImageConvert::convert(Iload, Iin_hsvd);
  vpImage<unsigned char> I_canny_hsvd = cannyDetector.detect(Iin_hsvd);

  vpCannyEdgeDetection cannyDetectorUC(options.m_gaussianKernelSize, options.m_gaussianStdev, uselessAperture,
    options.m_lowerThresh, options.m_upperThresh, options.m_lowerThreshRatio, options.m_upperThreshRatio);
  vpImage<unsigned char> Iin_convert;
  vpImageConvert::convert(Iload, Iin_convert);
  vpImage<unsigned char> I_canny_uc = cannyDetectorUC.detect(Iin_convert);

  // Initialization of the displays
#ifdef VISP_HAVE_DISPLAY
  vpImage<double> GIx, GIy, GI;
  vpImage<vpHSV<unsigned char, true>> Iblur_hsvuc;
  vpImageFilter::gaussianBlur(Iin_hsvuc, Iblur_hsvuc, options.m_gaussianKernelSize, options.m_gaussianStdev, true, p_mask);
  vpImageFilter::gradientFilter(Iblur_hsvuc, GIx, GIy, 1, p_mask, options.m_filteringType);
  double min = 0., max = 0.;
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvuc = convertToDisplay(GI, min, max);

  vpImage<vpHSV<unsigned char, true>> Iblur_hsvd;
  vpImageFilter::gaussianBlur(Iin_hsvd, Iblur_hsvd, options.m_gaussianKernelSize, options.m_gaussianStdev, true, p_mask);
  vpImageFilter::gradientFilter(Iblur_hsvd, GIx, GIy, 1, p_mask, options.m_filteringType);
  computeAbsoluteGradient(GIx, GIy, GI, min, max);
  vpImage<unsigned char> GIdisp_hsvd = convertToDisplay(GI, min, max);

  vpImage<double> GIx_uc, GIy_uc, GI_uc;
  vpImageFilter::computePartialDerivatives(Iin_convert, GIx_uc, GIy_uc, true, true, true,
    options.m_gaussianKernelSize, (double)options.m_gaussianStdev, uselessAperture, options.m_filteringType, vpImageFilter::CANNY_VISP_BACKEND, p_mask);
  computeAbsoluteGradient(GIx_uc, GIy_uc, GI_uc, min, max);
  vpImage<unsigned char> GIdisp_uc = convertToDisplay(GI_uc, min, max);

  if (options.m_useDisplay) {
    std::shared_ptr<vpDisplay> disp_input = vpDisplayFactory::createDisplay(Iload, -1, -1, "Input color image", vpDisplay::SCALE_AUTO);
    int posX = disp_input->getWidth() + 20;
    int posY = disp_input->getHeight() + 20;
    std::shared_ptr<vpDisplay> disp_canny = vpDisplayFactory::createDisplay(I_canny_hsvuc, posX, -1, "HSV Canny", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_input_uc = vpDisplayFactory::createDisplay(Iin_convert, -1, posY, "Input converted image", vpDisplay::SCALE_AUTO);
    std::shared_ptr<vpDisplay> disp_canny_uc = vpDisplayFactory::createDisplay(I_canny_uc, posX, posY, "UC Canny", vpDisplay::SCALE_AUTO);

    std::shared_ptr<vpDisplay> disp_GI_hsvuc = vpDisplayFactory::createDisplay(GIdisp_hsvuc, 2 * posX, -1, "Gradient");
    vpDisplay::display(GIdisp_hsvuc);
    vpDisplay::flush(GIdisp_hsvuc);

    std::shared_ptr<vpDisplay> disp_GI_hsvd = vpDisplayFactory::createDisplay(GIdisp_hsvd, 3 * posX, -1, "Gradient");
    vpDisplay::display(GIdisp_hsvd);
    vpDisplay::flush(GIdisp_hsvd);

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
    vpDisplay::displayText(I_canny_hsvuc, vpImagePoint(20, 20), "Click to leave.", vpColor::red);
    vpDisplay::flush(I_canny_hsvuc);
    vpDisplay::getClick(I_canny_hsvuc);
  }
#else
  options.m_saveImages = true;
#endif
  if (options.m_saveImages) {
    std::string basename = vpIoTools::getNameWE(options.m_img);
    vpImageIo::write(I_canny_hsvuc, "Canny_HSVUC_" + basename + ".jpg");
    vpImageIo::write(I_canny_hsvd, "Canny_HSVD_" + basename + ".jpg");
    vpImageIo::write(I_canny_uc, "Canny_UC_" + basename + ".jpg");
    vpImageIo::write(GIdisp_hsvuc, "Gradient_HSVUC_" + basename + ".jpg");
    vpImageIo::write(GIdisp_hsvd, "Gradient_HSVD_" + basename + ".jpg");
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
