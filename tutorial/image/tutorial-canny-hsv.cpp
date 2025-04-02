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
#include <visp3/core/vpRGBa.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)
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

  SoftwareArguments()
    : m_img("")
    , m_gaussianKernelSize(3)
    , m_gaussianStdev(1.)
    , m_lowerThresh(-1.)
    , m_upperThresh(-1.)
    , m_lowerThreshRatio(0.6f)
    , m_upperThreshRatio(0.8f)
    , m_filteringType(vpImageFilter::CANNY_COUNT_FILTERING)
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
    << " [-r, --ratio <lowerThreshRatio upperThreshRatio>]"
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
  vpImage<vpHSV<double>> I_canny_input;
  if (!options.m_img.empty()) {
    // Detection on the user image
    vpImageIo::read(Iload, options.m_img);
  }
  else {
    std::cout << "This example only works on a real image. Please use the -i option." << std::endl;
    return EXIT_SUCCESS;
  }
  vpImageConvert::convert(Iload, I_canny_input);
  vpImage<unsigned char> I_canny_visp = cannyDetector.detect(I_canny_input);

  vpCannyEdgeDetection cannyDetectorUC(options.m_gaussianKernelSize, options.m_gaussianStdev, uselessAperture,
    options.m_lowerThresh, options.m_upperThresh, options.m_lowerThreshRatio, options.m_upperThreshRatio);
  vpImage<unsigned char> Iin_convert;
  vpImageConvert::convert(Iload, Iin_convert);
  vpImage<unsigned char> I_canny_uc = cannyDetectorUC.detect(Iin_convert);

  // Initialization of the displays
#ifdef VISP_HAVE_DISPLAY
  std::shared_ptr<vpDisplay> disp_input = vpDisplayFactory::createDisplay(Iload, -1, -1, "Input color image", vpDisplay::SCALE_AUTO);
  int posX = disp_input->getWidth() + 20;
  int posY = disp_input->getHeight() + 20;
  std::shared_ptr<vpDisplay> disp_canny = vpDisplayFactory::createDisplay(I_canny_visp, posX, -1, "HSV Canny", vpDisplay::SCALE_AUTO);
  std::shared_ptr<vpDisplay> disp_input_uc = vpDisplayFactory::createDisplay(Iin_convert, -1, posY, "Input converted image", vpDisplay::SCALE_AUTO);
  std::shared_ptr<vpDisplay> disp_canny_uc = vpDisplayFactory::createDisplay(I_canny_uc, posX, posY, "UC Canny", vpDisplay::SCALE_AUTO);
  vpDisplay::display(Iload);
  vpDisplay::flush(Iload);
  vpDisplay::display(Iin_convert);
  vpDisplay::flush(Iin_convert);
  vpDisplay::display(I_canny_uc);
  vpDisplay::flush(I_canny_uc);
  vpDisplay::display(I_canny_visp);
  vpDisplay::displayText(I_canny_visp, vpImagePoint(20, 20), "Click to leave.", vpColor::red);
  vpDisplay::flush(I_canny_visp);
  vpDisplay::getClick(I_canny_visp);
#else
  vpImageIo::write(I_canny_visp, "HSVCanny.jpg");
#endif
  return EXIT_SUCCESS;
}
#else
void main()
{
  std::cout << "C++11 is needed to work with vpHSV." << std::endl;
}
#endif
