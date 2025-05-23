/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Test keypoints detection with OpenCV, specially the Pyramid implementation
 * feature misssing in OpenCV 3.0.
 */

/*!
  \example testKeyPoint-5.cpp

  \brief   Test keypoints detection with OpenCV, specially the Pyramid
  implementation feature missing in OpenCV 3.0.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x050000)  && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_FEATURES2D)) || \
   ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_3D) && defined(HAVE_OPENCV_FEATURES)))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpKeyPoint.h>

// List of allowed command line options
#define GETOPTARGS "cdh"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test keypoints detection.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], nullptr);
      return false;

    default:
      usage(argv[0], optarg_);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

template <typename Type>
void run_test(const std::string &env_ipath, bool opt_click_allowed, bool opt_display, vpImage<Type> &Iinput,
              vpImage<Type> &I)
{
  // Set the path location of the image sequence
  std::string dirname = vpIoTools::createFilePath(env_ipath, "Klimt");

  // Build the name of the image files
  std::string filename = vpIoTools::createFilePath(dirname, "/Klimt.png");
  vpImageIo::read(Iinput, filename);
  Iinput.halfSizeImage(I);

  vpDisplay *display = nullptr;

  if (opt_display) {
#ifdef VISP_HAVE_DISPLAY
    display = vpDisplayFactory::allocateDisplay(I, 0, 0, "KeyPoints detection.");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
  }

  // Here, we want to test feature detection on a pyramid of images even for
  // features that are scale invariant to detect potential problem in ViSP.
  std::cout << "INFORMATION: " << std::endl;
  std::cout << "Here, we want to test feature detection on a pyramid of images  even for features "
    << "that are scale invariant to detect potential problem in ViSP." << std::endl
    << std::endl;
  vpKeyPoint keyPoints;

  // Will test the different types of keypoints detection to see if there is
  // a problem  between OpenCV versions, modules or constructors
  std::vector<std::string> detectorNames;
#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_FEATURES2D)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES)))
  detectorNames.push_back("PyramidFAST");
  detectorNames.push_back("FAST");
#endif
#if ((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_FEATURES2D)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES))
  detectorNames.push_back("PyramidMSER");
  detectorNames.push_back("MSER");
  detectorNames.push_back("PyramidGFTT");
  detectorNames.push_back("GFTT");
  detectorNames.push_back("PyramidSimpleBlob");
  detectorNames.push_back("SimpleBlob");
#endif

  // In contrib modules
#if defined(HAVE_OPENCV_XFEATURES2D)
  detectorNames.push_back("PyramidSTAR");
  detectorNames.push_back("STAR");
#endif
#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_FEATURES2D)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES)))
  detectorNames.push_back("PyramidORB");
  detectorNames.push_back("ORB");
#endif
#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_FEATURES2D)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_XFEATURES2D)))
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  detectorNames.push_back("PyramidAGAST");
  detectorNames.push_back("AGAST");
#endif
  detectorNames.push_back("PyramidBRISK");
  detectorNames.push_back("BRISK");
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  detectorNames.push_back("PyramidKAZE");
  detectorNames.push_back("KAZE");
  detectorNames.push_back("PyramidAKAZE");
  detectorNames.push_back("AKAZE");
#endif
#endif

#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_XFEATURES2D)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES)))
#if (VISP_HAVE_OPENCV_VERSION != 0x040504) && (VISP_HAVE_OPENCV_VERSION != 0x040505) && \
    (VISP_HAVE_OPENCV_VERSION != 0x040600) && (VISP_HAVE_OPENCV_VERSION != 0x040700) && \
    (VISP_HAVE_OPENCV_VERSION != 0x040900) && (VISP_HAVE_OPENCV_VERSION != 0x040A00) && \
    (defined(__APPLE__) && defined(__MACH__))
  // SIFT is known to be unstable with OpenCV 4.5.4 and 4.5.5 on macOS (see #1048)
  // Same for OpenCV 4.6.0 (see #1106) where it produces an Illegal Instruction error when OpenCV 4.6.0 is
  // installed with brew. It seems working when OpenCV is build from source
  std::cout << "-- Add SIFT detector" << std::endl;
  detectorNames.push_back("PyramidSIFT");
  detectorNames.push_back("SIFT");
#endif
#endif
#if defined(OPENCV_ENABLE_NONFREE) && defined(HAVE_OPENCV_XFEATURES2D)
  detectorNames.push_back("PyramidSURF");
  detectorNames.push_back("SURF");
#endif

  for (std::vector<std::string>::const_iterator itd = detectorNames.begin(); itd != detectorNames.end(); ++itd) {
    keyPoints.setDetector(*itd);

    std::vector<cv::KeyPoint> kpts;

    keyPoints.detect(I, kpts);
    std::cout << "Nb keypoints detected: " << kpts.size() << " for " << *itd << " method." << std::endl;
    if (kpts.empty()) {
      std::stringstream ss;
      ss << "No keypoints detected with " << *itd << " and image: " << filename << "." << std::endl;
      throw(vpException(vpException::fatalError, ss.str()));
    }

    if (opt_display) {
      vpDisplay::display(I);

      for (std::vector<cv::KeyPoint>::const_iterator it = kpts.begin(); it != kpts.end(); ++it) {
        vpImagePoint imPt;
        imPt.set_uv(it->pt.x, it->pt.y);

        vpDisplay::displayCross(I, imPt, 4, vpColor::red);
      }

      vpDisplay::flush(I);

      if (opt_click_allowed) {
        vpDisplay::getClick(I);
      }
    }
  }

  std::cout << "\n\n";

  std::map<vpKeyPoint::vpFeatureDetectorType, std::string> mapOfDetectorNames = keyPoints.getDetectorNames();
  for (int i = 0; i < vpKeyPoint::DETECTOR_TYPE_SIZE; i++) {
#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && ((VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || \
     (VISP_HAVE_OPENCV_VERSION >= 0x040400)) && defined(HAVE_OPENCV_FEATURES2D))) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES))
#if ((VISP_HAVE_OPENCV_VERSION == 0x040504) || (VISP_HAVE_OPENCV_VERSION == 0x040505) ||  \
     (VISP_HAVE_OPENCV_VERSION == 0x040600) || (VISP_HAVE_OPENCV_VERSION == 0x040700) || \
     (VISP_HAVE_OPENCV_VERSION == 0x040900) || (VISP_HAVE_OPENCV_VERSION == 0x040A00)) && \
    (defined(__APPLE__) && defined(__MACH__))
    // SIFT is known to be unstable with OpenCV 4.5.4 and 4.5.5 on macOS (see #1048)
    // Same for OpenCV 4.6.0 (see #1106) where it produces an Illegal Instruction error when OpenCV 4.6.0 is
    // installed with brew. It seems working when OpenCV is build from source
    if (i == vpKeyPoint::DETECTOR_SIFT) {
      std::cout << "-- Skip SIFT detector" << std::endl;
      continue;
    }
#endif
#endif
    keyPoints.setDetector((vpKeyPoint::vpFeatureDetectorType)i);

    std::vector<cv::KeyPoint> kpts;

    keyPoints.detect(I, kpts);
    std::cout << "Nb keypoints detected: " << kpts.size() << " for "
      << mapOfDetectorNames[(vpKeyPoint::vpFeatureDetectorType)i] << " method." << std::endl;
    if (kpts.empty()) {
      std::stringstream ss;
      ss << "No keypoints detected with " << mapOfDetectorNames[(vpKeyPoint::vpFeatureDetectorType)i]
        << " method  and image: " << filename << "." << std::endl;
      throw(vpException(vpException::fatalError, ss.str()));
    }

    if (opt_display) {
      vpDisplay::display(I);

      for (std::vector<cv::KeyPoint>::const_iterator it = kpts.begin(); it != kpts.end(); ++it) {
        vpImagePoint imPt;
        imPt.set_uv(it->pt.x, it->pt.y);

        vpDisplay::displayCross(I, imPt, 4, vpColor::red);
      }

      vpDisplay::flush(I);

      if (opt_click_allowed) {
        vpDisplay::getClick(I);
      }
    }
  }

  if (display) {
    delete display;
  }
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit(EXIT_FAILURE);
    }

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if (env_ipath.empty()) {
      std::cerr << "Please set the VISP_INPUT_IMAGE_PATH environment "
        "variable value."
        << std::endl;
      return EXIT_FAILURE;
    }

    {
      vpImage<unsigned char> Iinput, I;

      std::cout << "-- Test on gray level images" << std::endl;
      run_test(env_ipath, opt_click_allowed, opt_display, Iinput, I);
    }

    {
      vpImage<vpRGBa> Iinput, I;

      std::cout << "-- Test on color images" << std::endl;
      run_test(env_ipath, opt_click_allowed, opt_display, Iinput, I);
    }

  }
  catch (const vpException &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "testKeyPoint-5 is ok !" << std::endl;
  return EXIT_SUCCESS;
}
#else
#include <cstdlib>

int main()
{
  std::cerr << "You need OpenCV library." << std::endl;

  return EXIT_SUCCESS;
}

#endif
