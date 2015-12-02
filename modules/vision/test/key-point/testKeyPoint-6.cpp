/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
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
 * Test descriptor computation.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpKeyPoint.h>

// List of allowed command line options
#define GETOPTARGS	"cdh"

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
Test keypoint descriptor extraction.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
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
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  Get the string OpenCV type.

  \param type : OpenCV type.

  \return The string OpenCV type.

*/
std::string getOpenCVType(const int type) {
  std::string type_string = "";

  switch(type) {
  case CV_8U:
    type_string = "CV_8U";
    break;

  case CV_8S:
    type_string = "CV_8S";
    break;

  case CV_16U:
    type_string = "CV_16U";
    break;

  case CV_16S:
    type_string = "CV_16S";
    break;

  case CV_32S:
    type_string = "CV_32S";
    break;

  case CV_32F:
    type_string = "CV_32F";
    break;

  case CV_64F:
    type_string = "CV_64F";
    break;

  default:
    type_string = "Problem with type !";
    break;
  }

  return type_string;
}

/*!
  \example testKeyPoint-6.cpp

  \brief   Test descriptor extraction.
*/
int main(int argc, const char ** argv) {
  try {
    std::string env_ipath;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit (-1);
    }

    //Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if(env_ipath.empty()) {
      std::cerr << "Please set the VISP_INPUT_IMAGE_PATH environment variable value." << std::endl;
      return -1;
    }

    vpImage<unsigned char> I;

    //Set the path location of the image sequence
    std::string dirname = vpIoTools::createFilePath(env_ipath, "ViSP-images/Klimt");

    //Build the name of the image files
    std::string filename = vpIoTools::createFilePath(dirname, "/Klimt.png");
    vpImageIo::read(I, filename);

#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#else
    vpDisplayOpenCV display;
#endif

    if(opt_display) {
      display.init(I, 0, 0, "KeyPoints detection.");
    }

    vpKeyPoint keyPoints;

    std::vector<std::string> descriptorNames;
#if defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)
    descriptorNames.push_back("SIFT");
    descriptorNames.push_back("SURF");
#endif
    descriptorNames.push_back("ORB");
#if (VISP_HAVE_OPENCV_VERSION >= 0x020403)
    descriptorNames.push_back("BRISK");
#endif
#if defined(VISP_HAVE_OPENCV_XFEATURES2D) || (VISP_HAVE_OPENCV_VERSION < 0x030000)
    descriptorNames.push_back("BRIEF");
#if (VISP_HAVE_OPENCV_VERSION >= 0x020402)
    descriptorNames.push_back("FREAK");
#endif
#endif
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
    descriptorNames.push_back("DAISY");
    descriptorNames.push_back("LATCH");
//    descriptorNames.push_back("LUCID"); //Problem
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    descriptorNames.push_back("KAZE");
    descriptorNames.push_back("AKAZE");
#endif

    std::string detectorName = "FAST";
    keyPoints.setDetector(detectorName);
    std::vector<cv::KeyPoint> kpts;
    double elapsedTime;
    keyPoints.detect(I, kpts, elapsedTime);
    std::cout << "Nb keypoints detected: " << kpts.size() << " for " << detectorName << " method." << std::endl;
    if(kpts.empty()) {
      std::cerr << "No keypoints detected with " << detectorName << " and image:" << filename << "." << std::endl;
      return -1;
    }

    for(std::vector<std::string>::const_iterator itd = descriptorNames.begin(); itd != descriptorNames.end(); ++itd) {
      if(*itd == "KAZE") {
        detectorName = "KAZE";
        keyPoints.setDetector(detectorName);
        keyPoints.detect(I, kpts, elapsedTime);
        std::cout << "Nb keypoints detected: " << kpts.size() << " for " << detectorName << " method." << std::endl;
        if(kpts.empty()) {
          std::cerr << "No keypoints detected with " << detectorName << " and image:" << filename << "." << std::endl;
          return -1;
        }
      } else if(*itd == "AKAZE") {
        detectorName = "AKAZE";
        keyPoints.setDetector(detectorName);
        keyPoints.detect(I, kpts, elapsedTime);
        std::cout << "Nb keypoints detected: " << kpts.size() << " for " << detectorName << " method." << std::endl;
        if(kpts.empty()) {
          std::cerr << "No keypoints detected with " << detectorName << " and image:" << filename << "." << std::endl;
          return -1;
        }
      }

      keyPoints.setExtractor(*itd);

      double t = vpTime::measureTimeMs();
      cv::Mat descriptor;
      keyPoints.extract(I, kpts, descriptor, elapsedTime);
      t = vpTime::measureTimeMs() - t;

      std::cout << "Descriptor: " << descriptor.rows << "x" << descriptor.cols << " (rows x cols) ; type=" <<
          getOpenCVType(descriptor.type()) << " for " << *itd << " method in " << t << " ms." << std::endl;
      if(descriptor.empty()) {
        std::cerr << "No descriptor extracted with " << *itd << " and image:" << filename << "." << std::endl;
        return -1;
      }

      if (opt_display) {
        vpDisplay::display(I);

        for(std::vector<cv::KeyPoint>::const_iterator it = kpts.begin(); it != kpts.end(); ++it) {
          vpImagePoint imPt;
          imPt.set_uv(it->pt.x, it->pt.y);

          vpDisplay::displayCross(I, imPt, 4, vpColor::red);
        }

        vpDisplay::flush(I);

        if(opt_click_allowed) {
          vpDisplay::getClick(I);
        }
      }
    }

  } catch(vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "testKeyPoint-6 is ok !" << std::endl;
  return 0;
}
#else
int main() {
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
