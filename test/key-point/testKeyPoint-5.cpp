/****************************************************************************
 *
 * $Id: testKeyPoint-5.cpp 5202 2015-01-24 09:29:06Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Test keypoints detection with OpenCV, specially the Pyramid implementation
 * feature misssing in OpenCV 3.0.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iostream>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <visp/vpKeyPoint.h>

// List of allowed command line options
#define GETOPTARGS	"cdi:h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test key points matching.\n\
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
  \example testKeyPoint-5.cpp

  \brief   Test keypoints detection with OpenCV, specially the Pyramid implementation
  feature misssing in OpenCV 3.0.
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
      std::cerr << "Please get the visp-images-data package path or set the VISP_INPUT_IMAGE_PATH "
          "environment variable value." << std::endl;
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

    std::vector<std::string> detectorNames;
    detectorNames.push_back("PyramidFAST");
    detectorNames.push_back("FAST");
    detectorNames.push_back("PyramidMSER");
    detectorNames.push_back("MSER");
    detectorNames.push_back("PyramidGFTT");
    detectorNames.push_back("GFTT");
    detectorNames.push_back("PyramidSimpleBlob");
    detectorNames.push_back("SimpleBlob");
    //In contrib modules
//    detectorNames.push_back("PyramidSTAR");
//    detectorNames.push_back("STAR");

    for(std::vector<std::string>::const_iterator it = detectorNames.begin(); it != detectorNames.end(); ++it) {
      keyPoints.setDetector(*it);

      std::vector<cv::KeyPoint> kpts;
      double elapsedTime;
      keyPoints.detect(I, kpts, elapsedTime);
      std::cout << "Nb keypoints detected:" << kpts.size() << " for " << *it << " method." << std::endl;
      if(kpts.empty()) {
        std::cerr << "No keypoints detected with " << *it << " and image:" << filename << "." << std::endl;
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

  return 0;
}
#else
int main() {
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
