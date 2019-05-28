/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Example of dot tracking.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file trackKltOpencv.cpp

  \brief Example of KLT tracking using OpenCV library.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <vector>

#if defined(VISP_HAVE_MODULE_KLT) &&                                                                                   \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/klt/vpKltOpencv.h>

// List of allowed command line options
#define GETOPTARGS "cdf:i:n:p:s:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned nimages, unsigned step);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &nimages,
                unsigned &step, bool &click_allowed, bool &display);
/*!
  \example trackKltOpencv.cpp

  Example of KLT tracking using OpenCV (Intel's Open source Computer
  Vision library).
*/

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.


*/
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned nimages, unsigned step)
{
  fprintf(stdout, "\n\
Example of KLT tracking using OpenCV library.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-f <first image>] [-n <number of images>] [-s <step>]\n\
     [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"mire-2/image.%%04d.pgm\". These \n\
     images come from ViSP-images-x.y.z.tar.gz available \n\
     on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
 -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/cube/image.%%04d.pgm\"\n\
     %%04d is for the image numbering.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -n <number of images>                                %u\n\
     Number of images to load from the sequence.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n", ipath.c_str(), ppath.c_str(), first, nimages, step);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param ppath : Personal image path.
  \param first : First image.
  \param nimages : Number of images to display.
  \param step : Step between two images.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &nimages,
                unsigned &step, bool &click_allowed, bool &display)
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
    case 'i':
      ipath = optarg_;
      break;
    case 'p':
      ppath = optarg_;
      break;
    case 'f':
      first = (unsigned)atoi(optarg_);
      break;
    case 'n':
      nimages = (unsigned)atoi(optarg_);
      break;
    case 's':
      step = (unsigned)atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, ipath, ppath, first, nimages, step);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath, first, nimages, step);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath, first, nimages, step);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_ppath;
    std::string dirname;
    std::string filename;
    unsigned opt_first = 1;
    unsigned opt_nimages = 500;
    unsigned opt_step = 1;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_nimages, opt_step, opt_click_allowed,
                   opt_display) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty() && opt_ppath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty() && opt_ppath.empty()) {
      usage(argv[0], NULL, ipath, opt_ppath, opt_first, opt_nimages, opt_step);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << "  Use -p <personal image path> option if you want to " << std::endl
                << "  use personal images." << std::endl
                << std::endl;

      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> vpI; // This is a ViSP image used for display only
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    IplImage *cvI = NULL; // This is an OpenCV IPL image used by the tracker
#else
    cv::Mat cvI;
#endif

    unsigned iter = opt_first;
    std::ostringstream s;
    char cfilename[FILENAME_MAX];

    if (opt_ppath.empty()) {

      // Warning :
      // the image sequence is not provided with the ViSP package
      // therefore the program will return you an error :
      //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file
      //        ViSP-images/mire-2/image.0001.pgm
      //  !!    vpDotExample.cpp: main(#95) :Error while reading the image
      //  terminate called after throwing an instance of 'vpImageException'
      //
      //  The sequence is available on the visp www site
      //  https://visp.inria.fr/download/
      //  in the download section. It is named "ViSP-images.tar.gz"

      // Set the path location of the image sequence
      dirname = vpIoTools::createFilePath(ipath, "mire-2");

      // Build the name of the image file

      s.setf(std::ios::right, std::ios::adjustfield);
      s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
      filename = vpIoTools::createFilePath(dirname, s.str());
    } else {
      sprintf(cfilename, opt_ppath.c_str(), iter);
      filename = cfilename;
    }

    // Read the PGM image named "filename" on the disk, and put the
    // bitmap into the image structure I.  I is initialized to the
    // correct size
    //
    // exception readPGM may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      std::cout << "Load: " << filename << std::endl;

      // Load a ViSP image used for the display
      vpImageIo::read(vpI, filename);
      vpImageConvert::convert(vpI, cvI);
    } catch (...) {
      // an exception is throwned if an exception from readPGM has been
      // catched here this will result in the end of the program Note that
      // another error message has been printed from readPGM to give more
      // information about the error
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      std::cerr << "  Check your -i " << ipath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      exit(-1);
    }

// We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display.init(vpI, 100, 100, "Display...");
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(vpI);
      vpDisplay::flush(vpI);
    }

    // KLT tracker
    vpKltOpencv tracker;

    // Event manager
    // tracker.setOnNewFeature(&newFeature);
    // tracker.setOnFeatureLost(&lostFeature);
    // tracker.setIsFeatureValid(&isValid);

    // Tracker parameters
    tracker.setTrackerId(1);
    // tracker.setOnMeasureFeature(&modifyFeature);
    tracker.setMaxFeatures(200);
    tracker.setWindowSize(10);
    tracker.setQuality(0.01);
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);

    // Point detection using Harris. In input we have an OpenCV IPL image
    tracker.initTracking(cvI);

    if (opt_display) {
      // Plot the Harris points on ViSP image
      tracker.display(vpI, vpColor::red);
    }

    // tracking is now initialized. We can start the tracker.
    while (iter < opt_first + opt_nimages * opt_step) {
      // set the new image name
      if (opt_ppath.empty()) {
        s.str("");
        s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
        filename = vpIoTools::createFilePath(dirname, s.str());
      } else {
        sprintf(cfilename, opt_ppath.c_str(), iter);
        filename = cfilename;
      }
      // read the image
      vpImageIo::read(vpI, filename);
      vpImageConvert::convert(vpI, cvI);

      // track the dot and returns its coordinates in the image
      // results are given in float since many many are usually considered
      //
      // an expcetion is thrown by the track method if
      //  - dot is lost

      if (opt_display) {
        // Display the image
        vpDisplay::display(vpI);
      }

      std::cout << "Tracking on image: " << filename << std::endl;
      double time = vpTime::measureTimeMs();
      // Tracking of the detected points
      tracker.track(cvI);
      std::cout << "Tracking performed in " << vpTime::measureTimeMs() - time << " ms" << std::endl;

      if (opt_display) {
        // Display the tracked points
        tracker.display(vpI, vpColor::red);

        vpDisplay::flush(vpI);
      }
      iter += opt_step;
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "\nA click to exit..." << std::endl;
      // Wait for a blocking mouse click
      vpDisplay::getClick(vpI);
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You do not have OpenCV functionalities to display images..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install OpenCV, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
#else
#include <iostream>

int main()
{
  std::cout << "visp_klt module or X11, GTK, GDI or OpenCV display "
               "functionalities are required..."
            << std::endl;
}

#endif
