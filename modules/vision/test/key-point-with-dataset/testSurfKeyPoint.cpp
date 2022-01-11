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
 * Test auto detection of dots.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)) // Require opencv >= 1.1.0 < 3.0.0

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpKeyPointSurf.h>

/*!
  \example testSurfKeyPoint.cpp

  \brief   Test matching points thanks to the surf key points.
*/

// List of allowed command line options
#define GETOPTARGS "cdi:h"

void usage(const char *name, const char *badparam, std::string ipath);
bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.

*/
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
Test dot tracking.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read image \n\
     \"ellipse/ellipse.pgm\"\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display)
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
    case 'h':
      usage(argv[0], NULL, ipath);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
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
    std::string dirname;
    std::string filenameRef;
    std::string filenameCur;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> Iref;
    vpImage<unsigned char> Icur;

    // Set the path location of the image sequence
    dirname = vpIoTools::createFilePath(ipath, "cube");

    // Build the name of the image file
    filenameRef = vpIoTools::createFilePath(dirname, "image.0000.pgm");
    filenameCur = vpIoTools::createFilePath(dirname, "image.0079.pgm");

    // Read the PGM image named "filename" on the disk, and put the
    // bitmap into the image structure I.  I is initialized to the
    // correct size
    //
    // exception readPGM may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      std::cout << "Load: " << filenameRef << std::endl;

      vpImageIo::read(Iref, filenameRef);

      std::cout << "Load: " << filenameCur << std::endl;

      vpImageIo::read(Icur, filenameCur);
    } catch (...) {
      // an exception is throwned if an exception from readPGM has been
      // catched here this will result in the end of the program Note that
      // another error message has been printed from readPGM to give more
      // information about the error
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filenameRef << "or" << filenameCur << std::endl;
      std::cerr << "  Check your -i " << ipath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      exit(-1);
    }

// We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
    vpDisplayX display[2];
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display[2];
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display[2];
#else
    vpDisplayOpenCV display[2];
#endif

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display[0].init(Iref, 100, 100, "Reference image");
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(Iref);
      // Flush the display
      vpDisplay::flush(Iref);
    }

    vpKeyPointSurf surf;
    unsigned int nbrRef;

    if (opt_click_allowed && opt_display) {
      std::cout << "Select a part of the image where the reference points "
                   "will be computed. This part is a rectangle."
                << std::endl;
      std::cout << "Click first on the top left corner and then on the "
                   "bottom right corner."
                << std::endl;
      vpImagePoint corners[2];
      for (int i = 0; i < 2; i++) {
        vpDisplay::getClick(Iref, corners[i]);
      }

      vpDisplay::displayRectangle(Iref, corners[0], corners[1], vpColor::red);
      vpDisplay::flush(Iref);
      unsigned int height, width;
      height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
      width = (unsigned int)(corners[1].get_j() - corners[0].get_j());

      // Computes the reference points
      nbrRef = surf.buildReference(Iref, corners[0], height, width);
    }

    else {
      nbrRef = surf.buildReference(Iref);
    }

    if (nbrRef < 1) {
      std::cerr << "No reference point" << std::endl;
      exit(-1);
    }

    unsigned int nbrPair;
    if (opt_display) {
      display[1].init(Icur, (int)(100 + Iref.getWidth()), 100, "Current image");
      // display variable.
      vpDisplay::display(Icur);
      // Flush the display
      vpDisplay::flush(Icur);
    }

    if (opt_click_allowed && opt_display) {
      std::cout << "Select a part of the current image where the reference "
                   "will be search. This part is a rectangle."
                << std::endl;
      std::cout << "Click first on the top left corner and then on the "
                   "bottom right corner."
                << std::endl;
      vpImagePoint corners[2];
      for (int i = 0; i < 2; i++) {
        vpDisplay::getClick(Icur, corners[i]);
      }
      vpDisplay::displayRectangle(Icur, corners[0], corners[1], vpColor::green);
      vpDisplay::flush(Icur);
      unsigned int height, width;
      height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
      width = (unsigned int)(corners[1].get_j() - corners[0].get_j());

      // Computes the reference points
      nbrPair = surf.matchPoint(Icur, corners[0], height, width);
    }

    else {
      nbrPair = surf.matchPoint(Icur);
    }

    if (nbrPair < 1) {
      std::cout << "No point matched" << std::endl;
    }

    if (opt_display) {
      surf.display(Iref, Icur, 7);
      vpDisplay::flush(Iref);
      vpDisplay::flush(Icur);
      if (opt_click_allowed) {
        std::cout << "A click on the reference image to exit..." << std::endl;
        vpDisplay::getClick(Iref);
      }
    }
    return (0);
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return (1);
  }
}
#else
int main()
{
  std::cerr << "You do not have 1.1.0 <= OpenCV < 2.3.0 that contains "
               "opencv_nonfree component..."
            << std::endl;
}

#endif
