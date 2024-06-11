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
 * Description:
 * Example of dot tracking.
 *
*****************************************************************************/
/*!
  \file trackDot2.cpp

  \brief Example of dot tracking on an image sequence using vpDot2.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#if defined(VISP_HAVE_MODULE_BLOB) &&                                                                                  \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "cdf:i:l:p:s:h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
  \example trackDot2.cpp
  Example of dot tracking on an image sequence using vpDot2.
*/

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param last : Last image.
  \param step : Step between two images.

*/
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned last, unsigned step)
{
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
  fprintf(stdout, "\n\
Test dot tracking using vpDot2 class.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-f <first image>] [-l <last image>] [-s <step>]\n\
     [-c] [-d] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"mire-2/image.%%04d.%s\". These \n\
     images come from visp-images-x.y.z.tar.gz available \n\
     on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
 -p <personal image path>                              %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     Example : \"C:/Temp/visp-images/cube/image.%%04d.%s\"\n\
     %%04d is for the image numbering.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -l <last image>                                      %u\n\
     Last image number of the sequence.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n",
          ipath.c_str(), ext.c_str(), ppath.c_str(), ext.c_str(), first, last, step);

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
  \param last : Last image.
  \param step : Step between two images.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &last,
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
    case 'l':
      last = (unsigned)atoi(optarg_);
      break;
    case 's':
      step = (unsigned)atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, ppath, first, last, step);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath, first, last, step);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, ppath, first, last, step);
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
    unsigned opt_last = 500;
    unsigned opt_step = 1;
    bool opt_click_allowed = true;
    bool opt_display = true;

#if VISP_HAVE_DATASET_VERSION >= 0x030600
    std::string ext("png");
#else
    std::string ext("pgm");
#endif

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_last, opt_step, opt_click_allowed,
                   opt_display) == false) {
      return EXIT_FAILURE;
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path coming from the command line option
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
      usage(argv[0], nullptr, ipath, opt_ppath, opt_first, opt_last, opt_step);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << "  Use -p <personal image path> option if you want to " << std::endl
        << "  use personal images." << std::endl
        << std::endl;

      return EXIT_FAILURE;
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> I;

    unsigned iter = opt_first;
    std::ostringstream s;
    char cfilename[FILENAME_MAX];

    if (opt_ppath.empty()) {

      // Warning :
      // The image sequence is not provided with the ViSP package
      // therefore the program will return an error :
      //  !!    couldn't read file visp-images/mire-2/image.0001.png
      //
      // ViSP dataset is available on the visp www site
      // https://visp.inria.fr/download/.

      // Set the path location of the image sequence
      dirname = vpIoTools::createFilePath(ipath, "mire-2");

      // Build the name of the image file

      s.setf(std::ios::right, std::ios::adjustfield);
      s << "image." << std::setw(4) << std::setfill('0') << iter << "." << ext;
      filename = vpIoTools::createFilePath(dirname, s.str());
    }
    else {
      snprintf(cfilename, FILENAME_MAX, opt_ppath.c_str(), iter);
      filename = cfilename;
    }

    // Read the image named "filename", and put the bitmap into the image structure I.
    // I is initialized to the correct size
    //
    // vpImageIo::read() may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      vpCTRACE << "Load: " << filename << std::endl;

      vpImageIo::read(I, filename);
    }
    catch (...) {
   // If an exception is thrown by vpImageIo::read() it will result in the end of the program.
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      if (opt_ppath.empty()) {
        std::cerr << "  Check your -i " << ipath << " option " << std::endl
          << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      }
      else {
        std::cerr << "  Check your -p " << opt_ppath << " option " << std::endl;
      }
      return EXIT_FAILURE;
    }

// We open a window using either X11, GTK or GDI.
#if defined(VISP_HAVE_X11)
    vpDisplayX display;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display;
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100, "Display...");
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    // define the vpDot structure.

    // vpDot and vpDot2 correspond to two different algorithms designed to
    // track a dot. vpDot is based on recurse connex componants (all the
    // pixels of the dot are parsed), while vpDot2 is based on freeman chain
    // code (only the contour of the dot is parsed)

    vpDot2 d;
    vpImagePoint cog;

    if (opt_display) {
      // by using setGraphics, we request to see the all the pixel of the dot
      // in green on the screen.
      // It uses the overlay image plane.
      // The default of this setting is that it is time consuming

      d.setGraphics(true);
    }
    else {

      d.setGraphics(false);
    }
    // We want to track an ellipsoid shape. If you want to track a non
    // ellipsoid object, use d.setEllipsoidShape(0); we also request to
    // compute the dot moment m00, m10, m01, m11, m20, m02
    d.setComputeMoments(true);
    d.setGrayLevelPrecision(0.90);

    // tracking is initalized if no other parameters are given to the
    // iniTracking(..) method a right mouse click on the dot is expected
    // dot location can also be specified explicitly in the
    // initTracking method : d.initTracking(I,ip) where ip is the image
    // point from which the dot is searched

    if (opt_display && opt_click_allowed) {
      std::cout << "Click on a dot to track it." << std::endl;
      d.initTracking(I);
    }
    else {
      vpImagePoint ip;
      ip.set_u(160);
      ip.set_v(212);
      d.initTracking(I, ip);
    }
    if (1) {
      std::cout << "COG: " << std::endl;
      cog = d.getCog();
      std::cout << "  u: " << cog.get_u() << " v: " << cog.get_v() << std::endl;
      std::cout << "Size:" << std::endl;
      std::cout << "  w: " << d.getWidth() << " h: " << d.getHeight() << std::endl;
      std::cout << "Area: " << d.getArea() << std::endl;
      std::cout << "Centered normalized moments nij:" << std::endl;
      std::cout << "  n20: " << d.get_nij()[0] << std::endl;
      std::cout << "  n11: " << d.get_nij()[1] << std::endl;
      std::cout << "  n02: " << d.get_nij()[2] << std::endl;
      std::cout << "Settings:" << std::endl;
      std::cout << "  gray level min: " << d.getGrayLevelMin() << std::endl;
      std::cout << "  gray level max: " << d.getGrayLevelMax() << std::endl;
      std::cout << "  size precision: " << d.getSizePrecision() << std::endl;
      std::cout << "  gray level precision: " << d.getGrayLevelPrecision() << std::endl;
    }

    while (iter < opt_last) {
      // set the new image name
      if (opt_ppath.empty()) {
        s.str("");
        s << "image." << std::setw(4) << std::setfill('0') << iter << "." << ext;
        filename = vpIoTools::createFilePath(dirname, s.str());
      }
      else {
        snprintf(cfilename, FILENAME_MAX, opt_ppath.c_str(), iter);
        filename = cfilename;
      }
      // read the image
      std::cout << "read : " << filename << std::endl;
      vpImageIo::read(I, filename);

      // track the dot and returns its coordinates in the image
      // results are given in float since many many are usually considered
      //
      // an exception is thrown by the track method if
      //  - dot is lost

      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
      }

      std::cout << "Tracking on image: " << filename << std::endl;
      double time = vpTime::measureTimeMs();
      d.track(I);

      std::cout << "COG (" << vpTime::measureTimeMs() - time << " ms): " << std::endl;
      cog = d.getCog();
      std::cout << "  u: " << cog.get_u() << " v: " << cog.get_v() << std::endl;
      std::cout << "Size:" << std::endl;
      std::cout << "  w: " << d.getWidth() << " h: " << d.getHeight() << std::endl;
      std::cout << "Area: " << d.getArea() << std::endl;
      std::cout << "Centered normalized moments nij:" << std::endl;
      std::cout << "  n20: " << d.get_nij()[0] << std::endl;
      std::cout << "  n11: " << d.get_nij()[1] << std::endl;
      std::cout << "  n02: " << d.get_nij()[2] << std::endl;
      std::cout << "Settings:" << std::endl;
      std::cout << "  gray level min: " << d.getGrayLevelMin() << std::endl;
      std::cout << "  gray level max: " << d.getGrayLevelMax() << std::endl;
      std::cout << "  size precision: " << d.getSizePrecision() << std::endl;
      std::cout << "  gray level precision: " << d.getGrayLevelPrecision() << std::endl;

      if (opt_display) {
        if (0) {
          std::list<vpImagePoint> edges;
          d.getEdges(edges);
          std::list<vpImagePoint>::const_iterator it;
          for (it = edges.begin(); it != edges.end(); ++it) {
            vpDisplay::displayPoint(I, *it, vpColor::blue);
          }
        }

        // display a green cross (size 10) in the image at the dot center
        // of gravity location
        vpDisplay::displayCross(I, cog, 10, vpColor::green);
        // flush the X11 buffer

        vpDisplay::flush(I);
      }
      iter++;
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "\nA click to exit..." << std::endl;
      // Wait for a blocking mouse click
      vpDisplay::getClick(I);
    }
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
#include <iostream>

int main()
{
  std::cout << "visp_me module or X11, GTK, GDI or OpenCV display "
    "functionalities are required..."
    << std::endl;
  return EXIT_SUCCESS;
  }

#endif
