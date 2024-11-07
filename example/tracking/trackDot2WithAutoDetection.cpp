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
 * Test auto detection of dots.
 *
*****************************************************************************/
/*!
  \file trackDot2WithAutoDetection.cpp

  \brief Example of auto detection of dots using vpDot2.
*/

/*!
  \example trackDot2WithAutoDetection.cpp

  Example of auto detection of dots using vpDot2.
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
#define GETOPTARGS "cdi:p:f:l:s:S:G:E:h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param last : Last image.
  \param step : Step between two images.
  \param sizePrecision : precision of the size of dots.
  \param grayLevelPrecision : precision of the gray level of dots.
  \param ellipsoidShapePrecision : precision of the ellipsoid shape of dots.


*/
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned last, unsigned step, double sizePrecision, double grayLevelPrecision,
           double ellipsoidShapePrecision)
{
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
  fprintf(stdout, "\n\
Test auto detection of dots using vpDot2.\n\
          \n\
SYNOPSIS\n\
  %s [-i <input image path>] [-p <personal image path>]\n\
     [-f <first image>] [-l <last image>] [-s <step>] \n\
     [-S <size precision>] [-G <gray level precision>]\n\
     [-E <ellipsoid shape precision>] [-c] [-d] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"mire-2/image.%%04d.%s\"\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
            \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     The format is selected by analysing the filename extension.\n\
     Example : \"/Temp/visp-images/mire-2/image.%%04d.%s\"\n\
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
  -S <size precision>                                  %f\n\
     Precision of the size of the dot. \n\
     It is a double precision float witch value is in ]0,1].\n\
     1 means full precision, the sizes (width, heigth, surface) \n\
     of the dots must the same, whereas values close to 0 \n\
     show a very bad precision.\n\
\n\
  -G <gray level precision>                            %f\n\
     Precision of the gray level of the dot. \n\
     It is a double precision float witch value is in ]0,1].\n\
     1 means full precision, the gray level must the same in \n\
     the wall dot, whereas values close to 0 \n\
     show a very bad precision.\n\
\n\
  -E <ellipsoid shape precision>                       %f\n\
     Precision of the ellipsoid shape of the dot. \n\
     It is a double precision float witch value is in [0,1].\n\
     1 means full precision, the shape should be a perfect ellipsoid,\n\
     whereas values close to 0 show a very bad precision.\n\
     0 means the shape of dots is not tested \n\
\n",
ipath.c_str(), ext.c_str(), ppath.c_str(), ext.c_str(), first, last, step, sizePrecision, grayLevelPrecision,
ellipsoidShapePrecision);

  fprintf(stdout, "\
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
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param last : Last image.
  \param step : Step between two images.
  \param sizePrecision : Precision of the size of dots.
  \param grayLevelPrecision : Precision of the gray level of dots.
  \param ellipsoidShapePrecision : Precision of the ellipsoid shape.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &last,
                unsigned &step, double &sizePrecision, double &grayLevelPrecision, double &ellipsoidShapePrecision,
                bool &click_allowed, bool &display)
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
    case 'S':
      sizePrecision = atof(optarg_);
      break;
    case 'G':
      grayLevelPrecision = atof(optarg_);
      break;
    case 'E':
      ellipsoidShapePrecision = atof(optarg_);
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, ppath, first, last, step, sizePrecision, grayLevelPrecision,
            ellipsoidShapePrecision);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath, first, last, step, sizePrecision, grayLevelPrecision,
            ellipsoidShapePrecision);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, ppath, first, last, step, sizePrecision, grayLevelPrecision,
          ellipsoidShapePrecision);
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
    unsigned opt_last = 10;
    unsigned opt_step = 1;
    double opt_sizePrecision = 0.65;
    double opt_grayLevelPrecision = 0.85;
    double opt_ellipsoidShapePrecision = 0.8;
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
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_last, opt_step, opt_sizePrecision,
                   opt_grayLevelPrecision, opt_ellipsoidShapePrecision, opt_click_allowed, opt_display) == false) {
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
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], nullptr, ipath, opt_ppath, opt_first, opt_last, opt_step, opt_sizePrecision,
            opt_grayLevelPrecision, opt_ellipsoidShapePrecision);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << std::endl
        << "  Use -p <personal image path> option if you want to " << std::endl
        << "  use personal images." << std::endl;
      return EXIT_FAILURE;
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> I;
    std::ostringstream s;
    char cfilename[FILENAME_MAX];
    unsigned iter = opt_first; // Image number

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

// We open a window using either GTK, X11 or GDI.
#if defined(VISP_HAVE_GTK)
    vpDisplayGTK display;
#elif defined(VISP_HAVE_X11)
    vpDisplayX display;
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

    // Dot declaration
    vpDot2 d;

    d.setGraphics(true);
    if (opt_click_allowed & opt_display) {
      d.setGrayLevelPrecision(opt_grayLevelPrecision);

      std::cout << "Please click on a dot to initialize detection" << std::endl;

      d.initTracking(I);
      if (opt_display) {
        vpImagePoint cog;
        cog = d.getCog();
        vpDisplay::displayCross(I, cog, 10, vpColor::green);
        vpDisplay::flush(I);
      }
      d.setSizePrecision(opt_sizePrecision);
      d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
      printf("Dot characteristics: \n");
      printf("  width : %lf\n", d.getWidth());
      printf("  height: %lf\n", d.getHeight());
      printf("  area: %lf\n", d.getArea());
      printf("  gray level min: %u\n", d.getGrayLevelMin());
      printf("  gray level max: %u\n", d.getGrayLevelMax());
      printf("  grayLevelPrecision: %lf\n", d.getGrayLevelPrecision());
      printf("  sizePrecision: %lf\n", d.getSizePrecision());
      printf("  ellipsoidShapePrecision: %lf\n", d.getEllipsoidShapePrecision());
    }
    else {
   //  Set dot characteristics for the auto detection
      d.setGraphics(true);
      d.setWidth(15.0);
      d.setHeight(12.0);
      d.setArea(124);
      d.setGrayLevelMin(164);
      d.setGrayLevelMax(255);
      d.setGrayLevelPrecision(opt_grayLevelPrecision);
      d.setSizePrecision(opt_sizePrecision);
      d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
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
      vpImageIo::read(I, filename);

      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
      }

      std::cout << "Search dots in image" << filename << std::endl;
      std::list<vpDot2> list_d;
      d.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), list_d);

      if (list_d.empty()) {
        std::cout << "Dot auto detection did not work." << std::endl;
        return EXIT_FAILURE;
      }
      else {
        std::cout << std::endl << list_d.size() << " dots are detected" << std::endl;

        if (opt_display) {
          int i = 0;
          // Parse all founded dots for display
          for (std::list<vpDot2>::const_iterator it = list_d.begin(); it != list_d.end(); ++it) {
            vpImagePoint cog = (*it).getCog();

            std::cout << "Dot " << i++ << " : " << cog.get_u() << " " << cog.get_v() << std::endl;

            vpDisplay::displayCross(I, cog, 16, vpColor::blue, 3);
          }
          vpDisplay::flush(I);
        }
      }

      // If click is allowed, wait for a mouse click to launch the next
      // iteration
      if (opt_display && opt_click_allowed) {
        std::cout << "\nA click to continue..." << std::endl;
        // Wait for a blocking mouse click
        vpDisplay::getClick(I);
      }

      iter += opt_step;
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
