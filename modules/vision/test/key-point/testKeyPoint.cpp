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
 * Test keypoint matching.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpKeyPoint.h>

// List of allowed command line options
#define GETOPTARGS "cdh"

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
          Test keypoints matching.\n\
          \n\
          SYNOPSIS\n\
          %s [-c] [-d] [-h]\n", name);

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
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
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

template<typename Type>
void run_test(const std::string &env_ipath, bool opt_click_allowed, bool opt_display,
              vpImage<Type> &Iref, vpImage<Type> &Icur, vpImage<Type> &Imatch)
{
  // Set the path location of the image sequence
  std::string dirname = vpIoTools::createFilePath(env_ipath, "mbt/cube");

  // Build the name of the image files
  std::string filenameRef = vpIoTools::createFilePath(dirname, "image0000.pgm");
  vpImageIo::read(Iref, filenameRef);
  std::string filenameCur = vpIoTools::createFilePath(dirname, "image%04d.pgm");

  // Init keypoints
  vpKeyPoint keypoints("ORB", "ORB", "BruteForce-Hamming");
  std::cout << "Build " << keypoints.buildReference(Iref) << " reference points." << std::endl;

  vpVideoReader g;
  g.setFileName(filenameCur);
  g.open(Icur);
  g.acquire(Icur);

  Imatch.resize(Icur.getHeight(), 2 * Icur.getWidth());
  Imatch.insert(Iref, vpImagePoint(0, 0));

#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#else
  vpDisplayOpenCV display;
#endif

  if (opt_display) {
    display.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display.init(Imatch, 0, 0, "ORB keypoints matching");
  }

  bool opt_click = false;
  vpMouseButton::vpMouseButtonType button;
  while (!g.end()) {
    g.acquire(Icur);
    Imatch.insert(Icur, vpImagePoint(0, Icur.getWidth()));

    if (opt_display) {
      vpDisplay::display(Imatch);
    }

    // Match keypoints
    keypoints.matchPoint(Icur);
    // Display image with keypoints matched
    keypoints.displayMatching(Iref, Imatch);

    if (opt_display) {
      vpDisplay::flush(Imatch);
    }

    // Click requested to process next image
    if (opt_click_allowed && opt_display) {
      if (opt_click) {
        vpDisplay::getClick(Imatch, button, true);
        if (button == vpMouseButton::button3) {
          opt_click = false;
        }
      } else {
        // Use right click to enable/disable step by step tracking
        if (vpDisplay::getClick(Imatch, button, false)) {
          if (button == vpMouseButton::button3) {
            opt_click = true;
          } else if (button == vpMouseButton::button1) {
            break;
          }
        }
      }
    }
  }
}

/*!
  \example testKeyPoint.cpp

  \brief   Test keypoint matching.
*/
int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if (env_ipath.empty()) {
      std::cerr << "Please set the VISP_INPUT_IMAGE_PATH environment "
                   "variable value."
                << std::endl;
      return -1;
    }

    {
      vpImage<unsigned char> Iref, Icur, Imatch;

      std::cout << "-- Test on gray level images" << std::endl;
      run_test(env_ipath, opt_click_allowed, opt_display, Iref, Icur, Imatch);
    }

    {
      vpImage<vpRGBa> Iref, Icur, Imatch;

      std::cout << "-- Test on color images" << std::endl;
      run_test(env_ipath, opt_click_allowed, opt_display, Iref, Icur, Imatch);
    }

  } catch (const vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "testKeyPoint is ok !" << std::endl;
  return 0;
}
#else
int main()
{
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
