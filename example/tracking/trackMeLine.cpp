/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Tracking of a line.
 */

/*!
  \file trackMeLine.cpp
  \example trackMeLine.cpp

  \brief Tracking of a line using vpMe.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_ME) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>

// List of allowed command line options
#define GETOPTARGS "cdf:hi:l:p:s:"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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
#if defined(VISP_HAVE_DATASET)
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
#else
  // We suppose that the user will download a recent dataset
  std::string ext("png");
#endif
  fprintf(stdout, "\n\
Tracking of a line.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-p <personal image path>]\n\
     [-f <first image>] [-l <last image>] [-s <step>]\n\
     [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"line/image.%%04d.%s\"\n\
     images. \n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     Example : \"C:/Temp/visp-images/line/image.%%04d.%s\"\n\
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
#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV)
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string opt_ppath;
  std::string videoname;
  unsigned int opt_first = 1;
  unsigned int opt_last = 30;
  unsigned int opt_step = 1;
  bool opt_click_allowed = true;
  bool opt_display = true;
  unsigned int thickness = 1;

  vpImage<unsigned char> I;
  vpDisplay *display = nullptr;
  vpVideoReader g;
#if defined(VISP_HAVE_DATASET)
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
#else
  // We suppose that the user will download a recent dataset
  std::string ext("png");
#endif

  try {
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
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
    }

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

    vpVideoReader g;
    if (opt_ppath.empty()) {
      // Set the path location of the image sequence
      videoname = vpIoTools::createFilePath(ipath, "line/image.%04d." + ext);
      g.setFileName(videoname);
    }
    else {
      g.setFileName(opt_ppath);
    }

    if (opt_first > 0) {
      g.setFirstFrameIndex(opt_first);
    }
    if (opt_last > 0) {
      g.setLastFrameIndex(opt_last);
    }
    g.setFrameStep(opt_step);
    g.open(I);

    if (opt_display) {
      // We open a window using either X11, GTK, GDI or OpenCV
      display = vpDisplayFactory::allocateDisplay();

      // Display size is automatically defined by the image (I) size
      display->init(I, 10, 10, "Current image");
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    vpMeLine me_line;

    vpMe me;
    me.setRange(20);
    me.setPointsToTrack(160);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20);

    me_line.setMe(&me);
    me_line.setDisplay(vpMeSite::RANGE_RESULT);
    // ajout FC
    const bool useIntensity = true;
    me_line.setRhoSignFromIntensity(useIntensity);
    // fin ajout FC

    std::cout << "Video settings" << std::endl;
    std::cout << "  Name       : " << g.getFrameName() << std::endl;
    std::cout << "  First image: " << g.getFirstFrameIndex() << std::endl;
    std::cout << "  Last image : " << g.getLastFrameIndex() << std::endl;
    std::cout << "  Step       : " << g.getFrameStep() << std::endl;
    std::cout << "  Image size : " << I.getWidth() << " x " << I.getHeight() << std::endl;

    std::cout << "Moving-edges settings" << std::endl;
    std::cout << "  Sample step   : " << me_line.getMe()->getSampleStep() << std::endl;
    std::cout << "  Range         : " << me_line.getMe()->getRange() << std::endl;
    std::cout << "  Threshold type: " << (me_line.getMe()->getLikelihoodThresholdType() == vpMe::NORMALIZED_THRESHOLD ? "normalized" : "old threshold (to be avoided)") << std::endl;
    std::cout << "  Threshold     : " << me_line.getMe()->getThreshold() << std::endl;

    if (opt_display && opt_click_allowed)
      me_line.initTracking(I);
    else {
      vpImagePoint ip1, ip2;
      ip1.set_i(96);
      ip1.set_j(191);
      ip2.set_i(122);
      ip2.set_j(211);
      me_line.initTracking(I, ip1, ip2);
    }
    me_line.track(I);

    if (opt_display) {
      me_line.display(I, vpColor::green);
      vpDisplay::flush(I);
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
    std::cout << "----------------------------------------------------------" << std::endl;

    vpFeatureLine l;

    vpCameraParameters cam;

    bool quit = false;
    while (!g.end() && !quit) {
      g.acquire(I);
      std::cout << "Process image " << g.getFrameIndex() << std::endl;
      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
        if (opt_click_allowed) {
          vpDisplay::displayText(I, 40, 10, "Click to exit...", vpColor::red);
        }
      }

      me_line.track(I);

      vpTRACE("me_line: rho %lf theta (dg) %lf", me_line.getRho(), vpMath::deg(me_line.getTheta()));
      vpFeatureBuilder::create(l, cam, me_line);
      vpTRACE("fe_line: rho %lf theta (dg) %lf", l.getRho(), vpMath::deg(l.getTheta()));

      // FC print
      // printf("me_line: rho %lf theta (dg) %lf\n", me_line.getRho(), vpMath::deg(me_line.getTheta()));
      // printf("fe_line: rho %lf theta (dg) %lf\n", l.getRho(), vpMath::deg(l.getTheta()));
      // FC fin print

      if (opt_display) {
        me_line.display(I, vpColor::green, thickness);
        vpDisplay::flush(I);
        if (opt_click_allowed) {
          if (vpDisplay::getClick(I, false)) {
            quit = true;
          }
        }
        // ajout FC
        // std::cout << "A click to continue..." << std::endl;
        // vpDisplay::getClick(I);
        // fin ajout FC
      }
    }
    if (opt_display && opt_click_allowed && !quit) {
      vpDisplay::getClick(I);
    }
    if (display) {
      delete display;
    }
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
#endif
}

#else
#include <iostream>

int main()
{
  std::cout << "visp_me module or X11, GTK, GDI or OpenCV display functionalities are required..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
