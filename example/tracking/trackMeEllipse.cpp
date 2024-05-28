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
 * Tracking of an ellipse.
 *
*****************************************************************************/

/*!
  \file trackMeEllipse.cpp

  \brief Tracking of an ellipse using vpMe.
*/

/*!
  \example trackMeEllipse.cpp

  Tracking of an ellipse using vpMe.
*/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#if defined(VISP_HAVE_MODULE_ME) &&                                                                                    \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpVideoWriter.h>
#include <visp3/me/vpMeEllipse.h>

// List of allowed command line options
#define GETOPTARGS "Aabcdf:hi:l:p:r:s:S:t:vw:"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param video_in_ipath : Input image path.
  \param video_in_ppath : Personal image path.
  \param video_in_first : First image to process.
  \param video_in_last : Last image to process.
  \param video_in_step : Step between two images.
  \param me_range : Moving-edges range.
  \param me_sample_step : Moving-edges sample step.
  \param me_threshold : Moving-edges threshold.
*/
void usage(const char *name, const char *badparam, const std::string &video_in_ipath, const std::string &video_in_ppath,
  unsigned video_in_first, int video_in_last, int video_in_step, int me_range, int me_sample_step, int me_threshold)
{
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif
  fprintf(stdout, "\n\
Example of ellipse/circle or arc of ellipse/circle tracking using vpMeEllipse.\n\
\n\
SYNOPSIS\n\
  %s [-i <visp dataset directory>] [-p <personal image path>]\n\
     [-f <video first image>] [-l <video last image>] [-s <video step>]\n\
     [-r <moving-edge range] [-t <moving-edge threshold] [-S <moving-edge sample step>]\n\
     [-w <output images sequence name>] \n\
     [-c] [-d] [-a] [-A] [-b] [-v] [-h]\n",
    name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <visp dataset directory>                                %s\n\
     Set visp dataset directory location.\n\
     From this directory read images \n\
     \"ellipse-1/image.%%04d.%s\"\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
  \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     The format is selected by analyzing \n\
     the filename extension.\n\
     Example : \"C:/Temp/ViSP-images/ellipse-1/image.%%04d.%s\"\n\
     %%04d is for the image numbering.\n\
  \n\
  -f <video first image>                                     %d\n\
     First image number to process.\n\
     Set -1 to process the first image of the sequence.\n\
  \n\
  -l <video last image>                                      %d\n\
     Last image number to process. \n\
     Set -1 to process images until the last image\n\
     of the sequence.\n\
  \n\
  -s <video step>                                            %d\n\
     Step between two images.\n\
  \n\
  -r <moving-edge range>                                     %d\n\
     Moving-edge range.\n\
     Increase value to consider large displacement. \n\
     When set to -1, use default value.             \n\
  \n\
  -S <moving-edge sample step>                               %d\n\
     Moving-edge sample step.\n\
     Distance between two moving-edges samples in degrees. \n\
     When set to -1, use default value.             \n\
  \n\
  -t <moving-edge threshold>                                 %d\n\
     Moving-edge threshold corresponding to the minimum        \n\
     contrast to consider. Value in range [0 ; 255] \n\
     When set to -1, use default value.             \n\
  \n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
  \n\
  -d \n\
     Turn off the display.\n\
  \n\
  -a \n\
     Enable arc of ellipse tracking.\n\
  \n\
  -b \n\
     Enable circle tracking.\n\
  \n\
  -w <output images sequence name>                        \n\
     Save images with tracking results in overlay.\n\
     Example: \"result/I%%04d.png\"                   \n\
  \n\
  -A                         \n\
     When display is activated using -d option, enable\n\
     windows auto scaling to fit the screen size. \n\
  \n\
  -v\n\
     Enable verbosity.\n\
  \n\
  -h\n\
     Print the help.\n",
    video_in_ipath.c_str(), ext.c_str(), video_in_ppath.c_str(), ext.c_str(), video_in_first, video_in_last,
    video_in_step, me_range, me_sample_step, me_threshold);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param video_in_ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param video_in_ppath : Personal image path.
  \param video_in_first : First image to process.
  \param video_in_last : Last image to process.
  \param video_in_step : Step between two images.
  \param display : Display activation.
  \param display_scale_auto : When display is activated, enable windows auto scaling.
  \param track_circle : Enable circle tracking.
  \param track_arc : Enable arc of an ellipse or circle tracking.
  \param video_out_save : Save resulting images sequence with tracking results in overlay.
  \param me_range : Moving-edges range.
  \param me_sample_step : Moving-edges sample step.
  \param me_threshold : Moving-edges threshold.
  \param verbose : Enable verbosity.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &video_in_ipath, std::string &video_in_ppath,
  int &video_in_first, int &video_in_last, int &video_in_step,
  bool &click_allowed, bool &display, bool &display_scale_auto, bool &track_circle, bool &track_arc,
  std::string &video_out_save, int &me_range, int &me_sample_step, int &me_threshold, bool &verbose)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'A':
      display_scale_auto = true;
      break;
    case 'a':
      track_arc = true;
      break;
    case 'b':
      track_circle = true;
      break;
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'f':
      video_in_first = atoi(optarg_);
      break;
    case 'i':
      video_in_ipath = std::string(optarg_);
      break;
    case 'l':
      video_in_last = atoi(optarg_);
      break;
    case 'p':
      video_in_ppath = std::string(optarg_);
      break;
    case 'r':
      me_range = atoi(optarg_);
      break;
    case 's':
      video_in_step = atoi(optarg_);
      break;
    case 'S':
      me_sample_step = atoi(optarg_);
      break;
    case 't':
      me_threshold = atoi(optarg_);
      break;
    case 'w':
      video_out_save = std::string(optarg_);
      break;
    case 'v':
      verbose = true;
      break;
    case 'h':
      usage(argv[0], nullptr, video_in_ipath, video_in_ppath, video_in_first, video_in_last, video_in_step, me_range, me_sample_step, me_threshold);
      return false;
      break;

    default:
      usage(argv[0], optarg_, video_in_ipath, video_in_ppath, video_in_first, video_in_last, video_in_step, me_range, me_sample_step, me_threshold);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, video_in_ipath, video_in_ppath, video_in_first, video_in_last, video_in_step, me_range, me_sample_step, me_threshold);
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
  int opt_first = -1;
  int opt_last = -1;
  int opt_step = 1;
  int opt_me_range = 30;
  int opt_me_sample_step = 5;
  int opt_me_threshold = 20; // Value in [0 ; 255]
  bool opt_click_allowed = true;
  bool opt_display = true;
  bool opt_display_scale_auto = false;
  bool opt_track_circle = false;
  bool opt_track_arc = false;
  bool opt_verbose = false;
  std::string opt_save;
  unsigned int thickness = 1;

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image is
  // read on the disk
  vpImage<unsigned char> I;

  vpDisplay *display = nullptr;

  try {
    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_last, opt_step, opt_click_allowed,
                   opt_display, opt_display_scale_auto, opt_track_circle, opt_track_arc, opt_save,
                   opt_me_range, opt_me_sample_step, opt_me_threshold, opt_verbose) == false) {
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
      usage(argv[0], nullptr, ipath, opt_ppath, opt_first, opt_last, opt_step, opt_me_range, opt_me_sample_step, opt_me_threshold);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << "  Use -p <personal image path> option if you want to " << std::endl
        << "  use personal images." << std::endl
        << std::endl;

      return EXIT_FAILURE;
    }

    // Create output folder if needed
    if (!opt_save.empty()) {
      std::string parent = vpIoTools::getParent(opt_save);
      if (!parent.empty()) {
        std::cout << "Create output directory: " << parent << std::endl;
        vpIoTools::makeDirectory(parent);
      }
      thickness += 1;
    }

    vpVideoReader g;
    if (opt_ppath.empty()) {
      // Set the path location of the image sequence
#if VISP_HAVE_DATASET_VERSION >= 0x030600
      videoname = vpIoTools::createFilePath(ipath, "ellipse-1/image.%04d.png");
#else
      videoname = vpIoTools::createFilePath(ipath, "ellipse-1/image.%04d.pgm");
#endif
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
      // We open a window using either X11, GTK or GDI.
#if defined(VISP_HAVE_X11)
      display = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
      display = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
      display = new vpDisplayGDI;
#elif defined(HAVE_OPENCV_HIGHGUI)
      display = new vpDisplayOpenCV;
#endif
      if (opt_display_scale_auto) {
        display->setDownScalingFactor(vpDisplay::SCALE_AUTO);
      }
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

    vpVideoWriter *writer = nullptr;
    vpImage<vpRGBa> O;
    if (!opt_save.empty()) {
      writer = new vpVideoWriter();
      writer->setFileName(opt_save);
      writer->open(O);
    }
    vpMeEllipse me_ellipse;

    vpMe me;
    if (opt_me_range > 0) {
      me.setRange(opt_me_range);
    }
    if (opt_me_sample_step > 0) {
      me.setSampleStep(opt_me_sample_step);
    }
    if (opt_me_threshold > 0) {
      me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
      me.setThreshold(opt_me_threshold);
    }

    me_ellipse.setMe(&me);
    me_ellipse.setDisplay(vpMeSite::RANGE_RESULT);

    std::cout << "Video settings" << std::endl;
    std::cout << "  Name       : " << g.getFrameName() << std::endl;
    std::cout << "  First image: " << g.getFirstFrameIndex() << std::endl;
    std::cout << "  Last image : " << g.getLastFrameIndex() << std::endl;
    std::cout << "  Step       : " << g.getFrameStep() << std::endl;
    std::cout << "  Image size : " << I.getWidth() << " x " << I.getHeight() << std::endl;

    std::cout << "Moving-edges settings" << std::endl;
    std::cout << "  Sample step   : " << me_ellipse.getMe()->getSampleStep() << std::endl;
    std::cout << "  Range         : " << me_ellipse.getMe()->getRange() << std::endl;
    std::cout << "  Threshold type: " << (me_ellipse.getMe()->getLikelihoodThresholdType() == vpMe::NORMALIZED_THRESHOLD ? "normalized" : "old threshold (to be avoided)") << std::endl;
    std::cout << "  Threshold     : " << me_ellipse.getMe()->getThreshold() << std::endl;

    if (opt_click_allowed) {
      me_ellipse.initTracking(I, opt_track_circle, opt_track_arc);
    }
    else {
      // Create a list of clockwise points to automate the test
      std::vector<vpImagePoint> ip;
      ip.push_back(vpImagePoint(195, 329));
      ip.push_back(vpImagePoint(243, 164));
      ip.push_back(vpImagePoint(201, 36));
      ip.push_back(vpImagePoint(83, 126));
      ip.push_back(vpImagePoint(33, 276));

      me_ellipse.initTracking(I, ip, opt_track_circle, opt_track_arc);
    }
    if (opt_display) {
      me_ellipse.display(I, vpColor::green, thickness);
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
    bool quit = false;

    while (!g.end() && !quit) {
      // Read the image
      g.acquire(I);
      std::stringstream ss;
      ss << "Process image " << g.getFrameIndex();
      if (opt_verbose) {
        std::cout << "-- " << ss.str() << std::endl;
      }
      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
        vpDisplay::displayText(I, 20, 10, ss.str(), vpColor::red);
        if (opt_click_allowed) {
          vpDisplay::displayText(I, 40, 10, "Click to exit...", vpColor::red);
        }
      }
      me_ellipse.track(I);

      if (opt_display) {
        me_ellipse.display(I, vpColor::green, thickness);
        vpDisplay::flush(I);
      }
      if (!opt_save.empty()) {
        vpDisplay::getImage(I, O);
        writer->saveFrame(O);
      }
      if (opt_display && opt_click_allowed) {
        if (vpDisplay::getClick(I, false)) {
          quit = true;
        }
      }
    }

    if (opt_display && opt_click_allowed && !quit) {
      vpDisplay::getClick(I);
    }

    if (writer) {
      delete writer;
    }
    if (display) {
      delete display;
    }
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    if (opt_display && opt_click_allowed) {
      vpDisplay::getClick(I);
    }
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
  std::cout << "visp_me module or X11, GTK, GDI or OpenCV display "
    "functionalities are required..."
    << std::endl;
  return EXIT_SUCCESS;
}

#endif
