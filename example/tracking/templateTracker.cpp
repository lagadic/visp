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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example templateTracker.cpp

  \brief Example of template tracking.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_TT) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoReader.h>

#include <visp3/tt/vpTemplateTrackerSSD.h>
#include <visp3/tt/vpTemplateTrackerSSDESM.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerZNCCInverseCompositional.h>

#include <visp3/tt/vpTemplateTrackerWarpAffine.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>
#include <visp3/tt/vpTemplateTrackerWarpHomographySL3.h>
#include <visp3/tt/vpTemplateTrackerWarpRT.h>
#include <visp3/tt/vpTemplateTrackerWarpSRT.h>
#include <visp3/tt/vpTemplateTrackerWarpTranslation.h>

#ifdef VISP_HAVE_MODULE_TT_MI
#include <visp3/tt_mi/vpTemplateTrackerMIESM.h>
#include <visp3/tt_mi/vpTemplateTrackerMIForwardAdditional.h>
#include <visp3/tt_mi/vpTemplateTrackerMIForwardCompositional.h>
#include <visp3/tt_mi/vpTemplateTrackerMIInverseCompositional.h>
#endif

#define GETOPTARGS "cdhi:l:pt:w:"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef enum {
  WARP_AFFINE,
  WARP_HOMOGRAPHY,
  WARP_HOMOGRAPHY_SL3,
  WARP_SRT,
  WARP_TRANSLATION,
#ifdef VISP_HAVE_MODULE_TT_MI
  WARP_RT,
#endif
  WARP_LAST
} WarpType;

typedef enum {
  TRACKER_SSD_ESM,
  TRACKER_SSD_FORWARD_ADDITIONAL,
  TRACKER_SSD_FORWARD_COMPOSITIONAL,
  TRACKER_SSD_INVERSE_COMPOSITIONAL, // The most efficient
  TRACKER_ZNCC_FORWARD_ADDITIONEL,
  TRACKER_ZNCC_INVERSE_COMPOSITIONAL,
#ifdef VISP_HAVE_MODULE_TT_MI
  TRACKER_MI_ESM,
  TRACKER_MI_FORWARD_ADDITIONAL,
  TRACKER_MI_FORWARD_COMPOSITIONAL,
  TRACKER_MI_INVERSE_COMPOSITIONAL, // The most efficient
#endif
  TRACKER_LAST
} TrackerType;

#endif

void usage(const char *name, const char *badparam, const WarpType &warp_type, TrackerType &tracker_type,
           const long &last_frame);
bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display, bool &pyramidal,
                WarpType &warp_type, TrackerType &tracker_type, long &last_frame);

void usage(const char *name, const char *badparam, const WarpType &warp_type, TrackerType &tracker_type,
           const long &last_frame)
{
  fprintf(stdout, "\n\
Example of template tracking.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-c] [-d] [-p] \n\
     [-w <warp type>] [-t <tracker type>] \n\
     [-l <last frame number>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                                            Default\n\
  -i <input image path>                                \n\
     Set image input path.\n\
     From this path read images \n\
     \"cube/image%%04d.pgm\". These \n\
     images come from ViSP-images-x.y.z.tar.gz available \n\
     on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
          \n\
  -l <last frame number>                                              %ld\n\
     Last frame number to consider.\n\
          \n\
  -d \n\
     Turn off the display.\n\
          \n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
          \n", last_frame);

#ifdef VISP_HAVE_MODULE_TT_MI
  fprintf(stdout, "\n\
  -w <warp type=[0,1,2,3,4,5]>                                        %d\n\
     Set the model used to warp the template. \n\
     Authorized values are:\n\
     %d : Affine\n\
     %d : Homography\n\
     %d : Homography in SL3\n\
     %d : SRT (scale, rotation, translation)\n\
     %d : RT (rotation, translation)\n\
     %d : Translation\n\n", (int)warp_type, (int)WARP_AFFINE, (int)WARP_HOMOGRAPHY, (int)WARP_HOMOGRAPHY_SL3, (int)WARP_SRT,
          (int)WARP_TRANSLATION, (int)WARP_RT);
#else
  fprintf(stdout, "\n\
  -w <warp type=[0,1,2,3,4]>                                          %d\n\
     Set the model used to warp the template. \n\
     Authorized values are:\n\
     %d : Affine\n\
     %d : Homography\n\
     %d : Homography in SL3\n\
     %d : SRT (scale, rotation, translation)\n\
     %d : Translation\n\n", (int)warp_type, (int)WARP_AFFINE, (int)WARP_HOMOGRAPHY, (int)WARP_HOMOGRAPHY_SL3, (int)WARP_SRT,
          (int)WARP_TRANSLATION);
#endif

#ifdef VISP_HAVE_MODULE_TT_MI
  fprintf(stdout, "\n\
  -t <tracker type=[0,1,2,3,4,5,6,7,8,9]>                             %d\n\
     Set the tracker used to track the template. \n\
     Authorized values are:\n\
     %d : SSD ESM\n\
     %d : SSD forward additional\n\
     %d : SSD forward compositional\n\
     %d : SSD inverse compositional\n\
     %d : ZNCC forward additional\n\
     %d : ZNCC inverse compositional\n\
     %d : MI ESM\n\
     %d : MI forward additional\n\
     %d : MI forward compositional\n\
     %d : MI inverse compositional\n\n", (int)tracker_type, (int)TRACKER_SSD_ESM, (int)TRACKER_SSD_FORWARD_ADDITIONAL,
          (int)TRACKER_SSD_FORWARD_COMPOSITIONAL, (int)TRACKER_SSD_INVERSE_COMPOSITIONAL,
          (int)TRACKER_ZNCC_FORWARD_ADDITIONEL, (int)TRACKER_ZNCC_INVERSE_COMPOSITIONAL, (int)TRACKER_MI_ESM,
          (int)TRACKER_MI_FORWARD_ADDITIONAL, (int)TRACKER_MI_FORWARD_COMPOSITIONAL,
          (int)TRACKER_MI_INVERSE_COMPOSITIONAL);
#else
  fprintf(stdout, "\n\
  -t <tracker type=[0,1,2,3,4,5]>                                     %d\n\
     Set the tracker used to track the template. \n\
     Authorized values are:\n\
     %d : SSD ESM\n\
     %d : SSD forward additional\n\
     %d : SSD forward compositional\n\
     %d : SSD inverse compositional\n\
     %d : ZNCC forward additional\n\
     %d : ZNCC inverse compositional\n\n", (int)tracker_type, (int)TRACKER_SSD_ESM, (int)TRACKER_SSD_FORWARD_ADDITIONAL,
          (int)TRACKER_SSD_FORWARD_COMPOSITIONAL, (int)TRACKER_SSD_INVERSE_COMPOSITIONAL,
          (int)TRACKER_ZNCC_FORWARD_ADDITIONEL, (int)TRACKER_ZNCC_INVERSE_COMPOSITIONAL);

#endif
  fprintf(stdout, "\n\
  -p\n\
     Enable pyramidal tracking.\n\
                  \n\
  -h \n\
     Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, const char **argv, std::string &ipath, bool &click_allowed, bool &display, bool &pyramidal,
                WarpType &warp_type, TrackerType &tracker_type, long &last_frame)
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
      usage(argv[0], NULL, warp_type, tracker_type, last_frame);
      return false;
      break;
    case 'i':
      ipath = optarg_;
      break;
    case 'l':
      last_frame = (long)atoi(optarg_);
      break;
    case 'p':
      pyramidal = true;
      break;
    case 't':
      tracker_type = (TrackerType)atoi(optarg_);
      break;
    case 'w':
      warp_type = (WarpType)atoi(optarg_);
      break;

    default:
      usage(argv[0], optarg_, warp_type, tracker_type, last_frame);
      return false;
      break;
    }
  }

  if (warp_type >= WARP_LAST) {
    usage(argv[0], NULL, warp_type, tracker_type, last_frame);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument -w <warp type> with \"warp type\"=" << (int)warp_type << std::endl << std::endl;
    return false;
  }
  if (tracker_type >= TRACKER_LAST) {
    usage(argv[0], NULL, warp_type, tracker_type, last_frame);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument -t <tracker type> with \"tracker type\"=" << (int)tracker_type << std::endl
              << std::endl;
    return false;
  }
  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, warp_type, tracker_type, last_frame);
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
    bool opt_click_allowed = true;
    bool opt_display = true;
    bool opt_pyramidal = false;
    TrackerType opt_tracker_type = TRACKER_SSD_INVERSE_COMPOSITIONAL;
    WarpType opt_warp_type = WARP_AFFINE;
    long opt_last_frame = 30;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_pyramidal, opt_warp_type,
                    opt_tracker_type, opt_last_frame)) {
      return (-1);
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, opt_warp_type, opt_tracker_type, opt_last_frame);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;

      return (-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = vpIoTools::createFilePath(opt_ipath, "mire-2/image.%04d.pgm");
    else
      ipath = vpIoTools::createFilePath(env_ipath, "mire-2/image.%04d.pgm");

    vpImage<unsigned char> I;
    vpVideoReader reader;

    reader.setFileName(ipath.c_str());
    reader.setFirstFrameIndex(1);
    reader.setLastFrameIndex(opt_last_frame);
    try {
      reader.open(I);
    } catch (...) {
      std::cout << "Cannot open sequence: " << ipath << std::endl;
      return -1;
    }
    reader.acquire(I);

    vpDisplay *display = NULL;
    if (opt_display) {
// initialise a  display
#if defined VISP_HAVE_X11
      display = new vpDisplayX;
#elif defined VISP_HAVE_GDI
      display = new vpDisplayGDI;
#elif defined VISP_HAVE_OPENCV
      display = new vpDisplayOpenCV;
#elif defined VISP_HAVE_D3D9
      display = new vpDisplayD3D;
#elif defined VISP_HAVE_GTK
      display = new vpDisplayGTK;
#else
      opt_display = false;
#endif
#if defined(VISP_HAVE_DISPLAY)
      display->init(I, 100, 100, "Test tracking");
#endif
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    vpTemplateTrackerWarp *warp = NULL;
    switch (opt_warp_type) {
    case WARP_AFFINE:
      warp = new vpTemplateTrackerWarpAffine;
      break;
    case WARP_HOMOGRAPHY:
      warp = new vpTemplateTrackerWarpHomography;
      break;
    case WARP_HOMOGRAPHY_SL3:
      warp = new vpTemplateTrackerWarpHomographySL3;
      break;
    case WARP_SRT:
      warp = new vpTemplateTrackerWarpSRT;
      break;
    case WARP_TRANSLATION:
      warp = new vpTemplateTrackerWarpTranslation;
      break;
#ifdef VISP_HAVE_MODULE_TT_MI
    case WARP_RT:
      warp = new vpTemplateTrackerWarpRT;
      break;
#endif
    default:
      return 0;
    }

    vpTemplateTracker *tracker = NULL;
    switch (opt_tracker_type) {
    case TRACKER_SSD_ESM:
      tracker = new vpTemplateTrackerSSDESM(warp);
      break;
    case TRACKER_SSD_FORWARD_ADDITIONAL:
      tracker = new vpTemplateTrackerSSDForwardAdditional(warp);
      break;
    case TRACKER_SSD_FORWARD_COMPOSITIONAL:
      tracker = new vpTemplateTrackerSSDForwardCompositional(warp);
      break;
    case TRACKER_SSD_INVERSE_COMPOSITIONAL:
      tracker = new vpTemplateTrackerSSDInverseCompositional(warp);
      break;
    case TRACKER_ZNCC_FORWARD_ADDITIONEL:
      tracker = new vpTemplateTrackerZNCCForwardAdditional(warp);
      break;
    case TRACKER_ZNCC_INVERSE_COMPOSITIONAL:
      tracker = new vpTemplateTrackerZNCCInverseCompositional(warp);
      break;
#ifdef VISP_HAVE_MODULE_TT_MI
    case TRACKER_MI_ESM:
      tracker = new vpTemplateTrackerMIESM(warp);
      break;
    case TRACKER_MI_FORWARD_ADDITIONAL:
      tracker = new vpTemplateTrackerMIForwardAdditional(warp);
      break;
    case TRACKER_MI_FORWARD_COMPOSITIONAL:
      tracker = new vpTemplateTrackerMIForwardCompositional(warp);
      break;
    case TRACKER_MI_INVERSE_COMPOSITIONAL:
      tracker = new vpTemplateTrackerMIInverseCompositional(warp);
      break;
#endif
    default:
      return 0;
    }

    tracker->setSampling(2, 2);
    tracker->setLambda(0.001);
    tracker->setThresholdGradient(60.);
    tracker->setIterationMax(800);
    if (opt_pyramidal) {
      tracker->setPyramidal(2, 1);
    }
    bool delaunay = false;
    if (opt_display && opt_click_allowed)
      tracker->initClick(I, delaunay);
    else {
      std::vector<vpImagePoint> v_ip;
      vpImagePoint ip;
      ip.set_ij(166, 54);
      v_ip.push_back(ip);
      ip.set_ij(284, 55);
      v_ip.push_back(ip);
      ip.set_ij(259, 284);
      v_ip.push_back(ip); // ends the first triangle
      ip.set_ij(259, 284);
      v_ip.push_back(ip); // start the second triangle
      ip.set_ij(149, 240);
      v_ip.push_back(ip);
      ip.set_ij(167, 58);
      v_ip.push_back(ip);

      tracker->initFromPoints(I, v_ip, false);
    }

    while (!reader.end()) {
      // Acquire a new image
      reader.acquire(I);
      std::cout << "Process image number " << reader.getFrameIndex() << std::endl;
      // Display the image
      vpDisplay::display(I);
      // Track the template
      tracker->track(I);

      // Simulate a re-init
      if (reader.getFrameIndex() == 10) {
        std::cout << "re-init simulation" << std::endl;
        if (opt_click_allowed)
          vpDisplay::getClick(I);

        tracker->resetTracker();

        if (opt_display && opt_click_allowed) {
          vpDisplay::displayText(I, 10, 10, "Re-init simulation", vpColor::red);
          vpDisplay::flush(I);
          tracker->initClick(I, delaunay);
        } else {
          std::vector<vpImagePoint> v_ip;
          vpImagePoint ip;
          ip.set_ij(146, 60);
          v_ip.push_back(ip);
          ip.set_ij(254, 74);
          v_ip.push_back(ip);
          ip.set_ij(228, 288);
          v_ip.push_back(ip); // ends the first triangle
          ip.set_ij(228, 288);
          v_ip.push_back(ip); // start the second triangle
          ip.set_ij(126, 242);
          v_ip.push_back(ip);
          ip.set_ij(146, 60);
          v_ip.push_back(ip);

          tracker->initFromPoints(I, v_ip, false);
        }
      }

// Display the template
#if 1
      tracker->display(I, vpColor::red, 3);
#else
      vpTemplateTrackerZone zoneWarped_, zoneRef_ = tracker->getZoneRef();
      vpTemplateTrackerWarp *warp_ = tracker->getWarp();
      vpColVector p_ = tracker->getp();
      warp_->warpZone(zoneRef_, p_, zoneWarped_);
      zoneWarped_.display(I, vpColor::red, 3);
      zoneRef_.display(I, vpColor::green, 3);
#endif

      vpDisplay::flush(I);
    }
    if (opt_click_allowed) {
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }
    reader.close();
    if (display)
      delete display;

    delete warp;
    delete tracker;

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else

int main()
{
  std::cout << "visp_tt module or display not available." << std::endl;
  return EXIT_SUCCESS;
}

#endif
