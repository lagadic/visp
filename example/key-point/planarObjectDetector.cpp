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
 * Detection of planar surface using Fern classifier.
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/
/*!
  \file planarObjectDetector.cpp

  \brief Tracking of planar surface using Fern classifier.
*/

/*!
  \example planarObjectDetector.cpp

  Tracking of planar surface using Fern classifier.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#if ((defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)) &&                                   \
     (VISP_HAVE_OPENCV_VERSION >= 0x020000) && (VISP_HAVE_OPENCV_VERSION < 0x030000))

#include <iomanip>
#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/vision/vpPlanarObjectDetector.h>

#define GETOPTARGS "hlcdb:i:p"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &isLearning, std::string &dataFile, bool &click_allowed,
                bool &display, bool &displayPoints, std::string &ipath);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test of detection of planar surface using a Fern classifier. The object needs \
  first to be learned (-l option). This learning process will create a file used\
  to detect the object.\n\
\n\
SYNOPSIS\n\
  %s [-l] [-h] [-b] [-c] [-d] [-p] [-i] [-s]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
  -l\n\
     learn an object.\n\
\n\
  -i <input image path>                                \n\
     Set image input path.\n\
     From this path read \"line/image.%%04d.pgm\"\n\
     images. \n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -b\n\
     database filename to use (default is ./dataPlanar).\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -s \n\
     Turn off the use of the sequence and use a webcam.\n\
\n\
  -p \n\
     display points of interest.\n\
\n\
  -h\n\
     Print this help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param isLearning : learning surface activation.
  \param dataFile : filename of the database.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \param displayPoints : Display points of interests activation.
  \param ipath : Input image path.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &isLearning, std::string &dataFile, bool &click_allowed,
                bool &display, bool &displayPoints, std::string &ipath)
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
    case 'l':
      isLearning = true;
      break;
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;
    case 'b':
      dataFile = optarg_;
      break;
    case 'p':
      displayPoints = true;
      break;
    case 'i':
      ipath = optarg_;
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

int main(int argc, const char **argv)
{
  try {
    bool isLearning = false;
    std::string dataFile("./dataPlanar");
    bool opt_click_allowed = true;
    bool opt_display = true;
    std::string objectName("object");
    bool displayPoints = false;
    std::string opt_ipath;
    std::string ipath;
    std::string env_ipath;
    std::string dirname;
    std::string filename;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty()) {
      ipath = env_ipath;
    }

    // Read the command line options
    if (getOptions(argc, argv, isLearning, dataFile, opt_click_allowed, opt_display, displayPoints, opt_ipath) ==
        false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
    }

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
      usage(argv[0], NULL);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    // Declare two images, these are gray level images (unsigned char)
    vpImage<unsigned char> I;
    vpImage<unsigned char> Iref;

    // Set the path location of the image sequence
    dirname = vpIoTools::createFilePath(ipath, "cube");

    // Build the name of the image file
    unsigned iter = 0; // Image number
    std::ostringstream s;
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
    filename = vpIoTools::createFilePath(dirname, s.str());

    // Read the PGM image named "filename" on the disk, and put the
    // bitmap into the image structure I.  I is initialized to the
    // correct size
    //
    // exception readPGM may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      std::cout << "Load: " << filename << std::endl;
      vpImageIo::read(Iref, filename);
      I = Iref;
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

#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#endif

#if defined VISP_HAVE_X11
    vpDisplayX displayRef;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK displayRef;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI displayRef;
#endif

    // declare a planar object detector
    vpPlanarObjectDetector planar;

    vpImagePoint corners[2];
    if (isLearning) {
      if (opt_display) {
        displayRef.init(Iref, 100, 100, "Reference image");
        vpDisplay::display(Iref);
        vpDisplay::flush(Iref);
      }
      if (opt_display && opt_click_allowed) {
        std::cout << "Click on the top left and the bottom right corners to "
                     "define the reference plane"
                  << std::endl;
        for (int i = 0; i < 2; i++) {
          vpDisplay::getClick(Iref, corners[i]);
          std::cout << corners[i] << std::endl;
        }
      } else {
        corners[0].set_ij(50, I.getWidth() - 100); // small ROI for the automated test
        corners[1].set_ij(I.getHeight() - 100, I.getWidth() - 2);
      }

      if (opt_display) {
        // Display the rectangle which defines the part of the image where the
        // reference points are computed.
        vpDisplay::displayRectangle(Iref, corners[0], corners[1], vpColor::green);
        vpDisplay::flush(Iref);
      }

      if (opt_click_allowed) {
        std::cout << "Click on the image to continue" << std::endl;
        vpDisplay::getClick(Iref);
      }

      vpRect roi(corners[0], corners[1]);

      std::cout << "> train the classifier on the selected plane (may take "
                   "up to several minutes)."
                << std::endl;
      if (opt_display) {
        vpDisplay::display(Iref);
        vpDisplay::flush(Iref);
      }
      double t0 = vpTime::measureTimeMs();
      planar.buildReference(Iref, roi);
      std::cout << "build reference in " << vpTime::measureTimeMs() - t0 << " ms" << std::endl;
      t0 = vpTime::measureTimeMs();
      planar.recordDetector(objectName, dataFile);
      std::cout << "record detector in " << vpTime::measureTimeMs() - t0 << " ms" << std::endl;
    } else {
      if (!vpIoTools::checkFilename(dataFile)) {
        vpERROR_TRACE("cannot load the database with the specified name. Has "
                      "the object been learned with the -l option? ");
        exit(-1);
      }
      try {
        // load a previously recorded file
        planar.load(dataFile, objectName);
      } catch (...) {
        vpERROR_TRACE("cannot load the database with the specified name. Has "
                      "the object been learned with the -l option? ");
        exit(-1);
      }
    }

    if (opt_display) {
      display.init(I, 110 + (int)Iref.getWidth(), 100, "Current image");
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "Click on the reference image to continue" << std::endl;
      vpDisplay::displayText(Iref, vpImagePoint(15, 15), "Click on the reference image to continue", vpColor::red);
      vpDisplay::flush(Iref);
      vpDisplay::getClick(Iref);
    }

    for (;;) {
      // acquire a new image
      iter++;
      if (iter >= 80) {
        break;
      }
      s.str("");
      s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
      filename = vpIoTools::createFilePath(dirname, s.str());
      // read the image
      vpImageIo::read(I, filename);

      if (opt_display) {
        vpDisplay::display(I);
      }

      double t0 = vpTime::measureTimeMs();
      // detection  of the reference planar surface
      bool isDetected = planar.matchPoint(I);
      std::cout << "matching in " << vpTime::measureTimeMs() - t0 << " ms" << std::endl;

      if (isDetected) {
        vpHomography H;
        planar.getHomography(H);
        std::cout << " > computed homography:" << std::endl << H << std::endl;
        if (opt_display) {
          if (isLearning) {
            vpDisplay::display(Iref);
            vpDisplay::displayRectangle(Iref, corners[0], corners[1], vpColor::green);
            planar.display(Iref, I, displayPoints);
            vpDisplay::flush(Iref);
          } else {
            planar.display(I, displayPoints);
          }
        }
      } else {
        std::cout << " > reference is not detected in the image" << std::endl;
      }
      if (opt_display) {
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false)) {
          break;
        }
      }
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
#if (!(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)))
  std::cout << "You do not have X11, or GTK, or GDI (Graphical Device Interface) functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
#else
  std::cout << "You do not have OpenCV functionalities" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install OpenCV, configure again ViSP using cmake and build again this example" << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif
