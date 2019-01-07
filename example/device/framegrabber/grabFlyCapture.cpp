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
 * Acquire images using OpenCV cv::VideoCapture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

/*!
  \example grabFlyCapture.cpp

  \brief Example of framegrabbing using OpenCV cv::VideoCapture class.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FLYCAPTURE)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

#define GETOPTARGS "cdhi:n:o:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param icamera : Camera index.
  \param opath : Image filename when saving.

*/
void usage(const char *name, const char *badparam, unsigned int icamera, std::string &opath)
{
  fprintf(stdout, "\n\
Acquire and display images using PointGrey FlyCapture SDK.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-i <camera index>] [-o <output image filename>] [-h] \n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c \n\
     Disable mouse click and acquire only 10 images.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -i [%%d]                                               %u\n\
     Camera index to connect (0 for the first one).     \n\
\n\
  -o [%%s]\n\
     Filename for image saving.                    \n\
     Example: -o %s\n\
     The %%d is for the image numbering.\n\
\n\
  -h \n\
     Print the help.\n\
\n", icamera, opath.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  Print the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param display : Display activation.
  \param save : Image saving activation.
  \param opath : Image filename when saving.
  \param deviceType : Type of device to detect.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &display, bool &click, bool &save, std::string &opath,
                unsigned int &icamera)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click = false;
      break;
    case 'd':
      display = false;
      break;
    case 'i':
      icamera = (unsigned int)atoi(optarg_);
      break;
    case 'o':
      save = true;
      opath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, icamera, opath);
      return false;
      break;

    default:
      usage(argv[0], optarg_, icamera, opath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, icamera, opath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

// usage: binary <device name>
// device name: 0 is the default to dial with the first camera,
//              1 to dial with a second camera attached to the computer
int main(int argc, const char **argv)
{
  try {
    bool opt_display = true;
    bool opt_click = true;
    bool opt_save = false;
    unsigned int opt_icamera = 0;
    std::string opt_opath = "I%04d.pgm";
    // vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images

    // Read the command line options
    if (getOptions(argc, argv, opt_display, opt_click, opt_save, opt_opath, opt_icamera) == false) {
      return 0;
    }

    std::cout << "Use device   : " << opt_icamera << std::endl;
    vpFlyCaptureGrabber g;
    g.setCameraIndex(opt_icamera); // open the default camera
    g.open(I);
    std::cout << "Camera serial: " << g.getCameraSerial(g.getCameraIndex()) << std::endl;
    std::cout << "Image size   : " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpDisplay *display = NULL;
    if (opt_display) {
#if defined(VISP_HAVE_X11)
      display = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      display = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
      display = new vpDisplayOpenCV(I);
#else
      std::cout << "No image viewer is available..." << std::endl;
#endif
    }

    for (;;) {
      g.acquire(I); // get a new frame from camera

      if (opt_save) {
        static unsigned int frame = 0;
        char buf[FILENAME_MAX];
        sprintf(buf, opt_opath.c_str(), frame++);
        std::string filename(buf);
        std::cout << "Write: " << filename << std::endl;
        vpImageIo::write(I, filename);
      }

      vpDisplay::display(I);
      vpDisplay::displayText(I, 10, 10, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (opt_click && opt_display) {
        if (vpDisplay::getClick(I, false) == true)
          break;
      } else {
        static unsigned int cpt = 0;
        if (cpt++ == 10)
          break;
      }
    }
    if (display)
      delete display;

    // The camera connection will be closed automatically in vpFlyCapture
    // destructor
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main() {
  std::cout << "You do not have PointGrey FlyCapture SDK enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install FlyCapture SDK, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
