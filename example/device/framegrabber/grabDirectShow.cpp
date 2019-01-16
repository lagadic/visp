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
 * Acquire images using DirectShow (under Windows only) and display it
 * using GTK or GDI.
 *
 * Authors:
 * Bruno Renier
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

/*!
  \file grabDirectShow.cpp

  \brief Example of framegrabbing using vpDirectShowGrabber class.

*/

#if defined(VISP_HAVE_DIRECTSHOW)
#if (defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vpDirectShowGrabber.h>

// List of allowed command line options
#define GETOPTARGS "dhn:o:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.

*/
void usage(const char *name, const char *badparam, unsigned &nframes, std::string &opath)
{
  fprintf(stdout, "\n\
Acquire images using DirectShow (under Windows only) and display\n\
it using GTK or the windows GDI if GTK is not available.\n\
\n\
SYNOPSIS\n\
  %s [-d] [-n] [-o] [-h] \n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -d \n\
     Turn off the display.\n\
\n\
  -n [%%u]                                               %u\n\
     Number of frames to acquire.               \n\
\n\
  -o [%%s] \n\
     Filename for image saving.                    \n\
     Example: -o %s\n\
     The %%d is for the image numbering.\n\
\n\
  -h \n\
     Print the help.\n\
\n", nframes, opath.c_str());
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
  \param nframes : Number of frames to acquire.
  \param save : Image saving activation.
  \param opath : Image filename when saving.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &display, unsigned &nframes, bool &save, std::string &opath)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd':
      display = false;
      break;
    case 'n':
      nframes = atoi(optarg);
      break;
    case 'o':
      save = true;
      opath = optarg;
      break;
    case 'h':
      usage(argv[0], NULL, nframes, opath);
      return false;
      break;

    default:
      usage(argv[0], optarg, nframes, opath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, nframes, opath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  \example grabDirectShow.cpp

  Example of framegrabbing using vpDirectShowGrabber class.

  Grab grey level images using DirectShow frame grabbing capabilities. Display
  the images using the GTK or GDI display.
*/
int main(int argc, const char **argv)
{
  try {
    bool opt_display = true;
    unsigned nframes = 50;
    bool save = false;

// Declare an image. It size is not defined yet. It will be defined when the
// image will acquired the first time.
#ifdef GRAB_COLOR
    vpImage<vpRGBa> I; // This is a color image (in RGBa format)
#else
    vpImage<unsigned char> I; // This is a B&W image
#endif

// Set default output image name for saving
#ifdef GRAB_COLOR
    // Color images will be saved in PGM P6 format
    std::string opath = "C:/temp/I%04d.ppm";
#else
    // B&W images will be saved in PGM P5 format
    std::string opath = "C:/temp/I%04d.pgm";
#endif

    // Read the command line options
    if (getOptions(argc, argv, opt_display, nframes, save, opath) == false) {
      exit(-1);
    }
    // Create the grabber
    vpDirectShowGrabber *grabber = new vpDirectShowGrabber();

    // test if a camera is connected
    if (grabber->getDeviceNumber() == 0) {
      vpCTRACE << "there is no camera detected on your computer." << std::endl;
      grabber->close();
      exit(0);
    }
    // Initialize the grabber
    grabber->open(I);

    // Acquire an image
    grabber->acquire(I);

    std::cout << "Image size: width : " << I.getWidth() << " height: " << I.getHeight() << std::endl;

// Creates a display
#if defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#endif

    if (opt_display) {
      display.init(I, 100, 100, "DirectShow Framegrabber");
    }

    double tbegin = 0, ttotal = 0;

    ttotal = 0;
    tbegin = vpTime::measureTimeMs();
    // Loop for image acquisition and display
    for (unsigned i = 0; i < nframes; i++) {
      // Acquires an RGBa image
      grabber->acquire(I);

      if (opt_display) {
        // Displays the grabbed rgba image
        vpDisplay::display(I);
        vpDisplay::flush(I);
      }

      if (save) {
        char buf[FILENAME_MAX];
        sprintf(buf, opath.c_str(), i);
        std::string filename(buf);
        std::cout << "Write: " << filename << std::endl;
        vpImageIo::write(I, filename);
      }
      double tend = vpTime::measureTimeMs();
      double tloop = tend - tbegin;
      tbegin = tend;
      std::cout << "loop time: " << tloop << " ms" << std::endl;
      ttotal += tloop;
    }
    std::cout << "Mean loop time: " << ttotal / nframes << " ms" << std::endl;
    std::cout << "Mean frequency: " << 1000. / (ttotal / nframes) << " fps" << std::endl;

    // Release the framegrabber
    delete grabber;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else  // (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

int main()
{
  std::cout << "You do not have GDI (Graphical Device Interface), or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif // (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))
#else  // defined (VISP_HAVE_DIRECTSHOW)
int main()
{
  std::cout << "This example requires Direct Show SDK. " << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install Direct Show, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif // defined (VISP_HAVE_DIRECTSHOW)
