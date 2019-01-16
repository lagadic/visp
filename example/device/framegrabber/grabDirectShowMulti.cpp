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
 * Anthony Saunier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

/*!
\file grabDirectShowMulti.cpp

\brief Example of framegrabbing using vpDirectShowGrabber class.

*/

#include <vector>

#include <iostream>
#include <sstream>

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
//#define GETOPTARGS	"dhn:o:"
#define GETOPTARGS "c:df:hmn:io:st:?"

#define GRAB_COLOR

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param camera : Active camera identifier.
\param nframes : Number of frames to acquire.
\param opath : Image filename when saving.

*/
void usage(const char *name, const char *badparam, unsigned int camera, unsigned int &nframes, std::string &opath)
{
  if (badparam)
    fprintf(stderr, "\nERREUR: Bad parameter [%s]\n", badparam);

  fprintf(stdout, "\n\
Acquire images using DirectShow (under Windows only) and display\n\
it using GTK or the windows GDI if GTK is not available.\n\
For a given camera, mediatype (or video mode) as well as framerate\n\
can be set.\n\
If more than one camera is connected, this example allows also to \n\
acquire images from all the cameras.\n\
\n\
SYNOPSIS\n\
%s [-t <mediatype>] [-f <framerate>] \n\
  [-c <camera id>] [-m] [-n <frames>] [-i] [-s] [-d] \n\
  [-o <filename>] [-h]\n\
  \n\
OPTIONS                                                    Default\n\
  -t [%%u] \n\
     MediaType (or video mode) to set for the active \n\
     camera. Use -s option so see which are the supported \n\
     Mediatypes. You can select the active camera \n\
     using -c option.\n\
\n\
  -f [%%d] \n\
     Framerate to set for the active camera.\n\
     You can select the active camera using -c option.\n", name);

  fprintf(stdout, "\n\
  -c [%%u]                                                    %u\n\
     Active camera identifier.\n\
     Zero is for the first camera found on the bus.\n\
\n\
  -m      \n\
     Flag to active multi camera acquisition.       \n\
     You need at least two cameras connected on the bus.\n\
\n\
  -n [%%u]                                                    %u\n\
     Number of frames to acquire.\n\
\n\
  -i      \n\
     Flag to print camera information.\n\
\n\
  -s      \n\
     Print camera settings capabilities such as MediaType \n\
     and sizes available and exit.\n\
\n\
  -d      \n\
     Flag to turn off image display.\n\
\n\
  -o [%%s] \n\
     Filename for image saving.                     \n\
     Example: -o %s\n\
     The first %%d is for the camera id, %%04d\n\
     is for the image numbering.\n\
\n\
  -h \n\
     Print the help.\n\
\n", camera, nframes, opath.c_str());

  exit(0);
}

/*!

Set the program options.

\param argc : Command line number of parameters.
\param argv : Array of command line parameters.
\param multi : Multi camera framegrabbing activation.
\param camera : Active camera identifier.
\param nframes : Number of frames to acquire.

\param verbose_info : Camera information printing.
\param verbose_settings : Camera settings printing.

\param mediatype_is_set : New mediatype setting.
\param mediatypeID : Mediatype setting.

\param framerate_is_set : New framerate setting.
\param framerate : Framerate setting.

\param display : Display activation.
\param save : Image saving activation.
\param opath : Image filename when saving.

*/

void read_options(int argc, const char **argv, bool &multi, unsigned int &camera, unsigned int &nframes,
                  bool &verbose_info, bool &verbose_settings, bool &mediatype_is_set, unsigned int &mediatypeID,
                  bool &framerate_is_set, double &framerate, bool &display, bool &save, std::string &opath)
{
  const char *optarg;
  int c;
  /*
   * Lecture des options.
   */

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {
    switch (c) {
    case 'c':
      camera = atoi(optarg);
      break;
    case 'd':
      display = false;
      break;
    case 'f':
      framerate_is_set = true;
      framerate = atoi(optarg);
      break;
    case 'i':
      verbose_info = true;
      break;
    case 'm':
      multi = true;
      break;
    case 'n':
      nframes = atoi(optarg);
      break;
    case 'o':
      save = true;
      opath = optarg;
      break;
    case 's':
      verbose_settings = true;
      break;
    case 't':
      mediatype_is_set = true;
      mediatypeID = atoi(optarg);
      break;
    default:
      usage(argv[0], NULL, camera, nframes, opath);
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, camera, nframes, opath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
  }
}

/*!
\example grabDirectShowMulti.cpp

Example of framegrabbing using vpDirectShowGrabber class.

Grab grey level images using DirectShow frame grabbing capabilities. Display
the images using the GTK or GDI display.
*/
int main(int argc, const char **argv)
{
  try {
    unsigned int camera = 0;
    bool multi = false;
    bool verbose_info = false;
    bool verbose_settings = false;
    bool display = true;
    unsigned int nframes = 50;
    bool mediatype_is_set = false;
    unsigned int mediatypeID;
    bool framerate_is_set = false;
    double framerate;
    bool save = false;

#ifdef GRAB_COLOR
    vpImage<vpRGBa> *I;
    std::string opath = "C:/temp/I%d-%04d.ppm";
#else
    vpImage<unsigned char> *I;
    std::string opath = "C:/temp/I%d-%04d.pgm";
#endif
#if defined VISP_HAVE_GDI
    vpDisplayGDI *d;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK *d;
#endif
    read_options(argc, argv, multi, camera, nframes, verbose_info, verbose_settings, mediatype_is_set, mediatypeID,
                 framerate_is_set, framerate, display, save, opath);

    // Number of cameras connected on the bus
    vpDirectShowGrabber *g;
    g = new vpDirectShowGrabber();
    unsigned int ncameras = g->getDeviceNumber();
    // Check the consistancy of the options
    if (multi) {
      // ckeck if two cameras are connected
      if (ncameras < 2) {
        std::cout << "You have only " << ncameras << " camera connected on the bus." << std::endl;
        std::cout << "It is not possible to active multi-camera acquisition." << std::endl;
        std::cout << "Disable -m command line option, or connect an other " << std::endl;
        std::cout << "cameras on the bus." << std::endl;
        g->close();
        delete g;
        return (0);
      }
    }
    if (camera >= ncameras) {
      std::cout << "You have only " << ncameras;
      std::cout << " camera connected on the bus." << std::endl;
      std::cout << "It is not possible to select camera " << camera << std::endl;
      std::cout << "Check your -c <camera> command line option." << std::endl;
      g->close();
      delete g;
      return (0);
    }
    if (multi) {
      camera = 0; // to over write a bad option usage
      // reinitialize the grabbers with the right number of devices (one
      // grabber per device)
      delete[] g;
      g = new vpDirectShowGrabber[ncameras];
      for (unsigned int i = 0; i < ncameras; i++) {
        g[i].open();
      }

    } else {
      ncameras = 1; // acquisition from only one camera
      delete[] g;
      g = new vpDirectShowGrabber[1];
      g[0].open();
      g[0].setDevice(camera);
    }

// allocate an image and display for each camera to consider
#ifdef GRAB_COLOR
    I = new vpImage<vpRGBa>[ncameras];
#else
    I = new vpImage<unsigned char>[ncameras];
#endif
    if (display)

#ifdef VISP_HAVE_GDI
      d = new vpDisplayGDI[ncameras];
#else
      d = new vpDisplayGTK[ncameras];
#endif
    // If required modify camera settings

    if (mediatype_is_set) {
      g[0].setMediaType(mediatypeID);
    }

    if (framerate_is_set) {
      for (unsigned int i = 0; i < ncameras; i++) {
        unsigned int c;
        if (multi)
          c = i;
        else
          c = camera;
        std::cout << "camera " << c << std::endl;
        if (!g[i].setFramerate(framerate))
          std::cout << "Set Framerate failed !!" << std::endl << std::endl;
      }
    }

    // Display information for each camera
    if (verbose_info || verbose_settings) {

      std::cout << "----------------------------------------------------------" << std::endl;
      std::cout << "---- Device List : " << std::endl;
      std::cout << "----------------------------------------------------------" << std::endl;
      g[0].displayDevices();
      for (unsigned i = 0; i < ncameras; i++) {
        unsigned int c;
        if (multi)
          c = i;
        else
          c = camera;

        if (verbose_info) {
          unsigned int width, height;
          g[i].getFormat(width, height, framerate);
          std::cout << "----------------------------------------------------------" << std::endl
                    << "---- MediaType and framerate currently used by device " << std::endl
                    << "---- (or camera) " << c << std::endl
                    << "---- Current MediaType : " << g[i].getMediaType() << std::endl
                    << "---- Current format : " << width << " x " << height << " at " << framerate << " fps"
                    << std::endl
                    << "----------------------------------------------------------" << std::endl;
        }
        if (verbose_settings) {
          std::cout << "----------------------------------------------------------" << std::endl
                    << "---- MediaTypes supported by device (or camera) " << c << std::endl
                    << "---- One of the MediaType below can be set using " << std::endl
                    << "---- option -t <mediatype>." << std::endl
                    << "----------------------------------------------------------" << std::endl;
          g[i].getStreamCapabilities();
        }
      }
      delete[] g;
      delete[] I;
      if (display)
        delete[] d;

      return 0;
    }

    // Do a first acquisition to initialise the display
    for (unsigned int i = 0; i < ncameras; i++) {
      // Acquire the first image
      g[i].acquire(I[i]);
      unsigned int c;
      if (multi)
        c = i;
      else
        c = camera;

      std::cout << "Image size for camera " << c << " : width: " << I[i].getWidth() << " height: " << I[i].getHeight()
                << std::endl;

      if (display) {
        // Initialise the display
        char title[100];
        sprintf(title, "Images captured by camera %u", c);
        d[i].init(I[i], 100 + i * 50, 100 + i * 50, title);
      }
    }

    // Main loop for single or multi-camera acquisition and display
    std::cout << "Capture in process..." << std::endl;

    double tbegin = 0, ttotal = 0;

    ttotal = 0;
    tbegin = vpTime::measureTimeMs();
    for (unsigned i = 0; i < nframes; i++) {
      for (unsigned c = 0; c < ncameras; c++) {
        // Acquire an image
        g[c].acquire(I[c]);
        if (display) {
          // Display the last image acquired
          vpDisplay::display(I[c]);
          vpDisplay::flush(I[c]);
        }
        if (save) {
          char buf[FILENAME_MAX];
          sprintf(buf, opath.c_str(), c, i);
          std::string filename(buf);
          std::cout << "Write: " << filename << std::endl;
          vpImageIo::write(I[c], filename);
        }
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
    delete[] g;

    // Free memory
    delete[] I;
    if (display)
      delete[] d;

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
