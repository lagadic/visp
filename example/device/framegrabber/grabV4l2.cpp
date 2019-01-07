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
 * Acquire images using 1394 device with cfox (MAC OSX) and display it
 * using GTK or GTK.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
/*!
  \file grabV4l2.cpp

  \brief Example of image framegrabbing using vpV4l2Grabber class.

*/

#ifdef VISP_HAVE_V4L2

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vpV4l2Grabber.h>

// List of allowed command line options
#define GETOPTARGS "df:i:hn:o:p:s:t:v:x"

typedef enum {
  grey_image = 0, // for ViSP unsigned char grey images
  color_image     // for ViSP vpRGBa color images
} vpImage_type;

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param fps : Framerate.
  \param input : Card input number.
  \param scale : Subsampling factor.
  \param niter : Number of images to acquire.
  \param device : Video device name.
  \param pixelformat : Pixel format.
  \param image_type : 0 for unsigned char, 1 for vpRGBa images
  \param opath : Image filename when saving.

*/
void usage(const char *name, const char *badparam, unsigned fps, unsigned input, unsigned scale, long niter,
           char *device, vpV4l2Grabber::vpV4l2PixelFormatType pixelformat, const vpImage_type &image_type,
           const std::string &opath)
{
  fprintf(stdout, "\n\
Grab grey level images using the Video For Linux Two framegrabber. \n\
Display these images using X11 or GTK.\n\
\n\
SYNOPSIS\n\
  %s [-v <video device>] [-f <fps=25|50>] \n\
     [-i <input=0|1|2|3> [-s <scale=1|2|4>] [-p <pixel format>]\n\
     [-n <niter>] [-t <image type>] [-o <filename>] [-x] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                                  Default\n\
  -v <video device>                                         %s\n\
     Video device to access to the camera\n\
\n\
  -f <fps>                                                  %u\n\
     Framerate in term od number of images per second.\n\
     Possible values are 25 (for 25Hz) or 50 (for %%) Hz)\n\
\n\
  -i <input>                                                %u\n\
     Framegrabber active input. Values can be 0, 1, 2, 4\n\
\n\
  -p <pixel format>                                         %d\n\
     Camera pixel format. Values must be in [0-%d]:\n\
       0 for gray format\n\
       1 for RGB24 format\n\
       2 for RGB32 format\n\
       3 for BGR24 format\n\
       4 for YUYV format\n\
\n\
  -t <image type>                                           %d\n\
     Kind of images that are acquired/displayed by ViSP. \n\
     Values must be in [0-1]:\n\
       0 for grey images in unsigned char \n\
       1 for color images in vpRGBa\n\
\n\
  -s <scale>                                                %u\n\
     Framegrabber subsampling factor. \n\
     If 1, full resolution image acquisition.\n\
     If 2, half resolution image acquisition. The \n\
     subsampling is achieved by the hardware.\n\
\n\
  -n <niter>                                                %ld\n\
     Number of images to acquire.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -x \n\
     Activates the extra verbose mode.\n\
\n\
  -o [%%s] : Filename for image saving.                     \n\
     Example: -o %s\n\
     The %%d is for the image numbering. The format is set \n\
     by the extension of the file (ex .png, .pgm, ...) \n\
                    \n\
  -h \n\
     Print the help.\n\n", device, fps, input, pixelformat, vpV4l2Grabber::V4L2_MAX_FORMAT - 1, image_type, scale, niter,
          opath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param fps : Framerate.
  \param input : Card input.
  \param scale : Subsampling factor.
  \param display : Display activation.
  \param verbose : Verbose mode activation.
  \param niter : Number of images to acquire.
  \param device : Video device name.
  \param pixelformat : Pixel format.
  \param image_type : 0 for unsigned char, 1 for vpRGBa images
  \param save : Image saving activation.
  \param opath : Image filename when saving.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, unsigned &fps, unsigned &input, unsigned &scale, bool &display,
                bool &verbose, long &niter, char *device, vpV4l2Grabber::vpV4l2PixelFormatType &pixelformat,
                vpImage_type &image_type, bool &save, std::string &opath)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'd':
      display = false;
      break;
    case 'f':
      fps = (unsigned)atoi(optarg_);
      break;
    case 'i':
      input = (unsigned)atoi(optarg_);
      break;
    case 'n':
      niter = atol(optarg_);
      break;
    case 'o':
      save = true;
      opath = optarg_;
      break;
    case 'p':
      pixelformat = (vpV4l2Grabber::vpV4l2PixelFormatType)atoi(optarg_);
      break;
    case 's':
      scale = (unsigned)atoi(optarg_);
      break;
    case 't':
      image_type = (vpImage_type)atoi(optarg_);
      break;
    case 'v':
      sprintf(device, "%s", optarg_);
      break;
    case 'x':
      verbose = true;
      break;
    case 'h':
      usage(argv[0], NULL, fps, input, scale, niter, device, pixelformat, image_type, opath);
      return false;
      break;

    default:
      usage(argv[0], optarg_, fps, input, scale, niter, device, pixelformat, image_type, opath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, fps, input, scale, niter, device, pixelformat, image_type, opath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  \example grabV4l2.cpp

  Example of grey level images framegrabbing using vpV4l2Grabber class.

  Test frame grabbing capabilities using video for linux two (V4L2) video
  device.  Only grabbing of grey level images is possible in this
  example. Display these images using X11 or GTK.
*/
int main(int argc, const char **argv)
{
  try {
    unsigned int opt_fps = 25;
    unsigned int opt_input = 0;
    unsigned int opt_scale = 1;
    vpV4l2Grabber::vpV4l2PixelFormatType opt_pixelformat = vpV4l2Grabber::V4L2_YUYV_FORMAT;
    long opt_iter = 100;
    bool opt_verbose = false;
    bool opt_display = true;
    char opt_device[20];
    bool opt_save = false;
    sprintf(opt_device, "/dev/video0");
    // Default output path for image saving
    std::string opt_opath = "/tmp/I%04d.ppm";

    vpImage_type opt_image_type = color_image;

    // Read the command line options
    if (getOptions(argc, argv, opt_fps, opt_input, opt_scale, opt_display, opt_verbose, opt_iter, opt_device,
                   opt_pixelformat, opt_image_type, opt_save, opt_opath) == false) {
      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char) and
    // an other one that is a color image. There size is not defined
    // yet. It will be defined when the image will acquired the first
    // time.
    vpImage<unsigned char> Ig; // grey level image
    vpImage<vpRGBa> Ic;        // color image

    // Creates the grabber
    vpV4l2Grabber g;

    // Initialize the grabber
    g.setVerboseMode(opt_verbose);
    g.setDevice(opt_device);
    g.setInput(opt_input);
    g.setScale(opt_scale);
    g.setPixelFormat(opt_pixelformat);
    if (opt_fps == 25)
      g.setFramerate(vpV4l2Grabber::framerate_25fps);
    else
      g.setFramerate(vpV4l2Grabber::framerate_50fps);
    if (opt_image_type == grey_image) {
      // Open the framegrabber with the specified settings on grey images
      g.open(Ig);
      // Acquire an image
      g.acquire(Ig);
      std::cout << "Grey image size: width : " << Ig.getWidth() << " height: " << Ig.getHeight() << std::endl;
    } else {
      // Open the framegrabber with the specified settings on color images
      g.open(Ic);
      // Acquire an image
      g.acquire(Ic);
      std::cout << "Color image size: width : " << Ic.getWidth() << " height: " << Ic.getHeight() << std::endl;
    }

// We open a window using either X11 or GTK.
// Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#endif

    if (opt_display) {
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      if (opt_image_type == grey_image) {
        display.init(Ig, 100, 100, "V4L2 grey images framegrabbing");
        vpDisplay::display(Ig);
        vpDisplay::flush(Ig);
      } else {
        display.init(Ic, 100, 100, "V4L2 color images framegrabbing");
        vpDisplay::display(Ic);
        vpDisplay::flush(Ic);
      }
    }
    // Acquisition loop
    long cpt = 1;
    while (cpt++ < opt_iter) {
      // Measure the initial time of an iteration
      double t = vpTime::measureTimeMs();
      // Acquire the image
      if (opt_image_type == grey_image) {
        g.acquire(Ig);
        if (opt_display) {
          // Display the image
          vpDisplay::display(Ig);
          // Flush the display
          vpDisplay::flush(Ig);
        }
      } else {
        g.acquire(Ic);
        if (opt_display) {
          // Display the image
          vpDisplay::display(Ic);
          // Flush the display
          vpDisplay::flush(Ic);
        }
      }

      if (opt_save) {
        char buf[FILENAME_MAX];
        sprintf(buf, opt_opath.c_str(), cpt);
        std::string filename(buf);
        std::cout << "Write: " << filename << std::endl;
        if (opt_image_type == grey_image) {
          vpImageIo::write(Ig, filename);
        } else {
          vpImageIo::write(Ic, filename);
        }
      }

      // Print the iteration duration
      std::cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << std::endl;
    }

    g.close();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You do not have X11, or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GTK, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
#else
int main()
{
  std::cout << "You do not have Video 4 Linux 2 functionality enabled" << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install libv4l2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
