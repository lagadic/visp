/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

/*!
  \file grabOpenCV.cpp

  \brief Example of framegrabbing using vpOpenCVGrabber class.

*/

#if defined (VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408)


#include <visp3/sensor/vpOpenCVGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpTime.h>

// List of allowed command line options
#define GETOPTARGS	"dhn:o:D:"

void usage(const char *name, const char *badparam, unsigned int &nframes, std::string &opath);
bool getOptions(int argc, const char **argv, bool &display,
                unsigned int &nframes, bool &save, std::string &opath, int &deviceType);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.

*/
void usage(const char *name, const char *badparam, unsigned int &nframes, std::string &opath)
{
  fprintf(stdout, "\n\
Acquire and display images using OpenCV library.\n\
\n\
SYNOPSIS\n\
  %s [-d] [-n] [-o] [-h] \n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -d \n\
     Turn off the display.\n\
\n\
  -D [%%s]                                               ANY\n\
     Type of device to detect.                           \n\
     It can be ANY, MIL, VFW, V4L, V4L2, DC1394, CMU1394, DSHOW. \n\
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
    fprintf(stderr, "ERROR: \n" );
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
  \param deviceType : Type of device to detect.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &display,
                unsigned int &nframes, bool &save, std::string &opath, int &deviceType)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'D':
      if (strcmp( optarg_ ,"ANY") == 0 ) {deviceType = CV_CAP_ANY;}
      else if ( strcmp( optarg_ ,"MIL") == 0) {deviceType = CV_CAP_MIL;}
      else if ( strcmp( optarg_ ,"VFW") == 0) {deviceType = CV_CAP_VFW;}
      else if ( strcmp( optarg_ ,"V4L") == 0) {deviceType = CV_CAP_V4L;}
      else if ( strcmp( optarg_ ,"V4L2") == 0) {deviceType = CV_CAP_V4L2;}
      else if ( strcmp( optarg_ ,"DC1394") == 0) {deviceType = CV_CAP_DC1394;}
      else if ( strcmp( optarg_ ,"CMU1394") == 0) {deviceType = CV_CAP_CMU1394;}
      else if ( strcmp( optarg_ ,"DSHOW") == 0) {deviceType = CV_CAP_DSHOW;}
      else {std::cout << "Unknown type of device" << std::endl;
	      deviceType = 0;}
      break;
    case 'n':
      nframes = (unsigned int)atoi(optarg_); break;
    case 'o':
      save = true;
      opath = optarg_; break;
    case 'h': usage(argv[0], NULL, nframes, opath); return false; break;

    default:
      usage(argv[0], optarg_, nframes, opath);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, nframes, opath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}


/*!
  \example grabOpenCV.cpp

  Example of framegrabbing using vpOpenCVGrabber class.

  Grab and display grey level images using OpenCV frame grabbing capabilities.
*/
int
main(int argc, const char ** argv)
{
  try {
    bool opt_display = true;
    unsigned nframes = 50;
    bool save = false;
    int deviceType = CV_CAP_ANY;

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
#  if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    std::string opath = "/tmp/I%04d.ppm";
#  elif defined(_WIN32)
    std::string opath = "C:/temp/I%04d.ppm";
#  endif
#else
    // B&W images will be saved in PGM P5 format
#  if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    std::string opath = "/tmp/I%04d.pgm";
#  elif defined(_WIN32)
    std::string opath = "C:/temp/I%04d.pgm";
#  endif
#endif

    // Read the command line options
    if (getOptions(argc, argv, opt_display, nframes, save, opath, deviceType) == false) {
      exit (-1);
    }
    // Create the grabber
    vpOpenCVGrabber grabber ;
    try {
      // Set the type of device to detect. Here for example we expect to find a firewire camera.
      grabber.setDeviceType(deviceType);

      // Initialize the grabber
      grabber.open(I);

      // Acquire an image
      grabber.acquire(I);
    }
    catch(...)
    {
      vpCTRACE << "Cannot acquire an image... "
               << "Check if a camera is connected to your computer."
               << std::endl ;
      return 0;
    }

    std::cout << "Image size: width : " << I.getWidth() <<  " height: "
              << I.getHeight() << std::endl;

    // Creates a display
    vpDisplayOpenCV display;

    if (opt_display) {
      display.init(I,100,100,"OpenCV framegrabber");
    }

    double tbegin=0, tend=0, tloop=0, ttotal=0;

    ttotal = 0;
    tbegin = vpTime::measureTimeMs();
    // Loop for image acquisition and display
    for (unsigned i = 0; i < nframes; i++) {
      //Acquires an RGBa image
      grabber.acquire(I);

      if (opt_display) {
        //Displays the grabbed image
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
      tend = vpTime::measureTimeMs();
      tloop = tend - tbegin;
      tbegin = tend;
      std::cout << "loop time: " << tloop << " ms" << std::endl;
      ttotal += tloop;
    }
    std::cout << "Mean loop time: " << ttotal / nframes << " ms" << std::endl;
    std::cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << std::endl;

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
#else // defined (VISP_HAVE_OPENCV) 
int
main()
{
  vpTRACE("OpenCV is not available...") ;
}
#endif // defined (VISP_HAVE_OPENCV) 


