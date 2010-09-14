/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \file grabV4l2Grey.cpp

  \brief Example of grey level image framegrabbing using vpV4l2Grabber class.

*/

#ifdef VISP_HAVE_V4L2

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <visp/vpV4l2Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"df:i:hn:s:v"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param fps : Framerate.
  \param input : Card input number.
  \param scale : Subsampling factor.
  \param niter : Number of images to acquire.

*/
void usage(const char *name, const char *badparam, unsigned fps, 
	   unsigned input, unsigned scale, long niter)
{
  fprintf(stdout, "\n\
Grab grey level images using the Video For Linux Two framegrabber. \n\
Display these images using X11 or GTK.\n\
\n\
SYNOPSIS\n\
  %s [-f <fps=25|50>] [-i <input=0|1|2|3> \n\
     [-s <scale=1|2|4>] [-n <niter>] [-v] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                                  Default\n\
  -f <fps>                                                  %u\n\
     Framerate in term od number of images per second.\n\
     Possible values are 25 (for 25Hz) or 50 (for %%) Hz)\n\
\n\
  -i <input>                                                %u\n\
     Framegrabber active input. Values can be 0, 1, 2, 4\n\
\n\
  -s <scale>                                                %u\n\
     Framegrabber subsampling factor. \n\
     If 1, full resolution image acquisition 768x576.\n\
     If 2, half resolution acquisition 384x288. The \n\
     subsampling is achieved by the hardware.\n\
\n\
  -n <niter>                                                %ld\n\
     Number of images to acquire.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -v \n\
     Activates the verbose mode.\n\
\n\
  -h \n\
     Print the help.\n\n",
	  fps, input, scale, niter);

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

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, unsigned &fps, unsigned &input,
		unsigned &scale, bool &display, bool &verbose, long &niter)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'f': fps = (unsigned) atoi(optarg); break;
    case 'i': input = (unsigned) atoi(optarg); break;
    case 'n': niter = atol(optarg); break;
    case 's': scale = (unsigned) atoi(optarg); break;
    case 'v': verbose = true; break;
    case 'h': usage(argv[0], NULL, fps, input, scale, niter); 
      return false; break;

    default:
      usage(argv[0], optarg, fps, input, scale, niter); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, fps, input, scale, niter);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  \example grabV4l2Grey.cpp

  Example of grey level images framegrabbing using vpV4l2Grabber class.

  Test frame grabbing capabilities using video for linux two (V4L2) video
  device.  Only grabbing of grey level images is possible in this
  example. Display these images using X11 or GTK.
*/
int
main(int argc, const char ** argv)
{
  unsigned fps = 25;
  unsigned input = vpV4l2Grabber::DEFAULT_INPUT;
  unsigned scale = vpV4l2Grabber::DEFAULT_SCALE;
  bool opt_display = true;
  bool opt_verbose = false;
  long opt_iter    = 20;

  // Read the command line options
  if (getOptions(argc, argv, fps, input, scale, opt_display, 
		 opt_verbose, opt_iter) == false) {
    exit (-1);
  }

  // Declare an image, this is a gray level image (unsigned char). It
  // size is not defined yet. It will be defined when the image will
  // acquired the first time.
  vpImage<unsigned char> I ;

  // Creates the grabber
  vpV4l2Grabber g(opt_verbose);

  try{
    // Initialize the grabber
    g.setInput(input);
    if (1) {
      g.setScale(scale);
    }
    else {
      g.setWidth(768/scale);
      g.setHeight(576/scale);
    }
    if (fps == 25)
      g.setFramerate(vpV4l2Grabber::framerate_25fps);
    else
      g.setFramerate(vpV4l2Grabber::framerate_50fps);
    // Open the framegrabber with the specified settings
    g.open(I) ;
    // Acquire an image
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot acquire an image...") ;
    return(0);
  }

  std::cout << "Image size: width : " << I.getWidth() <<  " height: "
       << I.getHeight() << std::endl;

  // We open a window using either X11 or GTK.
  // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#endif

  if (opt_display) {
    try{
      display.init(I, 100, 100, "V4L2 Framegrabber") ;
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  // Acquisition loop
  long cpt = 1;
  while(cpt ++ < opt_iter)
  {
    // Measure the initial time of an iteration
    double t = vpTime::measureTimeMs();
    // Acquire the image
    g.acquire(I) ;
    if (opt_display) {
      // Display the image
      vpDisplay::display(I) ;
      // Flush the display
      vpDisplay::flush(I) ;
    }
    // Print the iteration duration
    std::cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << std::endl;
  }

  g.close();
}
#else
int
main()
{
  vpTRACE("X11 or GTK display are not available") ;
}
#endif
#else
int
main()
{
  vpTRACE("Video 4 Linux 2 frame grabber drivers are not available") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
