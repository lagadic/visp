/****************************************************************************
 *
 * $Id: grab1394.cpp,v 1.1 2007-01-24 15:04:20 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file grab1394.cpp
  \brief Test firewire cameras video capture using vp1394Grabber.

  \warning This class needs at least libdc1394-1.0.0 and
  libraw1394-1.1.0. These libraries are available from
  http://sourceforge.net/projects/libdc1394 and
  http://sourceforge.net/projects/libraw1394 .

  vp1394Grabber was tested with MF-033C and F-131B Marlin cameras.
*/

#include <iostream>
#include <sstream>

using namespace std;

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined(VISP_HAVE_DC1394_1)

#include <visp/vp1394Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!
  \example grab1394.cpp

  Test frame grabbing capabilities using ieee 1394 video device.
*/

// List of allowed command line options
#define GETOPTARGS	"c:df:g:m:n:r:s:t:h"

/*

  Print the program options.

 */
void usage(char *name, char *badparam, vp1394Grabber &g, long &req_number,
	   bool &req_display, bool &req_color)
{
  unsigned int act_camera;
  int act_format;
  int act_mode;
  int act_framerate;
  unsigned int act_shutter;
  unsigned int min_shutter, max_shutter;
  unsigned int act_gain;
  unsigned int min_gain, max_gain;
  vpList<int> l_framerates; // list of supported framerates
  vpList<int> l_formats; // list of supported formats
  vpList<int> l_modes; // list of supported modes
  int n_formats; // number of supported formats
  int n_modes; // number of supported mode for this format
  int n_framerates; // number of supported framerates

  g.getCamera(act_camera);
  g.getFormat(act_format);
  g.getMode(act_mode);
  g.getFramerate(act_framerate);
  g.getShutter(min_shutter, act_shutter, max_shutter);
  g.getGain(min_gain, act_gain, max_gain);

  n_framerates = g.getFramerateSupported(act_format, act_mode, l_framerates);

  fprintf(stdout, "\n\
Test firewire 1394 framegrabbing.\n\
\n\
SYNOPSIS\n\
  %s [-c <camera>] [-f <format>] [-m <mode>] \n\
  [-f <framerate>] [-s <shutter>] [-g <gain>] [-n <number>]\n\
  [-d] [-t <color>] [-h]\n\
", name);

  fprintf(stdout, "\n\
CAMERA OPTIONS:                                   Default\n\
  -c <camera>                                        %d\n\
     Select the active camera on the bus:\n\
\n\
  -f <format>                                        %d\n\
     Change the format for the active camera. \n\
     Available formats are:\n", act_camera, act_format);

  // Display the available values for the format
  n_formats = g.getFormatSupported(l_formats);
  l_formats.front();
  while (! l_formats.outside()) {
    int value = l_formats.value();
    string _value = g.convertFormat(value);
    fprintf(stdout, "       %d (%s)\n", value, _value.c_str());
    l_formats.next();
  }

  fprintf(stdout, "\n\
  -m <mode>                                          %d\n\
     Change the mode associated to the format for \n\
     the active camera. \n\
     Available modes are:\n", act_mode);

  // Get the available values for the mode considering the specified format
  n_formats = g.getModeSupported(act_format, l_modes);
  // Display the available values for the mode
  l_modes.front();
  while (! l_modes.outside()) {
    int value = l_modes.value();
    string _value = g.convertMode(value);
    fprintf(stdout, "       %d (%s)\n", value, _value.c_str());
    l_modes.next();
  }

  fprintf(stdout, "\n\
  -r <framerate>                                     %d\n\
     Change the framerate for the active camera.\n\
     Values depend on the camera format and mode. \n\
     For the actual format and mode available \n\
     values are:\n", act_framerate);

  // Get the available values for the framerate considering the specified
  // camera format and mode
  n_framerates = g.getFramerateSupported(act_format, act_mode, l_framerates);

  // Display the available values for the framerate
  l_framerates.front();
  while (! l_framerates.outside()) {
    int value = l_framerates.value();
    string _value = g.convertFramerate(value);
    fprintf(stdout, "       %d (%s)\n", value, _value.c_str());
    l_framerates.next();
  }

  fprintf(stdout, "\n\
  -s <shutter>                                       %d\n\
     Change the shutter for the active camera.\n\
     Value must be comprised between %u and %u.\n",
	  act_shutter, min_shutter, max_shutter);

  fprintf(stdout, "\n\
  -g <gain>                                          %d\n\
     Change the gain for the active camera.\n\
     Value must be comprised between %u and %u.\n",
	  act_gain, min_gain, max_gain);

  fprintf(stdout, "\n\
ACQUISITION OPTIONS:\n\
  -n <number>                                        %ld\n\
     Number of frame to acquire.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -t <color>                                         %d\n\
     Type of display (greylevel or color images).\n\
     If 0, convert acquired images into greylevel,\n\
     if 1, convert acquired images to color RGBa.\n",
	  req_number, req_color);


  fprintf(stdout, "\n\
OTHER OPTIONS:\n\
  -h\n\
     Print the help.\n\
\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }

}

/*

  Set the program options.

*/
void getOptions(int argc, char **argv,
		vp1394Grabber &g,
		unsigned int &camera,
		int &format, int &mode, int &framerate,
		unsigned int &shutter, unsigned int &gain,
		long &number, bool &display, bool &color)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': camera = (unsigned int) atoi(optarg); break;
    case 'd': display = false; break;
    case 'f': format = atoi(optarg); break;
    case 'g': gain = (unsigned int) atoi(optarg); break;
    case 'm': mode = atoi(optarg); break;
    case 'n': number = (long) atoi(optarg); break;
    case 'r': framerate = atoi(optarg); break;
    case 's': shutter = (unsigned int) atoi(optarg); break;
    case 't': color = (bool) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, g, number, display, color); exit(0); break;

    default:
      usage(argv[0], optarg, g, number, display, color); exit(0); break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, g, number, display, color);
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "  Bad argument %s\n", optarg);
    fprintf(stderr, "\n");
    exit(0);
  }
}


int
main(int argc, char ** argv)
{
  unsigned int req_camera = 0;
  int req_format = 0;
  int req_mode = 0;
  int req_framerate = 0;
  unsigned int req_shutter = 0;
  unsigned int req_gain = 0;
  long req_number = 100;
  bool req_display = true;
  bool req_color = false;
  long cpt = 0;

  vpImage<unsigned char> I ;
  vpImage<vpRGBa> Irgb ;

  vp1394Grabber g ;

  try {
    g.open(I) ;
  }
  catch(...) {
    cout << "The program was stopped..." << endl;
    return 0;
  }

  getOptions(argc, argv, g, req_camera, req_format, req_mode, req_framerate,
	     req_shutter, req_gain, req_number, req_display, req_color);

  unsigned int cameras;
  g.getNumCameras(cameras);

  cout << endl;
  cout << "Number of cameras on the bus: " << cameras << endl;
  cout << endl;

  int act_format;
  int act_mode;
  int act_framerate;
  unsigned int act_shutter;
  unsigned int min_shutter, max_shutter;
  unsigned int act_gain;
  unsigned int min_gain, max_gain;
  for (int i=0; i < cameras; i++) {
    g.setCamera(i);
    g.getFormat(act_format);
    g.getMode(act_mode);
    g.getFramerate(act_framerate);
    g.getShutter(min_shutter, act_shutter, max_shutter);
    g.getGain(min_gain, act_gain, max_gain);

    cout << "Actual camera settings for camera: " << i << endl;
    cout << "  Format: " << act_format
	 << " (" << g.convertFormat(act_format) << ")" << endl;
    cout << "  Mode: " << act_mode
	 << " (" << g.convertMode(act_mode) << ")" << endl;
    cout << "  Framerate: " << act_framerate
	 << " (" << g.convertFramerate(act_framerate) << ")" << endl;
    cout << "  Shutter: " << act_shutter << endl;
    cout << "  Min shutter: " << min_shutter << endl;
    cout << "  Max shutter: " << max_shutter << endl;
    cout << "  Gain: " << act_gain << endl;
    cout << "  Min gain: " << min_gain << endl;
    cout << "  Max gain: " << max_gain << endl;
    cout << endl;
  }
  cout << endl;

  if (req_camera) {
    try {
      cout << "Set active camera: " << req_camera << endl;
      g.setCamera(req_camera);
    }
    catch(...) {
      vpERROR_TRACE("The requested camera %d is not on the bus",
		    req_camera);
      vpERROR_TRACE("The program is stopped...");
      exit(-1);
    }
  }

  if (req_format) {
    try {
      cout << "Set format to: " << req_format
	   << " (" << g.convertFormat(req_format) << ")" << endl;
      g.setFormat(req_format);
    }
    catch(...) {
      vpERROR_TRACE("The requested format %d is not supported by the camera",
		    req_format);
      vpERROR_TRACE("The program is stopped...");
      exit(-1);
    }
  }

  if (req_mode) {
    try {
      cout << "Set mode to: " << req_mode
	   << " (" << g.convertMode(req_mode) << ")" << endl;
      g.setMode(req_mode);
    }
    catch(...) {
      vpERROR_TRACE("The requested mode %d is not supported by the camera",
		    req_mode);
      vpERROR_TRACE("The program was stopped...");
      exit(-1);
    }
  }

  if (req_framerate) {
    try {
      cout << "Set framerate to: " << req_framerate
	   << " (" << g.convertFramerate(req_framerate) << ")" << endl;
      g.setFramerate(req_framerate);
    }
    catch(...) {
      vpERROR_TRACE("The requested framerate %d is not supported by the camera",
		    req_framerate);
      vpERROR_TRACE("The program was stopped...");
      exit(-1);
    }
  }


  if (req_shutter) {
    try {
      cout << "Set shutter to: " << req_shutter << endl;
      g.setShutter(req_shutter);
    }
    catch(...) {
      vpERROR_TRACE("The requested shutter %d is not supported by the camera",
		    req_shutter);
      vpERROR_TRACE("The program was stopped...");
      exit(-1);
    }
  }


  if (req_gain) {
    try {
      cout << "Set gain to: " << req_gain << endl;
      g.setGain(req_gain);
    }
    catch(...) {
      vpERROR_TRACE("The requested gain %d is not supported by the camera",
		    req_gain);
      vpERROR_TRACE("The program was stopped...");
      exit(-1);
    }
  }

  // Acquire an image to get the image size
  try{
    if (req_color)
      g.acquire(Irgb) ;
    else
      g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  cout << endl;
  cout << "Image size: " << I.getCols() <<  " cols "
       << I.getRows() << " rows" << endl;

  vpDisplayX d;

  if (req_display) {
    try{
      if (req_color) {
	d.init(Irgb, 100, 100, "1394 grabbing (color)... ") ;
	vpDisplay::display(Irgb) ;
	vpDisplay::flush(Irgb) ;
      }
      else {
	d.init(I, 100, 100, "1394 grabbing (greylevel)... ") ;
	vpDisplay::display(I) ;
	vpDisplay::flush(I) ;
      }
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }

  cpt = 0;
  double t, t0, tend;
  t0 = vpTime::measureTimeMs();
  while(cpt < req_number)
  {
    t = vpTime::measureTimeMs();
    if (req_color) {
      g.acquire(Irgb) ;
      if (req_display) {
	vpDisplay::display(Irgb) ;
	vpDisplay::flush(Irgb) ;
      }
    }
    else {
      g.acquire(I) ;
      if (req_display) {
	vpDisplay::display(I) ;
	vpDisplay::flush(I) ;
      }
    }
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
    cpt ++;
  }
  tend = vpTime::measureTimeMs();
  cout << "Mean acq time: " << (tend - t0) / cpt << " (ms) " << endl ;
  cout << "Mean fps: " << cpt / (tend - t0) * 1000 << " (Hz) " << endl ;
}
#else
int
main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available..."
	  "You should install libdc1394-1 to use this example.") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
