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
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file grab1394Two.cpp

  \brief Example of framegrabbing using vp1394TwoGrabber class.

  \warning This class needs at least libdc1394-2.0.0-rc4 and
  libraw1394-1.1.0. These libraries are available from
  http://sourceforge.net/projects/libdc1394 and
  http://sourceforge.net/projects/libraw1394 .

  vp1394TwoGrabber was tested with MF-033C and F-131B Marlin cameras.
*/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined(VISP_HAVE_DC1394_2)

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>
#include <visp/vpRGBa.h>

#define GRAB_COLOR


// List of allowed command line options
#define GETOPTARGS	"b:c:df:g:hH:L:mn:io:p:rsT:v:W:"


#define DUAL_ACQ


/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param camera : Active camera identifier.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.
  \param roi_left, roi_top, roi_width, roi_height : Region of interest in
  format 7.
  \param ringbuffersize : Ring buffer size used for capture.
  \param panControl : Pan control value.

*/
void usage(const char *name, const char *badparam, unsigned int camera,
	   const unsigned int &nframes, const std::string &opath,
	   const unsigned int &roi_left, const unsigned int &roi_top,
	   const unsigned int &roi_width, const unsigned int &roi_height,
	   const unsigned int &ringbuffersize, const unsigned int &panControl)
{
  if (badparam)
    fprintf(stderr, "\nERROR: Bad parameter [%s]\n", badparam);

  fprintf(stderr, "\n\
SYNOPTIQUE\n\
    %s [-v <video mode>] [-f <framerate>] \n\
    [-g <color coding>] [-c <camera id>] [-m] [-n <frames>] \n\
    [-i] [-s] [-d] [-o <filename>] [-L <format 7 roi left position>] \n\
    [-T <format 7 roi top position>] [-W <format 7 roi width>] \n\
    [-H <format 7 roi height>] [-b <ring buffer size>] \n\
    [-p <pan control value>] [-R] [-h]\n\
\n\
DESCRIPTION\n\
    Test for firewire camera image acquisition.\n\
\n\
EXAMPLES\n\
    If two cameras are connected on the bus:\n\
    %s -s\n\
       Indicates the current settings for the first camera found on the bus.\n\
    %s -i\n\
       Gives information on the first camera found on the bus.\n\
    %s -s -m\n\
       Indicates the current settings for all the cameras found on the bus.\n\
    %s -i -m\n\
       Gives information on all the cameras found on the bus.\n\
    %s -c 1\n\
       Grab images from camera 1.\n\
    %s -m\n\
       Grab images from all the cameras.\n\
\n\
    If a stereo camera is connected to the bus like the PointGrey Bumblebee,\n\
    you may set the pan control to select the camera view:\n\
    %s -p 0\n\
       Transmit right imge.\n\
    %s -p 1\n\
       Transmit left imge.\n\
\n\
OPTIONS                                                    Default\n\
    -v [%%u] : Video mode to set for the active camera.\n\
              Use -s option so see which are the supported \n\
              video modes. You can select the active \n\
              camera using -c option.\n\
\n\
    -f [%%u] : Framerate to set for the active camera.\n\
              Use -s option so see which are the supported \n\
              framerates. You can select the active \n\
              camera using -c option.\n\
\n\
    -g [%%u] : Color coding to set for the active camera\n\
              in format 7 video mode. Use -s option so see if \n\
              format 7 is supported by the camera and if so, \n\
              which are the supported color codings. You can \n\
              select the active camera using -c option.\n\
              See -t <top>, -l <left>, -w <width>, \n\
              -h <height> option to set format 7 roi.\n\
\n\
    -L [%%u] : Format 7 region of interest (roi) left         %u\n\
              position. This option is only used if video\n\
              mode is format 7.\n\
\n\
    -T [%%u] : Format 7 region of interest (roi) top          %u\n\
              position. This option is only used if video\n\
              mode is format 7.\n\
\n\
    -W [%%u] : Format 7 region of interest (roi) width.       %u\n\
              Is set to zero, use the maximum width. This\n\
              option is only used if video mode is format 7.\n\
\n\
    -H [%%u] : Format 7 region of interest (roi) height.      %u\n\
              Is set to zero, use the maximum height. This\n\
              option is only used if video mode is format 7.\n\
\n\
    -c [%%u] : Active camera identifier.                      %u\n\
              Zero is for the first camera found on the bus.\n\
\n\
    -m      : Flag to active multi camera acquisition.       \n\
              You need at least two cameras connected on \n\
              the bus.\n\
\n\
    -n [%%u] : Number of frames to acquire.                   %u\n\
\n\
    -i      : Flag to print camera informations.\n\
\n\
    -s      : Print camera settings capabilities such as \n\
              video mode and framerates available and exit.\n\
\n\
    -d      : Flag to turn off image display.\n\
\n\
    -b [%%u] : Ring buffer size used during capture           %u\n\
\n\
    -p [%%u] : Pan control value used to control single or    %u\n\
              multiple image transmission from stereo vision \n\
              cameras by setting the PAN register 0x884.\n\
\n\
    -o [%%s] : Filename for image saving.                     \n\
              Example: -o %s\n\
              The first %%d is for the camera id. The second\n\
              %%04d is for the image numbering. If color images \n\
              are acquired the format of the .ppm file is PNM P6.\n\
              If grey level images are acquired, the format of \n\
              the .ppm file is PNM P5.\n\
\n\
    -r      : Reset the bus attached to the first camera found.\n\
              Bus reset may help to make firewire working if the\n\
              program was not properly stopped by a CTRL-C.\n\
\n\
    -h      : Print this help.\n\
\n",
	  name, name, name, name, name, name, name, name, name,
	  roi_left, roi_top, roi_width, roi_height,
	  camera, nframes, ringbuffersize, panControl, opath.c_str());

}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param multi : Multi camera framegrabbing activation.
  \param camera : Active camera identifier.
  \param nframes : Number of frames to acquire.

  \param verbose_info : Camera informations printing.
  \param verbose_settings : Camera settings printing.

  \param videomode_is_set : New video mode setting.
  \param videomode : Video mode setting.

  \param framerate_is_set : New framerate setting.
  \param framerate : Framerate setting.

  \param colorcoding_is_set : New color coding setting.
  \param colorcoding : Color coding setting (usefull only for format 7).

  \param ringbuffersize_is_set : New ring buffer size.
  \param ringbuffersize : Ring buffer size used during capture.

  \param display : Display activation.
  \param save : Image saving activation.
  \param opath : Image filename when saving.

  \param roi_left, roi_top, roi_width, roi_height : Region of interest in
  format 7.

  \param reset : If "true", reset the bus attached to the first
  camera found. Bus reset may help to make firewire working if the
  program was not properly stopped by a CTRL-C.

  \param panControl : Value used to set the pan control register.

  \param panControl_is_set : Indicates if the pan contraol register
  has to be set.
  
*/
void read_options(int argc, const char **argv, bool &multi, unsigned int &camera,
		  unsigned int &nframes, bool &verbose_info,
		  bool &verbose_settings,
		  bool &videomode_is_set,
		  vp1394TwoGrabber::vp1394TwoVideoModeType &videomode,
		  bool &framerate_is_set,
		  vp1394TwoGrabber::vp1394TwoFramerateType &framerate,
		  bool &colorcoding_is_set,
		  vp1394TwoGrabber::vp1394TwoColorCodingType &colorcoding,
		  bool &ringbuffersize_is_set,
		  unsigned int &ringbuffersize,
		  bool &display, bool &save, std::string &opath,
		  unsigned int &roi_left, unsigned int &roi_top,
		  unsigned int &roi_width, unsigned int &roi_height,
		  bool &reset,
		  unsigned int &panControl, bool & panControl_is_set)
{
  /*
   * Lecture des options.
   */
  const char *optarg;
  int	c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {
    switch (c) {
    case 'c':
      camera = atoi(optarg); break;
    case 'd':
      display = false; break;
    case 'f':
      framerate_is_set = true;
      framerate = (vp1394TwoGrabber::vp1394TwoFramerateType) atoi(optarg);
      break;
    case 'g':
      colorcoding_is_set = true;
      colorcoding = (vp1394TwoGrabber::vp1394TwoColorCodingType) atoi(optarg);
      break;
    case 'H':
      roi_height = (unsigned int) atoi(optarg); break;
    case 'i':
      verbose_info = true; break;
    case 'L':
      roi_left = (unsigned int) atoi(optarg); break;
    case 'm':
      multi = true; break;
    case 'n':
      nframes = atoi(optarg); break;
    case 'o':
      save = true;
      opath = optarg; break;
    case 'b':
      ringbuffersize_is_set = true;
      ringbuffersize = (unsigned int) atoi(optarg); break;
    case 'p':
      panControl = (unsigned int) atoi(optarg);
      panControl_is_set = true;
      break;
    case 'r':
      reset = true; break;
    case 's':
      verbose_settings = true; break;
    case 'T':
      roi_top = (unsigned int) atoi(optarg); break;
    case 'v':
      videomode_is_set = true;
      videomode = (vp1394TwoGrabber::vp1394TwoVideoModeType) atoi(optarg);
      break;
    case 'W':
      roi_width = (unsigned int) atoi(optarg); break;
    case 'h':
    case '?':
      usage(argv[0], NULL, camera, nframes, opath,
	    roi_left, roi_top, roi_width, roi_height, ringbuffersize, 
            panControl);
      exit(0);
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, camera, nframes, opath,
	  roi_left, roi_top, roi_width, roi_height, ringbuffersize, panControl);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    exit(-1);
  }
}

/*!
  \example grab1394Two.cpp

  Example of framegrabbing using vp1394TwoGrabber class.

  Grab images from a firewire camera using vp1394TwoGrabber, an interface for
  the libdc1394-2.0.0 driver. Display these images using X11 or GTK.

*/
int
main(int argc, const char ** argv)
{
  try  {
    unsigned int camera = 0;
    bool multi = false;
    bool verbose_info = false;
    bool verbose_settings = false;
    bool display = true;
    unsigned int nframes = 50;
    unsigned int offset;
    bool videomode_is_set = false;
    vp1394TwoGrabber::vp1394TwoVideoModeType videomode;
    bool framerate_is_set = false;
    vp1394TwoGrabber::vp1394TwoFramerateType framerate;
    bool colorcoding_is_set = false;
    vp1394TwoGrabber::vp1394TwoColorCodingType colorcoding;
    bool ringbuffersize_is_set = false;
    unsigned int ringbuffersize = 4;
    bool save = false;
    bool reset = false;
    unsigned int panControl = 0;
    bool panControl_is_set = false;

    // Format 7 roi
    unsigned int roi_left=0, roi_top=0, roi_width=0, roi_height=0;

    // Default output path for image saving
    std::string opath = "/tmp/I%d-%04d.ppm";

    read_options(argc, argv, multi, camera, nframes,
		 verbose_info, verbose_settings,
		 videomode_is_set, videomode,
		 framerate_is_set, framerate,
		 colorcoding_is_set, colorcoding,
		 ringbuffersize_is_set, ringbuffersize,
		 display, save, opath,
		 roi_left, roi_top, roi_width, roi_height, reset,
		 panControl, panControl_is_set);

    // Create a grabber
    vp1394TwoGrabber g(reset);

    if (reset) {
      // The experience shows that for some Marlin cameras (F131B and
      // F033C) a tempo of 1s is requested after a bus reset.
      vpTime::wait(1000); // Wait 1000 ms
    }
    
    // Number of cameras connected on the bus
    unsigned int ncameras = 0;
    g.getNumCameras(ncameras);

    std::cout << "Number of cameras on the bus: " << ncameras << std::endl;

    // Check the consistancy of the options
    if (multi) {
      // ckeck if two cameras are connected
      if (ncameras < 2) {
	std::cout << "You have only " << ncameras << " camera connected on the bus." << std::endl;
	std::cout << "It is not possible to active multi-camera acquisition." << std::endl;
	std::cout << "Disable -m command line option, or connect an other " << std::endl;
	std::cout << "cameras on the bus." << std::endl;
	g.close();
	return(0);
      }
    }
    if (camera >= ncameras) {
      std::cout << "You have only " << ncameras;
      std::cout << " camera connected on the bus." << std::endl;
      std::cout << "It is not possible to select camera " << camera << std::endl;
      std::cout << "Check your -c <camera> command line option." << std::endl;
      g.close();
      return(0);
    }

    if (multi) {
      camera = 0; // to over write a bad option usage
    }
    else {
      ncameras = 1; // acquisition from only one camera
    }
    // Offset is used to set the correspondancy between and image and the
    // camera. For example, images comming from camera (i+offset) are available
    // in I[i]
    offset = camera;

    // Display information for each camera
    if (verbose_info || verbose_settings) {
      for (unsigned int i=0; i < ncameras; i ++) {

	g.setCamera(i+offset);

	if (verbose_info)
	  g.printCameraInfo();

	if (verbose_settings) {
	  vp1394TwoGrabber::vp1394TwoVideoModeType curmode;
	  vp1394TwoGrabber::vp1394TwoFramerateType curfps;
	  vp1394TwoGrabber::vp1394TwoColorCodingType curcoding;
	  vpList<vp1394TwoGrabber::vp1394TwoVideoModeType> lmode;
	  vpList<vp1394TwoGrabber::vp1394TwoFramerateType> lfps;
	  vpList<vp1394TwoGrabber::vp1394TwoColorCodingType> lcoding;
	  uint64_t guid;

	  g.getVideoMode(curmode);
	  g.getFramerate(curfps);
	  g.getColorCoding(curcoding);
	  g.getVideoModeSupported(lmode);
	  g.getGuid(guid);

	  std::cout << "----------------------------------------------------------"
	       << std::endl
	       << "---- Video modes and framerates supported by camera "
	       << i+offset << " ----" << std::endl
		    << "---- with guid 0x" << std::hex << guid << "                       ----" << std::endl
	       << "---- * is for the current settings                    ----"
	       << std::endl
	       << "---- between ( ) you have the corresponding option    ----"
	       << std::endl
	       << "---- to use.                                          ----"
	       << std::endl
	       << "----------------------------------------------------------"
	       << std::endl;
	  lmode.front();
	  while (! lmode.outside() ) {

	    // Parse the list of supported modes
	    vp1394TwoGrabber::vp1394TwoVideoModeType supmode = lmode.value();
	    if (curmode == supmode)
	      std::cout << " * " << vp1394TwoGrabber::videoMode2string(supmode)
		   << " (-v " << supmode << ")" << std::endl;
	    else
	      std::cout << "   " << vp1394TwoGrabber::videoMode2string(supmode)
		   << " (-v " << supmode << ")" << std::endl;

	    if (g.isVideoModeFormat7(supmode)){
	      // Format 7 video mode; no framerate setting, but color
	      // coding setting
	      lcoding.kill();
	      g.getColorCodingSupported(supmode, lcoding);
	      lcoding.front();
	      while (! lcoding.outside() ) {
		vp1394TwoGrabber::vp1394TwoColorCodingType supcoding;
		supcoding = lcoding.value();
		if ( (curmode == supmode) && (supcoding == curcoding) )
		  std::cout << "    * "
			    << vp1394TwoGrabber::colorCoding2string(supcoding)
			    << " (-g " << supcoding << ")" << std::endl;
		else
		  std::cout << "      "
			    << vp1394TwoGrabber::colorCoding2string(supcoding)
			    << " (-g " << supcoding << ")" << std::endl;
		lcoding.next();
	      }
	    }
	    else {

	      // Parse the list of supported framerates for a supported mode
	      lfps.kill();
	      g.getFramerateSupported(supmode, lfps);
	      lfps.front();
	      while (! lfps.outside() ) {
		vp1394TwoGrabber::vp1394TwoFramerateType supfps = lfps.value();
		if ( (curmode == supmode) && (supfps == curfps) )
		  std::cout << "    * "
			    << vp1394TwoGrabber::framerate2string(supfps)
			    << " (-f " << supfps << ")" << std::endl;
		else
		  std::cout << "      "
			    << vp1394TwoGrabber::framerate2string(supfps)
			    << " (-f " << supfps << ")" << std::endl;
		lfps.next();
	      }
	    }

	    lmode.next();
	  }
	  std::cout << "----------------------------------------------------------"
	       << std::endl;

	}
      }
      return 0;
    }

    // If requested set the PAN register 0x884 to control single or
    // multiple image transmission from stereo vision cameras.
    if (panControl_is_set) {
      g.setPanControl(panControl);
    }

    // If required modify camera settings
    if (videomode_is_set) {
      g.setCamera(camera);
      g.setVideoMode(videomode);
    }
    else {
      // get The actual video mode
      g.setCamera(camera);
      g.getVideoMode(videomode);
    }
    if (framerate_is_set) {
      g.setCamera(camera);
      g.setFramerate(framerate);
    }
    if (colorcoding_is_set) {
      g.setCamera(camera);
      g.setColorCoding(colorcoding);
    }
    if (ringbuffersize_is_set) {
      g.setRingBufferSize(ringbuffersize);
    }

    // In format 7 set roi to the hole image
    if (g.isVideoModeFormat7(videomode))
      g.setFormat7ROI(roi_left, roi_top, roi_width, roi_height);

    // Array to know if color images or grey level images are acquired
    bool *grab_color = new bool [ncameras];

#ifdef VISP_HAVE_X11
    // allocate a display for each camera to consider
    vpDisplayX *d = NULL;
    if (display)
      d = new vpDisplayX [ncameras];
#endif

    // allocate an Grey and color image for each camera to consider
    vpImage<vpRGBa> *Ic        = new vpImage<vpRGBa> [ncameras];
    vpImage<unsigned char> *Ig = new vpImage<unsigned char> [ncameras];

    // Do a first acquisition to initialise the display
    for (unsigned int i=0; i < ncameras; i ++) {
      // Set the active camera on the bus
      g.setCamera(i+offset);
      // Ask each camera to know if color images or grey level images are
      // acquired
      grab_color[i] = g.isColor();
      // Acquire the first image
      if (grab_color[i]) {
	g.acquire(Ic[i]);
	std::cout << "Image size for camera " << i+offset << " : width: "
		  << Ic[i].getWidth() << " height: " << Ic[i].getHeight()
		  << std::endl;

#ifdef VISP_HAVE_X11
	if (display) {
	  // Initialise the display
	  char title[100];
	  sprintf(title, "Images captured by camera %u", i+offset);
	  d[i].init(Ic[i], 100+i*50, 100+i*50, title) ;
	  vpDisplay::display(Ic[i]);
	  vpDisplay::flush(Ic[i]);
	}
#endif
      }
      else {
	g.acquire(Ig[i]);
	std::cout << "Image size for camera " << i+offset << " : width: "
		  << Ig[i].getWidth() << " height: " << Ig[i].getHeight()
		  << std::endl;

#ifdef VISP_HAVE_X11
	if (display) {
	  // Initialise the display
	  char title[100];
	  sprintf(title, "Images captured by camera %u", i+offset);
	  d[i].init(Ig[i], 100+i*50, 100+i*50, title) ;
	  vpDisplay::display(Ig[i]);
	  vpDisplay::flush(Ig[i]);
	}
#endif

      }
    }

    // Main loop for single or multi-camera acquisition and display
    std::cout << "Capture in process..." << std::endl;

    double tbegin=0, tend=0, tloop=0, ttotal=0;

    ttotal = 0;
    tbegin = vpTime::measureTimeMs();
    for (unsigned int i = 0; i < nframes; i++) {
      for (unsigned int c = 0; c < ncameras; c++) {
	// Set the active camera on the bus
	g.setCamera(c+offset);
	// Acquire an image
	if (grab_color[c]) {
	  g.acquire(Ic[c]);
#ifdef VISP_HAVE_X11
	  if (display) {
	    // Display the last image acquired
	    vpDisplay::display(Ic[c]);
	    vpDisplay::flush(Ic[c]);
	  }
#endif
	}
	else {
	  g.acquire(Ig[c]);
#ifdef VISP_HAVE_X11
	  if (display) {
	    // Display the last image acquired
	    vpDisplay::display(Ig[c]);
	    vpDisplay::flush(Ig[c]);
	  }
#endif

	}
	if (save) {
	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), c+offset, i);
	  std::string filename(buf);
	  std::cout << "Write: " << filename << std::endl;
	  if (grab_color[c]) {
	    vpImageIo::writePPM(Ic[c], filename);
	  }
	  else {
	    vpImageIo::writePGM(Ig[c], filename);
	  }
	}
      }
      tend = vpTime::measureTimeMs();
      tloop = tend - tbegin;
      tbegin = tend;
      std::cout << "loop time: " << tloop << " ms" << std::endl;
      ttotal += tloop;
    }

    std::cout << "Mean loop time: " << ttotal / nframes << " ms" << std::endl;
    std::cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << std::endl;

    // Release the framegrabber
    g.close();

    // Free memory

    delete [] Ic;
    delete [] Ig;
    delete [] grab_color;

#ifdef VISP_HAVE_X11
    if (display)
      delete [] d;
#endif
  }
  catch (...) {
    vpCERROR << "Failure: exit" << std::endl;
  }

  std::cout << " the end" << std::endl;
}
#else
int
main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available...\n"
	  "You should install libdc1394-2 to use this example.") ;
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
