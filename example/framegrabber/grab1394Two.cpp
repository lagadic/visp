/****************************************************************************
 *
 * $Id: grab1394Two.cpp,v 1.3 2007-02-02 10:50:59 fspindle Exp $
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
  \file grab1394Two.cpp

  \brief Example of framegrabbing using vp1394TwoGrabber class.

  \warning This class needs at least libdc1394-2.0.0-rc4 and
  libraw1394-1.1.0. These libraries are available from
  http://sourceforge.net/projects/libdc1394 and
  http://sourceforge.net/projects/libraw1394 .

  vp1394TwoGrabber was tested with MF-033C and F-131B Marlin cameras.
*/

#include <iostream>
#include <sstream>

using namespace std;

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

//#define GRAB_COLOR


// List of allowed command line options
#define GETOPTARGS	"c:df:g:hmn:io:sv:?"


#define DUAL_ACQ


/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param camera : Active camera identifier.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.

*/
void usage(char *name, char *badparam, unsigned camera, unsigned &nframes,
	   string &opath)
{
  if (badparam)
    fprintf(stderr, "\nERREUR: Mauvais paramètre [%s]\n", badparam);

  fprintf(stderr, "\n\
SYNOPTIQUE\n\
    %s [-v <video mode>] [-f <framerate>] \n\
    [-g <color coding>] [-c <camera id>] [-m] [-n <frames>] \n\
    [-i] [-s] [-d] [-o <filename>] [-?]\n\
\n\
DESCRIPTION\n\
    Test for firewire camera image acquisition.\n\
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
    -o [%%s] : Filename for image saving.                     \n\
              Example: -o %s\n\
              The first %%d is for the camera id, %%04d\n\
              is for the image numbering.\n\
\n",
	  name, camera, nframes, opath.c_str());

  exit(0);
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

  \param display : Display activation.
  \param save : Image saving activation.
  \param opath : Image filename when saving.

*/
void read_options(int argc, char **argv, bool &multi, unsigned &camera,
		  unsigned &nframes, bool &verbose_info,
		  bool &verbose_settings,
		  bool &videomode_is_set,
		  vp1394TwoGrabber::vp1394TwoVideoMode &videomode,
		  bool &framerate_is_set,
		  vp1394TwoGrabber::vp1394TwoFramerate &framerate,
		  bool &colorcoding_is_set,
		  vp1394TwoGrabber::vp1394TwoColorCoding &colorcoding,
		  bool &display, bool &save, string &opath)
{
  int	c;
  /*
   * Lecture des options.
   */

  while ((c = getopt(argc, argv, GETOPTARGS)) != EOF)
    switch (c) {
    case 'c':
      camera = atoi(optarg); break;
    case 'd':
      display = false; break;
    case 'f':
      framerate_is_set = true;
      framerate = (vp1394TwoGrabber::vp1394TwoFramerate) atoi(optarg); break;
    case 'g':
      colorcoding_is_set = true;
      colorcoding = (vp1394TwoGrabber::vp1394TwoColorCoding) atoi(optarg); break;
    case 'i':
      verbose_info = true; break;
    case 'm':
      multi = true; break;
    case 'n':
      nframes = atoi(optarg); break;
    case 'o':
      save = true;
      opath = optarg; break;
    case 's':
      verbose_settings = true; break;
    case 'v':
      videomode_is_set = true;
      videomode = (vp1394TwoGrabber::vp1394TwoVideoMode) atoi(optarg); break;
    default:
      usage(argv[0], NULL, camera, nframes, opath);
      break;
    }

  /* expect no args left over */
  if (argv[optind]) {
    usage(argv[0], argv[optind], camera, nframes, opath);
  }
}

/*!
  \example grab1394Two.cpp

  Example of framegrabbing using vp1394TwoGrabber class.

  Grab images from a firewire camera using vp1394TwoGrabber, an interface for
  the libdc1394-2.0.0 driver. Display these images using X11 or GTK.

*/
int
main(int argc, char ** argv)
{
  try  {
    unsigned camera = 0;
    bool multi = false;
    bool verbose_info = false;
    bool verbose_settings = false;
    bool display = true;
    unsigned nframes = 50;
    unsigned offset;
    bool videomode_is_set = false;
    vp1394TwoGrabber::vp1394TwoVideoMode videomode;
    bool framerate_is_set = false;
    vp1394TwoGrabber::vp1394TwoFramerate framerate;
    bool colorcoding_is_set = false;
    vp1394TwoGrabber::vp1394TwoColorCoding colorcoding;
    bool save = false;

#ifdef GRAB_COLOR
    vpImage<vpRGBa> *I;
    string opath = "/tmp/I%d-%04d.ppm";
#else
    vpImage<unsigned char> *I;
    string opath = "/tmp/I%d-%04d.pgm";
#endif
    vpDisplayX *d;
    vp1394TwoGrabber g ;

    read_options(argc, argv, multi, camera, nframes,
		 verbose_info, verbose_settings,
		 videomode_is_set, videomode,
		 framerate_is_set, framerate,
		 colorcoding_is_set, colorcoding,
		 display, save, opath);

    // Number of cameras connected on the bus
    unsigned ncameras = 0;
    g.getNumCameras(ncameras);

    cout << "Number of cameras on the bus: " << ncameras << endl;

    // Check the consistancy of the options
    if (multi) {
      // ckeck if two cameras are connected
      if (ncameras != 2) {
	cout << "You have only " << ncameras << " camera connected on the bus." << endl;
	cout << "It is not possible to active multi-camera acquisition." << endl;
	cout << "Disable -m command line option, or connect an other " << endl;
	cout << "cameras on the bus." << endl;
	g.close();
	return(0);
      }
    }
    if (camera >= ncameras) {
      cout << "You have only " << ncameras;
      cout << " camera connected on the bus." << endl;
      cout << "It is not possible to select camera " << camera << endl;
      cout << "Check your -c <camera> command line option." << endl;
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
    // camera. For example, images comming from camera (i+offset) are avalaible
    // in I[i]
    offset = camera;

    // allocate an image and display for each camera to consider
#ifdef GRAB_COLOR
    I = new vpImage<vpRGBa> [ncameras];
#else
    I = new vpImage<unsigned char> [ncameras];
#endif
    if (display)
      d = new vpDisplayX [ncameras];

    // Display information for each camera
    if (verbose_info || verbose_settings) {
      for (unsigned i=0; i < ncameras; i ++) {

	g.setCamera(i+offset);

	if (verbose_info)
	  g.printCameraInfo();

	if (verbose_settings) {
	  vp1394TwoGrabber::vp1394TwoVideoMode curmode;
	  vp1394TwoGrabber::vp1394TwoFramerate curfps;
	  vp1394TwoGrabber::vp1394TwoColorCoding curcoding;
	  vpList<vp1394TwoGrabber::vp1394TwoVideoMode> lmode;
	  vpList<vp1394TwoGrabber::vp1394TwoFramerate> lfps;
	  vpList<vp1394TwoGrabber::vp1394TwoColorCoding> lcoding;

	  g.getVideoMode(curmode);
	  g.getFramerate(curfps);
	  g.getColorCoding(curcoding);
	  g.getVideoModeSupported(lmode);

	  cout << "----------------------------------------------------------"
	       << endl
	       << "---- Video modes and framerates supported by camera "
	       << camera << " ----" << endl
	       << "---- * is for the current settings                    ----"
	       << endl
	       << "---- between ( ) you have the corresponding option    ----"
	       << endl
	       << "---- to use.                                          ----"
	       << endl
	       << "----------------------------------------------------------"
	       << endl;
	  lmode.front();
	  while (! lmode.outside() ) {

	    // Parse the list of supported modes
	    vp1394TwoGrabber::vp1394TwoVideoMode supmode = lmode.value();
	    if (curmode == supmode)
	      cout << " * " << vp1394TwoGrabber::videoMode2string(supmode)
		   << " (-v " << supmode << ")" << endl;
	    else
	      cout << "   " << vp1394TwoGrabber::videoMode2string(supmode)
		   << " (-v " << supmode << ")" << endl;

	    if (g.isVideoModeFormat7(supmode)){
	      // Format 7 video mode; no framerate setting, but color coding setting
	      lcoding.kill();
	      g.getColorCodingSupported(supmode, lcoding);
	      lcoding.front();
	      while (! lcoding.outside() ) {
		vp1394TwoGrabber::vp1394TwoColorCoding supcoding = lcoding.value();
		if ( (curmode == supmode) && (supcoding == curcoding) )
		  cout << "    * " << vp1394TwoGrabber::colorCoding2string(supcoding)
		       << " (-g " << supcoding << ")" << endl;
		else
		  cout << "      " << vp1394TwoGrabber::colorCoding2string(supcoding)
		       << " (-g " << supcoding << ")" << endl;
		lcoding.next();
	      }
	    }
	    else {

	      // Parse the list of supported framerates for a supported mode
	      lfps.kill();
	      g.getFramerateSupported(supmode, lfps);
	      lfps.front();
	      while (! lfps.outside() ) {
		vp1394TwoGrabber::vp1394TwoFramerate supfps = lfps.value();
		if ( (curmode == supmode) && (supfps == curfps) )
		  cout << "    * " << vp1394TwoGrabber::framerate2string(supfps)
		       << " (-f " << supfps << ")" << endl;
		else
		  cout << "      " << vp1394TwoGrabber::framerate2string(supfps)
		       << " (-f " << supfps << ")" << endl;
		lfps.next();
	      }
	    }

	    lmode.next();
	  }
	  cout << "----------------------------------------------------------"
	       << endl;

	}
      }
      return 0;
    }

    // If required modify camera settings
    if (videomode_is_set) {
      g.setCamera(camera);
      g.setVideoMode(videomode);
    }
    else {
      // get The actual video mode
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

    // In format 7 set roi to the hole image
    if (g.isVideoModeFormat7(videomode))
      g.setFormat7ROI();

    // Do a first acquisition to initialise the display
    for (unsigned i=0; i < ncameras; i ++) {
      // Set the active camera on the bus
      g.setCamera(i+offset);
      // Acquire the first image
      g.acquire(I[i]);
      cout << "Image size for camera " << i+offset << " : "
	   << I[i].getCols() << " " << I[i].getRows() << endl;

      if (display) {
	// Initialise the display
	char title[100];
	sprintf(title, "Images captured by camera %u", i+offset);
	d[i].init(I[i], 100+i*50, 100+i*50, title) ;
      }
    }

    // Main loop for single or multi-camera acquisition and display
    cout << "Capture in process..." << endl;

    double tbegin=0, tend=0, tloop=0, ttotal=0;

    ttotal = 0;
    tbegin = vpTime::measureTimeMs();
    for (unsigned i = 0; i < nframes; i++) {
      for (unsigned c = 0; c < ncameras; c++) {
	// Set the active camera on the bus
	g.setCamera(c+offset);
	// Acquire an image
	g.acquire(I[c]);
	if (display) {
	  // Display the last image acquired
	  vpDisplay::display(I[c]);
	}
	if (save) {
	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), c+offset, i);
	  string filename(buf);
	  cout << "Write: " << filename << endl;
#ifdef GRAB_COLOR
	  vpImageIo::writePPM(I[c], filename);
#else
	  vpImageIo::writePGM(I[c], filename);
#endif
	}
      }
      tend = vpTime::measureTimeMs();
      tloop = tend - tbegin;
      tbegin = tend;
      cout << "loop time: " << tloop << " ms" << endl;
      ttotal += tloop;
    }

    cout << "Mean loop time: " << ttotal / nframes << " ms" << endl;
    cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << endl;

    // Release the framegrabber
    g.close();

    // Free memory
    delete [] I;
    if (display)
      delete [] d;

  }
  catch (...) {
    vpCERROR << "Failure: exit" << endl;
  }

  cout << " the end" << endl;
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
