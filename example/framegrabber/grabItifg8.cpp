/****************************************************************************
 *
 * $Id: grabItifg8.cpp,v 1.6 2007-02-26 17:39:42 fspindle Exp $
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
 * Acquire images from the itifg-8.x device.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file grabItifg8.cpp
  \brief Example of framegrabbing using vpItifg8Grabber class.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_ITIFG8

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <visp/vpItifg8Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>
#include <visp/vpRGBa.h>


// List of allowed command line options
#define GETOPTARGS	"b:c:df:hi:n:o:p:s:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param board : Board number
  \param fps : Framerate.
  \param input : Camera port number.
  \param scale : Subsampling factor.
  \param buffer : Number of buffers.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.
  \param conffile : Camera configuration file.

*/
void usage(char *name, char *badparam, unsigned board, float fps,
	   unsigned input, unsigned scale, unsigned buffer,
	   unsigned &nframes, string opath, string conffile)
{
  fprintf(stdout, "\n\
Grab grey level images using the itifg-8.x framegrabber device from\n\
Coreco Imaging. Display these images using X11 or GTK.\n\
\n\
SYNOPSIS\n\
  %s [-b <board=[0-7]>] [-c <conffile>]  \n\
   [-f <fps=0.01-100.0>] [-i <input=0|1|2|3>] [-s <scale=1|2|4>] \n\
   [-p <buffer=1-8>] [-n <nframes>] [-d] [-o <filename>] [-h]\n", name);

  if (badparam != NULL)
  fprintf(stdout, "\n\
ERROR:\n\
     Bad parameter: %s\n\
     Check program usage below.\n", badparam);


  fprintf(stdout, "\n\
OPTIONS:                                                  Default\n\
  -b <board>                                                \n\
     Board number [0-7]. Useful only if multiple boards \n\
     are connected to the computer.\n\
\n\
  -c <conffile>                  %s\n\
     Camera configuration file.\n\
\n\
  -f <fps>                                                  %f\n\
     Framerate in term of number of images per second.\n\
     Possible values are 25 (for 25Hz) or 50 (for 50) Hz) \n\
     for AM_STD COMP boards, or [0.01-100.0] for other \n\
     boards.\n\
\n\
  -i <input>                                                %u\n\
     Framegrabber active camera input. Values can be 0, 1,\n\
     2, or 3.\n\
\n\
  -s <scale>                                                %u\n\
     Framegrabber subsampling factor. \n\
     If 1, full resolution image acquisition.\n\
     If 2, half resolution acquisition. The \n\
     subsampling is achieved by the hardware.\n\
\n\
  -p <buffer>                                               %u\n\
     Number of buffers for the acquisition [1-8].\n\
\n\
  -n <nframes>                                              %u\n\
     Number of frames to acquire.\n\
\n\
  -o <filename> \n\
     Generic filename for image saving.                     \n\
     Example: -o %s \n\
     where %%04d is for the image numbering.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h \n\
     Print the help.\n\n",
	  conffile.c_str(), fps, input, scale, buffer, nframes, opath.c_str());

}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param board : Selected board number.
  \param fps : Framerate.
  \param input : Camera port number.
  \param scale : Subsampling factor.
  \param buffer : Number of buffers.
  \param display : Display activation.
  \param nframes : Number of frames to acquire.
  \param save : Image saving activation.
  \param opath : Image filename when saving.
  \param conffile : Camera configuration file.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, unsigned &board, float &fps,
		unsigned &input, unsigned &scale, unsigned &buffer,
		bool &display, unsigned &nframes, bool &save,
		string &opath, string &conffile)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'b': board = atoi(optarg); break;
    case 'c': conffile = optarg; break;
    case 'd': display = false; break;
    case 'f': fps = atof(optarg); break;
    case 'i': input = (unsigned) atoi(optarg); break;
    case 'n': nframes = atoi(optarg); break;
    case 'p': buffer = (unsigned) atoi(optarg); break;
    case 'o': save = true; opath = optarg; break;
    case 's': scale = (unsigned) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, board, fps, input, scale, buffer,
		    nframes, opath, conffile);
      return false; break;

    default:
      usage(argv[0], optarg, board, fps, input, scale, buffer,
	    nframes, opath, conffile);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], optarg, board, fps, input, scale, buffer,
	  nframes, opath, conffile);
    cerr << endl << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}

/*!
  \example grabItifg8.cpp

  Example of framegrabbing using vpItifg8Grabber class.

  Grab grey level images using vpItifg8Grabber, an interface for the itifg-8.x
  framegrabber driver from Coreco Imaging. Display these images using X11 or
  GTK.

*/
int
main(int argc, char ** argv)
{
  unsigned board = 0;
  float fps = 25.;
  unsigned nframes = 50;
  unsigned input = vpItifg8Grabber::DEFAULT_INPUT;
  unsigned scale = vpItifg8Grabber::DEFAULT_SCALE;
  unsigned buffer = 2;
  bool opt_display = true;
  bool save = false;
  string conffile = "/usr/share/itifg/conffiles/robot.cam";

  // Declare an image. It size is not defined yet. It will be defined when the
  // image will acquired the first time.
#ifdef GRAB_COLOR
  vpImage<vpRGBa> I; // This is a color image (in RGBa format)
#else
  vpImage<unsigned char> I; // This is a B&W image
#endif

  // Set default output image name for saving
#ifdef GRAB_COLOR
  string opath = "/tmp/I%04d.ppm"; // Color images will be saved in PGM P6 format
#else
  string opath = "/tmp/I%04d.pgm"; // B&W images will be saved in PGM P5 format
#endif

  // Read the command line options
  if (getOptions(argc, argv, board, fps, input, scale, buffer,
		 opt_display, nframes, save, opath, conffile) == false) {
    exit (-1);
  }

  // Declare a framegrabber to acquire images with the IC-comp framegrabber
  // card (Imaging Technology)
  vpItifg8Grabber g;
  unsigned nboards = g.getNumBoards();

  try{
    // Initialize the grabber board
    g.setBoard(board);
    g.setConfFile(conffile);
    g.setVerboseMode(false);
    g.setScale(scale);
    g.setInput(input);
    g.setBuffer(buffer); //
    unsigned module  = g.getModule();
    if (module == ICP_AMCMP) {
      if (fps == 25.) {
	g.setFramerate(vpItifg8Grabber::framerate_25fps);
	fps = 25.;
      }
      else {
	g.setFramerate(vpItifg8Grabber::framerate_50fps);
	fps = 50.;
      }
    }
    else
      g.setFramerate(fps);

    // Open the framegrabber with the specified settings
    g.open(I) ;
    // Acquire an image
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot acquire the image... ") ;
    exit(-1);
  }

  cout << "Image size: width : " << I.getWidth() <<  " height: "
       << I.getHeight() << endl;

  // We open a window using either X11 or GTK.
  // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#endif
  if (opt_display) {
    try{
      display.init(I, 100, 100,"Itifg-8.x Framegrabber...") ;
      // display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Cannot display the image... ") ;
      exit(-1);
    }
  }

  try {
    double tbegin=0, tend=0, tloop=0, ttotal=0;

    // Loop to acquire images
    for (unsigned i = 0; i < nframes; i++) {

	tbegin = vpTime::measureTimeMs();

	// Grab the image
	g.acquire(I) ;

	if (opt_display) {
	  // Display the image
	  vpDisplay::display(I) ;
	  // Flush the display
	  vpDisplay::flush(I) ;
	}

	if (save) {
	  char buf[FILENAME_MAX];
	  sprintf(buf, opath.c_str(), i);
	  string filename(buf);
	  cout << "Write: " << filename << endl;
#ifdef GRAB_COLOR
	  vpImageIo::writePPM(I, filename);
#else
	  vpImageIo::writePGM(I, filename);
#endif
	}
	vpTime::wait(tbegin, 1000./fps);
	tend = vpTime::measureTimeMs();
	tloop = tend - tbegin;
	tbegin = tend;
	cout << "loop time: " << tloop << " ms" << endl;
	ttotal += tloop;

      }
      cout << "Mean loop time: " << ttotal / nframes << " ms" << endl;
      cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << endl;
  }
  catch(...)  {
    vpCERROR << "Failure: exit" << endl;
  }
}
#else
int
main()
{
  vpTRACE("You do not have X11 or GTK display functionalities...") ;
}
#endif
#else
int
main()
{
  vpTRACE("Itifg-8.x frame grabber driver is not available") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
