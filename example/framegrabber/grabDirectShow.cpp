/****************************************************************************
 *
 * $Id: grabDirectShow.cpp,v 1.6 2007-04-19 09:01:05 asaunier Exp $
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
 * Acquire images using DirectShow (under Windows only) and display it
 * using GTK or GDI.
 *
 * Authors:
 * Bruno Renier
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \file grabDirectShow.cpp

  \brief Example of framegrabbing using vpDirectShowGrabber class.

*/

#if defined (VISP_HAVE_DIRECTSHOW)

#include <visp/vpDirectShowGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpParseArgv.h>
#include <visp/vpTime.h>

// List of allowed command line options
#define GETOPTARGS	"dhn:o:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param nframes : Number of frames to acquire.
  \param opath : Image filename when saving.

*/
void usage(char *name, char *badparam, unsigned &nframes, string &opath)
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
bool getOptions(int argc, char **argv, bool &display,
		unsigned &nframes, bool &save, string &opath)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'n':
      nframes = atoi(optarg); break;
    case 'o':
      save = true;
      opath = optarg; break;
    case 'h': usage(argv[0], NULL, nframes, opath); return false; break;

    default:
      usage(argv[0], optarg, nframes, opath);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, nframes, opath);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}


/*!
  \example grabDirectShow.cpp

  Example of framegrabbing using vpDirectShowGrabber class.

  Grab grey level images using DirectShow frame grabbing capabilities. Display the
  images using the GTK or GDI display.
*/
int
main(int argc, char ** argv)
{
	try 
	{
		bool opt_display = true;
		unsigned nframes = 50;
		bool save = false;

		// Declare an image. It size is not defined yet. It will be defined when the image will
		// acquired the first time.
		#ifdef GRAB_COLOR
		vpImage<vpRGBa> I; // This is a color image (in RGBa format)
		#else
		vpImage<unsigned char> I; // This is a B&W image
		#endif

		// Set default output image name for saving
		#ifdef GRAB_COLOR
		string opath = "C:/temp/I%04d.ppm"; // Color images will be saved in PGM P6 format
		#else
		string opath = "C:/temp/I%04d.pgm"; // B&W images will be saved in PGM P5 format
		#endif

		// Read the command line options
		if (getOptions(argc, argv, opt_display, nframes, save, opath) == false) {
		exit (-1);
		}
		vpDirectShowGrabber* grabber ;
		try {
		    // Create the grabber
		    grabber = new vpDirectShowGrabber();

		    // Initialize the grabber
		    grabber->open(I);

		    // Acquire an RGBa image
		    grabber->acquire(I);
		}
		catch(...)
		{
		    vpERROR_TRACE("Cannot acquire an image...") ;
		    delete grabber;
		    exit(-1);
		}

		cout << "Image size: width : " << I.getWidth() <<  " height: "
		   << I.getHeight() << endl;

		// Creates a display
		#ifdef VISP_HAVE_GTK
		vpDisplayGTK display;
		#else
		vpDisplayGDI display;
		#endif

		if (opt_display) {
		display.init(I,100,100,"DirectShow Framegrabber");
		}

		try {
		double tbegin=0, tend=0, tloop=0, ttotal=0;

		ttotal = 0;
		tbegin = vpTime::measureTimeMs();
		// Loop for image acquisition and display
		for (unsigned i = 0; i < nframes; i++) {
		  //Acquires an RGBa image
		  grabber->acquire(I);

		  if (opt_display) {
		//Displays the grabbed rgba image
		vpDisplay::display(I);
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
		  tend = vpTime::measureTimeMs();
		  tloop = tend - tbegin;
		  tbegin = tend;
		  cout << "loop time: " << tloop << " ms" << endl;
		  ttotal += tloop;
		}
		cout << "Mean loop time: " << ttotal / nframes << " ms" << endl;
		cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << endl;

		// Release the framegrabber
		delete grabber;

		}
		catch(...)
		{
		  vpCERROR << "Failure: exit" << std::endl;
		  delete grabber;
		  return(-1);
		}
	}
	catch(...)
	{
	  vpERROR_TRACE("Error caught");
	  delete grabber;
	  return(-1);
	}
}
#else
int
main()
{
  vpTRACE("DirectShow is not available...") ;
}
#endif


