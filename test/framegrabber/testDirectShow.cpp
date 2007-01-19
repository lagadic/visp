/****************************************************************************
 *
 * $Id: testDirectShow.cpp,v 1.5 2007-01-19 17:28:35 fspindle Exp $
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
  \example testDirectShow.cpp

  Test frame grabbing capabilities using DirectShow video device. Display the
  images using the GTK display.
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

 */
void usage(char *name, char *badparam, unsigned &nframes, string &opath)
{
  fprintf(stdout, "\n\
Acquire images using DirectShow (under Windows only) and display\n\
it using GTK or the windows GDI if GTK is not available.\n\
\n\
SYNOPSIS\n\
  %s [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -d \n\
     Turn off the display.\n\
\n\
  -n [%%u] : Number of frames to acquire.                   %u\n\
\n\
  -o [%%s] : Filename for image saving.                     %s\n\
            The %%d is for the image numbering.\n\
\n\
  -h \n\
     Print the help.\n\
\n", nframes, opath.c_str());

}
/*!

  Set the program options.

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


int
main(int argc, char ** argv)
{
  bool opt_display = true;
  unsigned nframes = 50;
  bool save = false;

  // Read the command line options
  if (getOptions(argc, argv, opt_display, nframes, save, opath) == false) {
    exit (-1);
  }

  // Declare an image, this is a color image (in RGBa format). It
  // size is not defined yet. It will be defined when the image will
  // acquired the first time.
#ifdef GRAB_COLOR
  vpImage<vpRGBa> I ;
  string opath = "C:/temp/I%04d.ppm";
#else
  vpImage<unsigned char> I ;
  string opath = "C:/temp/I%04d.pgm";
#endif

  // Create the grabber
  vpDirectShowGrabber grabber;

  try {
    // Initialize the grabber
    grabber.open(I);

    // Acquire an RGBa image
    grabber.acquire(I);
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot acquire an image...") ;
    exit(-1);
  }

  cout << "Image size: " << I.getCols() << "  " << I.getRows() <<endl  ;

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
      grabber.acquire(I);

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

    grabber.close();
  }
  catch(...)
  {
    vpCERROR << "Failure: exit" << endl;
  }
}
#else
int
main()
{
  vpTRACE("DirectShow is not available...") ;
}
#endif


