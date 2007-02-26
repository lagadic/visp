/****************************************************************************
 *
 * $Id: grabOSXcfoxColor.cpp,v 1.3 2007-02-26 17:39:42 fspindle Exp $
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
 * Acquire images using 1394 device with cfox (MAC OSX) and display it
 * using GTK or GTK.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \file grabOSXcfoxColor.cpp

  \brief Example of framegrabbing using vpOSXcfoxGrabber class.

*/

#if ( defined(APPLE) && defined(VISP_HAVE_CFOX) )

#if (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_X11) )

// List of allowed command line options
#define GETOPTARGS	"dh"

#include <visp/vpOSXcfoxGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayX11.h>

#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Acquire images using firewire 1394 framegrabber capabilities \n\
with cfox (under MAC OSX only) and display it using the GTK or \n\
X11 display.\n\
\n\
SYNOPSIS\n\
  %s [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h \n\
     Print the help.\n\
\n");

}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}

/*!
  \example grabOSXcfoxColor.cpp

  Example of framegrabbing using vpOSXcfoxGrabber class.

  Handle frame grabbing capabilities using 1394 device with cfox.  Only
  grabbing of grey level images is tested. Display these images using X11 or
  GTK.
*/
int
main(int argc, char ** argv)
{
  bool opt_display = true;

  // Read the command line options
  if (getOptions(argc, argv, opt_display) == false) {
    exit (-1);
  }

  // Declare an image, this is a gray level image (unsigned char). It
  // size is not defined yet. It will be defined when the image will
  // acquired the first time.
  vpImage<unsigned char> I ;

  // Creates the grabber
  vpOSXcfoxGrabber g;

  try{
    // Initialize the grabber
    g.setInput(vpOSXcfoxGrabber::DEFAULT_INPUT);
    g.setScale(vpOSXcfoxGrabber::DEFAULT_SCALE);
    g.setFramerate(vpOSXcfoxGrabber::framerate_30fps);
    // Open the framegrabber with the specified settings
    g.open(I) ;
    // Acquire an image
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot acquire an image...") ;
    exit(-1);
  }

  cout << "Image size: width : " << I.getWidth() <<  " height: "
       << I.getHeight() << endl;

  // We open a window using either GTK or X11.
  // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_X11
  vpDisplayX display;
#endif

  if (opt_display) {
    try{
      display.init(I, 100, 100, "Cfox Framegrabber") ;
      // Display the image
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

  // Acquisition loop
  long cpt = 1;
  while(cpt ++ < 100)
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
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
 }

  g.close();

}
#else
int
main()
{
  vpTRACE("GTK or X11 are not activated...") ;
}
#endif
#else
int
main()
{
  vpTRACE("Cfox OSX frame grabber drivers are not available...") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
