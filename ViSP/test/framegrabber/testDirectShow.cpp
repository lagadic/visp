/****************************************************************************
 *
 * $Id: testDirectShow.cpp,v 1.3 2006-08-25 07:58:54 brenier Exp $
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
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

 */
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Acquire images using DirectShow (under Windows only) and display\n\
it using GTK.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h                                     %s\n\
     Print the help.\n\
\n");

}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
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


int
main(int argc, char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  // Declare an image, this is a color image (in RGBa format). It
  // size is not defined yet. It will be defined when the image will
  // acquired the first time.
  vpImage<vpRGBa> I ;

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
  vpDisplayGTK display(I,100,100,"DirectShow Framegrabber");
#else
  vpDisplayGDI display(I,100,100,"DirectShow Framegrabber");
#endif
  

  try {
    // Loop for image acquisition and display
    for(int i=0 ; i<100 ; i++)
      {
	//Acquires an RGBa image
	grabber.acquire(I);
	
	//Displays the grabbed rgba image
	vpDisplay::display(I);
      }
    
  }
  catch(...)
    {
      exit(-1);
    }

  grabber.close();

}
#else
int
main()
{
  vpTRACE("DirectShow is not available...") ;
}
#endif


