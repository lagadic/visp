/****************************************************************************
 *
 * $Id: trackDot2.cpp,v 1.3 2007-01-31 09:59:14 asaunier Exp $
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
 * Example of dot tracking.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file trackDot2.cpp

  \brief Example of dot tracking on an image sequence using vpDot2.
*/

#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(WIN32))

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDot2.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

// List of allowed command line options
#define GETOPTARGS	"cdi:h"

//int gsl_warnings_off;
#define ADD_TEST 0

/*!
  \example trackDot2.cpp
  Example of dot tracking on an image sequence using vpDot2.
*/

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.

*/
void usage(char *name, char *badparam, string ipath)
{
  fprintf(stdout, "\n\
Test dot tracking using vpDot2 class.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"ViSP-images/mire-2/image.%%04d.pgm\"\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -c\n\
     Disable the mouse click. Usefull to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n",
	  ipath.c_str());

}
/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, string &ipath,
		bool &click_allowed, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'h': usage(argv[0], NULL, ipath); return false; break;

    default:
      usage(argv[0], optarg, ipath);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}


int
main(int argc, char ** argv)
{
  string env_ipath;
  string opt_ipath;
  string ipath;
  string dirname;
  string filename;
  bool opt_click_allowed = true;
  bool opt_display = true;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;


  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed,
		 opt_display) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (opt_ipath.empty()) {
    if (ipath != env_ipath) {
      cout << endl
	   << "WARNING: " << endl;
      cout << "  Since -i <visp image path=" << ipath << "> "
	   << "  is different from VISP_IMAGE_PATH=" << env_ipath << endl
	   << "  we skip the environment variable." << endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty()){
    usage(argv[0], NULL, ipath);
    cerr << endl
	 << "ERROR:" << endl;
    cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << endl
	 << "  environment variable to specify the location of the " << endl
	 << "  image path where test images are located." << endl << endl;
    exit(-1);
  }


  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  // Set the path location of the image sequence
  dirname = ipath +  vpIoTools::path("/ViSP-images/mire-2/");

  // Build the name of the image file
  unsigned iter = 1; // Image number
  std::ostringstream s;
  s.setf(ios::right, ios::adjustfield);
  s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
  filename = dirname + s.str();

  // Read the PGM image named "filename" on the disk, and put the
  // bitmap into the image structure I.  I is initialized to the
  // correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpCTRACE << "Load: " << filename << endl;

    vpImageIo::readPGM(I, filename) ;
  }
  catch(...)
    {
      // an exception is throwned if an exception from readPGM has been catched
      // here this will result in the end of the program
      // Note that another error message has been printed from readPGM
      // to give more information about the error
      cerr << endl
	   << "ERROR:" << endl;
      cerr << "  Cannot read " << filename << endl;
      cerr << "  Check your -i " << ipath << " option " << endl
	   << "  or VISP_INPUT_IMAGE_PATH environment variable."
	   << endl;
      exit(-1);
    }

  // We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined WIN32
  vpDisplayGDI display;
#endif

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100,"Display...") ;
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
    }
    catch(...)
      {
	// an exception is throwned if an exception from readPGM has been catched
	// here this will result in the end of the program
	// Note that another error message has been printed from readPGM
	// to give more information about the error

	vpERROR_TRACE("Error while displaying the image") ;
	exit(-1);
      }
  }
  // define the vpDot structure.

  // vpDot and vpDot2 correspond to two different algorithms designed to track
  // a dot. vpDot is based on recurse connex componants (all the pixels of the
  // dot are parsed), while vpDot2 is based on freeman chain code (only the
  // contour of the dot is parsed)

  vpDot2 d ;

  if (opt_display) {
    // by using setGraphics, we request to see the all the pixel of the dot
    // in green on the screen.
    // It uses the overlay image plane.
    // The default of this setting is that it is time consumming

    d.setGraphics(true) ;
  }
  else {

    d.setGraphics(false) ;
  }
  // we also request to compute the dot moment m00, m10, m01, m11, m20, m02
  d.setComputeMoments(true);

  // tracking is initalized
  // if no other parameters are given to the iniTracking(..) method
  // a right mouse click on the dot is expected
  // dot location can also be specified explicitely in the initTracking
  // method  : d.initTracking(I,u,v)  where u is the column index and v is
  // the row index

  try{
    if (opt_display && opt_click_allowed) {
      d.initTracking(I) ;
    }
    else {
      d.initTracking(I, 160, 212) ;
    }
    if (1) {
      cout << "COG: " << endl;
      cout << d.get_u() << " " << d.get_v()
	   << " - "
	   << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
      cout << "Size:" << endl;
      cout << "w: " << d.getWidth() << " h: " << d.getHeight() << endl;
      cout << "Moments:" << endl;
      cout << "m00: " << d.m00 << endl;
      cout << "m11: " << d.m11 << endl;
      cout << "m02: " << d.m02 << endl;
      cout << "m20: " << d.m20 << endl;
      cout << "m10: " << d.m10 << endl;
      cout << "m01: " << d.m01 << endl;
    }
  }
  catch(...)
    {
      vpERROR_TRACE("Cannot initialise the tracking... ") ;
      exit(-1);
    }

  try {
    while (iter < 1500)
      {
	// set the new image name
	s.str("");
	s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
	filename = dirname + s.str();
	// read the image
	vpImageIo::readPGM(I, filename);

	// track the dot and returns its coordinates in the image
	// results are given in float since many many are usually considered
	//
	// an expcetion is thrown by the track method if
	//  - dot is lost
	//  - the number of pixel is too small
	//  - too many pixels are detected (this is usual when a "big" specularity
	//    occurs. The threshold can be modified using the
	//    setNbMaxPoint(int) method

	d.track(I) ;

	cout << "COG: " << endl;
	cout << d.get_u() << " " << d.get_v()
	     << " - "
	     << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
	cout << "Size:" << endl;
	cout << "w: " << d.getWidth() << " h: " << d.getHeight() << endl;
	cout << "Moments:" << endl;
	cout << "m00: " << d.m00 << endl;
	cout << "m11: " << d.m11 << endl;
	cout << "m02: " << d.m02 << endl;
	cout << "m20: " << d.m20 << endl;
	cout << "m10: " << d.m10 << endl;
	cout << "m01: " << d.m01 << endl;
	if (opt_display) {
	  // Display the image
	  vpDisplay::display(I) ;

	  // display a red cross (size 10) in the image at the dot center
	  // of gravity location
	  //
	  // WARNING
	  // in the vpDisplay class member's when pixel coordinates
	  // are considered the first element is the row index and the second
	  // is the column index:
	  //   vpDisplay::displayCross(Image, row index, column index, size, color)
	  //   therefore u and v are inverted wrt to the vpDot specification
	  // Alternatively, to avoid this problem another set of member have
	  // been defined in the vpDisplay class.
	  // If the method name is postfixe with _uv the specification is :
	  //   vpDisplay::displayCross_uv(Image, column index, row index, size, color)

	  vpDisplay::displayCross_uv(I,(int)d.get_u(), (int)d.get_v(),10,vpColor::green) ;
	  // flush the X11 buffer

	  vpDisplay::flush(I) ;
	}
	iter ++;
      }
  }
  catch (...) {
    cerr << "Error during the tracking..." << endl;
    cerr << "The progam was stopped." << endl;
    exit(-1);
  }
  if (opt_display && opt_click_allowed) {
    cout << "\nA click to exit..." << endl;
    // Wait for a blocking mouse click
    vpDisplay::getClick(I) ;
  }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11, GTK or GDI display functionalities...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
