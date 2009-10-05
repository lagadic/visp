/****************************************************************************
 *
 * $Id: trackMeLine.cpp,v 1.11 2008-06-17 08:08:28 asaunier Exp $
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
 * Tracking of Surf key points.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file keyPointSurf.cpp

  \brief Tracking of Surf key-points.
*/

/*!
  \example keyPointSurf.cpp

  Tracking of Surf key-points.
*/

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if ((defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)) && (VISP_HAVE_OPENCV_VERSION >= 0x010100))

#include <visp/vpKeyPointSurf.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

// List of allowed command line options
#define GETOPTARGS	"cdi:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.

*/
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
Tracking of a line.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/line/image.%%04d.pgm\"\n\
     images. \n\
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

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
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
bool getOptions(int argc, const char **argv, std::string &ipath,
		bool &click_allowed, bool &display)
{
  const char *optarg;
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
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string dirname;
  std::string filename;
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
  if (!opt_ipath.empty() && !env_ipath.empty()) {
    if (ipath != env_ipath) {
      std::cout << std::endl
	   << "WARNING: " << std::endl;
      std::cout << "  Since -i <visp image path=" << ipath << "> "
	   << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
	   << "  we skip the environment variable." << std::endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty()){
    usage(argv[0], NULL, ipath);
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << std::endl
	 << "  environment variable to specify the location of the " << std::endl
	 << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> Iref ;
  vpImage<unsigned char> Icur ;

  // Set the path location of the image sequence
  dirname = ipath +  vpIoTools::path("/ViSP-images/cube/");

  // Build the name of the image file
  unsigned iter = 0; // Image number
  std::ostringstream s;
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
  filename = dirname + s.str();

  // Read the PGM image named "filename" on the disk, and put the
  // bitmap into the image structure I.  I is initialized to the
  // correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpCTRACE << "Load: " << filename << std::endl;

    vpImageIo::readPGM(Iref, filename) ;
  }
  catch(...)
  {
    // an exception is throwned if an exception from readPGM has been catched
    // here this will result in the end of the program
    // Note that another error message has been printed from readPGM
    // to give more information about the error
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Cannot read " << filename << std::endl;
    std::cerr << "  Check your -i " << ipath << " option " << std::endl
	 << "  or VISP_INPUT_IMAGE_PATH environment variable."
	 << std::endl;
    exit(-1);
  }

  // We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
  vpDisplayX display[2];
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display[2];
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display[2];
#endif

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display[0].init(Iref, 100, 100,"Display reference image") ;
      vpDisplay::display(Iref) ;
      vpDisplay::flush(Iref) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  vpImagePoint corners[2];
  if (opt_display && opt_click_allowed)
  {
    std::cout << "Click on the top left and the bottom right corners to define the part of the image where the reference points will be computed" << std::endl;
    for (int i=0 ; i < 2 ; i++)
    {
      vpDisplay::getClick(Iref, corners[i]);
std::cout << corners[i] << std::endl;
    }
  }
  else
  {
    corners[0].set_ij(156,209);
    corners[1].set_ij(272,378);
  }

  if (opt_display)
  {
    //Display the rectangle which defines the part of the image where the reference points are computed.
    vpDisplay::displayRectangle(Iref, corners[0], corners[1], vpColor::green);
    vpDisplay::flush(Iref);
  }

  if (opt_click_allowed)
  {
    std::cout << "Click on the image to continue" << std::endl;
    vpDisplay::getClick(Iref);
  }

  vpKeyPointSurf surf;
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());

  //Computes the reference points
  nbrRef = surf.buildReference(Iref, corners[0], height, width);

  int nbrPair = 0;

  vpImageIo::readPGM(Icur, filename);

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display[1].init(Icur, 100, 100,"Display current image") ;
      vpDisplay::display(Icur) ;
      vpDisplay::flush(Icur) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  for (int iter = 1 ; iter < 30 ; iter++)
  {
    std::cout <<"----------------------------------------------------------"<<std::endl;
    // set the new image name
    s.str("");
    s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
    filename = dirname + s.str();
    // read the image
    vpImageIo::readPGM(Icur, filename);
    if (opt_display) {
      // Display the image
      vpDisplay::display(Iref) ;
      vpDisplay::display(Icur) ;
    }

    nbrPair = surf.matchPoint(Icur);
    std::cout << "Number of matched point : " << nbrPair <<std::endl;

    if (opt_display)
    {
      // Display the matched features
      surf.display(Iref,Icur);

      vpDisplay::flush(Iref) ;
      vpDisplay::flush(Icur) ;
    }
  }
}

#else
int
main()
{
#if ( ! (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)) ) 
  vpERROR_TRACE("You do not have X11, GTK or GDI display functionalities...");
#else
  vpERROR_TRACE("You do not have OpenCV-1.1.0 or a more recent release...");
#endif
}

#endif
