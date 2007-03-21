/****************************************************************************
 *
 * $Id: displayGDI.cpp,v 1.5 2007-03-21 14:10:51 asaunier Exp $
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
 * Windows' GDI Display Test
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/
/*!
  \file displayGDI.cpp

  \brief Read an image on the disk, display it using vpDisplayGDI class, display some
  features (line, circle, caracters) in overlay and finaly write the image and
  the overlayed features in an image on the disk.

*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if ( defined(WIN32) && defined(VISP_HAVE_GDI) )

#include <visp/vpDisplayGDI.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

/*!
  \example displayGDI.cpp

  Read an image on the disk, display it using vpDisplayGDI class, display some
  features (line, circle, caracters) in overlay and finaly write the image and
  the overlayed features in an image on the disk.

*/

// List of allowed command line options
#define GETOPTARGS	"cdi:o:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.


 */
void usage(char *name, char *badparam, string ipath, string opath, string user)
{
  fprintf(stdout, "\n\
Read an image on the disk, display it using GDI, display some\n\
features (line, circle, caracters) in overlay and finaly write \n\
the image and the overlayed features in an image on the disk\n\
\n\
SYNOPSIS\n\
  %s [-p <input image path>] [-o <output image path>]\n\
     [-c] [-d] [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/Klimt/Klimt.pgm\"\n\
     image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     Klimt_grey.overlay.ppm output image is written.\n\
\n\
  -c\n\
     Disable the mouse click. Usefull to automate the \n\
     execution of this program without humain intervention.\n\
\n\
  -d                                             \n\
     Disable the image display. This can be usefull \n\
     for automatic tests using the task manager under \n\
     Windows.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), opath.c_str(), user.c_str());

}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param click_allowed : Enable/disable mouse click.
  \param user : Username.
  \param display :Activates Display

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv,
		string &ipath, string &opath, bool &click_allowed,
		string user, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'o': opath = optarg; break;
    case 'h': usage(argv[0], NULL, ipath, opath, user); return false; break;

    default:
      usage(argv[0], optarg, ipath, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
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
  string opt_opath;
  string ipath;
  string opath;
  string filename;
  string username;
  bool opt_click_allowed = true;
  bool opt_display = true;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Set the default output path
  opt_opath = "C:\\temp";


  // Get the user login name
  vpIoTools::getUserName(username);

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_opath,
		 opt_click_allowed, username, opt_display) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;
  if (!opt_opath.empty())
    opath = opt_opath;

  // Append to the output path string, the login name of the user
  string dirname = opath +  vpIoTools::path("/") + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(dirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(dirname);
    }
    catch (...) {
      usage(argv[0], NULL, ipath, opath, username);
      cerr << endl
	   << "ERROR:" << endl;
      cerr << "  Cannot create " << dirname << endl;
      cerr << "  Check your -o " << opath << " option " << endl;
      exit(-1);
    }
  }

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
    usage(argv[0], NULL, ipath, opath, username);
    cerr << endl
	 << "ERROR:" << endl;
    cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << endl
	 << "  environment variable to specify the location of the " << endl
	 << "  image path where test images are located." << endl << endl;
    exit(-1);
  }

  // Create a grey level image
  vpImage<unsigned char> I ;

  // Load a grey image from the disk
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
  vpImageIo::readPGM(I, filename) ;

  // For this grey level image, open a GDI display at position 100,100
  // in the screen, and with title "GDI display"
  vpDisplayGDI display;

  if (opt_display) {
    // We open a window using GDI.
    // Its size is automatically defined by the image (I) size
    display.init(I, 100, 100, "GDI display") ;

    // Display the image
    vpDisplay::display(I) ;

    // Display in overlay a red cross at position 10,10 in the
    // image. The lines are 10 pixels long
    vpDisplay::displayCross(I, 100,10, 20, vpColor::red) ;

    // Display in overlay horizontal red lines
    for (unsigned i=0 ; i < I.getHeight() ; i+=20)
      vpDisplay::displayLine(I,i,0,i,I.getWidth(), vpColor::red) ;

    // Display in overlay vertical green dot lines
    for (unsigned i=0 ; i < I.getWidth() ; i+=20)
      vpDisplay::displayDotLine(I,0,i,I.getWidth(), i,vpColor::green) ;

    // Display in overlay a blue arrow
    vpDisplay::displayArrow(I,0,0,100,100,vpColor::blue) ;

    // Display in overlay some circles. The position of the center is 200, 200
    // the radius is increased by 20 pixels for each circle
    for (unsigned i=0 ; i < 100 ; i+=20)
      vpDisplay::displayCircle(I,200,200,20+i,vpColor::yellow) ;

    // Display in overlay a yellow string
    vpDisplay::displayCharString(I, 85, 100,
				 "ViSP is a marvelous software",
				 vpColor::yellow) ;

    // Create a color image
    vpImage<vpRGBa> Ioverlay ;
    // Updates the color image with the original loaded image and the overlay
    vpDisplay::getImage(I, Ioverlay) ;

    // Write the color image on the disk
    filename = opath +  vpIoTools::path("/Klimt_grey.overlay.ppm");
    vpImageIo::writePPM(Ioverlay, filename) ;

    // If click is allowed, wait for a mouse click to close the display
    if (opt_click_allowed) {
      cout << "\nA click to close the windows..." << endl;
      // Wait for a blocking mouse click
      vpDisplay::getClick(I) ;
    }

    // Close the display
    vpDisplay::close(I);
  }

  vpTRACE("-------------------------------------");

  // Create a color image
  vpImage<vpRGBa> Irgba ;

  // Load a grey image from the disk and convert it to a color image
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
  vpImageIo::readPGM(Irgba, filename) ;

  // For this color image, open a GDI display at position 100,100
  // in the screen, and with title "GDI color display"
  vpDisplayGDI displayRGBa;

  if (opt_display) {
    // We open a window using GDI.
    // Its size is automatically defined by the image (Irgba) size
    displayRGBa.init(Irgba, 100, 100, "GDI color display");

    // Display the color image
    vpDisplay::display(Irgba) ;

    // If click is allowed, wait for a blocking mouse click to display
    // a cross at the clicked pixel position
    if (opt_click_allowed) {
      cout << "\nA click to display a cross..." << endl;
      unsigned i,j;
      // Blocking wait for a click. Get the position of the selected pixel
      // (i correspond to the row and j to the column coordinates in the image)
      vpDisplay::getClick(Irgba, i, j);
      // Display a red cross on the click pixel position
      cout << "Cross position: " << i << ", " << j << endl;
      vpDisplay::displayCross(Irgba,i,j,15,vpColor::red);
    }
    else {
      unsigned i=10,j=20;
      // Display a red cross at position i, j (i correspond to the row
      // and j to the column coordinates in the image)
      cout << "Cross position: " << i << ", " << j << endl;
      vpDisplay::displayCross(Irgba,i,j,15,vpColor::red);

    }
    // If click is allowed, wait for a blocking mouse click to exit.
    if (opt_click_allowed) {
      cout << "\nA click to exit the program..." << endl;
      vpDisplay::getClick(Irgba) ;
      cout << "Bye" << endl;
    }
  }
}
#else
int
main()
{
  vpERROR_TRACE("GDI display only works under windows...");
}
#endif
