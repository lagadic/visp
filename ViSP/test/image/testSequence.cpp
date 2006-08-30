/****************************************************************************
 *
 * $Id: testSequence.cpp,v 1.7 2006-08-30 15:56:44 fspindle Exp $
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
 * Read an image sequence from the disk and display it.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>

#include <visp/vpTime.h>

/*!
  \example testSequence.cpp

  \brief Read an image sequence from the disk and display it.

  The sequence is made of separate images. Each image corresponds to a
  PGM file.

*/

// List of allowed command line options
#define GETOPTARGS	"di:hn:s:"

/*!

  Print the program options.

  \param ipath : Input image path.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.

 */
void usage(char *name, char *badparam, string ipath,
	   unsigned nimages, unsigned step)
{
  fprintf(stdout, "\n\
Read an image sequence from the disk and display it.\n\
The sequence is made of separate images. Each image corresponds\n\
to a PGM file.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-n <number of images> \n\
   [-s <step>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/cube/image.%%04d.pgm\"\n\
     images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -n <number of images>                                %u\n\
     Number of images to load from the sequence.\n\
\n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -d                                             \n\
     Disable the image display. This can be usefull \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), nimages, step);

}
/*!

  Set the program options.

  \param ipath : Input image path.
  \param nimages : Number of images to display.

  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be usefull for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, string &ipath,
		unsigned &nimages, unsigned &step, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'n': nimages = (unsigned) atoi(optarg); break;
    case 's': step = (unsigned) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, ipath, nimages, step); return false; break;

    default:
      usage(argv[0], optarg, ipath, nimages, step); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, nimages, step);
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
  unsigned opt_nimages = 79;
  unsigned opt_step = 1;
  bool opt_display = true;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_nimages, opt_step,
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
    usage(argv[0], NULL, ipath, opt_nimages, opt_step);
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

  // Warning :
  // the image sequence is not provided with the ViSP package
  // therefore the program will return you an error :
  //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file
  //        ViSP-images/cube/image.0001.pgm
  //  !!    vpDotExample.cpp: main(#95) :Error while reading the image
  //  terminate called after throwing an instance of 'vpImageException'
  //
  //  The sequence is available on the visp www site
  //  http://www.irisa.fr/lagadic/visp/visp.html
  //  in the download section. It is named "ViSP-images.tar.gz"

  // Set the path location of the image sequence
  dirname = ipath + vpIoTools::path("/ViSP-images/cube/");

  // Build the name of the image file
  unsigned iter = 0;
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

#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#endif
  if (opt_display) {
    try {
      // We open a window using either X11 or GTK.
      // Its size is automatically defined by the image (I) size
      display.init(I, 100, 100,"Display...") ;

      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
      // flush the display buffer
      vpDisplay::flush(I) ;
    }
    catch(...) {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }



  unsigned niter=0 ;
  double totaltms =0 ;
  // this is the loop over the image sequence
  while (iter < opt_nimages)
  {
    try {
      double tms = vpTime::measureTimeMs() ;
      // set the new image name
      s.str("");
      s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
      filename = dirname + s.str();
      // read the image
      vpImageIo::readPGM(I, filename);
      if (opt_display) {
	// Display the image
	vpDisplay::display(I) ;
	// Flush the display
	vpDisplay::flush(I) ;
      }
      // Synchronise the loop to 40 ms
      vpTime::wait(tms, 40) ;
      niter++ ;
    }
    catch(...) {
      exit(-1) ;
    }
    iter += opt_step ;
  }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11 or GTK display functionalities...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
