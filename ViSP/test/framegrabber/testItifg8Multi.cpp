/****************************************************************************
 *
 * $Id: testItifg8Multi.cpp,v 1.1 2006-09-29 12:51:19 fspindle Exp $
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
 * Acquire images from multiple itifg-8.x devices.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_ITIFG8

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <vpItifg8Grabber.h>
//#include <visp/vpItifg8Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!
  \example testIcComp.cpp

  \brief   Test framegrabbing using Imaging Technology IC-comp framegrabber.
*/

// List of allowed command line options
#define GETOPTARGS	"df:i:hs:"

/*!

  Print the program options.

  \param fps : Framerate.
  \param input : Camera port number.
  \param scale : Subsampling factor.

 */
void usage(char *name, char *badparam,
	   unsigned fps, unsigned input, unsigned scale)
{
  fprintf(stdout, "\n\
Grab grey level images using the itifg-8.x framegrabber device from\n\
Coreco Imaging. Display these images using X11 or GTK.\n\
\n\
SYNOPSIS\n\
  %s [-f <fps=25|50>] [-i <input=0|1|2|3> \n\
   [-s <scale=1|2|4>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                                  Default\n\
  -f <fps>                                                  %u\n\
     Framerate in term of number of images per second.\n\
     Possible values are 25 (for 25Hz) or 50 (for 50) Hz)\n\
\n\
  -i <input>                                                %u\n\
     Camera input number [0|1|2|3].\n\
\n\
  -s <scale>                                                %u\n\
     Framegrabber subsampling factor. \n\
     If 1, full resolution image acquisition 768x576.\n\
     If 2, half resolution acquisition 384x288. The \n\
     subsampling is achieved by the hardware.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h \n\
     Print the help.\n\n",
	  fps, input, scale);

}

/*!

  Set the program options.

  \param fps : Framerate.
  \param input : Camera port number.
  \param scale : Subsampling factor.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, unsigned &fps, unsigned &input,
		unsigned &scale, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'f': fps = (unsigned) atoi(optarg); break;
    case 'i': input = (unsigned) atoi(optarg); break;
    case 's': scale = (unsigned) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, fps, input, scale); return false; break;

    default:
      usage(argv[0], optarg, fps, input, scale); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, fps, input, scale);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}

int
main(int argc, char ** argv)
{
  unsigned fps = 25;
  unsigned input = vpItifg8Grabber::DEFAULT_INPUT;
  unsigned scale = vpItifg8Grabber::DEFAULT_SCALE;
  bool opt_display = true;


  // Read the command line options
  if (getOptions(argc, argv, fps, input, scale, opt_display) == false) {
    exit (-1);
  }
  // Declare a framegrabber to acquire images with the IC-comp framegrabber
  // card (Imaging Technology)
  vpItifg8Grabber g;
  unsigned nboards = g.getNumBoards();

  cout << "Number of detected boards: " << nboards << endl;

  // Declare an image pointer, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> *I ;

  // We create an image for each board detected
  I = new vpImage<unsigned char> [nboards];


  try{
    for (int i=0; i < nboards; i ++) {
      // Initialize each grabber
      g.setBoard(i);
      g.setConfile("/udd/fspindle/robot/driver/itifg/itifg-8.2.2-0-irisa/conffiles/robot.cam");

      g.setInput(0);
      g.setScale(scale);
      g.setDepth(8);
      g.setBuffer(1); //
      g.setFramerate(1.); // Default framerate
      g.setOpmode(vpItifg8Grabber::MMAP_MODE);     // operation mode
      g.setSyncmode(vpItifg8Grabber::SIGNAL_MODE); // synchronisation mode
      g.setAcqmode(vpItifg8Grabber::NORMAL_MODE);  // special acq mode
//     if (fps == 25)
//       g.setFramerate(vpItifg8Grabber::framerate_25fps);
//     else
//       g.setFramerate(vpItifg8Grabber::framerate_50fps);
    // Open the framegrabber with the specified settings
      g.open(I[i]) ;
      // Acquire an image
      g.acquire(I[i]) ;

      cout << "Image size: " << I[i].getCols() << "  " << I[i].getRows() <<endl  ;

    }
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot acquire the image... ") ;
    delete [] I;
    exit(-1);
  }

  // We open a window using either X11 or GTK.
  // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
  vpDisplayX *display;
  display = new vpDisplayX [nboards];
#elif defined VISP_HAVE_GTK
  vpDisplayGTK *display;
  display = new vpDisplayGTK [nboards];
#endif
  if (opt_display) {
    try{
      for (int i=0; i < nboards; i ++) {
	char title[100];
	sprintf(title, "Itifg-8.x Framegrabber: board %d", i);
	display[i].init(I[i], 100+30*i, 100+30*i, title) ;
	// display the image
	// The image class has a member that specify a pointer toward
	// the display that has been initialized in the display declaration
	// therefore is is no longuer necessary to make a reference to the
	// display variable.
	vpDisplay::display(I[i]) ;
	vpDisplay::flush(I[i]) ;
      }
    }
    catch(...)
    {
      vpERROR_TRACE("Cannot display the image... ") ;
      delete [] I;
      delete [] display;
      exit(-1);
    }
  }

  child = 0;
  children[child] = getpid();
  for (i = 1; i < boards; i++)
    {
      if ((children[i] = fork ()) < 0)
	{
	  perror ("Can't create child process");
	  exit (EXIT_FAILURE);
	}
      if (children[i] == 0)
	{
	  child = i;
	  break;
	}
    }

//   cout << "A click to continue..." << endl;
//   vpDisplay::getClick(I[0]);

  long cpt = 1;
  // Loop to acquire 100 images
  while(cpt ++ < 100)
  {
    double t = vpTime::measureTimeMs();
    for (int i=0; i < nboards; i ++) {
      g.setBoard(i);
      // read the image
      g.acquire(I[i]) ;

      if (opt_display) {
	// Display the image
	vpDisplay::display(I[i]) ;
	// Flush the display
	vpDisplay::flush(I[i]) ;
      }
    }
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
//     cout << "A click to continue..." << endl;
//     vpDisplay::getClick(I[0]);
  }

  delete [] I;
  delete [] display;
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
