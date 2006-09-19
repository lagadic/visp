/****************************************************************************
 *
 * $Id: testIcComp.cpp,v 1.11 2006-09-19 11:47:43 fspindle Exp $
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
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_ICCOMP

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <visp/vpIcCompGrabber.h>
#include <visp/vpImage.h>
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
  \param input : Card input number.
  \param scale : Subsampling factor.

 */
void usage(char *name, char *badparam, unsigned fps, unsigned input,
	   unsigned scale)
{
  fprintf(stdout, "\n\
Grab grey level images using the IC-comp framegrabber card from\n\
Imaging technology. Display these images using X11 or GTK.\n\
\n\
SYNOPSIS\n\
  %s [-f <fps=25|50>] [-i <input=0|1|2|3> \n\
   [-s <scale=1|2|4>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                                  Default\n\
  -f <fps>                                                  %ud\n\
     Framerate in term od number of images per second.\n\
     Possible values are 25 (for 25Hz) or 50 (for %) Hz)\n\
\n\
  -i <input>                                                %ud\n\
     Framegrabber active input. Values can be 0, 1, 2, 4\n\
\n\
  -s <scale>                                                %ud\n\
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
  \param input : Card input.
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
  unsigned input = vpIcCompGrabber::DEFAULT_INPUT;
  unsigned scale = vpIcCompGrabber::DEFAULT_SCALE;
  bool opt_display = true;

  // Read the command line options
  if (getOptions(argc, argv, fps, input, scale, opt_display) == false) {
    exit (-1);
  }

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  // Declare a framegrabber to acquire images with the IC-comp framegrabber
  // card (Imaging Technology)
  vpIcCompGrabber g;

  try{
    // Initialize the grabber
    g.setScale(scale);
    g.setInput(input);
    if (fps == 25)
      g.setFramerate(vpIcCompGrabber::framerate_25fps);
    else
      g.setFramerate(vpIcCompGrabber::framerate_50fps);
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

  cout << "Image size: " << I.getCols() << "  " << I.getRows() <<endl  ;

  // We open a window using either X11 or GTK.
  // Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#endif
  if (opt_display) {
    try{
      display.init(I, 100, 100,"IcComp Framegrabber...") ;
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

  long cpt = 1;
  // Loop to acquire 100 images
  while(cpt ++ < 100)
  {
    double t = vpTime::measureTimeMs();
    // read the image
    g.acquire(I) ;
    if (opt_display) {
      // Display the image
      vpDisplay::display(I) ;
      // Flush the display
      vpDisplay::flush(I) ;
    }
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
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
  vpTRACE("ICcomp frame grabber drivers are not available") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
