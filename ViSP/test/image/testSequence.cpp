
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDotExample.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testSequence.cpp,v 1.2 2006-04-19 09:01:23 fspindle Exp $
 *
 * Description
 * ============
 *   test the display of an image sequence
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>

#include <visp/vpTime.h>

/*!
  \example testSequence.cpp

  \brief   test the display of an image sequence
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testSequence.cpp" <<endl << endl ;

  cout <<  " exampe of  dots tracking on an image sequence" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;



  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  // Warning :
  // the image sequence is not provided with the ViSP package
  // therefore the program will return you an error :
  //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file /Images/cube-1/image.0001.pgm
  //  !!      vpDotExample.cpp: main(#95) :Error while reading the image
  //  terminate called after throwing an instance of 'vpImageException'
  //
  //  The sequence is available on the visp www site
  //  http://www.irisa.fr/lagadic/visp/visp.html
  //  in the download section. It is named "cube.tar.gz"

  // directory name
  char dir[FILENAME_MAX] ;
  sprintf(dir,"/Images/cube") ;

  // image base name.
  char s[FILENAME_MAX] ;
  int iter = 0 ;
  // set image name to [dir]/image.0000.pgm
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;


  // Read the PGM image named "s" on the disk, and put the bitmap into the
  // image structure I.
  // I is initialized to the correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    // an exception is throwned if an exception from readPGM has been catched
    // here this will result in the end of the program
    // Note that another error message has been printed from readPGM
    // to give more information about the error
    ERROR_TRACE("Error while reading the image") ;
    throw ;
  }

  // We open a window using the  window manager
  // it will be located in 100,100 and titled "tracking using vpDot"
  // its size is automatically defined by the image (I) size
  vpDisplayX display(I,100,100,"tracking using vpDot") ;

  try{
    // display the image
    // The image class has a member that specify a pointer toward
    // the display that has been initialized in the display declaration
    // therefore is is no longuer necessary to make a reference to the
    // display variable.
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE("Error while displaying the image") ;
    throw ;
  }


  // flush the X11 buffer
  vpDisplay::flush(I) ;
  
  int niter=0 ;
  double totaltms =0 ;
  // this is the loop over the image sequence
  while (iter < 400)
  {
    try {
      // set the new image name
      sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
      cout << s <<endl ;
     // read the image
      vpImageIo::readPGM(I,s) ;
      double tms  =  vpTime::measureTimeMs() ;
      vpDisplay::display(I) ;
      totaltms += vpTime::measureTimeMs()  - tms ;
      niter++ ;
    }
    catch(...)
      {
	ERROR_TRACE("Error in tracking loop") ;
	throw ;
      }
    iter +=5 ;
  }
  cout << "mean time (display) " << totaltms/niter << " ms" << endl ;
}
#else
int
main()
{
  ERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
