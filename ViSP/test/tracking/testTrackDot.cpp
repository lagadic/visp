
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testIoPGM.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testTrackDot.cpp,v 1.1.1.1 2005-06-08 07:08:15 fspindle Exp $
 *
 * Description
 * ============
 *   test reading and writting of PGM image
 *   read an image that does not exist
 *   write in a directory that does no exist
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot.h>

/*!
  \example testTrackDot.cpp

  \brief   test dot tracking on an image sequence
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testTrackDot.cpp" <<endl << endl ;

  cout <<  "  test dot tracking on an image sequence" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.

  char s[FILENAME_MAX] ;
  char dir[FILENAME_MAX] ;

  int iter = 1 ;
  sprintf(dir,"/udd/marchand/dd/t/images/pattern/mire-2/") ;
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;

  vpDot d ;

  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  try{
    d.initTracking(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  for (iter = 1 ; iter < 1500 ; iter++)
  {
    sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
    vpImageIo::readPGM(I,s) ;
    d.track(I) ;
    vpDisplay::display(I) ;
    vpDisplay::displayCross(I,(int)d.I(), (int)d.J(),
			   10,vpColor::green) ;
    vpDisplay::flush(I) ;
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
