
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
 *  $Id: testTrackDot.cpp,v 1.3 2005-12-05 14:03:47 marchand Exp $
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

  cout <<  " test dot tracking on an image sequence" << endl ;
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
  d.setGraphics(true);
  d.setComputeMoments(true);

  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpDisplayX display(I,100,100,"Test tracking using vpDot") ;

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

  while (iter < 1500)
  {
    sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
    vpImageIo::readPGM(I,s) ;
    vpDisplay::display(I) ;
    d.track(I) ;
    cout << "Moments: \n" << endl;
    cout << "m00: " << d.m00 << endl;
    cout << "m11: " << d.m11 << endl;
    cout << "m02: " << d.m02 << endl;
    cout << "m20: " << d.m20 << endl;
    cout << "m10: " << d.m10 << endl;
    cout << "m01: " << d.m01 << endl;

    vpDisplay::displayCross(I,(int)d.get_v(), (int)d.get_u(),
			   10,vpColor::green) ;
    vpDisplay::flush(I) ;
//     vpDisplay::getClick(I);
    iter ++;
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
