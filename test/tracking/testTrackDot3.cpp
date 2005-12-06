
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: testTrackDot3.cpp,v 1.3 2005-12-06 13:19:50 fspindle Exp $
 *
 * Description
 * ============
 *   test dot tracking
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

/*!
  \example testTrackDot3.cpp

  \brief   Test dot tracking on an image sequence using vpDot2
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testTrackDot3.cpp" <<endl << endl ;

  cout <<  " test dot tracking on an image sequence using vpDot2" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.

  char s[FILENAME_MAX] ;
  char dir[FILENAME_MAX] ;

  int iter = 1 ;
  sprintf(dir,"/udd/marchand/dd/t/images/pattern/mire-2/") ;
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;

  vpDot2 d ;

  d.setGraphics(true);
  d.setComputeMoments(true);

  try{
    TRACE("Load: %s", s);
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
    TRACE("Load: %s", s);
    vpImageIo::readPGM(I,s) ;
    vpDisplay::display(I) ;
    d.track(I) ;
    cout << "Moments:" << endl;
    cout << "m00: " << d.m00 << endl;
    cout << "m11: " << d.m11 << endl;
    cout << "m02: " << d.m02 << endl;
    cout << "m20: " << d.m20 << endl;
    cout << "m10: " << d.m10 << endl;
    cout << "m01: " << d.m01 << endl;
    vpDisplay::displayCross_uv(I,(int)d.get_u(), (int)d.get_v(),
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
