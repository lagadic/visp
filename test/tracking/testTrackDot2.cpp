
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
 *  $Id: testTrackDot2.cpp,v 1.2 2005-07-13 10:42:02 fspindle Exp $
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
#include <visp/vpFeatureEllipse.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpFeatureBuilder.h>

/*!
  \example testTrackDot2.cpp

  \brief   Test dot tracking on an image sequence by using vpDot.
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testTrackDot2.cpp" <<endl << endl ;

  cout <<  " test dot tracking on an image sequence" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.

  char s[FILENAME_MAX] ;
  char dir[FILENAME_MAX] ;

  sprintf(dir,".") ;
  sprintf(s,"%s/test-ellipse.pgm",dir) ;


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

  vpDot dot ;
  dot.setNbMaxPoint(8000);
  dot.initTracking(I) ;
  dot.setGraphics(true) ;
  dot.setComputeMoments(true) ;
  dot.track(I) ;

  vpFeatureEllipse e ;

  vpCameraParameters cam ;
  vpFeatureBuilder::create(e,cam,dot) ;
  e.display(cam, I, vpColor::red) ;

  vpDisplay::getClick(I) ;


}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
