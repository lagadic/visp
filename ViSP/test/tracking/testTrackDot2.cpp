
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
 *  $Id: testTrackDot2.cpp,v 1.1.1.1 2005-06-08 07:08:15 fspindle Exp $
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

  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpDot dot ;
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
