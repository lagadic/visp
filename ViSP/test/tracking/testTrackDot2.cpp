
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testTrackDot2.cpp,v 1.5 2006-06-23 14:45:09 brenier Exp $
 *
 * Description
 * ============
 *   test dot tracking
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

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
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpDisplayX display(I,100,100,"Test tracking using vpDot") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpDot dot ;
  dot.setNbMaxPoint(8000);
  dot.initTracking(I, 140, 140) ;
  dot.setGraphics(true) ;
  dot.setComputeMoments(true) ;
  dot.track(I) ;

  vpFeatureEllipse e ;

  vpCameraParameters cam ;
  vpFeatureBuilder::create(e,cam,dot) ;
  e.display(cam, I, vpColor::red) ;

}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
