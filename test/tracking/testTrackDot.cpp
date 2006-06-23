
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
 *  $Id: testTrackDot.cpp,v 1.6 2006-06-23 14:45:09 brenier Exp $
 *
 * Description
 * ============
 *   test reading and writting of PGM image
 *   read an image that does not exist
 *   write in a directory that does no exist
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#define ADD_TEST 1

#include <stdio.h>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

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
  sprintf(dir,"/local/seq/mire-2/") ;
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;

  vpDot d ;
  d.setGraphics(true);
  d.setComputeMoments(true);
  d.setConnexity(vpDot::CONNEXITY_8);

  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  if (ADD_TEST) {
    I = 0;

    int half_w = 10;
    int half_h = 10;
    int u0 = 200;
    int v0 = 110;

    for (int v = v0-half_h; v < v0+half_h; v ++)
      for (int u = u0-half_w; u < u0+half_w; u ++)
	if (u >=0 && u < I.getCols() && v>=0 && v < I.getRows())
	  I[v][u] = 255;

  }

  vpDisplayX display(I,100,100,"Test tracking using vpDot") ;

  try{
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
  try{
    d.initTracking(I) ;
    if (ADD_TEST) {
      cout << "COG: " << endl;
      cout << d.get_u() << " " << d.get_v()
	   << " - "
	   << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
      cout << "Moments:" << endl;
      cout << "m00: " << d.m00 << endl;
      cout << "m11: " << d.m11 << endl;
      cout << "m02: " << d.m02 << endl;
      cout << "m20: " << d.m20 << endl;
      cout << "m10: " << d.m10 << endl;
      cout << "m01: " << d.m01 << endl;
      vpDisplay::getClick(I);
    }
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  while (iter < 1500)
  {
    sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
    vpImageIo::readPGM(I,s) ;

    if (ADD_TEST) {
      I = 0;

      int half_w = 15;
      int half_h = 10;
      int u0 = 240;
      int v0 = 110;

      for (int v = v0-half_h; v < v0+half_h; v ++)
	for (int u = u0-half_w; u < u0+half_w; u ++)
	  if (u >=0 && u < I.getCols() && v>=0 && v < I.getRows())
	    I[v][u] = 255;

    }

    vpDisplay::display(I) ;
    d.track(I) ;

    cout << "COG: " << endl;
    cout << d.get_u() << " " << d.get_v()
	 << " - "
	 << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
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
    vpDisplay::getClick(I);
    iter ++;
  }
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
