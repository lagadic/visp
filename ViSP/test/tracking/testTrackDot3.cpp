
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
 *  $Id: testTrackDot3.cpp,v 1.4 2006-04-19 09:01:25 fspindle Exp $
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
#include <visp/vpDot2.h>

int gsl_warnings_off;
#define ADD_TEST 0
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

  int iter = 1;
  sprintf(dir,"/local/seq/mire-2/") ;
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
  //  sprintf(dir,"/udd/marchand/dd/t/images/pattern/mire-2/") ;
  //sprintf(s,"%s/image.%04d.pgm",dir,iter) ;


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

  if (ADD_TEST) {
    I = 0;

//     int i0 = 150;
//     int j0 = 200;
//     int r = 10;

//     for (int i = i0-r; i < i0+r; i ++) {
//       double theta = asin((double)(i-i0)/r);
//       int jmax = (int) (r*cos(theta));
//       for (int j=j0-jmax; j < j0+jmax; j++)
// 	I[i][j] = 255;
//     }


    int half_w = 10;
    int half_h = 10;
    int u0 = 200;
    int v0 = 110;

    for (int v = v0-half_h; v <= v0+half_h; v ++)
      for (int u = u0-half_w; u <= u0+half_w; u ++)
	if (u >=0 && u < I.getCols() && v>=0 && v < I.getRows())
	  I[v][u] = 255;

//     I[100][200] = 0;
//     I[100][220] = 0;
//     I[120][200] = 0;
//     I[120][220] = 0;

  }

  vpDisplayX display(I,100,100,"Test tracking using vpDot2") ;

  try{
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  try{
    d.initTracking(I) ;
    if (1) {
      cout << "COG: " << endl;
      cout << d.get_u() << " " << d.get_v()
	   << " - "
	   << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
      cout << "Size:" << endl;
      cout << "w: " << d.getWidth() << " h: " << d.getHeight() << endl;
      cout << "Moments:" << endl;
      cout << "m00: " << d.m00 << endl;
      cout << "m11: " << d.m11 << endl;
      cout << "m02: " << d.m02 << endl;
      cout << "m20: " << d.m20 << endl;
      cout << "m10: " << d.m10 << endl;
      cout << "m01: " << d.m01 << endl;
      //vpDisplay::getClick(I);
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  while (iter < 20)
  {
    sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
    TRACE("Load: %s", s);
    vpImageIo::readPGM(I,s) ;

    if (ADD_TEST) {
//       int i0 = 150;
//       int j0 = 250;
//       int r = 10;

      I = 0;
//       for (int i = i0-r; i < i0+r; i ++) {
// 	double theta = asin((double)(i-i0)/r);
// 	int jmax = (int) (r*cos(theta));
// 	for (int j=j0-jmax; j < j0+jmax; j++)
// 	  I[i][j] = 255;
//       }

    int half_w = 15;
    int half_h = 10;
    int u0 = 240;
    int v0 = 110;

    for (int v = v0-half_h; v <= v0+half_h; v ++)
      for (int u = u0-half_w; u <= u0+half_w; u ++)
	if (u >=0 && u < I.getCols() && v>=0 && v < I.getRows())
	  I[v][u] = 255;


//       I[105][205] = 0;
//       I[105][225] = 0;
//       I[125][205] = 0;
//       I[125][225] = 0;
    }

    vpDisplay::display(I) ;
    d.track(I) ;

    cout << "COG: " << endl;
    cout << d.get_u() << " " << d.get_v()
	 << " - "
	 << d.m10 / d.m00 << " " << d.m01 / d.m00 << endl;
    cout << "Size:" << endl;
    cout << "w: " << d.getWidth() << " h: " << d.getHeight() << endl;
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
    //vpDisplay::getClick(I);
    iter ++;
  }
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
