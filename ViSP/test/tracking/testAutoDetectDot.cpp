
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
 *  $Id: testAutoDetectDot.cpp,v 1.1 2005-07-18 13:57:04 fspindle Exp $
 *
 * Description
 * ============
 *   Test auto detection of dots
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>

#include <stdio.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpList.h>
#include <visp/vpDot2.h>

/*!
  \example testAutoDetectDot.cpp

  \brief   Test dot tracking on an image sequence using vpDot2
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testAutoDetectDot.cpp" <<endl << endl ;

  cout <<  " test auto detection of dots using vpDot2" << endl ;
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
  vpList<vpDot2> * list_d;

  d.setGraphics(true);

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
//     d.initTracking(I) ;
//     cout << "Dot characteristics: " << endl;
//     cout << d.getWidth() << endl;
//     cout << d.getHeight()<< endl;
//     cout << d.getSurface()<< endl;
//     cout << d.getInLevel()<< endl;
//     cout << d.getOutLevel()<< endl;
//     cout << d.getAccuracy()<< endl;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }


  // Set dot characteristics for the auto detection
  d.setWidth(15.0);
  d.setHeight(12.0);
  d.setSurface(124);
  d.setInLevel(164);
  d.setOutLevel(164);
  d.setAccuracy(0.65);

  while (iter < 3)
  {
    sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
    TRACE("Load: %s", s);
    vpImageIo::readPGM(I,s) ;
    vpDisplay::display(I) ;

    list_d = d.searchDotsInArea(I, 0, 0, I.getRows(), I.getCols()) ;

    if( list_d->nbElement() == 0 ) {
      cout << "Dot auto detection did not work, "
	   << "Please click on a dot to perform a manual detection"
	   << endl;

      d.initTracking( I );
      vpDisplay::displayCross(I,(int)d.I(), (int)d.J(),
			      10,vpColor::green) ;
      vpDisplay::flush(I) ;
    }
    else {
      cout << endl << list_d->nbElement() << " dots are detected" << endl;

      // Parse all founded dots for display
      list_d->front();
      while (!list_d->outside()) {
	vpDot2 tmp_d;
	tmp_d = list_d->value() ;
	list_d->next() ;
	vpDisplay::displayCross(I,(int)tmp_d.I(), (int)tmp_d.J(),
				10, vpColor::red) ;
      }
      vpDisplay::flush(I) ;
    }


    TRACE("Click in the image to continue...");
    vpDisplay::getClick(I);
    iter ++;
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
