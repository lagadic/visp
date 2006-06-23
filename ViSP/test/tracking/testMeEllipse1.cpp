
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testMeLine1.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testMeEllipse1.cpp,v 1.3 2006-06-23 14:45:09 brenier Exp $
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpColor.h>

#include <visp/vpMeEllipse.h>


int
main()
{

  int im = 1 ;

  char rep[FILENAME_MAX] ;
  sprintf(rep,"/local/images/primitives/ellipse1/") ;

  vpImage<unsigned char> I ; char s[FILENAME_MAX] ;
  sprintf(s,"%s/image.%04d.pgm",rep,im) ;

  cout << "chargement de " << endl <<s << endl ;

  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
  vpMeEllipse E1 ;

  vpMe me ;
  me.setRange(20) ;
  me.setSampleStep(2) ;
  me.setPointsToTrack(60) ;
  me.setThreshold(15000) ;


  E1.setMe(&me) ;
  E1.setDisplay(vpMeTracker::RANGE_RESULT) ;
  E1.initTracking(I) ;
  E1.display(I, vpColor::green) ;

  vpERROR_TRACE("sample step %f ",E1.me->sample_step) ;
  E1.track(I) ;
  vpDisplay::getClick(I) ;
  cout <<"------------------------------------------------------------"<<endl;


  for (int iter = 1 ; iter < 1500 ; iter++)
  {
    sprintf(s,"%s/image.%04d.pgm",rep,iter) ;
    vpImageIo::readPGM(I,s) ;
    vpDisplay::display(I) ;

    try
    {
      E1.track(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error in tracking vpMeLine ") ;
      exit(1) ;
    }

    E1.display(I,vpColor::green) ;
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I) ;

  }
  vpDisplay::getClick(I) ;


}
#else
int
main()
{
  vpERROR_TRACE("You do not have a biclops robot connected to your computer...");
}

#endif
