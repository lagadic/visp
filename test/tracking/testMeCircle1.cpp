
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
 *  $Id: testMeCircle1.cpp,v 1.3 2006-06-23 14:45:09 brenier Exp $
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
  sprintf(s,"./test-circle.pgm") ;

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


  E1.setCircle(true) ;
  E1.setMe(&me) ;
  E1.setDisplay(vpMeTracker::RANGE_RESULT) ;
#if 0
  E1.initTracking(I) ;
#else
  // Create a list of points to automate the test
  int n=5 ;
  int *i, *j ;
  i = new int[n] ;
  j = new int[n] ;
  i[0] = 71; j[0] = 196;
  i[1] = 54; j[1] = 154;
  i[2] = 84; j[2] = 105;
  i[3] = 129; j[3] = 101;
  i[4] = 158; j[4] = 125;
  E1.initTracking(I, n, i, j) ;
  delete []i ;
  delete []j ;

#endif
  E1.display(I, vpColor::green) ;

  vpERROR_TRACE("sample step %f ",E1.me->sample_step) ;
  E1.track(I) ;
#if 0
  vpDisplay::getClick(I) ;
#endif
  cout <<"------------------------------------------------------------"<<endl;


  for (int iter = 1 ; iter < 1500 ; iter++)
  {
    sprintf(s,"%s/image.%04d.pgm",rep,iter) ;
    sprintf(s,"./test-circle.pgm") ;
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
#if 0
    vpDisplay::getClick(I) ;
#endif
  }
#if 0
  vpDisplay::getClick(I) ;
#endif
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
