



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
  vpMeEllipse E1 ;

  vpMe me ;
  me.setRange(20) ;
  me.setSampleStep(2) ;
  me.setPointsToTrack(60) ;
  me.setThreshold(15000) ;


  E1.setCircle(true) ;
  E1.setMe(&me) ;
  E1.setDisplay(vpMeTracker::RANGE_RESULT) ;
  E1.initTracking(I) ;
  E1.display(I, vpColor::green) ;

  ERROR_TRACE("sample step %f ",E1.me->sample_step) ;
  E1.track(I) ;
  vpDisplay::getClick(I) ;
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
      ERROR_TRACE("Error in tracking vpMeLine ") ;
      exit(1) ;
    }

    E1.display(I,vpColor::green) ;
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I) ;

  }
  vpDisplay::getClick(I) ;


}
