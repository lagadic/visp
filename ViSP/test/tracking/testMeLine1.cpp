



#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpColor.h>

#include <visp/vpMeLine.h>

#include <visp/vpFeatureLine.h>
#include <visp/vpFeatureBuilder.h>

int
main()
{

  int im = 1 ;

  char rep[FILENAME_MAX] ;
  sprintf(rep,"/udd/marchand/im2/") ;

  vpImage<unsigned char> I ; char s[FILENAME_MAX] ;
  sprintf(s,"%s/image%04d.pgm",rep,im) ;

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
  vpMeLine L1 ;

  vpMe me ;
  me.setRange(5) ;
  me.setPointsToTrack(160) ;
  me.setThreshold(15000) ;


  L1.setMe(&me) ;
  L1.setDisplay(vpMeTracker::RANGE_RESULT) ;
  L1.initTracking(I) ;
  L1.display(I, vpColor::green) ;

  L1.track(I) ;
  vpDisplay::getClick(I) ;
  cout <<"------------------------------------------------------------"<<endl;

  vpFeatureLine l ;

  vpCameraParameters cam ;
  vpImage<vpRGBa> Ic ;
  for (int iter = 1 ; iter < 1500 ; iter++)
  {
   cout <<"------------------------------------------------------------"<<endl;
   sprintf(s,"%s/image%04d.pgm",rep,iter) ;
    vpImageIo::readPGM(I,s) ;
    vpDisplay::display(I) ;

    try
    {
      L1.track(I) ;
    }
    catch(...)
    {
      ERROR_TRACE("Error in tracking vpMeLine ") ;
      exit(1) ;
    }

    TRACE("L1 : %f %f", L1.getRho(), vpMath::deg(L1.getTheta())) ;
    vpFeatureBuilder::create(l,cam,L1) ;
    TRACE("L1 : %f %f", l.getRho(), vpMath::deg(l.getTheta())) ;


    L1.display(I,vpColor::green) ;
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I) ;

    sprintf(s,"/tmp/marchand/image.%04d.ppm",iter) ;

  }
  vpDisplay::getClick(I) ;


}
