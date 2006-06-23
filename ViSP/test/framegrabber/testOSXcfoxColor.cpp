#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \example testOSXcfoxColor.cpp

  Test frame grabbing capabilities using 1394 device with cfox.
  Only grabbing of color level images is tested.
*/

#if ( defined(APPLE) && defined(VISP_HAVE_CFOX) && defined(VISP_HAVE_GTK) )


#include <visp/vpOSXcfoxGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

#include <visp/vpDisplayGTK.h>

#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>


int
main(int argc, char ** argv)
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  "  test frame grabbing with cfox OSX" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  vpOSXcfoxGrabber g;

  g.setInput(vpOSXcfoxGrabber::DEFAULT_INPUT);
  g.setScale(vpOSXcfoxGrabber::DEFAULT_SCALE);
  g.setFramerate(vpOSXcfoxGrabber::framerate_30fps);
  g.open(I) ;


  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpTRACE(" ") ;

  cout << I.getCols() << "  " << I.getRows() <<endl  ;


  vpDisplayGTK display(I,100,100,"Cfox OSX grabbing... ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


  long cpt = 1;
  while(cpt ++ < 1000)
  {
     double t = vpTime::measureTimeMs();
     g.acquire(I) ;
     vpDisplay::display(I) ;
     vpDisplay::flush(I) ;
     cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
 }

  g.close();



}
#else
int
main()
{
  vpTRACE("Cfox OSX frame grabber drivers are not available or GTK was not activated") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
