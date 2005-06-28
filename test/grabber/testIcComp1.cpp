#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef HAVE_FG_ICCOMP

#include <visp/vpIcCompGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!
  \example testTrackDot.cpp

  \brief   test dot tracking on an image sequence
*/

int
main(int argc, char ** argv)
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  "  test frame grabbing" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;

  int fps = 25;

  vpArgvInfo argTable[] =
    {
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL," test frame grabbing "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {"-fps", ARGV_INT, (char *) 1, (char *) &fps,
      "Frame per second (25 or 50)."},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_END, NULL,NULL,NULL}
    } ;
  //Parsing of the table
  if (vpParseArgv(&argc,argv,argTable,0))
  {
    cout << endl << "Usage : " << argv[0] << "  [-help] [-fps 50] [-fps 25] "<<endl ;
    exit(1) ;
  }

  vpImage<unsigned char> I ;


  vpIcCompGrabber g(2) ;
  g.open(I) ;
  if (fps == 25)
    g.setFramerate(vpIcCompGrabber::framerate_25fps);
  else
    g.setFramerate(vpIcCompGrabber::framerate_50fps);

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }


  cout << I.getCols() << "  " << I.getRows() <<endl  ;

  TRACE(" ") ;

  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
  TRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  TRACE(" ") ;

  long cpt = 1;
  while(cpt ++ < 100)
  {
    long t = vpTime::measureTimeMs();
    g.acquire(I) ;
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
  }
}
#else
int
main()
{
  TRACE("ICcomp frame grabber drivers are not available") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
