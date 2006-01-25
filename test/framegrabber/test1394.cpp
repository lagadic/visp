#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if ( defined (HAVE_LIBDC1394_CONTROL) & defined(HAVE_LIBRAW1394) )

#include <visp/vp1394Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!
  \example test1394.cpp

  Test frame grabbing capabilities using ieee 1394 video device.
*/

int
main(int argc, char ** argv)
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  "  test frame grabbing with ieee 1394" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;

  int fps = 30;

  vpArgvInfo argTable[] =
    {
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL," test frame grabbing "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {"-fps", ARGV_INT, (char *) 1, (char *) &fps,
      "Frame per second (7, 15 or 30)."},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_END, NULL,NULL,NULL}
    } ;
  //Parsing of the table
  if (vpParseArgv(&argc,argv,argTable,0))
  {
    cout << endl << "Usage : " << argv[0] << "  [-help] [-fps 7] [-fps 15] [-fps 30] "<<endl ;
    exit(1) ;
  }

  vpImage<unsigned char> I ;


  vp1394Grabber g ;

  g.open(I) ;

  switch (fps) {
  case 7:
    CTRACE << "Framerate is set to 7.5 fps" << endl;
    g.setFramerate(FRAMERATE_7_5);
    break;
  case 15:
    CTRACE << "Framerate is set to 15 fps" << endl;
    g.setFramerate(FRAMERATE_15);
    break;
  case 30:
    CTRACE << "Framerate is set to 30 fps" << endl;
    g.setFramerate(FRAMERATE_30);
    break;
  default:
    CTRACE << "Use of default framerate" << endl;
  }


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

  vpDisplayX display(I,100,100,"ieee 1394 grabbing... ") ;
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
    double t = vpTime::measureTimeMs();
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
  TRACE("Ieee 1394 grabber capabilities are not available...") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
