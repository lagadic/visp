#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \example testV4l2Grey.cpp

  Test frame grabbing capabilities using video 4 linux two video device.
  Only grabbing of grey level images is tested.
*/

#ifdef VISP_HAVE_V4L2

#include <visp/vpV4l2Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>


int
main(int argc, char ** argv)
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  "  test frame grabbing with Video 4 Linux 2" << endl ;
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


  vpV4l2Grabber g;

  g.setInput(vpV4l2Grabber::DEFAULT_INPUT);
  g.setScale(vpV4l2Grabber::DEFAULT_SCALE);
  g.setFramerate(vpV4l2Grabber::framerate_25fps);
  g.open(I) ;

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


  cout << I.getCols() << "  " << I.getRows() <<endl  ;

  vpTRACE(" ") ;

  vpDisplayX display(I,100,100,"Video4Linux2 grabbing... ") ;
  vpTRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpTRACE(" ") ;

  long cpt = 1;
  while(cpt ++ < 100)
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
  vpTRACE("Video 4 Linux 2 frame grabber drivers are not available") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
