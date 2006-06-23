#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_DC1394

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

  int req_fps = 30;
  unsigned int req_shutter = 0;
  unsigned int req_gain = 0;

  vpArgvInfo argTable[] =
    {
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL," test frame grabbing "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {"-fps", ARGV_INT, (char *) 1, (char *) &req_fps,
      "Set frame per second to 7, 15 or 30 fps."},
      {"-shutter", ARGV_INT, (char *) 1, (char *) &req_shutter,
      "Set shutter to the requested value.\n"
      "           If 0, the shutter is not modified"},
      {"-gain", ARGV_INT, (char *) 1, (char *) &req_gain,
      "Set gain to the requested value.\n"
      "           If 0, the gain is not modified"},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_END, NULL,NULL,NULL}
    } ;
  //Parsing of the table
  if (vpParseArgv(&argc,argv,argTable,0))
  {
    cout << endl << "Usage : " << argv[0]
	 << " [-fps <7|15|30]" << endl
	 << " [-shutter <value>] "
	 << " [-gain <value>] "
	 << " [-help] " << endl << endl;

    exit(1) ;
  }

  vpImage<unsigned char> I ;


  vp1394Grabber g ;

  g.open(I) ;

  unsigned int cameras;
  g.getNumCameras(cameras);
  vpTRACE("Number of cameras on the bus: %d\n", cameras);

  switch (req_fps) {
  case 7:
    vpTRACE("Framerate is set to 7.5 fps");
    g.setFramerate(FRAMERATE_7_5);
    break;
  case 15:
    vpTRACE("Framerate is set to 15 fps");
    g.setFramerate(FRAMERATE_15);
    break;
  case 30:
    vpTRACE("Framerate is set to 30 fps");
    g.setFramerate(FRAMERATE_30);
    break;
  default:
    vpCTRACE << "Use of default framerate" << endl;
  }

  if (req_shutter) {
    g.setShutter(req_shutter);
    vpTRACE("Set shutter to : %d", req_shutter);
  }

  unsigned int min_shutter, shutter, max_shutter;
  g.getShutter(min_shutter, shutter, max_shutter);

  vpTRACE("Shutter: %d < %d < %d", min_shutter, shutter, max_shutter);

  if (req_gain) {
    g.setGain(req_gain);
    vpTRACE("Set gain to : %d", req_gain);
  }

  unsigned int min_gain, gain, max_gain;
  g.getGain(min_gain, gain, max_gain);

  vpTRACE("Gain: %d < %d < %d", min_gain, gain, max_gain);

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


  vpTRACE("Image size: %d cold %d rows", I.getCols(), I.getRows() );

  vpTRACE(" ") ;

  vpDisplayX display(I,100,100,"ieee 1394 grabbing... ") ;
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
}
#else
int
main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available...") ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
