
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:     testDisplayX2.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testDisk.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *  read a pgm image from the disk
 *  open X display
 *  display red lines on the image
 *  wait for a mouse click
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpDiskGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>

/*!
  \example testDisk.cpp

  \brief   Test image sequence acquisition from disk..
*/

int
main(int argc, char ** argv)
{
  char *directory = new char [FILENAME_MAX];
  char *basename  = new char [FILENAME_MAX];
  char *_directory = NULL;
  char *_basename = NULL;

  sprintf(directory, "/udd/marchand/dd/t/images/pattern/mire-2");
  sprintf(basename, "image.");

  _directory = directory;
  _basename  = basename;

  vpArgvInfo argTable[] =
    {
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL," test disk frame grabbing "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {"-dir", ARGV_STRING, (char *) 1, (char *) &_directory,
      "Image sequence directory.\n"
       "         Default value:"},
      {"-basename", ARGV_STRING, (char *) 1, (char *) &_basename,
      "Image base name.\n"
       "         Default value:"},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_END, NULL,NULL,NULL}
    } ;
  //Parsing of the table
  if (vpParseArgv(&argc,argv,argTable,0))
  {
    cout << endl << "Usage : " << argv[0] << "  [-help] "<<endl ;
    delete [] directory;
    delete [] basename;
    exit(1) ;
  }

  if (_directory != NULL)
    sprintf(directory, "%s", _directory);
  if (_basename != NULL)
    sprintf(basename, "%s", _basename);

  vpImage<unsigned char> I ;


  vpDiskGrabber g;

  g.setDirectory(directory);
  g.setBaseName(basename);
  g.setStep(1);
  g.setNumberOfZero(4);
  g.setImageNumber(4);

  g.open(I) ;


  cout << I.getCols() << "  " << I.getRows() <<endl  ;

  vpTRACE(" ") ;

  vpDisplayX display(I,100,100,"Display...") ;
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

    // cout << "a click to continue..." << endl;
    //vpDisplay::getClick(I);
    cout << "time: " << vpTime::measureTimeMs() - t << " (ms)" << endl;
  }

  delete [] directory;
  delete [] basename;

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
