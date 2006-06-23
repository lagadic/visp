
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
 *  $Id: testDisplayX2.cpp,v 1.8 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *  read a pgm image
 *  open X display
 *  display red lines on the image
 *  wait for a mouse click
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>

/*!
  \example testDisplayX2.cpp

  \brief
   read a pgm image
   open X display
   display red lines on the image
   wait for a mouse click

   */

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testDisplayX2cpp" <<endl << endl ;
  cout << endl ;
  cout <<  " test the vpDisplayX class " << endl ;
  cout << endl ;
  cout <<  "  read a pgm image" << endl ;
  cout <<  "  open X display" << endl ;
  cout <<  "  display red lines on the image" << endl ;
  cout <<  "  display green dotted lines on the image" << endl ;
  cout <<  "  display circles, arrow, string on the image" << endl ;
  cout <<  "  get and save the pixmap " << endl ;

  cout <<  "  wait for a mouse click " << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;



  vpImage<vpRGBa> I ;
  vpImage<vpRGBa> I2 ;

  // test read write vpRGBa pgm image.

  try{
    vpImageIo::readPPM(I,"images/Klimt.ppm") ;
    vpImageIo::readPPM(I2,"images/Klimt.ppm") ;
  }
  catch(...)
  {
    throw ;
  }

  vpDisplayX display(I,00,00,"Display1...") ;
  vpDisplayX display2(I2,200,200,"Display2...") ;


  vpImageIo::readPPM(I,"images/Klimt.ppm") ;
  vpDisplay::display(I) ;
  vpImageIo::readPPM(I2,"images/Klimt.ppm") ;
  vpDisplay::display(I2) ;

  for (int i=0 ; i < I.getRows() ; i+=20)
    vpDisplay::displayLine(I,i,0,i,I.getCols(), vpColor::red) ;



  for (int i=0 ; i < I.getCols() ; i+=20)
    vpDisplay::displayDotLine(I,0,i,I.getCols(), i,vpColor::green) ;

  vpDisplay::displayArrow(I,0,0,100,100,vpColor::blue) ;
  for (int i=0 ; i < 100 ; i+=20)
    vpDisplay::displayCircle(I,200,200,20+i,vpColor::yellow) ;

  vpDisplay::displayCharString(I,100,100,
			       "ViSP is a marvelous software",
			       vpColor::blue) ;

#if 0
  cout << "\nA click to display a point..." << endl;

  vpDisplay::getClick(I);
  vpDisplay::displayCross(I, 50, 50, 3, vpColor::red);

  vpImage<vpRGBa> Iaug ;
  vpDisplay::getImage(I,Iaug) ;
  vpImageIo::writePPM(Iaug,"images-res/DisplayX2.Klimt-augmented.ppm") ;


  cout << "\nA click to exit..." << endl;
  vpDisplay::getClick(I) ;
#else
  // Suppress the getClick() call for automate test
  vpDisplay::displayCross(I, 50, 50, 3, vpColor::red);

  vpImage<vpRGBa> Iaug ;
  vpDisplay::getImage(I,Iaug) ;
  vpImageIo::writePPM(Iaug,"images-res/DisplayX2.Klimt-augmented.ppm") ;
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
