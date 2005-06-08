
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:     testDisplay1.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testDisplayX1.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
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


#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>

/*!
  \example testDisplay1.cpp

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
  cout <<  " testDisplay1.cpp" <<endl << endl ;
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



  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.


  vpImageIo::readPGM(I,"images/Klimt.pgm") ;


  vpDisplayX display(I,100,100,"testDisplayX.cpp") ;


  vpDisplay::display(I) ;

  vpDisplay::displayCross(I, 10,10, 10, vpColor::red) ;

  for (int i=0 ; i < I.getRows() ; i+=20)
    vpDisplay::displayLine(I,i,0,i,I.getCols(), vpColor::red) ;



  for (int i=0 ; i < I.getCols() ; i+=20)
    vpDisplay::displayDotLine(I,0,i,I.getCols(), i,vpColor::green) ;

  vpDisplay::displayArrow(I,0,0,100,100,vpColor::blue) ;
  for (int i=0 ; i < 100 ; i+=20)
    vpDisplay::displayCircle(I,200,200,20+i,vpColor::yellow) ;

  vpDisplay::displayCharString(I,100,100,
			       "ViSP is a marvelous software",
			       vpColor::yellow) ;

  vpImage<vpRGBa> Iaug ;
  vpDisplay::getImage(I,Iaug) ;
  vpImageIo::writePPM(Iaug,"images-res/DisplayX1.Klimt-augmented.ppm") ;

  vpDisplay::getClick(I) ;
  vpDisplay::flush(I) ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
