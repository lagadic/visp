
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:     testDisplayX1.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testDisplayGTK.cpp,v 1.1 2006-05-10 13:01:28 fspindle Exp $
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

#ifdef VISP_HAVE_GTK

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGTK.h>

/*!
  \example testDisplayGTK.cpp

  \brief
   read a pgm image
   open GTK display
   display red lines on the image
   wait for a mouse click

   */

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testDisplayGTK.cpp" <<endl << endl ;
  cout << endl ;
  cout <<  " test the vpDisplayGTK class " << endl ;
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


  vpDisplayGTK display(I,100,100,"testDisplayX.cpp") ;


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

#if 0
  cout << "\nA click to close the windows..." << endl;
  vpDisplay::getClick(I) ;
#endif
  vpDisplay::close(I);

  TRACE("-------------------------------------");
  vpImage<vpRGBa> Irgba ;

  vpImageIo::readPGM(Irgba,"images/Klimt.pgm") ;
  char name[100];

  sprintf(name, "Visualisation image : Klimt.ppm");
  vpDisplayGTK displayRGBa(Irgba,100,100,name);
  vpDisplay::display(Irgba) ;

#if 0
  cout << "\nA click to display a point..." << endl;

  {
    int i,j;
    vpDisplay::getClick(Irgba,i,j);
    vpDisplay::displayCross(Irgba,i,j,15,vpColor::red);
    cout << "Cood: " << i << ", " << j << endl;
  }
  cout << "Click to flush..." <<endl;
  vpDisplay::getClick(Irgba);
  cout << "flush the display" << endl;
  //vpDisplay::flush(Irgba);

  cout << "\nA click to exit the program..." << endl;
  vpDisplay::getClick(Irgba) ;
  cout << "Bye" << endl;

#else

  {
    int i=10,j=20;
    vpDisplay::displayCross(Irgba,i,j,15,vpColor::red);
    cout << "Cood: " << i << ", " << j << endl;
  }

#endif
}
#else
int
main()
{
  ERROR_TRACE("You do not have GTK functionalities to display images...");
}

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
