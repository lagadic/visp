
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testIoPPM.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testIoPPM.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *   test reading and writting of PGM image
 *   read an image that does not exist
 *   write in a directory that does no exist
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

/*!
  \example testIoPPM.cpp

  \brief  test reading and writting of PPM image
  test the exception :
  read an image that does not exist
  write in a directory that does no exist
 */

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testIoPPM.cpp" <<endl << endl ;

  cout <<  "  test reading and writting of PPM image" << endl ;
  cout <<  "  read an image that does not exist" << endl ;
  cout <<  "  write in a directory that does no exist" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.

  vpImageIo::readPPM(I,"images/Klimt.ppm") ;
  vpImageIo::writePPM(I,"images-res/IoPPM.Klimt_char.ppm") ;

  // test io error
  try
  {
    vpImageIo::readPPM(I,"images/image-that-does-not-exist.ppm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

 // test io error
  try
  {
    vpImageIo::writePPM(I,"directory-that-does-not-exist/Klimt.ppm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

  cout << "----------------------------------------------------" << endl ;
  vpImage<vpRGBa> Irgba ;

  // test read write unsigned char pgm image.

  vpImageIo::readPPM(Irgba,"images/Klimt.ppm") ;
  vpImageIo::writePPM(Irgba,"images-res/IoPGM.Klimt_rgba.ppm") ;

  // test io error
  try
  {
    vpImageIo::readPPM(Irgba,"images/image-that-does-not-exist.ppm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

 // test io error
  try
  {
    vpImageIo::writePPM(Irgba,"directory-that-does-not-exist/Klimt.ppm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }



}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
