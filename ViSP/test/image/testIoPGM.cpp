
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testIoPGM.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testIoPGM.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
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
  \example testIoPGM.cpp

  \brief  test reading and writting of PGM image
  test the exception :
   read an image that does not exist
   write in a directory that does no exist
 */

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " testIoPGM.cpp" <<endl << endl ;

  cout <<  "  test reading and writting of PGM image" << endl ;
  cout <<  "  read an image that does not exist" << endl ;
  cout <<  "  write in a directory that does no exist" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpImage<unsigned char> I ;

  // test read write unsigned char pgm image.

  vpImageIo::readPGM(I,"images/Klimt.pgm") ;
  vpImageIo::writePGM(I,"images-res/IoPGM.Klimt_char.pgm") ;

  // test io error
  try
  {
    vpImageIo::readPGM(I,"images/image-that-does-not-exist.pgm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }
 /*
 // test io error
  try
  {
    vpImageIo::writePGM(I,"directory-that-does-not-exist/Klimt.pgm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

  cout << "----------------------------------------------------" << endl ;
  vpImage<vpRGBa> Irgba ;

  // test read write unsigned char pgm image.

  vpImageIo::readPGM(Irgba,"images/Klimt.pgm") ;
  vpImageIo::writePGM(Irgba,"images-res/IoPGM.Klimt_rgba.pgm") ;

  // test io error
  try
  {
    vpImageIo::readPGM(Irgba,"images/image-that-does-not-exist.pgm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

 // test io error
  try
  {
    vpImageIo::writePGM(Irgba,"directory-that-does-not-exist/Klimt.pgm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }

  */

}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
