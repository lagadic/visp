
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageIoExample.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageIoExample.cpp,v 1.1 2005-11-30 11:15:59 marchand Exp $
 *
 * Description
 * ============
 *   test reading and writting of PPM color image
 *   read an image that does not exist
 *   write in a directory that does no exist
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

/*!
  \example vpImageIoExample.cpp

  \brief   reading and writting of PPM image
  example of exception handling the exception :
  read an image that does not exist
  write in a directory that does no exist
 */

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " vpImageIoExample.cpp" <<endl << endl ;

  cout <<  "  reading and writting of PPM image" << endl ;
  cout <<  "  read an image that does not exist" << endl ;
  cout <<  "  write in a directory that does no exist" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;

  // First we wanted to have gray level image (8bits)
  // vpImage is a template class you can declare vpImage of ... everything...
  vpImage<unsigned char> I ;

  // Although I is a gray level image you can read and write
  // color image. Obviously the color will be translated as a gray level
  vpImageIo::readPPM(I,"images/Klimt.ppm") ;
  vpImageIo::writePPM(I,"images-res/IoPPM.Klimt_char.ppm") ;

  // test io error
  // if the image you want to read on the disk does not exist
  // an exception is thrown
  try
  {
    vpImageIo::readPPM(I,"images/image-that-does-not-exist.ppm") ;
  }
  catch(vpImageException e)
  {
    ERROR_TRACE("at main level");
    cout << e << endl ;
  }
  
  // same thing if you to write in a directory that does not exist
  // or where you are not allowd to write.
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
 
 // Let's consider that the image is now a color image (32 bits RGBa)
  vpImage<vpRGBa> Irgba ;

  // read write unsigned char pgm image.
  // the color image is now load as is
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
