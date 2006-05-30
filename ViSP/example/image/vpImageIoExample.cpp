/****************************************************************************
 *
 * $Id: vpImageIoExample.cpp,v 1.2 2006-05-30 08:42:16 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Reading and writting images on the disk.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


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
