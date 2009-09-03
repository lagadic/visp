/****************************************************************************
 *
 * $Id: vpImageIo.h,v 1.12 2008-11-12 17:36:24 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
 * Read/write images.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpImageIo.h
  \brief Read/write images
*/

#ifndef vpIMAGEIO_H
#define vpIMAGEIO_H

#include <stdio.h>
#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

#if defined(VISP_HAVE_LIBJPEG)
#include <jpeglib.h>
#include <jerror.h>
#endif

/*!
  \class vpImageIo
  
  \ingroup ImageRW

  \brief Read/write images with various image format.

  The code below shows how to convert an PPM P6 image file format into
  a PGM P5 image file format. The extension of the filename is here
  used in read() and write() functions to set the image file format
  (".pgm" for PGM P5 and ".ppm" for PPM P6).

  \code
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

int main()
{
  vpImage<unsigned char> I;
#ifdef UNIX
  std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.ppm");
#elif WIN32
  std::string filename("C:/temp/ViSP-images/Klimt/Klimt.ppm");
#endif

  vpImageIo::read(I, filename); // Convert the color image in a gray level image
  vpImageIo::write(I, "Klimt.pgm"); // Write the image in a PGM P5 image file format 
}
  \endcode
*/

class VISP_EXPORT vpImageIo
{

private:
  
  typedef enum
  {
    FORMAT_PGM,
    FORMAT_PPM,
	FORMAT_JPEG,
    FORMAT_UNKNOWN
  } vpImageFormatType;
  
  static const int vpMAX_LEN;

  static FILE * openFileRead(const char *filename) ;
  static FILE * openFileWrite(const char *filename, const char *mode="w") ;

  static FILE * openFileRead(const std::string filename) ;
  static FILE * openFileWrite(const std::string filename, 
			      const std::string mode="w") ;

  static vpImageFormatType getFormat(const char *filename) ;
public:

  static
  void read(vpImage<unsigned char> &I, const char *filename) ;
  static
  void read(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void read(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void read(vpImage<vpRGBa> &I, const std::string filename) ;
  
  static
  void write(vpImage<unsigned char> &I, const char *filename) ;
  static
  void write(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void write(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void write(vpImage<vpRGBa> &I, const std::string filename) ;

  static
  void readPGM(vpImage<unsigned char> &I, const char *filename) ;
  static
  void readPGM(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void readPGM(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void readPGM(vpImage<vpRGBa> &I, const std::string filename) ;

  static
  void readPPM(vpImage<unsigned char> &I, const char *filename) ;
  static
  void readPPM(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void readPPM(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void readPPM(vpImage<vpRGBa> &I, const std::string filename) ;

#if defined(VISP_HAVE_LIBJPEG)
  static
  void readJPEG(vpImage<unsigned char> &I, const char *filename) ;
  static
  void readJPEG(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void readJPEG(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void readJPEG(vpImage<vpRGBa> &I, const std::string filename) ;
#endif

  static
  void writePGM(const vpImage<unsigned char> &I, const char *filename) ;
  static
  void writePGM(const vpImage<unsigned char> &I, const std::string filename) ;
  static
  void writePGM(const vpImage<short> &I, const char *filename) ;
  static
  void writePGM(const vpImage<short> &I, const std::string filename) ;
  static
  void writePGM(const vpImage<vpRGBa> &I, const char *filename) ;
  static
  void writePGM(const vpImage<vpRGBa> &I, const std::string filename) ;

  static
  void writePPM(const vpImage<unsigned char> &I, const char *filename) ;
  static
  void writePPM(const vpImage<unsigned char> &I, const std::string filename) ;
  static
  void writePPM(const vpImage<vpRGBa> &I, const char *filename) ;
  static
  void writePPM(const vpImage<vpRGBa> &I, const std::string filename) ;

#if defined(VISP_HAVE_LIBJPEG)
  static
  void writeJPEG(vpImage<unsigned char> &I, const char *filename) ;
  static
  void writeJPEG(vpImage<unsigned char> &I, const std::string filename) ;
  static
  void writeJPEG(vpImage<vpRGBa> &I, const char *filename) ;
  static
  void writeJPEG(vpImage<vpRGBa> &I, const std::string filename) ;
#endif

  } ;
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
