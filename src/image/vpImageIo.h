/****************************************************************************
 *
 * $Id: vpImageIo.h,v 1.11 2008-09-26 15:20:54 fspindle Exp $
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


/*!
  \class vpImageIo
  
  \ingroup ImageRW

  \brief Read/write images with various image format.
*/

class VISP_EXPORT vpImageIo
{

private:
  
  typedef enum
  {
    FORMAT_PGM,
    FORMAT_PPM,
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

  } ;
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
