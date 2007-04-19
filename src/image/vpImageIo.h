/****************************************************************************
 *
 * $Id: vpImageIo.h,v 1.7 2007-04-19 07:31:14 asaunier Exp $
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
  \brief Read/write images with various image format
*/

class VISP_EXPORT vpImageIo
{

private:
  static const int vpMAX_LEN;

  static FILE * openFileRead(const char filename[FILENAME_MAX]) ;
  static FILE * openFileWrite(const char filename[FILENAME_MAX],
			      const char *mode="w") ;

  static FILE * openFileRead(const string filename) ;
  static FILE * openFileWrite(const string filename, const string mode="w") ;

public:


  //! Read PGM images
  static
  void readPGM(vpImage<unsigned char> &I,
	       const char filename[FILENAME_MAX]) ;

  //! Read PGM images
  static
  void readPGM(vpImage<vpRGBa> &I,
	       const char filename[FILENAME_MAX]) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<unsigned char> &I,
		const char filename[FILENAME_MAX]) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<short> &I,
		const char filename[FILENAME_MAX]) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<vpRGBa> &I,
		const char filename[FILENAME_MAX]) ;


  //! Read PGM images
  static
  void readPGM(vpImage<unsigned char> &I,
	       const string filename) ;

  //! Read PGM images
  static
  void readPGM(vpImage<vpRGBa> &I,
	       const string filename) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<unsigned char> &I,
		const string filename) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<short> &I,
		const string filename) ;
  //! Write PGM images
  static
  void writePGM(const vpImage<vpRGBa> &I,
		const string filename) ;



  //! Read PPM images
  static
  void readPPM(vpImage<unsigned char> &I,
	       const char filename[FILENAME_MAX]) ;
  //! Read PPM images
  static
  void readPPM(vpImage<vpRGBa> &I,
	       const char filename[FILENAME_MAX]) ;
  //! Write PPM images
  static
  void writePPM(const vpImage<unsigned char> &I,
		const char filename[FILENAME_MAX]) ;
  //! Write PPM images
  static
  void writePPM(const vpImage<vpRGBa> &I,
		const char filename[FILENAME_MAX]) ;

  //! Read PPM images
  static
  void readPPM(vpImage<unsigned char> &I,
	       const string filename) ;
  //! Read PPM images
  static
  void readPPM(vpImage<vpRGBa> &I,
	       const string filename) ;
  //! Write PPM images
  static
  void writePPM(const vpImage<unsigned char> &I,
		const string filename) ;
  //! Write PPM images
  static
  void writePPM(const vpImage<vpRGBa> &I,
		const string filename) ;

  } ;
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
