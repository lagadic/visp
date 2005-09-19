
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageIO.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageIo.h,v 1.2 2005-09-19 13:39:02 fspindle Exp $
 *
 * Description
 * ============
 *
 * Read/write images
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpImageIo.h
  \brief Read/write images
*/

#ifndef vpIMAGEIO_H
#define vpIMAGEIO_H

#include <stdio.h>

// image
#include <visp/vpImage.h>

// color
#include <visp/vpRGBa.h>


/*!
  \class vpImageIo
  \brief Read/write images with various image format
*/

class vpImageIo
{

private:
  static FILE * openFileRead(const char filename[FILENAME_MAX]) ;
  static FILE * openFileWrite(const char filename[FILENAME_MAX],
			      const char *s="w") ;

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
		const char *filename) ;




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
		const char *filename) ;
  } ;
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
