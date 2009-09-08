/****************************************************************************
 *
 * $Id: vpImageIo.cpp,v 1.8 2008-04-17 12:44:58 asaunier Exp $
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
  \file vpImageIo.cpp
  \brief Read/write images
*/

#include <visp/vpImageIo.h>

/*!

  Open a file with read access.

  \param filename : Name of the file to open.

  \return File descriptor.
*/
FILE *
vpImageIo::openFileRead(const char *filename)
{

  FILE *fd ;

  // Lecture du nom du fichier image.
  if (filename == '\0')   {
    vpERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameError,
			    "filename empty ")) ;
  }

  // Ouverture de l'image.
  if ((fd = fopen(filename, "r")) == NULL)
  {
    vpERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioError,
			    "cannot open file")) ;
  }
  return fd ;
}

/*!

  Open a file with write access.

  \param filename : Name of the file to open.

  \param mode : Access mode. By default set to "w" for write
  access. Could be changed to set for example the access mode to "wa"
  to append data in the file.

  \return File descriptor.
*/
FILE *
vpImageIo::openFileWrite(const char *filename, const char *mode)
{
  FILE *fd ;

 // Lecture du nom du fichier image.
  if (filename == '\0')
  {
    vpERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameError,
			    "filename empty ")) ;
  }

  // Ouverture de l'image.
  if ((fd = fopen(filename, mode)) == NULL)
  {
    vpERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioError,
			    "cannot open file")) ;
  }
  return fd ;
}

/*!

  Open a file with read access.

  \param filename : Name of the file to open.

  \return File descriptor.
*/
FILE *
vpImageIo::openFileRead(const std::string filename)
{

  FILE *fd ;

  // Lecture du nom du fichier image.
  if (filename.empty()) {
    vpERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameError,
			    "filename empty ")) ;
  }
  
  // Ouverture de l'image.
  if ((fd = fopen(filename.c_str(), "r")) == NULL)
  {
    vpERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioError,
			    "cannot open file")) ;
  }
  return fd ;
}

/*!

  Open a file with write access.

  \param filename : Name of the file to open.

  \param mode : Access mode. By default set to "w" for write
  access. Could be changed to set for example the access mode to "wa"
  to append data in the file.

  \return File descriptor.
*/
FILE *
vpImageIo::openFileWrite(const std::string filename,
			 const std::string mode)
{
  FILE *fd ;

 // Lecture du nom du fichier image.
  if (filename.empty())
  {
    vpERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameError,
			    "filename empty ")) ;
  }

  // Ouverture de l'image.
  if ((fd = fopen(filename.c_str(), mode.c_str())) == NULL)
  {
    vpERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioError,
			    "cannot open file")) ;
  }
  return fd ;
}

vpImageIo::vpImageFormatType
vpImageIo::getFormat(const char *filename)
{
  std::string sfilename(filename);

  int PGM = sfilename.find("PGM");
  int pgm = sfilename.find("pgm");
  int PPM = sfilename.find("PPM");
  int ppm = sfilename.find("ppm");
  int JPG = sfilename.find("JPG");
  int jpg = sfilename.find("jpg");
  int JPEG = sfilename.find("JPEG");
  int jpeg = sfilename.find("jpeg");
  int PNG = sfilename.find("PNG");
  int png = sfilename.find("png");
  
  int size = sfilename.size();

  if ((PGM>0 && PGM<size ) || (pgm>0 && pgm<size))
    return FORMAT_PGM;
  else if ((PPM>0 && PPM<size) || ( ppm>0 && ppm<size))
    return FORMAT_PPM;
  else if ((JPG>0 && JPG<size) || ( jpg>0 && jpg<size) || (JPEG>0 && JPEG<size) || ( jpeg>0 && jpeg<size))
	return FORMAT_JPEG;
  else if ((PNG>0 && PNG<size) || ( png>0 && png<size))
    return FORMAT_PNG;
  else{ 
    return FORMAT_UNKNOWN;
  } 
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
