/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

  size_t PGM = sfilename.find("PGM");
  size_t pgm = sfilename.find("pgm");
  size_t PPM = sfilename.find("PPM");
  size_t ppm = sfilename.find("ppm");
  size_t JPG = sfilename.find("JPG");
  size_t jpg = sfilename.find("jpg");
  size_t JPEG = sfilename.find("JPEG");
  size_t jpeg = sfilename.find("jpeg");
  size_t PNG = sfilename.find("PNG");
  size_t png = sfilename.find("png");
  
  size_t size = sfilename.size();

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
