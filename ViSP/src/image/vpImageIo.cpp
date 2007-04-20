/****************************************************************************
 *
 * $Id: vpImageIo.cpp,v 1.6 2007-04-20 14:22:16 asaunier Exp $
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


FILE *
vpImageIo::openFileRead(const char filename[FILENAME_MAX])
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

FILE *
vpImageIo::openFileWrite(const char filename[FILENAME_MAX],
			 const char *mode)
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




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
