
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageIo.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageIo.cpp,v 1.1.1.1 2005-06-08 07:08:08 fspindle Exp $
 *
 * Description
 * ============
 *
 * Read/write images
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
    ERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameERR,
			    "filename empty ")) ;
  }

  // Ouverture de l'image.
  if ((fd = fopen(filename, "r")) == NULL)
  {
    ERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioERR,
			    "cannot open file")) ;
  }
  return fd ;
}

FILE *
vpImageIo::openFileWrite(const char filename[FILENAME_MAX],
			 const char *s)
{
  FILE *fd ;

 // Lecture du nom du fichier image.
  if (filename == '\0')
  {
    ERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameERR,
			    "filename empty ")) ;
  }

  // Ouverture de l'image.
  if ((fd = fopen(filename, s)) == NULL)
  {
    ERROR_TRACE("cannot open file") ;
    throw (vpImageException(vpImageException::ioERR,
			    "cannot open file")) ;
  }
  return fd ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
