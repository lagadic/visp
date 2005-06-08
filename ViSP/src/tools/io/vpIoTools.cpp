
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrixException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpIoTools.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 * io basic tools
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpIoException.h>

void
vpIoTools::checkDirectory(const char *dir )
{
  struct stat stbuf;

  if ( dir == NULL || dir[0] == '\0' ) {
    ERROR_TRACE( " invalid directory name\n" );
    throw(vpIoException(vpIoException::ERRInvalidDirectoryName,
			"invalid directory name")) ;
  }
  if ( stat( dir, &stbuf ) != 0 ) {
    ERROR_TRACE( "can't stat directory '%s' (doesn't exist?)\n", dir );
    throw(vpIoException(vpIoException::ERRCantStatDirectory,
			"can't stat directory")) ;
  }
  if ( (stbuf.st_mode & S_IFDIR) == 0 ) {
    ERROR_TRACE( "'%s' is not a directory\n",  dir );
    throw(vpIoException(vpIoException::ERRNotADirectory,
			"not a directory")) ;
  }
  if ( (stbuf.st_mode & S_IWUSR) == 0 ) {
    ERROR_TRACE( "'%s' is not writable\n", dir );
    throw(vpIoException(vpIoException::ERRNotWritable,
			"Directory not writable")) ;
  }
}

void
vpIoTools::makeDirectory(const  char *dir )
{
  struct stat stbuf;

  if ( dir == NULL || dir[0] == '\0' ) {
    ERROR_TRACE( "invalid directory name\n");
    throw(vpIoException(vpIoException::ERRInvalidDirectoryName,
			"invalid directory name")) ;
  }
  if ( stat( dir, &stbuf ) != 0 ) {
    if ( mkdir( dir, (mode_t)0755 ) != 0 ) {
      ERROR_TRACE("unable to create directory '%s'\n",  dir );
      throw(vpIoException(vpIoException::ERRCantCreateDirectory,
			  "unable to create directory")) ;
    }
    DEBUG_TRACE(2,"has created directory '%s'\n", dir );
  }

  try{
    checkDirectory( dir ) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
