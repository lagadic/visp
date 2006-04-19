
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
 *  $Id: vpIoTools.cpp,v 1.3 2006-04-19 09:01:22 fspindle Exp $
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
#if defined UNIX
#  include <unistd.h>
#elif defined WIN32
#  include <direct.h>
#endif
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpIoException.h>

void
vpIoTools::checkDirectory(const char *dir )
{
#if defined UNIX
  struct stat stbuf;
#elif defined WIN32
  struct _stat stbuf;
#endif

  if ( dir == NULL || dir[0] == '\0' ) {
    ERROR_TRACE( " invalid directory name\n" );
    throw(vpIoException(vpIoException::ERRInvalidDirectoryName,
			"invalid directory name")) ;
  }

#if defined UNIX
  if ( stat( dir, &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( dir, &stbuf ) != 0 )
#endif
  {
    ERROR_TRACE( "can't stat directory '%s' (doesn't exist?)\n", dir );
    throw(vpIoException(vpIoException::ERRCantStatDirectory,
			"can't stat directory")) ;
  }
  if ( (stbuf.st_mode & S_IFDIR) == 0 ) {
    ERROR_TRACE( "'%s' is not a directory\n",  dir );
    throw(vpIoException(vpIoException::ERRNotADirectory,
			"not a directory")) ;
  }
#if defined UNIX
  if ( (stbuf.st_mode & S_IWUSR) == 0 )
#elif defined WIN32
  if ( (stbuf.st_mode & S_IWRITE) == 0 )
#endif
  {
    ERROR_TRACE( "'%s' is not writable\n", dir );
    throw(vpIoException(vpIoException::ERRNotWritable,
			"Directory not writable")) ;
  }
}

void
vpIoTools::makeDirectory(const  char *dir )
{
#if defined UNIX
  struct stat stbuf;
#elif defined WIN32
  struct _stat stbuf;
#endif

  if ( dir == NULL || dir[0] == '\0' ) {
    ERROR_TRACE( "invalid directory name\n");
    throw(vpIoException(vpIoException::ERRInvalidDirectoryName,
			"invalid directory name")) ;
  }
#if defined UNIX
  if ( stat( dir, &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( dir, &stbuf ) != 0 )
#endif  
  {
#if defined UNIX
    if ( mkdir( dir, (mode_t)0755 ) != 0 )
#elif defined WIN32
    if ( _mkdir( dir) != 0 )
#endif  
	{
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

