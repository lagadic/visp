/****************************************************************************
 *
 * $Id: vpIoTools.cpp,v 1.8 2006-09-07 09:30:02 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Directory management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#if defined UNIX
#  include <unistd.h>
#elif defined WIN32
#  include <windows.h>
#  include <direct.h>
#endif
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpIoException.h>

/*!
  Get the user name.

  - Under unix, get the content of the LOGNAME environment variable.  For most
    purposes (especially in conjunction with crontab), it is more useful to use
    the environment variable LOGNAME to find out who the user is, rather than
    the getlogin() function.  This is more flexible precisely because the user
    can set LOGNAME arbitrarily.
  - Under windows, uses the GetUserName() function.

  \param username : The user name.

  \exception ERRCantGetUserName : If this method cannot get the user name.
*/
void
vpIoTools::getUserName(string &username)
{
#if defined UNIX
  // Get the user name.
  char *_username = NULL;
  _username = getenv("LOGNAME");
  if (_username == NULL) {
    vpERROR_TRACE( "Cannot get the username. Check your LOGNAME environment variable" );
    throw(vpIoException(vpIoException::ERRCantGetUserName,
			"Cannot get the username")) ;
  }
  username = _username;
#elif defined WIN32
  int info_buffer_size = 1024;
  TCHAR  *infoBuf = new TCHAR [info_buffer_size];
  DWORD  bufCharCount = info_buffer_size;
  // Get the user name.
  if( ! GetUserName( infoBuf, &bufCharCount ) ) {
    delete [] infoBuf;
    vpERROR_TRACE( "Cannot get the username" );
    throw(vpIoException(vpIoException::ERRCantGetUserName,
			"Cannot get the username")) ;

  }
  username = infoBuf;
  delete [] infoBuf;
#endif
}

/*!

  Check if a directory exists.

  \param dirname : Directory to test if it exists.

  \return true : If the directory exists and is accessible with write access.

  \return false : If dirname string is null, or is not a directory, or
  has no write access.

*/
bool
vpIoTools::checkDirectory(const char *dirname )
{
#if defined UNIX
  struct stat stbuf;
#elif defined WIN32
  struct _stat stbuf;
#endif

  if ( dirname == NULL || dirname[0] == '\0' ) {
    return false;
  }

#if defined UNIX
  if ( stat( dirname, &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( dirname, &stbuf ) != 0 )
#endif
  {
    return false;
  }
  if ( (stbuf.st_mode & S_IFDIR) == 0 ) {
    return false;
  }
#if defined UNIX
  if ( (stbuf.st_mode & S_IWUSR) == 0 )
#elif defined WIN32
  if ( (stbuf.st_mode & S_IWRITE) == 0 )
#endif
  {
    return false;
  }
  return true;
}

/*!
  Check if a directory exists.

  \param dirname : Directory to test if it exists.

  \return true : If the directory exists and is accessible with write access.

  \return false : If dirname string is null, or is not a directory, or
  has no write access.

*/
bool
vpIoTools::checkDirectory(const string dirname )
{
  return vpIoTools::checkDirectory(dirname.c_str());
}
/*!

  Create a new directory.

  \param dirname : Directory to create.

  \exception ERRInvalidDirectoryName : If the directory cannot be created.
*/
void
vpIoTools::makeDirectory(const  char *dirname )
{
#if defined UNIX
  struct stat stbuf;
#elif defined WIN32
  struct _stat stbuf;
#endif

  if ( dirname == NULL || dirname[0] == '\0' ) {
    vpERROR_TRACE( "invalid directory name\n");
    throw(vpIoException(vpIoException::ERRInvalidDirectoryName,
			"invalid directory name")) ;
  }
#if defined UNIX
  if ( stat( dirname, &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( dirname, &stbuf ) != 0 )
#endif
  {
#if defined UNIX
    if ( mkdir( dirname, (mode_t)0755 ) != 0 )
#elif defined WIN32
    if ( _mkdir( dirname) != 0 )
#endif
	{
      vpERROR_TRACE("unable to create directory '%s'\n",  dirname );
      throw(vpIoException(vpIoException::ERRCantCreateDirectory,
			  "unable to create directory")) ;
    }
    vpDEBUG_TRACE(2,"has created directory '%s'\n", dirname );
  }

  if ( checkDirectory( dirname ) == false) {
    vpERROR_TRACE("unable to create directory '%s'\n",  dirname );
    throw(vpIoException(vpIoException::ERRCantCreateDirectory,
			"unable to create directory")) ;
  }
}

/*!

  Create a new directory.

  \param dirname : Directory to create.

  \exception ERRInvalidDirectoryName : If the directory cannot be created.
*/
void
vpIoTools::makeDirectory(const string dirname )
{
  try {
    vpIoTools::makeDirectory(dirname.c_str());
  }
  catch (...) {
    throw;
  }
}

/*!
  Converts a pathname to the current system's format.
  \param _p Pathname to convert.
  \return The converted pathname.
*/
string
vpIoTools::path(const char * _p)
{
  string path(_p);

#ifdef WIN32
  for(unsigned int i=0 ; i<path.length() ; i++)
    if( path[i] == '/')	path[i] = '\\';
#else
  for(unsigned int i=0 ; i<path.length() ; i++)
    if( path[i] == '\\')	path[i] = '/';
#endif

  return path;
}


/*!
  Converts a pathname to the current system's format.
  \param _p Pathname to convert.
  \return The converted pathname.
*/
string
vpIoTools::path(const string& _p)
{
  return path(_p.c_str());
}
