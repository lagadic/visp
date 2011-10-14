/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Directory management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpIoTools.cpp
  \brief File and directories basic tools.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

  \exception vpIoException::cantGetUserName : If this method cannot get the
  user name.

  \sa getUserName()
*/
void
vpIoTools::getUserName(std::string &username)
{
#if defined UNIX
  // Get the user name.
  char *_username = NULL;
  _username = ::getenv("LOGNAME");
  if (_username == NULL) {
    vpERROR_TRACE( "Cannot get the username. Check your LOGNAME environment variable" );
    throw(vpIoException(vpIoException::cantGetUserName,
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
    throw(vpIoException(vpIoException::cantGetUserName,
			"Cannot get the username")) ;

  }
  username = infoBuf;
  delete [] infoBuf;
#endif
}
/*!
  Get the user name.

  - Under unix, get the content of the LOGNAME environment variable.  For most
    purposes (especially in conjunction with crontab), it is more useful to use
    the environment variable LOGNAME to find out who the user is, rather than
    the getlogin() function.  This is more flexible precisely because the user
    can set LOGNAME arbitrarily.
  - Under windows, uses the GetUserName() function.

  \return The user name.

  \exception vpIoException::cantGetUserName : If this method cannot get the
  user name.

  \sa getUserName(std::string &)
*/
std::string
vpIoTools::getUserName()
{
  std::string username;
#if defined UNIX
  // Get the user name.
  char *_username = NULL;
  _username = ::getenv("LOGNAME");
  if (_username == NULL) {
    vpERROR_TRACE( "Cannot get the username. Check your LOGNAME environment variable" );
    throw(vpIoException(vpIoException::cantGetUserName,
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
    throw(vpIoException(vpIoException::cantGetUserName,
			"Cannot get the username")) ;

  }
  username = infoBuf;
  delete [] infoBuf;
#endif
  return username;
}

/*!
  Get the content of an environment variable.

  \warning Under windows, this function is not implemented yet.

  \param env : Environment variable name (HOME, LOGNAME...).
  \return Value of the environment variable

  \exception vpException::notImplementedError : If this method is
  called under Windows.

  \exception vpIoException::cantGetenv : If an error occur while
  getting the environement variable value.

  \code
#include <iostream>
#include <string>
#include <visp/vpIoTools.h>

int main()
{
  std::string envvalue;
  try {
    envvalue = vpIoTools::getenv("HOME");
    std::cout << "$HOME = \"" << envvalue << "\"" << std::endl;
  }
  catch (...) {
    std::cout << "Cannot get the environment variable value" << std::endl;
    return -1;
  }
  return 0;
}
  \endcode

  \sa getenv(std::string &)
*/
std::string
vpIoTools::getenv(const char *env)
{
  std::string value;
#if defined UNIX
  // Get the environment variable value.
  char *_value = NULL;
  _value = ::getenv(env);
  if (_value == NULL) {
    vpERROR_TRACE( "Cannot get the environment variable value" );
    throw(vpIoException(vpIoException::cantGetenv,
			"Cannot get the environment variable value")) ;
  }
  value = _value;
#elif defined WIN32

  vpERROR_TRACE( "Not implemented!" );
  throw(vpIoException(vpException::notImplementedError,
		      "Not implemented!")) ;
#endif
  return value;
}

/*!
  Get the content of an environment variable.

  \warning Under windows, this function is not implemented yet.

  \param env : Environment variable name (HOME, LOGNAME...).
  \return Value of the environment variable

  \exception vpException::notImplementedError : If this method is
  called under Windows.

  \exception vpIoException::cantGetenv : If an error occur while
  getting the environement variable value.

  \code
#include <iostream>
#include <string>
#include <visp/vpIoTools.h>

int main()
{
  std::string envvalue;
  try {
    std::string env = "HOME";
    envvalue = vpIoTools::getenv(env);
    std::cout << "$HOME = \"" << envvalue << "\"" << std::endl;
  }
  catch (...) {
    std::cout << "Cannot get the environment variable value" << std::endl;
    return -1;
  }
  return 0;
}
  \endcode

  \sa getenv(const char *)
*/
std::string
vpIoTools::getenv(std::string &env)
{
  return (vpIoTools::getenv(env.c_str()));
}

/*!

  Check if a directory exists.

  \param dirname : Directory to test if it exists. The directory name
  is converted to the current system's format; see path().

  \return true : If the directory exists and is accessible with write access.

  \return false : If dirname string is null, or is not a directory, or
  has no write access.

  \sa checkDirectory(const std::string &)
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

  std::string _dirname = path(dirname);

#if defined UNIX
  if ( stat( _dirname.c_str(), &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( _dirname.c_str(), &stbuf ) != 0 )
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

  \param dirname : Directory to test if it exists. The directory name
  is converted to the current system's format; see path().

  \return true : If the directory exists and is accessible with write access.

  \return false : If dirname string is null, or is not a directory, or
  has no write access.

  \sa checkDirectory(const char *)
*/
bool
vpIoTools::checkDirectory(const std::string &dirname )
{
  return vpIoTools::checkDirectory(dirname.c_str());
}
/*!

  Create a new directory.

  \param dirname : Directory to create. The directory name
  is converted to the current system's format; see path().

  \exception vpIoException::invalidDirectoryName : The \e dirname is invalid.

  \exception vpIoException::cantCreateDirectory : If the directory cannot be
  created.

  \sa makeDirectory(const std::string &)
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
    throw(vpIoException(vpIoException::invalidDirectoryName,
			"invalid directory name")) ;
  }

  std::string _dirname = path(dirname);

#if defined UNIX
  if ( stat( _dirname.c_str(), &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( _dirname.c_str(), &stbuf ) != 0 )
#endif
  {
#if defined UNIX
    if ( mkdir( _dirname.c_str(), (mode_t)0755 ) != 0 )
#elif defined WIN32
    if ( _mkdir( _dirname.c_str()) != 0 )
#endif
	{
      vpERROR_TRACE("unable to create directory '%s'\n",  dirname );
      throw(vpIoException(vpIoException::cantCreateDirectory,
			  "unable to create directory")) ;
    }
    vpDEBUG_TRACE(2,"has created directory '%s'\n", dirname );
  }

  if ( checkDirectory( dirname ) == false) {
    vpERROR_TRACE("unable to create directory '%s'\n",  dirname );
    throw(vpIoException(vpIoException::cantCreateDirectory,
			"unable to create directory")) ;
  }
}

/*!

  Create a new directory.

  \param dirname : Directory to create. The directory name
  is converted to the current system's format; see path().

  \exception vpIoException::cantCreateDirectory : If the directory cannot be
  created.

  \sa makeDirectory(const  char *)
*/
void
vpIoTools::makeDirectory(const std::string &dirname )
{
  try {
    vpIoTools::makeDirectory(dirname.c_str());
  }
  catch (...) {
    vpERROR_TRACE("unable to create directory '%s'\n",dirname.c_str());
    throw(vpIoException(vpIoException::cantCreateDirectory,
			"unable to create directory")) ;
  }
}

/*!

  Check if a file exists.

  \param filename : Filename to test if it exists.

  \return true : If the filename exists and is accessible with read access.

  \return false : If filename string is null, or is not a filename, or
  has no read access.

  \sa checkFilename(const std::string &)
*/
bool
vpIoTools::checkFilename(const char *filename)
{
#if defined UNIX
  struct stat stbuf;
#elif defined WIN32
  struct _stat stbuf;
#endif

  if ( filename == NULL || filename[0] == '\0' ) {
    return false;
  }

  std::string _filename = path(filename);
#if defined UNIX
  if ( stat( _filename.c_str(), &stbuf ) != 0 )
#elif defined WIN32
  if ( _stat( _filename.c_str(), &stbuf ) != 0 )
#endif
  {
    return false;
  }
  if ( (stbuf.st_mode & S_IFREG) == 0 ) {
    return false;
  }
#if defined UNIX
  if ( (stbuf.st_mode & S_IRUSR) == 0 )
#elif defined WIN32
  if ( (stbuf.st_mode & S_IREAD) == 0 )
#endif
  {
    return false;
  }
  return true;
}

/*!
  Check if a file exists.

  \param filename : Filename to test if it exists.

  \return true : If the filename exists and is accessible with read access.

  \return false : If filename string is null, or is not a filename, or
  has no read access.

  \sa checkFilename(const char *)
*/
bool
vpIoTools::checkFilename(const std::string &filename)
{
  return vpIoTools::checkFilename(filename.c_str());
}

/*!

  Remove a file.

  \param filename : File to remove.

  \return true if the file was removed, false otherwise.

  \sa remove(const std::string &)
*/
bool
vpIoTools::remove(const char *filename)
{
  if (remove(filename) != 0)
    return false;
  else
    return true;
}
/*!

  Remove a file.

  \param filename : File to remove.

  \return true if the file was removed, false otherwise.

  \sa remove(const char *)
*/
bool
vpIoTools::remove(const std::string &filename)
{
  return vpIoTools::remove(filename.c_str());
}

/*!
  Converts a path name to the current system's format.

  \param pathname : Path name to convert. Path name to convert. Under
  windows, converts all the "/" characters in the \e pathname string
  into "\\" characters. Under Unix systems converts all the "\\"
  characters in the \e pathname string into "/" characters.

  \return The converted path name.

  \sa path(const std::string &)
*/
std::string
vpIoTools::path(const char *pathname)
{
  std::string path(pathname);

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
  Converts a path name to the current system's format.
  
  \param pathname : Path name to convert. Under windows, converts all
  the "/" characters in the \e pathname string into "\\"
  characters. Under Unix systems converts all the "\\" characters in
  the \e pathname string into "/" characters.

  \return The converted path name.

  \sa path(const char *)
*/
std::string
vpIoTools::path(const std::string &pathname)
{
  return path(pathname.c_str());
}
