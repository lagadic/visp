/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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

*/
bool
vpIoTools::checkDirectory(const std::string dirname )
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
*/
void
vpIoTools::makeDirectory(const std::string dirname )
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

*/
bool
vpIoTools::checkFilename(const std::string filename)
{
  return vpIoTools::checkFilename(filename.c_str());
}

/*!
  Converts a path name to the current system's format.

  \param pathname : Path name to convert. Path name to convert. Under
  windows, converts all the "/" characters in the \e pathname string
  into "\\" characters. Under Unix systems converts all the "\\"
  characters in the \e pathname string into "/" characters.

  \return The converted path name.
*/
std::string
vpIoTools::path(const char * pathname)
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
*/
std::string
vpIoTools::path(const std::string& pathname)
{
  return path(pathname.c_str());
}
