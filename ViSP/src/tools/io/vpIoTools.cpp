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
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpIoException.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
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

std::string vpIoTools::baseName = "";
std::string vpIoTools::baseDir = "";
std::string vpIoTools::configFile = "";
std::vector<std::string> vpIoTools::configVars = std::vector<std::string>();
std::vector<std::string> vpIoTools::configValues = std::vector<std::string>();


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
  unsigned int info_buffer_size = 1024;
  TCHAR  *infoBuf = new TCHAR [info_buffer_size];
  DWORD  bufCharCount = (DWORD) info_buffer_size;
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
  unsigned int info_buffer_size = 1024;
  TCHAR  *infoBuf = new TCHAR [info_buffer_size];
  DWORD  bufCharCount = (DWORD) info_buffer_size;
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
vpIoTools::getenv(const char *
#if defined UNIX
                  env
#endif
                  )
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

  return value;
#elif defined WIN32

  vpERROR_TRACE( "Not implemented!" );
  throw(vpIoException(vpException::notImplementedError,
		      "Not implemented!")) ;
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
	if (::remove(filename) != 0)
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

  Rename an existing file \e oldfilename in \e newfilename.

  \param oldfilename : File to rename.
  \param newfilename : New file name.

  \return true if the file was renamed, false otherwise.

  \sa rename(const std::string &, const std::string &)
*/
bool
vpIoTools::rename(const char *oldfilename, const char *newfilename)
{
  if (::rename(oldfilename, newfilename) != 0)
    return false;
  else
    return true;
}

/*!

  Rename an existing file \e oldfilename in \e newfilename.

  \param oldfilename : File to rename.
  \param newfilename : New file name.

  \return true if the file was renamed, false otherwise.

  \sa rename(const char *, const char *)
*/
bool
vpIoTools::rename(const std::string &oldfilename, const std::string &newfilename)
{
  return vpIoTools::rename(oldfilename.c_str(), newfilename.c_str());
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


/*!
 Reads the configuration file and parses it.

 \param confFile : path to the file containing the configuration parameters to parse.
 */
void vpIoTools::loadConfigFile(const std::string &confFile)
{
        configFile = confFile;
	configVars.clear();configValues.clear();
	std::ifstream confContent(confFile.c_str(), std::ios::in);

	if(confContent)
	{
		std::string line,var,val;
		int k,c,c2;
		std::string stop[3] = {" ", "\t", "#"};
		while(std::getline(confContent, line))
		{
			if((line.find("#",0,1) != 0) && (line.size() > 2))
			{
				try
				{
					// name of the variable
					k = line.find(" ");
					var = line.substr(0,k);
					// look for the end of the actual value
					c = 200;
					for(unsigned i=0;i<3;++i)
						c = vpMath::minimum(c,(int)line.find(stop[i],k+1));
					if(c==-1)
						c = line.size();
					val = line.substr(k+1,c-k-1);
					configVars.push_back(var);
					configValues.push_back(val);
				}
				catch(...){}
			}
		}
		confContent.close();
	}
}

/*!
 Tries to read the parameter named \e var as a \e double.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, double &value)
{
	bool found = false;
	for(unsigned int k=0;k<configVars.size() && found==false;++k)
	{
		if(configVars[k] == var)
		{
			value = atof(configValues[k].c_str());
			found = true;
		}
	}
	if(found == false)
		std::cout << var << " not found in config file" << std::endl;
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e int.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, int &value)
{
	bool found = false;
	for(unsigned int k=0;k<configVars.size() && found==false;++k)
	{
		if(configVars[k] == var)
		{
			value = atoi(configValues[k].c_str());
			found = true;
		}
	}
	if(found == false)
		std::cout << var << " not found in config file" << std::endl;
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e unsigned int.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, unsigned int &value)
{
	int v;
	bool found = readConfigVar(var,v);
	value = (unsigned int) v;
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e bool.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, bool &value)
{
	int v;
	bool found = readConfigVar(var,v);
	value = (v!=0);
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e vpColor.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read. See vpColor.cpp for the color number.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, vpColor &value)
{
	int v;
	bool found = readConfigVar(var,v);
	value = vpColor::getColor(v);
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e std::string.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, std::string &value)
{
	bool found = false;
	for(unsigned int k=0;k<configVars.size() && found==false;++k)
	{
		if(configVars[k] == var)
		{
			value = configValues[k];
			found = true;
		}
	}
	if(found == false)
		std::cout << var << " not found in config file" << std::endl;
	return found;
}

/*!
 Tries to read the parameter named \e var as a \e vpMatrix.
 If \e nCols and \e nRows are indicated, will resize the matrix.
 Otherwise, will try to read as many values as indicated by the dimension of \e value.

 \param var : Name of the parameter in the configuration file.
 \param value : Value to be read.
 \param nCols : Column dimension if resized.
 \param nRows : Row dimension if resized

 \return true if the parameter could be read.
 */
bool vpIoTools::readConfigVar(const std::string &var, vpMatrix &value, const int &nCols, const int &nRows)
{
	bool found = false;
	for(unsigned int k=0;k<configVars.size() && found==false;++k)
	{
		if(configVars[k] == var)
		{
			found = true;
			// resize or not
			if(nCols != 0 && nRows != 0)
				value.resize(nRows, nCols);
			int i,j,ind=0,ind2;
			for(i=0;i<value.getRows();++i)
				for(j=0;j<value.getCols();++j)
				{
					ind2 = configValues[k].find(",",ind);
					value[i][j] = atof(configValues[k].substr(ind,ind2-ind).c_str());
					ind = ind2+1;
				}
		}
	}
	if(found == false)
		std::cout << var << " not found in config file" << std::endl;
	return found;
}

// construct experiment filename & path

/*!
 Augments the prefix of the experiment files by \e strTrue if \e cond is verified, and by \e strFalse otherwise.

 \param strTrue : String to add if \e cond is true
 \param cond : Condition managing the file name
 \param strFalse : String to add if \e cond is false (default "")
 */
void vpIoTools::addNameElement(const std::string &strTrue, const bool &cond, const std::string &strFalse)
{
	if(cond)
		baseName += "_" + strTrue;
	else if(strFalse != "")
		baseName += "_" + strFalse;
}

/*!
 Augments the prefix of the experiment files by \e strTrue followed by \e val.

 \param strTrue : String to add
 \param val : Value to add

 */
void vpIoTools::addNameElement(const std::string &strTrue, const double &val)
{
	if(val != 0.)
	{
		char valC[256];
		sprintf(valC, "%.3f", val);
		std::string valS(valC);
		baseName += "_" + strTrue + valS;
	}
}

/*!
 Creates the directory \e baseDir/baseName. If already exists, empties it if \e empty is true.
 Useful to save the images corresponding to a particular experiment.

 \param empty : Indicates if the new directory has to be emptied

 */
void vpIoTools::createBaseNamePath(const bool &empty)
{
	if(vpIoTools::checkDirectory(baseDir + baseName) == false)
	{
		vpIoTools::makeDirectory(baseDir + baseName);
	std::cout << "creating directory " + baseDir + baseName << std::endl;
	}
	else
          if(empty)
          {
                  system(("rm -rf " + baseDir + baseName + "/*").c_str());
                  std::cout << "emptying directory " + baseDir + baseName << std::endl;
          }
}


// write configuration file
/*!
 Copy the initial configuration file to the experiment directory.

 \param actuallySave : If false, do not copy the file.

 */
void vpIoTools::saveConfigFile(const bool &actuallySave)
{
	if(actuallySave)
	{
	  std::string dest = baseDir + "/" + baseName + "_config.txt";
	  // file copy
	  system(("cp " + configFile + " " + dest).c_str());
	}
}



