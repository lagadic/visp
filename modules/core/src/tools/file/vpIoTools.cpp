/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Directory management.
 */

/*!
  \file vpIoTools.cpp
  \brief File and directories basic tools.
*/

// At this point, to make scandir() working as expected on armv7 virtualized on a x86-64bit architecture
// (see github/workflow/other-arch-isolated.yml) we need to define _FILE_OFFSET_BITS=64. Otherwise
// testVideo.cpp will fail.
// Since adding here something like:
//   #include <visp3/core/vpConfig.h>
//   #ifdef VISP_DEFINE_FILE_OFFSET_BITS
//   # define _FILE_OFFSET_BITS 64
//   #endif
// where VISP_DEFINE_FILE_OFFSET_BITS is defined in vpConfig.h doesn't work (my explanation is that the define
// should be done before any other includes; in vpConfig.h there is cstdlib that is included), the other way
// that was retained is to add to vpIoTools.cpp COMPILE_DEFINTIONS _FILE_OFFSET_BITS=64 (see CMakeLists.txt)

#include <algorithm>
#include <cctype>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <visp3/core/vpEndian.h>
#include <visp3/core/vpIoException.h>
#include <visp3/core/vpIoTools.h>
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <dirent.h>
#include <unistd.h>
#elif defined(_WIN32)
#include <direct.h>
#include <windows.h>
#endif
#if !defined(_WIN32)
#ifdef __ANDROID__
// Like IOS, wordexp.cpp is not defined for Android
#else
#include <wordexp.h>
#endif
#endif

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#include <TargetConditionals.h>             // To detect OSX or IOS using TARGET_OS_IOS macro
#endif

#ifndef PATH_MAX
#ifdef _MAX_PATH
#define PATH_MAX _MAX_PATH
#else
#define PATH_MAX 1024
#endif
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#define VP_STAT stat
#elif defined(_WIN32) && defined(__MINGW32__)
#define VP_STAT stat
#elif defined(_WIN32)
#define VP_STAT _stat
#else
#define VP_STAT stat
#endif
namespace
{
// The following code is not working on iOS since wordexp() is not available
// The function is not used on Android
#if defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
#if (TARGET_OS_IOS == 0) && !defined(__ANDROID__)
void replaceAll(std::string &str, const std::string &search, const std::string &replace)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(search, start_pos)) != std::string::npos) {
    str.replace(start_pos, search.length(), replace);
    start_pos += replace.length(); // Handles case where 'replace' is a
    // substring of 'search'
  }
}
#endif
#endif
} // namespace

BEGIN_VISP_NAMESPACE
/*!
  Return build informations (OS, compiler, build flags, used 3rd parties...).
 */
  const std::string &vpIoTools::getBuildInformation()
{
  static std::string build_info =
#include "version_string.inc"
    ;
  return build_info;
}

/*!
  Return path to the default temporary folder:
    - on Windows it returns `GetTempPath()`
    - on Unix it returns `/tmp/<username>`

  \warning This function is not implemented on WINRT.

  The following sample shows how to use this function to create unique temporary directories:
  \code
  include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::string tmp_path = vpIoTools::getTempPath();
    std::cout << "Temp path: " << tmp_path << std::endl;

    std::string tmp_dir1 = vpIoTools::makeTempDirectory(tmp_path);
    std::cout << "Created unique temp dir1: " << tmp_dir1 << std::endl;

    std::string tmp_dir2_template = tmp_path + vpIoTools::path("/") + "dir_XXXXXX";
    std::string tmp_dir2 = vpIoTools::makeTempDirectory(tmp_dir2_template);
    std::cout << "Created unique temp dir2: " << tmp_dir2 << std::endl;

    if (vpIoTools::remove(tmp_dir1)) {
      std::cout << "Temp dir1 was deleted" << std::endl;
    }
    if (vpIoTools::remove(tmp_dir2)) {
      std::cout << "Temp dir2 was deleted" << std::endl;
    }
  }
  \endcode
  On Windows it produces:
  \verbatim
  Temp path: C:\Users\<username>\AppData\Local\Temp
  Created unique temp dir1: C:\Users\<username>\AppData\Local\Temp\ddaac8c3-7a95-447f-8a1c-fe31bb2426f9
  Created unique temp dir2: C:\Users\<username>\AppData\Local\Temp\dir_8b9e6e9a-fe9b-4b44-8382-fc2368dfed68
  Temp dir1 was deleted
  Temp dir2 was deleted
  \endverbatim

  while on Unix it produces:
  \verbatim
  Temp path: /tmp/<username>
  Created unique temp dir1: /tmp/<username>/AMIsXF
  Created unique temp dir2: /tmp/<username>/dir_KP7119
  Temp dir1 was deleted
  Temp dir2 was deleted
  \endverbatim

  \sa makeTempDirectory(), remove()
 */
std::string vpIoTools::getTempPath()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::string username;
  vpIoTools::getUserName(username);
  return "/tmp/" + username;
#elif defined(_WIN32) && !defined(WINRT)
  // https://docs.microsoft.com/en-us/windows/win32/fileio/creating-and-using-a-temporary-file
  //  Gets the temp path env string (no guarantee it's a valid path).
  TCHAR lpTempPathBuffer[MAX_PATH];
  DWORD dwRetVal = GetTempPath(MAX_PATH /* length of the buffer */, lpTempPathBuffer /* buffer for path */);
  if (dwRetVal > MAX_PATH || (dwRetVal == 0)) {
    throw vpIoException(vpIoException::cantGetenv, "Error with GetTempPath() call!");
  }
  std::string temp_path(lpTempPathBuffer);
  if (!temp_path.empty()) {
    if (temp_path.back() == '\\') {
      temp_path.resize(temp_path.size() - 1);
    }
  }
  else {
    temp_path = "C:\temp";
    try {
      vpIoTools::makeDirectory(temp_path);
    }
    catch (...) {
      throw(vpException(vpException::fatalError, "Cannot set temp path to %s", temp_path.c_str()));
    }
  }
  return temp_path;
#else
  throw vpIoException(vpException::fatalError, "Not implemented on this platform!");
#endif
}

/*!
  Get the user name.

  - Under unix, get the content of the LOGNAME environment variable.  For most
    purposes (especially in conjunction with crontab), it is more useful to
    use the environment variable LOGNAME to find out who the user is, rather
    than the getlogin() function.  This is more flexible precisely because the
    user can set LOGNAME arbitrarily.
  - Under windows, uses the GetUserName() function.

  \param username : The user name. When the username cannot be retrieved, set \e username to
  "unknown" string.
*/
void vpIoTools::getUserName(std::string &username)
{
  // With MinGW, UNIX and _WIN32 are defined
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  // Get the user name.
  char *logname = ::getenv("LOGNAME");
  if (!logname) {
    username = "unknown";
  }
  else {
    username = logname;
  }
#elif defined(_WIN32)
#if (!defined(WINRT))
  unsigned int info_buffer_size = 1024;
  TCHAR *infoBuf = new TCHAR[info_buffer_size];
  DWORD bufCharCount = (DWORD)info_buffer_size;
  // Get the user name.
  if (!GetUserName(infoBuf, &bufCharCount)) {
    username = "unknown";
  }
  else {
    username = infoBuf;
  }
  delete[] infoBuf;
#else
  // Universal platform
  username = "unknown";
#endif
#else
  username = "unknown";
#endif
}

/*!
  Get the user name.

  - Under unix, get the content of the LOGNAME environment variable.  For most
    purposes (especially in conjunction with crontab), it is more useful to
  use the environment variable LOGNAME to find out who the user is, rather
  than the getlogin() function.  This is more flexible precisely because the
  user can set LOGNAME arbitrarily.
  - Under windows, uses the GetUserName() function.

  \return The user name.

  \sa getUserName(std::string &)
*/
std::string vpIoTools::getUserName()
{
  std::string username;
  getUserName(username);
  return username;
}

/*!
  Get the content of an environment variable.

  \param env : Environment variable name (HOME, LOGNAME...).
  \return Value of the environment variable

  \exception vpIoException::cantGetenv : If an error occur while
  getting the environment variable value.

  \code
  #include <iostream>
  #include <string>
  #include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::string envvalue;
    try {
      std::string env = "HOME";
      envvalue = vpIoTools::getenv(env);
      std::cout << "$HOME = \"" << envvalue << "\"" << std::endl;
    }
    catch (const vpException &e) {
      std::cout << e.getMessage() << std::endl;
      return -1;
    }
    return 0;
  }
  \endcode
*/
std::string vpIoTools::getenv(const std::string &env)
{
#if defined(_WIN32) && defined(WINRT)
  throw(vpIoException(vpIoException::cantGetenv, "Cannot get the environment variable value: not "
                      "implemented on Universal Windows Platform"));
#else
  std::string value;
  // Get the environment variable value.
  char *v_value = ::getenv(env.c_str());
  if (!v_value) {
    throw(vpIoException(vpIoException::cantGetenv, "Cannot get the environment variable value"));
  }
  value = v_value;

  return value;
#endif
}

/*!
  Extract major, minor and patch from a version given as "x.x.x".
  Ex: If version is "1.2.1", major will be 1, minor 2 and patch 1.

  \param version : String to extract the values.
  \param major : Extracted major.
  \param minor : Extracted minor.
  \param patch : Extracted patch.
*/
void vpIoTools::getVersion(const std::string &version, unsigned int &major, unsigned int &minor, unsigned int &patch)
{
  if (version.size() == 0) {
    major = 0;
    minor = 0;
    patch = 0;
  }
  else {
    size_t major_pos = version.find('.');
    std::string major_str = version.substr(0, major_pos);
    major = static_cast<unsigned>(atoi(major_str.c_str()));

    if (major_pos != std::string::npos) {
      size_t minor_pos = version.find('.', major_pos + 1);
      std::string minor_str = version.substr(major_pos + 1, (minor_pos - (major_pos + 1)));
      minor = static_cast<unsigned>(atoi(minor_str.c_str()));

      if (minor_pos != std::string::npos) {
        std::string patch_str = version.substr(minor_pos + 1);
        patch = static_cast<unsigned>(atoi(patch_str.c_str()));
      }
      else {
        patch = 0;
      }
    }
    else {
      minor = 0;
      patch = 0;
    }
  }
}

/*!
  Check if a directory exists.

  \param dirname : Directory to test if it exists. The directory name
  is converted to the current system's format; see path().

  \return true : If the directory exists and is accessible with write access.

  \return false : If dirname string is null, or is not a directory, or
  has no write access.
*/
bool vpIoTools::checkDirectory(const std::string &dirname)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  struct stat stbuf;
#elif defined(_WIN32) && defined(__MINGW32__)
  struct stat stbuf;
#elif defined(_WIN32)
  struct _stat stbuf;
#endif

  if (dirname.empty()) {
    return false;
  }

  std::string path_dirname = path(dirname);

  if (VP_STAT(path_dirname.c_str(), &stbuf) != 0) {
    // Test adding the separator if not already present
    if (path_dirname.at(path_dirname.size() - 1) != separator) {
      if (VP_STAT((path_dirname + separator).c_str(), &stbuf) != 0) {
        return false;
      }
    }
    // Test removing the separator if already present
    if (path_dirname.at(path_dirname.size() - 1) == separator) {
      if (VP_STAT((path_dirname.substr(0, path_dirname.size() - 1)).c_str(), &stbuf) != 0) {
        return false;
      }
    }
  }

#if defined(_WIN32) || (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
  if ((stbuf.st_mode & S_IFDIR) == 0)
#endif
  {
    return false;
  }
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if ((stbuf.st_mode & S_IWUSR) == 0)
#elif defined(_WIN32)
  if ((stbuf.st_mode & S_IWRITE) == 0)
#endif
  {
    return false;
  }
  return true;
}

/*!
  Check if a fifo file exists.

  \param fifofilename : Fifo filename to test if it exists.

  \return true : If the fifo file exists and is accessible with read access.

  \return false : If fifofilename string is null, or is not a fifo filename, or
                              has no read access.

  \sa checkFilename(const std::string &)
*/
bool vpIoTools::checkFifo(const std::string &fifofilename)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  struct stat stbuf;

  std::string v_filename = path(fifofilename);
  if (stat(v_filename.c_str(), &stbuf) != 0) {
    return false;
  }
  if ((stbuf.st_mode & S_IFIFO) == 0) {
    return false;
  }
  if ((stbuf.st_mode & S_IRUSR) == 0)

  {
    return false;
  }
  return true;
#elif defined(_WIN32)
  (void)fifofilename;
  throw(vpIoException(vpIoException::notImplementedError, "Fifo files are not supported on Windows platforms."));
#endif
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
int vpIoTools::mkdir_p(const std::string &path, int mode)
{
  errno = 0;
  if (path.size() > PATH_MAX) {
    errno = ENAMETOOLONG;
    return -1;
  }

  // Iterate over the string
  std::string cpy_path = path;
  std::string sub_path;
  for (size_t pos = 0; (pos = cpy_path.find(vpIoTools::separator)) != std::string::npos;) {
    sub_path += cpy_path.substr(0, pos + 1);
    // Continue if sub_path = separator
    bool stop_for_loop = false;
    if (pos == 0) {
      cpy_path.erase(0, pos + 1);
      stop_for_loop = true;
    }
    if (!stop_for_loop) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
      if (mkdir(sub_path.c_str(), static_cast<mode_t>(mode)) != 0)
#elif defined(_WIN32)
      (void)mode; // var not used
      if (!checkDirectory(sub_path) && _mkdir(sub_path.c_str()) != 0)
#endif
      {
        if (errno != EEXIST) {
          return -1;
        }
      }
      cpy_path.erase(0, pos + 1);
    }
  }

  if (!cpy_path.empty()) {
    sub_path += cpy_path;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    if (mkdir(sub_path.c_str(), static_cast<mode_t>(mode)) != 0)
#elif defined(_WIN32)

    if (_mkdir(sub_path.c_str()) != 0)
#endif
    {
      if (errno != EEXIST) {
        return -1;
      }
    }
  }

  return 0;
}

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  Create a new directory. It will create recursively the parent directories if
  needed.

  \param dirname : Directory to create. The directory name
  is converted to the current system's format; see path().

  \exception vpIoException::cantCreateDirectory : If the directory cannot be
  created.

  \sa makeTempDirectory(), remove()
*/
void vpIoTools::makeDirectory(const std::string &dirname)
{
#if ((!defined(__unix__) && !defined(__unix) && (!defined(__APPLE__) || !defined(__MACH__)))) && !defined(_WIN32)
  std::cerr << "Unsupported platform for vpIoTools::makeDirectory()!" << std::endl;
  return;
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  struct stat stbuf;
#elif defined(_WIN32) && defined(__MINGW32__)
  struct stat stbuf;
#elif defined(_WIN32)
  struct _stat stbuf;
#endif

  if (dirname.empty()) {
    throw(vpIoException(vpIoException::invalidDirectoryName, "invalid directory name"));
  }

  std::string v_dirname = path(dirname);

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (stat(v_dirname.c_str(), &stbuf) != 0)
#elif defined(_WIN32) && defined(__MINGW32__)
  if (stat(v_dirname.c_str(), &stbuf) != 0)
#elif defined(_WIN32)
  if (_stat(v_dirname.c_str(), &stbuf) != 0)
#endif
  {
    if (vpIoTools::mkdir_p(v_dirname, 0755) != 0) {
      throw(vpIoException(vpIoException::cantCreateDirectory, "Unable to create directory '%s'", dirname.c_str()));
    }
  }

  if (checkDirectory(dirname) == false) {
    throw(vpIoException(vpIoException::cantCreateDirectory, "Unable to create directory '%s'", dirname.c_str()));
  }
}

/*!
  Create a new FIFO file. A FIFO file is a special file, similar to a pipe, but actually existing on the hard drive. It
  can be used to communicate data between multiple processes.

  \warning This function is only implemented on unix-like OS.

  \param[in] fifoname : Pathname of the fifo file to create.

  \exception vpIoException::invalidDirectoryName : The \e dirname is invalid.

  \exception vpIoException::cantCreateDirectory : If the file cannot be created.
*/
void vpIoTools::makeFifo(const std::string &fifoname)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX

  // If dirname is a directory, we throw an error
  if (vpIoTools::checkDirectory(fifoname)) {
    throw(vpIoException(vpIoException::invalidDirectoryName,
                        "Unable to create fifo file. '%s' is an existing directory.", fifoname.c_str()));
  }

  // If dirname refers to an already existing file, we throw an error
  else if (vpIoTools::checkFilename(fifoname)) {
    throw(vpIoException(vpIoException::invalidDirectoryName, "Unable to create fifo file '%s'. File already exists.",
                        fifoname.c_str()));
    // If dirname refers to an already existing fifo, we throw an error
  }
  else if (vpIoTools::checkFifo(fifoname)) {
    throw(vpIoException(vpIoException::invalidDirectoryName, "Unable to create fifo file '%s'. Fifo already exists.",
                        fifoname.c_str()));
  }

  else if (mkfifo(fifoname.c_str(), 0666) < 0) {
    throw(vpIoException(vpIoException::cantCreateDirectory, "Unable to create fifo file '%s'.", fifoname.c_str()));
  }
#elif defined(_WIN32)
  (void)fifoname;
  throw(vpIoException(vpIoException::cantCreateDirectory, "Unable to create fifo on Windows platforms."));
#endif
}

#if defined(_WIN32) && !defined(WINRT)
std::string getUuid()
{
  UUID uuid;
  if (UuidCreate(&uuid) != RPC_S_OK) {
    throw(vpIoException(vpIoException::fatalError, "UuidCreate() failed!"));
  }

  RPC_CSTR stringUuid;
  if (UuidToString(&uuid, &stringUuid) != RPC_S_OK) {
    throw(vpIoException(vpIoException::fatalError, "UuidToString() failed!"));
  }

  return reinterpret_cast<char *>(stringUuid);
}
#endif

/*!
  Create a new temporary directory with a unique name based on dirname parameter.

  \warning This function is not implemented on WINRT.

  \param dirname : Parent directory in which a temporary directory will be created or temporary directory that ends with
  "XXXXXX", which will be converted into random characters in order to create a unique directory name.

  \return String corresponding to the absolute path to the generated directory name.

  \exception vpIoException::cantCreateDirectory : If the directory cannot be created.

  The following sample shows how to use this function to create unique temporary directories:
  \code
  include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::string tmp_path = vpIoTools::getTempPath();
    std::cout << "Temp path: " << tmp_path << std::endl;

    std::string tmp_dir1 = vpIoTools::makeTempDirectory(tmp_path);
    std::cout << "Created unique temp dir1: " << tmp_dir1 << std::endl;

    std::string tmp_dir2_template = tmp_path + vpIoTools::path("/") + "dir_XXXXXX";
    std::string tmp_dir2 = vpIoTools::makeTempDirectory(tmp_dir2_template);
    std::cout << "Created unique temp dir2: " << tmp_dir2 << std::endl;

    if (vpIoTools::remove(tmp_dir1)) {
      std::cout << "Temp dir1 was deleted" << std::endl;
    }
    if (vpIoTools::remove(tmp_dir2)) {
      std::cout << "Temp dir2 was deleted" << std::endl;
    }
  }
  \endcode
  On Windows it produces:
  \verbatim
  Temp path: C:\Users\<username>\AppData\Local\Temp
  Created unique temp dir1: C:\Users\<username>\AppData\Local\Temp\ddaac8c3-7a95-447f-8a1c-fe31bb2426f9
  Created unique temp dir2: C:\Users\<username>\AppData\Local\Temp\dir_8b9e6e9a-fe9b-4b44-8382-fc2368dfed68
  Temp dir1 was deleted
  Temp dir2 was deleted
  \endverbatim

  while on Unix it produces:
  \verbatim
  Temp path: /tmp/<username>
  Created unique temp dir1: /tmp/<username>/AMIsXF
  Created unique temp dir2: /tmp/<username>/dir_KP7119
  Temp dir1 was deleted
  Temp dir2 was deleted
  \endverbatim

  \sa makeDirectory(), getTempPath(), remove()
*/
std::string vpIoTools::makeTempDirectory(const std::string &dirname)
{
#if defined(WINRT) || !defined(_WIN32) && !(defined(__unix__) || defined(__unix) ||                                    \
                                            (defined(__APPLE__) && defined(__MACH__))) // not UNIX and not Windows
  throw(vpIoException(vpIoException::cantCreateDirectory, "makeTempDirectory() is not supported on this platform!"));
#endif

  std::string dirname_cpy = std::string(dirname);
  std::string correctEnding = "XXXXXX";

  // If dirname is an unexisting directory, it should end with XXXXXX in order to create a temp directory
  if (!vpIoTools::checkDirectory(dirname_cpy)) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    // Check if dirname ends with XXXXXX
    if (dirname_cpy.rfind(correctEnding) == std::string::npos) {
      if (dirname_cpy.at(dirname_cpy.length() - 1) != '/') {
        dirname_cpy = dirname_cpy + "/";
      }
      try {
        vpIoTools::makeDirectory(dirname_cpy);
      }
      catch (...) {
        throw(vpException(vpException::fatalError, "Cannot create temp directory %s", dirname_cpy.c_str()));
      }

      dirname_cpy = dirname_cpy + "XXXXXX";
    }

#elif defined(_WIN32) && !defined(WINRT)
    // Remove XXXXXX
    dirname_cpy = dirname_cpy.substr(0, dirname_cpy.rfind(correctEnding));
    // Append UUID
    dirname_cpy = dirname_cpy + getUuid();
#endif

  }
  else {
    // If dirname is an existing directory, we create a temp directory inside
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    if (dirname_cpy.at(dirname_cpy.length() - 1) != '/') {
      dirname_cpy = dirname_cpy + "/";
    }
    dirname_cpy = dirname_cpy + "XXXXXX";
#elif defined(_WIN32) && !defined(WINRT)
    dirname_cpy = createFilePath(dirname_cpy, getUuid());
#endif
  }

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  char *dirname_char = new char[dirname_cpy.length() + 1];
  strcpy(dirname_char, dirname_cpy.c_str());

  char *computedDirname = mkdtemp(dirname_char);

  if (!computedDirname) {
    delete[] dirname_char;
    throw(vpIoException(vpIoException::cantCreateDirectory, "Unable to create directory '%s'.", dirname_cpy.c_str()));
  }

  std::string res(computedDirname);
  delete[] dirname_char;
  return res;
#elif defined(_WIN32) && !defined(WINRT)
  makeDirectory(dirname_cpy);
  return dirname_cpy;
#endif
}

/*!
  Check if a file exists.

  \param filename : Filename to test if it exists.

  \return true : If the filename exists and is accessible with read access.

  \return false : If filename string is null, or is not a filename, or
  has no read access.
*/
bool vpIoTools::checkFilename(const std::string &filename)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  struct stat stbuf;
#elif defined(_WIN32)
  struct _stat stbuf;
#endif

  if (filename.empty()) {
    return false;
  }

  std::string v_filename = path(filename);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (stat(v_filename.c_str(), &stbuf) != 0)
#elif defined(_WIN32)
  if (_stat(v_filename.c_str(), &stbuf) != 0)
#endif
  {
    return false;
  }
  if ((stbuf.st_mode & S_IFREG) == 0) {
    return false;
  }
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if ((stbuf.st_mode & S_IRUSR) == 0)
#elif defined(_WIN32)
  if ((stbuf.st_mode & S_IREAD) == 0)
#endif
  {
    return false;
  }
  return true;
}

/*!

  Copy a \e src file or directory in \e dst.

  \param src : Existing file or directory to copy.
  \param dst : New copied file or directory.
*/
bool vpIoTools::copy(const std::string &src, const std::string &dst)
{
  // Check if we have to consider a file or a directory
  if (vpIoTools::checkFilename(src)) {
    // --comment: std::cout "copy file: " src " in " dst std::endl;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#if TARGET_OS_IOS == 0 // The following code is not working on iOS since
                       // wordexp() is not available
    std::stringstream cmd;
    cmd << "cp -p ";
    cmd << src;
    cmd << " ";
    cmd << dst;
    int ret = system(cmd.str().c_str());
    if (ret) {
    } // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot copy %s in %s: not implemented on iOS Platform", src.c_str(),
                        dst.c_str()));
#endif
#elif defined(_WIN32)
#if (!defined(WINRT))
    std::stringstream cmd;
    cmd << "copy ";
    cmd << vpIoTools::path(src);
    cmd << " ";
    cmd << vpIoTools::path(dst);
    int ret = system(cmd.str().c_str());
    if (ret) {
    }; // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot copy %s in %s: not implemented on Universal Windows Platform",
                        src.c_str(), dst.c_str()));
#endif
#endif
  }
  else if (vpIoTools::checkDirectory(src)) {
    // --comment: std::cout  "copy directory: "  src  " in "  dst
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#if TARGET_OS_IOS == 0 // The following code is not working on iOS since
                       // wordexp() is not available
    std::stringstream cmd;
    cmd << "cp -p ";
    cmd << src;
    cmd << " ";
    cmd << dst;
    int ret = system(cmd.str().c_str());
    if (ret) {
    } // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot copy %s in %s: not implemented on iOS Platform", src.c_str(),
                        dst.c_str()));
#endif
#elif defined(_WIN32)
#if (!defined(WINRT))
    std::stringstream cmd;
    cmd << "copy ";
    cmd << vpIoTools::path(src);
    cmd << " ";
    cmd << vpIoTools::path(dst);
    int ret = system(cmd.str().c_str());
    if (ret) {
    }; // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot copy %s in %s: not implemented on Universal Windows Platform",
                        src.c_str(), dst.c_str()));
#endif
#endif
  }
  else {
    std::cout << "Cannot copy: " << src << " in " << dst << std::endl;
    return false;
  }
}

/*!

  Remove a file or a directory.

  \param file_or_dir : File name or directory to remove.

  \return true if the file or the directory was removed, false otherwise.

  \sa makeDirectory(), makeTempDirectory()
*/
bool vpIoTools::remove(const std::string &file_or_dir)
{
  // Check if we have to consider a file or a directory
  if (vpIoTools::checkFilename(file_or_dir)
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    || vpIoTools::checkFifo(std::string(file_or_dir))
#endif
    ) {
    if (::remove(file_or_dir.c_str()) != 0) {
      return false;
    }
    else {
      return true;
    }
  }
  else if (vpIoTools::checkDirectory(file_or_dir)) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#if TARGET_OS_IOS == 0 // The following code is not working on iOS since
                       // wordexp() is not available
    std::stringstream cmd;
    cmd << "rm -rf \"";
    cmd << file_or_dir;
    cmd << "\"";
    int ret = system(cmd.str().c_str());
    if (ret) {
    } // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot remove %s: not implemented on iOS Platform",
                        file_or_dir.c_str()));
#endif
#elif defined(_WIN32)
#if (!defined(WINRT))
    std::stringstream cmd;
    cmd << "rmdir /S /Q ";
    cmd << vpIoTools::path(file_or_dir);
    cmd << "\"";
    int ret = system(cmd.str().c_str());
    if (ret) {
    }; // to avoid a warning
    // std::cout << cmd << " return value: " << ret << std::endl;
    return true;
#else
    throw(vpIoException(vpException::fatalError, "Cannot remove %s: not implemented on Universal Windows Platform",
                        file_or_dir.c_str()));
#endif
#endif
  }
  else {
    std::cout << "Cannot remove: " << file_or_dir << std::endl;
    return false;
  }
}

/*!

  Rename an existing file \e oldfilename in \e newfilename.

  \param oldfilename : File to rename.
  \param newfilename : New file name.

  \return true if the file was renamed, false otherwise.
*/
bool vpIoTools::rename(const std::string &oldfilename, const std::string &newfilename)
{
  if (::rename(oldfilename.c_str(), newfilename.c_str()) != 0) {
    return false;
  }
  else {
    return true;
  }
}

/*!
  Converts a path name to the current system's format.

  \param pathname : Path name to convert. Under windows, converts all
  the "/" characters in the \e pathname string into "\\"
  characters. Under Unix systems converts all the "\\" characters in
  the \e pathname string into "/" characters.

  \return The converted path name.
*/
std::string vpIoTools::path(const std::string &pathname)
{
  std::string path(pathname);

#if defined(_WIN32)
  for (unsigned int i = 0; i < path.length(); ++i)
    if (path[i] == '/')
      path[i] = '\\';
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
  unsigned int path_length = path.length();
  for (unsigned int i = 0; i < path_length; ++i) {
    if (path[i] == '\\') {
      path[i] = '/';
    }
  }
#if TARGET_OS_IOS == 0 // The following code is not working on iOS and android since
  // wordexp() is not available
#ifdef __ANDROID__
// Do nothing
#else
  wordexp_t exp_result;

  // escape quote character
  replaceAll(path, "'", "'\\''");
  // add quotes to handle special characters like parentheses and spaces
  wordexp(std::string("'" + path + "'").c_str(), &exp_result, 0);
  path = exp_result.we_wordc == 1 ? exp_result.we_wordv[0] : "";
  wordfree(&exp_result);
#endif
#endif
#endif

  return path;
}

/*!
  Get ViSP images data path. ViSP images data can be installed from Debian or
  Ubuntu \e visp-images-data package. It can be also installed from
  visp-images-3.x.y.zip that can be found on https://visp.inria.fr/download page.

  This function returns the path to the folder that contains the data.
  - It checks first if VISP_INPUT_IMAGE_PATH environment variable that gives the
  location of the data is set. In that case returns the content of this
  environment var.
  - Otherwise it checks if \e visp-images-data binary package (Ubuntu, Debian) is installed.
  In that case returns then \e /usr/share/visp-images-data".
  - If the path is not found, returns an empty string.
 */
std::string vpIoTools::getViSPImagesDataPath()
{
  std::string data_path;
  std::string file_to_test("mbt/cube.cao");
  std::string filename;
  // Test if VISP_INPUT_IMAGE_PATH env var is set
  data_path = vpIoTools::getenv("VISP_INPUT_IMAGE_PATH");
  filename = data_path + "/" + file_to_test;
  if (vpIoTools::checkFilename(filename)) {
    return data_path;
  }
  data_path = vpIoTools::getenv("VISP_INPUT_IMAGE_PATH") + "/ViSP-images";
  filename = data_path + "/" + file_to_test;
  if (vpIoTools::checkFilename(filename)) {
    return data_path;
  }
  data_path = vpIoTools::getenv("VISP_INPUT_IMAGE_PATH") + "/visp-images";
  filename = data_path + "/" + file_to_test;
  if (vpIoTools::checkFilename(filename)) {
    return data_path;
  }

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  // Test if visp-images-data package is installed (Ubuntu and Debian)
  data_path = "/usr/share/visp-images-data/ViSP-images";
  filename = data_path + "/" + file_to_test;
  if (vpIoTools::checkFilename(filename)) {
    return data_path;
  }
  data_path = "/usr/share/visp-images-data/visp-images";
  filename = data_path + "/" + file_to_test;
  if (vpIoTools::checkFilename(filename)) {
    return data_path;
  }
#endif
  data_path = "";
  return data_path;
}

/*!
  Returns the extension of the file or an empty string if the file has no
  extension. If checkFile flag is set, it will check first if the pathname
  denotes a directory and so return an empty string and second it will check
  if the file denoted by the pathanme exists. If so, it will return the
  extension if present.

  \param pathname : The pathname of the file we want to get the extension.
  \param checkFile : If true, the file must exist otherwise an empty string will be returned.
  \return The extension of the file including the dot "." or an empty string if the file has no extension or if the
  pathname is empty.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::string filename = "my/path/to/file.xml"
    std::string ext = vpIoTools::getFileExtension(opt_learning_data);
    std::cout << "ext: " << ext << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  ext: .xml
  \endcode
 */
std::string vpIoTools::getFileExtension(const std::string &pathname, bool checkFile)
{
  if (checkFile && (vpIoTools::checkDirectory(pathname) || (!vpIoTools::checkFilename(pathname)))) {
    return "";
  }

#if defined(_WIN32)
  std::string sep = "\\";
  std::string altsep = "/";
  std::string extsep = ".";
#else
  // On Unix, or on the Mac
  std::string sep = "/";
  std::string altsep = "";
  std::string extsep = ".";
#endif

  // Python 2.7.8 module.
  // # Split a path in root and extension.
  // # The extension is everything starting at the last dot in the last
  // # pathname component; the root is everything before that.
  // # It is always true that root + ext == p.
  //
  // # Generic implementation of splitext, to be parametrized with
  // # the separators
  // def _splitext(p, sep, altsep, extsep):
  //    """Split the extension from a pathname.
  //
  //    Extension is everything from the last dot to the end, ignoring
  //    leading dots.  Returns "(root, ext)"; ext may be empty."""
  //
  //    sepIndex = p.rfind(sep)
  //    if altsep:
  //        altsepIndex = p.rfind(altsep)
  //        sepIndex = max(sepIndex, altsepIndex)
  //
  //    dotIndex = p.rfind(extsep)
  //    if dotIndex > sepIndex:
  //        # skip all leading dots
  //        filenameIndex = sepIndex + 1
  //        while filenameIndex < dotIndex:
  //            if p[filenameIndex] != extsep:
  //                return p[:dotIndex], p[dotIndex:]
  //            filenameIndex += 1
  //
  //    return p, ''

  int sepIndex = static_cast<int>(pathname.rfind(sep));
  if (!altsep.empty()) {
    int altsepIndex = static_cast<int>(pathname.rfind(altsep));
    sepIndex = std::max<int>(sepIndex, altsepIndex);
  }

  size_t dotIndex = pathname.rfind(extsep);
  if (dotIndex != std::string::npos) {
    // The extsep character exists
    size_t npos = std::string::npos;
    if (((sepIndex != static_cast<int>(npos)) && (static_cast<int>(dotIndex) > sepIndex)) ||
      (sepIndex == static_cast<int>(npos))) {
      if (sepIndex == static_cast<int>(npos)) {
        sepIndex = 0;
      }
      size_t filenameIndex = static_cast<size_t>(sepIndex) + static_cast<size_t>(1);

      while (filenameIndex < dotIndex) {
        if (pathname.compare(filenameIndex, 1, extsep) != 0) {
          return pathname.substr(dotIndex);
        }
        filenameIndex++;
      }
    }
  }

  return "";
}

/*!
   Returns the name of the file or directory denoted by this pathname.
   \return The name of the file or directory denoted by this pathname, or an
   empty string if this pathname's name sequence is empty.
 */
std::string vpIoTools::getName(const std::string &pathname)
{
  if (pathname.size() > 0) {
    std::string convertedPathname = vpIoTools::path(pathname);

    size_t index = convertedPathname.find_last_of(vpIoTools::separator);
    if (index != std::string::npos) {
      return convertedPathname.substr(index + 1);
    }

    return convertedPathname;
  }

  return "";
}

/*!
   Returns the name of the file without extension or directory denoted by this
   pathname. \return The name of the file without extension or directory
   denoted by this pathname, or an empty string if this pathname's name
   sequence is empty.
 */
std::string vpIoTools::getNameWE(const std::string &pathname)
{
  std::string name = vpIoTools::getName(pathname);
  size_t found = name.find_last_of(".");
  std::string name_we = name.substr(0, found);
  return name_we;
}

/*!
  Checks file name format and extracts its index.

  Format must contain substring "%0xd", defining the length of image index.
  For example, format can be "img%04d.jpg". Then "img0001.jpg" and
  "img0000.jpg" satisfy it, while "picture001.jpg" and "img001.jpg" don't.

  \param filename : Name from which to extract the index.
  \param format : Format of the filename.
  \return Extracted index on success, -1 otherwise.

  The following sample code shows how to use this function:
  \code
  #include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::cout << vpIoTools::getIndex("file-1.txt", "file-%d.txt") << std::endl;
    std::cout << vpIoTools::getIndex("/tmp/file0040.txt", "/tmp/file%04d.txt") << std::endl;
    std::cout << vpIoTools::getIndex("file.txt", "file%d.txt") << std::endl;
    std::cout << vpIoTools::getIndex("file03.txt", "file%02d.txt") << std::endl;
    std::cout << vpIoTools::getIndex("file-03.txt", "file%02d.txt") << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  1
  40
  -1
  3
  -1
  \endcode
*/
long vpIoTools::getIndex(const std::string &filename, const std::string &format)
{
  size_t indexBegin = format.find_last_of('%');
  size_t indexEnd = format.find_first_of('d', indexBegin);
  size_t suffixLength = format.length() - indexEnd - 1;
  // Extracting index
  if (filename.length() <= (suffixLength + indexBegin)) {
    return -1;
  }
  size_t indexLength = filename.length() - suffixLength - indexBegin;
  std::string indexSubstr = filename.substr(indexBegin, indexLength);
  std::istringstream ss(indexSubstr);
  long index = 0;
  ss >> index;
  if (ss.fail() || (index < 0) || (!ss.eof())) {
    return -1;
  }

  // Checking that format with inserted index equals filename
  char nameByFormat[FILENAME_MAX];
  snprintf(nameByFormat, FILENAME_MAX, format.c_str(), index);
  if (std::string(nameByFormat) != filename) {
    return -1;
  }
  return index;
}

/*!
   Returns the pathname string of this pathname's parent.

   \param[in] pathname : Pathname from which parent name is extracted using vpIoTools::separator.
   When the separator is not found, it returns "." as the current parent folder.

   \return The parent of the `pathname`, or an empty string if the `pathname` is empty.

   For example
   - when pathname is set to "./executable" it returns "."
   - When pathname is set to "folder/executable" it returns "folder"
   - When pathname is set to "folder\subfolder\file.cpp" it returns "folder\subfolder"
   - When pathname is set to "executable" it returns "."

 */
std::string vpIoTools::getParent(const std::string &pathname)
{
  if (pathname.size() > 0) {
    std::string convertedPathname = vpIoTools::path(pathname);

    size_t index = convertedPathname.find_last_of(vpIoTools::separator);
    if (index != std::string::npos) {
      return convertedPathname.substr(0, index);
    }

    return ".";
  }
  else {
    return "";
  }
}

/**
 * @brief Return a lower-case version of the string \b input .
 * Numbers and special characters stay the same
 *
 * @param input The input string for which we want to ensure that all the characters are in lower case.
 * @return std::string A lower-case version of the string \b input, where
 * numbers and special characters stay the same
 */
std::string vpIoTools::toLowerCase(const std::string &input)
{
  std::string out;
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  for (std::string::const_iterator it = input.cbegin(); it != input.cend(); ++it) {
#else
  for (std::string::const_iterator it = input.begin(); it != input.end(); ++it) {
#endif
    out += std::tolower(*it);
  }
  return out;
}

/**
 * @brief Return a upper-case version of the string \b input .
 * Numbers and special characters stay the same
 *
 * @param input The input string for which we want to ensure that all the characters are in upper case.
 * @return std::string A upper-case version of the string \b input, where
 * numbers and special characters stay the same
 */
std::string vpIoTools::toUpperCase(const std::string &input)
{
  std::string out;
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  for (std::string::const_iterator it = input.cbegin(); it != input.cend(); ++it) {
#else
  for (std::string::const_iterator it = input.begin(); it != input.end(); ++it) {
#endif
    out += std::toupper(*it);
  }
  return out;
}

/*!
  Returns the absolute path using realpath() on Unix systems or
  GetFullPathName() on Windows systems. \return According to realpath()
  manual, returns an absolute pathname that names the same file, whose
  resolution does not involve '.', '..', or symbolic links for Unix systems.
  According to GetFullPathName() documentation, retrieves the full path of the
  specified file for Windows systems.
 */
std::string vpIoTools::getAbsolutePathname(const std::string &pathname)
{

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::string real_path_str = pathname;
  char *real_path = realpath(pathname.c_str(), nullptr);

  if (real_path) {
    real_path_str = real_path;
    free(real_path);
  }
  return real_path_str;
#elif defined(_WIN32)
#if (!defined(WINRT))
  std::string real_path_str = pathname;
  DWORD retval = 0;
  TCHAR buffer[4096] = TEXT("");

  retval = GetFullPathName(pathname.c_str(), 4096, buffer, 0);
  if (retval != 0) {
    real_path_str = buffer;
  }
  return real_path_str;
#else
  throw(vpIoException(vpException::fatalError,
                      "Cannot get absolute path of %s: not implemented on "
                      "Universal Windows Platform",
                      pathname.c_str()));
#endif
#endif
}

/*!
  Return the file path that corresponds to the concatenated
  \e parent and \e child string files
  by adding the corresponding separator for unix or windows.

  The corresponding path is also converted. Under
  windows, all the "/" characters are converted
  into "\\" characters. Under Unix systems all the "\\"
  characters are converted into "/" characters.
 */
std::string vpIoTools::createFilePath(const std::string &parent, const std::string &child)
{
  if ((child.size() == 0) && (parent.size() == 0)) {
    return "";
  }

  if (child.size() == 0) {
    return vpIoTools::path(parent);
  }

  if (parent.size() == 0) {
    return vpIoTools::path(child);
  }

  std::string convertedParent = vpIoTools::path(parent);
  std::string convertedChild = vpIoTools::path(child);

  std::stringstream ss;
  ss << vpIoTools::separator;
  std::string stringSeparator;
  ss >> stringSeparator;

  std::string lastConvertedParentChar = convertedParent.substr(convertedParent.size() - 1);
  std::string firstConvertedChildChar = convertedChild.substr(0, 1);

  if (lastConvertedParentChar == stringSeparator) {
    convertedParent = convertedParent.substr(0, convertedParent.size() - 1);
  }

  if (firstConvertedChildChar == stringSeparator) {
    convertedChild = convertedChild.substr(1);
  }

  return std::string(convertedParent + vpIoTools::separator + convertedChild);
}

/*!
   Return whether a path is absolute.

   \return true if the pathname is absolute, false otherwise.
 */
bool vpIoTools::isAbsolutePathname(const std::string &pathname)
{
  // # Inspired by the Python 2.7.8 module.
  // # Return whether a path is absolute.
  // # Trivial in Posix, harder on the Mac or MS-DOS.
  // # For DOS it is absolute if it starts with a slash or backslash (current
  // # volume), or if a pathname after the volume letter and colon / UNC
  //  resource # starts with a slash or backslash.
  //
  //  def isabs(s):
  //     """Test whether a path is absolute"""
  //     s = splitdrive(s)[1]
  //     return s != '' and s[:1] in '/\\'
  std::string path = splitDrive(pathname).second;
  return (path.size() > 0) && ((path.substr(0, 1) == "/") || (path.substr(0, 1) == "\\"));
}

/*!
   Return true if the two pathnames are identical.

   \return true if the two pathnames are identical, false otherwise.
   \note It uses path() to normalize the path and getAbsolutePathname() to get
   the absolute pathname.
 */
bool vpIoTools::isSamePathname(const std::string &pathname1, const std::string &pathname2)
{
  // Normalize path
  std::string path1_normalize = vpIoTools::path(pathname1);
  std::string path2_normalize = vpIoTools::path(pathname2);

  // Get absolute path
  path1_normalize = vpIoTools::getAbsolutePathname(path1_normalize);
  path2_normalize = vpIoTools::getAbsolutePathname(path2_normalize);

  return (path1_normalize == path2_normalize);
}

/*!
   Split a path in a drive specification (a drive letter followed by a colon)
   and the path specification. It is always true that drivespec + pathspec ==
   p Inspired by the Python 2.7.8 module. \return a pair whose the first
   element is the drive specification and the second element the path
   specification
 */
std::pair<std::string, std::string> vpIoTools::splitDrive(const std::string &pathname)
{
  // # Split a path in a drive specification (a drive letter followed by a
  // # colon) and the path specification.
  // # It is always true that drivespec + pathspec == p
  //  def splitdrive(p):
  //     """Split a pathname into drive/UNC sharepoint and relative path
  //     specifiers. Returns a 2-tuple (drive_or_unc, path); either part may be
  //     empty.
  //
  //     If you assign
  //         result = splitdrive(p)
  //     It is always true that:
  //         result[0] + result[1] == p
  //
  //     If the path contained a drive letter, drive_or_unc will contain
  //     everything up to and including the colon.  e.g. splitdrive("c:/dir")
  //     returns ("c:", "/dir")
  //
  //     If the path contained a UNC path, the drive_or_unc will contain the host
  //     name and share up to but not including the fourth directory separator
  //     character. e.g. splitdrive("//host/computer/dir") returns
  //     ("//host/computer", "/dir")
  //
  //     Paths cannot contain both a drive letter and a UNC path.
  //
  //     """
  //     if len(p) > 1:
  //         normp = p.replace(altsep, sep)
  //         if (normp[0:2] == sep*2) and (normp[2] != sep):
  //             # is a UNC path:
  //             # vvvvvvvvvvvvvvvvvvvv drive letter or UNC path
  //             # \\machine\mountpoint\directory\etc\...
  //             #           directory ^^^^^^^^^^^^^^^
  //             index = normp.find(sep, 2)
  //             if index == -1:
  //                 return '', p
  //             index2 = normp.find(sep, index + 1)
  //             # a UNC path can't have two slashes in a row
  //             # (after the initial two)
  //             if index2 == index + 1:
  //                 return '', p
  //             if index2 == -1:
  //                 index2 = len(p)
  //             return p[:index2], p[index2:]
  //         if normp[1] == ':':
  //             return p[:2], p[2:]
  //     return '', p

  // On Unix, the drive is always empty.
  // On the Mac, the drive is always empty (don't use the volume name -- it
  // doesn't have the same  syntactic and semantic oddities as DOS drive
  // letters, such as there being a separate current directory per drive).
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
  return std::pair<std::string, std::string>("", pathname);
#else
  const std::string sep = "\\";
  const std::string sepsep = "\\\\";
  const std::string altsep = "/";

  if (pathname.size() > 1) {
    std::string normPathname = pathname;
    std::replace(normPathname.begin(), normPathname.end(), *altsep.c_str(), *sep.c_str());

    if (normPathname.substr(0, 2) == sepsep && normPathname.substr(2, 1) != sep) {
      // is a UNC path:
      // vvvvvvvvvvvvvvvvvvvv drive letter or UNC path
      // \\machine\mountpoint\directory\etc\...
      //           directory ^^^^^^^^^^^^^^^
      size_t index = normPathname.find(sep, 2);
      if (index == std::string::npos) {
        return std::pair<std::string, std::string>("", pathname);
      }

      size_t index2 = normPathname.find(sep, index + 1);
      // # a UNC path can't have two slashes in a row
      // # (after the initial two)
      if (index2 == index + 1) {
        return std::pair<std::string, std::string>("", pathname);
      }

      if (index2 == std::string::npos) {
        index2 = pathname.size();
      }

      return std::pair<std::string, std::string>(pathname.substr(0, index2), pathname.substr(index2));
    }

    if (normPathname[1] == ':') {
      return std::pair<std::string, std::string>(pathname.substr(0, 2), pathname.substr(2));
    }
  }

  return std::pair<std::string, std::string>("", pathname);
#endif
}

/*!
  Split a chain.
  \param chain : Input chain to split.
  \param sep : Character separator.
  \return A vector that contains all the subchains.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpIoTools.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    {
      std::string chain("/home/user;/usr/local/include;/usr/include");
      std::string sep = ";";

      std::vector<std::string> subChain = vpIoTools::splitChain(chain, sep);
      std::cout << "Found the following subchains: " << std::endl;
      for (size_t i=0; i < subChain.size(); ++i)
        std::cout << subChain[i] << std::endl;
    }

    {
      std::string chain("This is an other example");
      std::string sep = " ";

      std::vector<std::string> subChain = vpIoTools::splitChain(chain, sep);
      std::cout << "Found the following subchains: " << std::endl;
      for (size_t i=0; i < subChain.size(); ++i)
        std::cout << subChain[i] << std::endl;
    }
  }
  \endcode

  It produces the following output:
  \code
  Found the following subchains:
  /home/user
  /usr/local/include
  /usr/include
  Found the following subchains:
  This
  is
  an
  other
  example
  \endcode
 */
std::vector<std::string> vpIoTools::splitChain(const std::string &chain, const std::string &sep)
{
  size_t startIndex = 0;

  std::string chainToSplit = chain;
  std::vector<std::string> subChain;
  size_t sepIndex = chainToSplit.find(sep);

  while (sepIndex != std::string::npos) {
    std::string sub = chainToSplit.substr(startIndex, sepIndex);
    if (!sub.empty()) {
      subChain.push_back(sub);
    }
    chainToSplit = chainToSplit.substr(sepIndex + 1, chain.size() - 1);

    sepIndex = chainToSplit.find(sep);
  }
  if (!chainToSplit.empty()) {
    subChain.push_back(chainToSplit);
  }

  return subChain;
}

/*!
   List of files in directory, in alphabetical order.
   There is no difference if pathname contains terminating backslash or not
   Unlike scandir(), does not return "." and ".."
   \param pathname : path to directory
   \return A vector of files' names in that directory
 */
std::vector<std::string> vpIoTools::getDirFiles(const std::string &pathname)
{

  if (!checkDirectory(pathname)) {
    throw(vpIoException(vpException::fatalError, "Directory %s doesn't exist'", pathname.c_str()));
  }
  std::string dirName = path(pathname);

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX

  std::vector<std::string> files;
  struct dirent **list = nullptr;
  int filesCount = scandir(dirName.c_str(), &list, nullptr, nullptr);
  if (filesCount == -1) {
    throw(vpIoException(vpException::fatalError, "Cannot read files of directory %s", dirName.c_str()));
  }
  for (int i = 0; i < filesCount; ++i) {
    std::string fileName = list[i]->d_name;
    if ((fileName != ".") && (fileName != "..")) {
      files.push_back(fileName);
    }
    free(list[i]);
  }
  free(list);
  std::sort(files.begin(), files.end());
  return files;

#elif defined(_WIN32)
#if (!defined(WINRT))

  std::vector<std::string> files;
  std::string fileMask = dirName;
  fileMask.append("\\*");
  WIN32_FIND_DATA FindFileData;
  HANDLE hFind = FindFirstFile(fileMask.c_str(), &FindFileData);
  // Directory is empty
  if (HandleToLong(&hFind) == ERROR_FILE_NOT_FOUND) {
    return files;
  }
  if (hFind == INVALID_HANDLE_VALUE) {
    throw(vpIoException(vpException::fatalError, "Cannot read files of directory %s", dirName.c_str()));
  }
  do {
    std::string fileName = FindFileData.cFileName;
    if (fileName != "." && fileName != "..") {
      files.push_back(fileName);
    }
  } while (FindNextFile(hFind, &FindFileData));
  FindClose(hFind);
  std::sort(files.begin(), files.end());
  return files;

#else
  throw(vpIoException(vpException::fatalError,
                      "Cannot read files of directory %s: not implemented on "
                      "Universal Windows Platform",
                      dirName.c_str()));
#endif
#endif
}

END_VISP_NAMESPACE
