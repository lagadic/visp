/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Test functions in vpIoTools.
 *
 *****************************************************************************/
/*!
  \example testIoTools.cpp

  \brief Test functions in IoTools.
*/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <visp3/core/vpIoTools.h>

namespace
{
template <typename T>
void checkReadBinaryValue(std::ifstream &file, const T checkValue)
{
  T value = (T)10;
  vpIoTools::readBinaryValueLE(file, value);
  if (value != checkValue) {
    std::stringstream ss;
    ss << "Read: " << value << " ; Expected: " << checkValue;
    throw vpException(vpException::badValue, ss.str());
  }
}

template <>
void checkReadBinaryValue<float>(std::ifstream &file, const float checkValue)
{
  float value = 10.0f;
  vpIoTools::readBinaryValueLE(file, value);
  if (!vpMath::equal(value, checkValue, std::numeric_limits<float>::epsilon())) {
    std::stringstream ss;
    ss << "Read: " << value << " ; Expected: " << checkValue;
    throw vpException(vpException::badValue, ss.str());
  }
}

template <>
void checkReadBinaryValue<double>(std::ifstream &file, const double checkValue)
{
  double value = 10.0;
  vpIoTools::readBinaryValueLE(file, value);
  if (!vpMath::equal(value, checkValue, std::numeric_limits<double>::epsilon())) {
    std::stringstream ss;
    ss << "Read: " << value << " ; Expected: " << checkValue;
    throw vpException(vpException::badValue, ss.str());
  }
}
}

int main(int argc, const char **argv)
{
  const char c = vpIoTools::separator;
  if (c == '\\') {
    std::cout << "The directory separator character is '" << c << "' (Windows platform)." << std::endl;
  } else {
    std::cout << "The directory separator character is '" << c << "' (Unix like platform)." << std::endl;
  }

#if defined(_WIN32)
  std::string pathname = "C:\\Program Files (x86)\\Java\\jre7";
#else
  std::string pathname = "/usr/bin/java";
#endif

  std::cout << "Parent of " << pathname << " is " << vpIoTools::getParent(pathname) << std::endl;
  std::cout << "Name of " << pathname << " is " << vpIoTools::getName(pathname) << std::endl;

  if (argc == 3 && std::string(argv[1]) == std::string("-i")) {
    std::cout << "Parent of " << argv[2] << " is " << vpIoTools::getParent(argv[2]) << std::endl;
    std::cout << "Name of " << argv[2] << " is " << vpIoTools::getName(argv[2]) << std::endl;
  }

  std::string windowsPathnameStyle = "\\usr\\bin\\java";
  std::cout << "Parent of " << windowsPathnameStyle << " is " << vpIoTools::getParent(windowsPathnameStyle)
            << std::endl;
  std::cout << "Name of " << windowsPathnameStyle << " is " << vpIoTools::getName(windowsPathnameStyle) << std::endl;

  std::string parent = "/usr/toto/", child = "\\blabla\\java";
  std::cout << "parent=" << vpIoTools::path(parent) << " ; child=" << vpIoTools::path(child) << std::endl;
  std::cout << "Create file path from parent=" << parent << " and child=" << child << " is "
            << vpIoTools::createFilePath(parent, child) << std::endl;

  std::string expandPath = "~/Documents/fictional directory/fictional file";
  std::cout << "Path for " << expandPath << " is " << vpIoTools::path(expandPath) << std::endl;

  std::cout << "Test get name with an empty pathname=" << vpIoTools::getName("") << std::endl;
  std::cout << "Get parent with an empty pathname=" << vpIoTools::getParent("") << std::endl;
  std::cout << "Get parent with a filename=" << vpIoTools::getParent("my_file.txt") << std::endl;
  expandPath = "~/Documents/fictional dir/fictional file.txt";
  std::cout << "Get name with a unix expand pathname " << expandPath << "=" << vpIoTools::getName(expandPath)
            << std::endl;
  std::cout << "Get parent with a unix expand pathname " << expandPath << "=" << vpIoTools::getParent(expandPath)
            << std::endl;

  pathname = "c:/dir";
  std::cout << "pathname=" << vpIoTools::splitDrive(pathname).first << " ; " << vpIoTools::splitDrive(pathname).second
            << std::endl;

  std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

  pathname = "c:/dir/fictional directory/fictional file.txt";
  std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

  pathname = "/home/user/Documents/fictional directory/fictional file.txt";
  std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

  pathname = "~/Documents/fictional directory/fictional file.txt";
  std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

  pathname = "fictional directory/fictional file.txt";
  std::cout << "isAbsolutePath of " << pathname << "=" << vpIoTools::isAbsolutePathname(pathname) << std::endl;

  // Test vpIoTools::splitDrive
  unsigned int nbFail = 0, nbOk = 0;
#if defined(_WIN32)
  if (strcmp(vpIoTools::splitDrive("c:\\foo\\bar").first.c_str(), "c:") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("c:\\foo\\bar").first << " should be=c:" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("c:\\foo\\bar").second.c_str(), "\\foo\\bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("c:\\foo\\bar").second << " should be=\\foo\\bar" << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("c:/foo/bar").first.c_str(), "c:") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("c:/foo/bar").first << " should be=c:" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("c:/foo/bar").second.c_str(), "/foo/bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("c:/foo/bar").second << " should be=/foo/bar" << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("\\\\conky\\mountpoint\\foo\\bar").first.c_str(), "\\\\conky\\mountpoint") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\conky\\mountpoint\\foo\\bar").first
              << " should be=\\\\conky\\mountpoint" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("\\\\conky\\mountpoint\\foo\\bar").second.c_str(), "\\foo\\bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\conky\\mountpoint\\foo\\bar").second << " should be=\\foo\\bar"
              << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("//conky/mountpoint/foo/bar").first.c_str(), "//conky/mountpoint") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("//conky/mountpoint/foo/bar").first << " should be=//conky/mountpoint"
              << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("//conky/mountpoint/foo/bar").second.c_str(), "/foo/bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("//conky/mountpoint/foo/bar").second << " should be=/foo/bar"
              << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("\\\\\\conky\\mountpoint\\foo\\bar").first.c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\\\conky\\mountpoint\\foo\\bar").first
              << " should be=" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("\\\\\\conky\\mountpoint\\foo\\bar").second.c_str(),
             "\\\\\\conky\\mountpoint\\foo\\bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\\\conky\\mountpoint\\foo\\bar").second
              << " should be=\\\\\\conky\\mountpoint\\foo\\bar" << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("///conky/mountpoint/foo/bar").first.c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("///conky/mountpoint/foo/bar").first << " should be=" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("///conky/mountpoint/foo/bar").second.c_str(), "///conky/mountpoint/foo/bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("///conky/mountpoint/foo/bar").second
              << " should be=///conky/mountpoint/foo/bar" << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("\\\\conky\\\\mountpoint\\foo\\bar").first.c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\conky\\\\mountpoint\\foo\\bar").first
              << " should be=" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("\\\\conky\\\\mountpoint\\foo\\bar").second.c_str(),
             "\\\\conky\\\\mountpoint\\foo\\bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("\\\\conky\\\\mountpoint\\foo\\bar").second
              << " should be=\\\\conky\\\\mountpoint\\foo\\bar" << std::endl;
  }

  if (strcmp(vpIoTools::splitDrive("//conky//mountpoint/foo/bar").first.c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("//conky//mountpoint/foo/bar").first << " should be=" << std::endl;
  }
  if (strcmp(vpIoTools::splitDrive("//conky//mountpoint/foo/bar").second.c_str(), "//conky//mountpoint/foo/bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::splitDrive("//conky//mountpoint/foo/bar").second
              << " should be=//conky//mountpoint/foo/bar" << std::endl;
  }

  std::cout << "Test vpIoTools::splitDrive (Win32) - passed: " << nbOk << "/" << (nbOk + nbFail) << std::endl;

  if (nbFail) {
    std::cerr << "Failed test: vpIoTools::splitDrive (Win32)" << std::endl;
    return EXIT_FAILURE;
  }
#endif

// Test vpIoTools::getFileExtension
#if defined(_WIN32)
  nbFail = 0;
  nbOk = 0;

  if (strcmp(vpIoTools::getFileExtension("foo.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("/foo/foo.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("/foo/foo.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension(".ext").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension(".ext") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("\\foo.ext\\foo").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("\\foo.ext\\foo") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("foo.ext\\").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.ext\\") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("foo.bar.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.bar.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("xx/foo.bar.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("xx/foo.bar.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("xx\\foo.bar.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("xx\\foo.bar.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("c:a/b\\c.d").c_str(), ".d") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("c:a/b\\c.d") << " should be=.d" << std::endl;
  }

  std::cout << "Test vpIoTools::getFileExtension (WIN32 platform) - passed: " << nbOk << "/" << (nbOk + nbFail)
            << std::endl;

  if (nbFail) {
    std::cerr << "Failed test: vpIoTools::getFileExtension (WIN32 platform)" << std::endl;
    return EXIT_FAILURE;
  }
#else
  nbFail = 0;
  nbOk = 0;

  if (strcmp(vpIoTools::getFileExtension("foo.bar").c_str(), ".bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.bar") << " should be=.bar" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("foo.boo.bar").c_str(), ".bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.boo.bar") << " should be=.bar" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("foo.boo.biff.bar").c_str(), ".bar") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("foo.boo.biff.bar") << " should be=.bar" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension(".csh.rc").c_str(), ".rc") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension(".csh.rc") << " should be=.rc" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("nodots").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("nodots") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension(".cshrc").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension(".cshrc") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("...manydots").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("...manydots") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("...manydots.ext").c_str(), ".ext") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("...manydots.ext") << " should be=.ext" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension(".").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension(".") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("..").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("..") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("........").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("........") << " should be=" << std::endl;
  }

  if (strcmp(vpIoTools::getFileExtension("").c_str(), "") == 0) {
    nbOk++;
  } else {
    nbFail++;
    std::cout << "Fail=" << vpIoTools::getFileExtension("") << " should be=" << std::endl;
  }

  std::cout << "Test vpIoTools::getFileExtension (Unix-like platform) - passed: " << nbOk << "/" << (nbOk + nbFail)
            << std::endl;
#endif

  // Test makeDirectory()
  try {
    std::string username, directory_filename;
    vpIoTools::getUserName(username);
#if defined(_WIN32)
    std::string tmp_dir = "C:/temp/" + username;
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    std::string tmp_dir = "/tmp/" + username;
#endif
#if defined(_WIN32)
    directory_filename = tmp_dir + "/test_directory1/test directory 2/";
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    directory_filename = tmp_dir + "/test_directory1/test directory 2/";
#endif
    vpIoTools::makeDirectory(directory_filename);
    vpIoTools::makeDirectory(directory_filename);
    std::cout << "Create directories: " << directory_filename
              << " ; check: " << vpIoTools::checkDirectory(directory_filename) << std::endl;

#if defined(_WIN32)
    directory_filename = tmp_dir + "/test_directory1/test directory 3";
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    directory_filename = tmp_dir + "/test_directory1/test directory 3";
#endif
    vpIoTools::makeDirectory(directory_filename);
    std::cout << "Create directories: " << directory_filename
              << " ; check: " << vpIoTools::checkDirectory(directory_filename) << std::endl;

#if defined(_WIN32)
    directory_filename = "C:\\temp/" + username + "\\test_directory1\\test directory 4";
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    directory_filename = "/tmp\\" + username + "\\test_directory1\\test directory 4";
#endif
    vpIoTools::makeDirectory(directory_filename);
    vpIoTools::makeDirectory(directory_filename);
    std::cout << "Create directories: " << directory_filename
              << " ; check: " << vpIoTools::checkDirectory(directory_filename) << std::endl;

#if defined(_WIN32)
    directory_filename = "C:\\temp/" + username + "\\test_directory1\\test directory 5 . dir/test directory 6";
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
    directory_filename = "/tmp\\" + username + "\\test_directory1\\test directory 5 . dir/test directory 6";
#endif
    vpIoTools::makeDirectory(directory_filename);
    std::cout << "Create directories: " << directory_filename
              << " ; check: " << vpIoTools::checkDirectory(directory_filename) << std::endl;

    //Delete test directory
    if (!vpIoTools::remove(tmp_dir + "/test_directory1")) {
      std::cerr << "Cannot remove directory: " << tmp_dir << "/test_directory1" << std::endl;
    }
  } catch (const vpException &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Get the user login name
  std::string username = "";
  vpIoTools::getUserName(username);
  std::ofstream dummy_file;

// Test isSamePathname()
#if defined(_WIN32)
  std::string path1 = "tmp/test/file.txt";
  std::string path2 = "tmp/test/../test/file.txt";

  nbOk = 0;
  nbFail = 0;
  bool res;

  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path1 = ".\\tmp/test/file.txt";
  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path1 = ".\\tmp/test\\../fake dir/..\\test\\file.txt";
  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path2 = "/tmp/test/../test/file.txt";
  res = vpIoTools::isSamePathname(path1, path2); // False
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk : nbOk + 1;
  nbFail = res ? nbFail + 1 : nbFail;

  std::cout << "Test vpIoTools::isSamePathname (WIN32 platform) - passed: " << nbOk << "/" << (nbOk + nbFail)
            << std::endl;
  if (nbFail) {
    std::cerr << "Failed test: vpIoTools::isSamePathname (WIN32 platform)" << std::endl;
    return EXIT_FAILURE;
  }
#else
  // realpath requires not fake path, so we create dummy file and directories

  vpIoTools::makeDirectory("/tmp/" + username + "/test");
  vpIoTools::makeDirectory("/tmp/" + username + "/dummy dir");

  std::string path1 = "/tmp/" + username + "/test/file.txt";
  std::string path2 = "/tmp/" + username + "/test/../test/file.txt";
  dummy_file.open(path1.c_str());
  if (!dummy_file.is_open()) {
    return EXIT_SUCCESS;
  }
  dummy_file.close();

  nbOk = 0;
  nbFail = 0;
  bool res;

  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path1 = "\\tmp/" + username + "/./test/file.txt";
  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path1 = "\\tmp/" + username + "/test\\../dummy dir/..\\test\\file.txt";
  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  path2 = "/tmp/" + username + "/test/../test";
  res = vpIoTools::isSamePathname(path1, path2); // False
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk : nbOk + 1;
  nbFail = res ? nbFail + 1 : nbFail;

  path1 = "/tmp/" + username + "/test/";
  res = vpIoTools::isSamePathname(path1, path2); // True
  std::cout << "vpIoTools::isSamePathname(" << path1 << ", " << path2 << ")? " << res << std::endl;
  nbOk = res ? nbOk + 1 : nbOk;
  nbFail = res ? nbFail : nbFail + 1;

  std::cout << "Test vpIoTools::isSamePathname (Unix platform) - passed: " << nbOk << "/" << (nbOk + nbFail)
            << std::endl;

  //Delete test directory
  if (!vpIoTools::remove("/tmp/" + username + "/test")) {
    std::cerr << "Cannot remove directory: " << "/tmp/" << username << "/test" << std::endl;
  }
  if (!vpIoTools::remove("/tmp/" + username + "/dummy dir")) {
    std::cerr << "Cannot remove directory: " << "/tmp/" << username << "/dummy dir" << std::endl;
  }

  if (nbFail) {
    std::cerr << "Failed test: vpIoTools::isSamePathname (Unix platform)" << std::endl;
    return EXIT_FAILURE;
  }
#endif

  // Test checkFilename()
  vpIoTools::makeDirectory("/tmp/" + username + "/directory (1) with ' quote and spaces");
  path1 = "/tmp/" + username +
          "/directory (1) with ' quote and spaces/file with ' quote (1) and "
          "spaces.txt";
  dummy_file.open(path1.c_str());
  if (!dummy_file.is_open()) {
    return EXIT_SUCCESS;
  }
  dummy_file.close();

  if (!vpIoTools::checkFilename(path1)) {
    std::cerr << "Problem with checkFilename(" << path1 << ")!" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Test vpIoTools::checkFilename() is ok." << std::endl;

  //Delete test directory
  if (!vpIoTools::remove("/tmp/" + username + "/directory (1) with ' quote and spaces")) {
    std::cerr << "Cannot remove directory: " << "/tmp/" << username << "/directory (1) with ' quote and spaces" << std::endl;
  }

  // Test endianness
  {
    std::string filename_endianness =  vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "endianness/test_endianness_little_endian.bin");
    std::ifstream file_endianness(filename_endianness.c_str(), std::ios::in | std::ios::binary);
    if (file_endianness.is_open()) {
      checkReadBinaryValue<short>(file_endianness, std::numeric_limits<short>::min());
      checkReadBinaryValue<short>(file_endianness, std::numeric_limits<short>::max());

      checkReadBinaryValue<unsigned short>(file_endianness, std::numeric_limits<unsigned short>::min());
      checkReadBinaryValue<unsigned short>(file_endianness, std::numeric_limits<unsigned short>::max());

      checkReadBinaryValue<int>(file_endianness, std::numeric_limits<int>::min());
      checkReadBinaryValue<int>(file_endianness, std::numeric_limits<int>::max());

      checkReadBinaryValue<unsigned int>(file_endianness, std::numeric_limits<unsigned int>::min());
      checkReadBinaryValue<unsigned int>(file_endianness, std::numeric_limits<unsigned int>::max());

      checkReadBinaryValue<float>(file_endianness, -std::numeric_limits<float>::max());
      checkReadBinaryValue<float>(file_endianness, std::numeric_limits<float>::max());

      checkReadBinaryValue<double>(file_endianness, -std::numeric_limits<double>::max());
      checkReadBinaryValue<double>(file_endianness, std::numeric_limits<double>::max());

      std::cout << "Test endianness is ok." << std::endl;
    } else {
      std::cout << "Cannot open file: " << filename_endianness << std::endl;
    }
  }

  std::cout << std::endl << "End" << std::endl;
  return EXIT_SUCCESS;
}
