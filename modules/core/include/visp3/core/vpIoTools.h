/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Directory management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpIoTools_HH
#define vpIoTools_HH

/*!
  \file vpIoTools.h
  \brief File and directories basic tools.
 */

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <visp3/core/vpColor.h>

/*!
  \class vpIoTools
  \ingroup group_core_files_io
  \brief File and directories basic tools.

  The example below shows how to manipulate the functions of this
  class to create first a directory which name corresponds to the user
  name and then create a file in this directory.

  \code
#include <iostream>
#include <string>
#include <fstream>
#include <visp3/core/vpIoTools.h>

int main()
{
  std::string username;
  vpIoTools::getUserName(username);

  // Test if a username directory exist. If no try to create it
  if (vpIoTools::checkDirectory(username) == false) {
     try {
       // Create a directory with name "username"
       vpIoTools::makeDirectory(username);
     }
     catch (...) {
       std::cout << "Cannot create " << username << " directory" << std::endl;
       return false;
     }
   }
  // Create a empty filename with name "username/file.txt"
  std::ofstream f;
  std::string filename = username + "/file.txt";
  // Under Windows converts the filename string into "username\\file.txt"
  filename = vpIoTools::path(filename);
  std::cout << "Create: " << filename << std::endl;
  f.open(filename.c_str());
  f.close();

  // Rename the file
  std::string newfilename = username + "/newfile.txt";
  std::cout << "Rename: " << filename << " in: " << newfilename << std::endl;
  if (vpIoTools::rename(filename, newfilename) == false)
    std::cout << "Unable to rename: " << filename << std::endl;

  // Remove the file
  std::cout << "Remove: " << newfilename << std::endl;
  if (vpIoTools::remove(newfilename) == false)
    std::cout << "Unable to remove: " << newfilename << std::endl;
}
  \endcode

  The example below shows how to read a configuration file and how to create a name
  for experiment files. We assume the following file "/home/user/demo/config.txt" :
  \code
expNumber 2
save 0
lambda 0.4
use2D 0
use3D 1
  \endcode

  \code
#include <iostream>
#include <string>
#include <visp3/core/vpIoTools.h>

int main()
{
  // reading configuration file
  vpIoTools::loadConfigFile("/home/user/demo/config.txt");
  std::string nExp;vpIoTools::readConfigVar("expNumber", nExp); // nExp <- "2"
  double lambda;vpIoTools::readConfigVar("lambda", lambda);     // lambda <- 0.4
  bool use2D;vpIoTools::readConfigVar("use2D", use2D);          // use2D <- false
  bool use3D;vpIoTools::readConfigVar("use3D", use3D);          // use3D <- true
  bool doSave;vpIoTools::readConfigVar("save", doSave);         //  doSave <- false

  // creating name for experiment files
  vpIoTools::setBaseDir("/home/user/data");
  // full name <- "/home/user/data/exp2"
  vpIoTools::setBaseName("exp" + nExp);
  // full name <- "/home/user/data/exp2" since use2D==false
  vpIoTools::addNameElement("2D", use2D);
  // full name <- "/home/user/data/exp2_3D"
  vpIoTools::addNameElement("3D", use3D);
  // full name <- "/home/user/data/exp2_3D_lambda0.4"
  vpIoTools::addNameElement("lambda", lambda);

  // Saving file.Would copy "/home/user/demo/config.txt" to
  // "/home/user/data/exp2_3D_lambda0.4_config.txt" if doSave was true
  vpIoTools::saveConfigFile(doSave);
  // create sub directory
  vpIoTools::createBaseNamePath();  // creates "/home/user/data/exp2_3D_lambda0.4/"
}
  \endcode

 */

class VISP_EXPORT vpIoTools
{

public:
  static const std::string &getBuildInformation();
  static void getUserName(std::string &username);
  static std::string getUserName();
  static std::string getenv(const char *env);
  static std::string getenv(const std::string &env);
  static std::string getViSPImagesDataPath();
  static void getVersion(const std::string &version, unsigned int &major, unsigned int &minor, unsigned int &patch);
  static bool checkDirectory(const char *dirname);
  static bool checkDirectory(const std::string &dirname);
  static bool checkFilename(const char *filename);
  static bool checkFilename(const std::string &filename);
  static bool copy(const char *src, const char *dst);
  static bool copy(const std::string &src, const std::string &dst);
  static void makeDirectory(const char *dirname);
  static void makeDirectory(const std::string &dirname);
  static bool remove(const char *filename);
  static bool remove(const std::string &filename);
  static bool rename(const char *oldfilename, const char *newfilename);
  static bool rename(const std::string &oldfilename, const std::string &newfilename);

  static std::string path(const char *pathname);
  static std::string path(const std::string &pathname);

  /*!
         Define the directory separator character, backslash ('\') for windows
     platform or slash ('/') otherwise.
   */
  static const char separator =
#if defined(_WIN32)
      '\\';
#else
      '/';
#endif

  static std::string getAbsolutePathname(const std::string &pathname);
  static std::string getFileExtension(const std::string &pathname, const bool checkFile = false);
  static std::string getName(const std::string &pathname);
  static std::string getNameWE(const std::string &pathname);
  static std::string getParent(const std::string &pathname);
  static std::string createFilePath(const std::string &parent, const std::string &child);
  static bool isAbsolutePathname(const std::string &pathname);
  static bool isSamePathname(const std::string &pathname1, const std::string &pathname2);
  static std::pair<std::string, std::string> splitDrive(const std::string &pathname);
  static std::vector<std::string> splitChain(const std::string &chain, const std::string &sep);
  static std::vector<std::string> getDirFiles(const std::string &dirname);

  /*!
    @name Configuration file parsing
  */
  //@{
  // read configuration file
  static bool loadConfigFile(const std::string &confFile);
  static bool readConfigVar(const std::string &var, float &value);
  static bool readConfigVar(const std::string &var, double &value);
  static bool readConfigVar(const std::string &var, int &value);
  static bool readConfigVar(const std::string &var, unsigned int &value);
  static bool readConfigVar(const std::string &var, bool &value);
  static bool readConfigVar(const std::string &var, std::string &value);
  static bool readConfigVar(const std::string &var, vpColor &value);
  static bool readConfigVar(const std::string &var, vpArray2D<double> &value, const unsigned int &nCols = 0,
                            const unsigned int &nRows = 0);

  // construct experiment filename & path
  static void setBaseName(const std::string &s);
  static void setBaseDir(const std::string &dir);
  static void addNameElement(const std::string &strTrue, const bool &cond = true, const std::string &strFalse = "");
  static void addNameElement(const std::string &strTrue, const double &val);
  static std::string getBaseName();
  static std::string getFullName();

  // write files
  static void saveConfigFile(const bool &actuallySave = true);
  static void createBaseNamePath(const bool &empty = false);
  //@}

protected:
  static std::string baseName;
  static std::string baseDir;
  static std::string configFile;
  static std::vector<std::string> configVars;
  static std::vector<std::string> configValues;

  static int mkdir_p(const char *path, const int mode);
};
#endif
