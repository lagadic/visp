/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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


#ifndef vpIoTools_HH
#define vpIoTools_HH

/*!
  \file vpIoTools.h
  \brief io basic tools
*/

#include <iostream>
#include <visp/vpConfig.h>



/*!
  \class vpIoTools
  \ingroup FileDirectories
  \brief File and directories basic tools.

  The example below shows how to manipulate the functions of this
  class to create first a directory which name corresponds to the user
  name and then create a file in this directory.

  \code
#include <iostream>
#include <string>
#include <fstream>
#include <visp/vpIoTools.h>

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
  filename = vpIoTools::path(filename); // Under Windows converts the filename string into "username\\file.txt"
  f.open(filename.c_str());
  f.close();
}
  \endcode

*/

class VISP_EXPORT vpIoTools
{

public:
  static void getUserName(std::string &username);
  static std::string getenv(const char *env);
  static std::string getenv(std::string &env);
  static bool checkDirectory(const char *dirname);
  static bool checkDirectory(const std::string dirname);
  static void makeDirectory(const char *dirname);
  static void makeDirectory(const std::string dirname);
  static bool checkFilename(const char *filename);
  static bool checkFilename(const std::string filename);

  static std::string path(const char * pathname);
  static std::string path(const std::string& _p);
} ;


#endif
