/****************************************************************************
 *
 * $Id: vpIoTools.h,v 1.9 2008-11-13 11:14:34 fspindle Exp $
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
