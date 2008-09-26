/****************************************************************************
 *
 * $Id: vpIoTools.h,v 1.8 2008-09-26 15:20:58 fspindle Exp $
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

*/

class VISP_EXPORT vpIoTools
{

public:
  static void getUserName(std::string &username);
  static bool checkDirectory(const char *dirname);
  static bool checkDirectory(const std::string dirname);
  static void makeDirectory(const char *dirname);
  static void makeDirectory(const std::string dirname);
  static bool checkFilename(const char *filename);
  static bool checkFilename(const std::string filename);

  static std::string path(const char * _p);
  static std::string path(const std::string& _p);
} ;


#endif
