/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Example of  command line parsing.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file parse-argv2.cpp

  \brief Parsing command line arguments.
*/


/*!
  \example parse-argv2.cpp

  Example of command line parsing.
*/



#include <visp3/core/vpDebug.h>
#include <visp3/io/vpParseArgv.h>
#include <stdio.h>
#include <sstream>
#include <iomanip>

int
main(int argc, const char ** argv)
{
  try {
    using ::std::cout;
    using ::std::endl;

    int    i_val = 3;
    float  f_val = 3.14f;
    double d_val = 3.1415;
    int    flag = 0;

    vpParseArgv::vpArgvInfo argTable[] =
    {
      {"-flag", vpParseArgv::ARGV_CONSTANT, 0, (char *) &flag,
       "Flag enabled."},
      {"-integer", vpParseArgv::ARGV_INT, (char*) NULL, (char *) &i_val,
       "An integer value."},
      {"-float", vpParseArgv::ARGV_FLOAT, (char*) NULL, (char *) &f_val,
       "A float value."},
      {"-double", vpParseArgv::ARGV_DOUBLE, (char*) NULL, (char *) &d_val,
       "A double value."},
      {(char*) NULL, vpParseArgv::ARGV_END, (char*) NULL, (char*) NULL, (char*) NULL}
    } ;

    // Read the command line options
    if(vpParseArgv::parse(&argc, argv, argTable, 0)) {
      return (-1);
    }

    cout << "Your parameters: " << endl;
    cout << "  Integer value: " << i_val << endl;
    cout << "  Float   value: " << f_val << endl;
    cout << "  Double  value: " << d_val << endl << endl;
    cout << "  Flag         : " << flag << endl << endl;

    cout << "Call  " << argv[0]
         << " -h to see how to change these parameters." << endl;

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return 1;
  }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
