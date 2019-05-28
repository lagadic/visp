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

#include <iomanip>
#include <sstream>
#include <stdio.h>

#include <visp3/core/vpDebug.h>
#include <visp3/io/vpParseArgv.h>

int main(int argc, const char **argv)
{
  try {
    using ::std::cout;
    using ::std::endl;

    bool bool_val = false;
    int int_val = 3;
    long long_val = 33333333;
    float float_val = 3.14f;
    double double_val = 3.1415;
    char *string_val = NULL;

    vpParseArgv::vpArgvInfo argTable[] = {
        {"-bool", vpParseArgv::ARGV_CONSTANT_BOOL, 0, (char *)&bool_val, "Bool enabled."},
        {"-integer", vpParseArgv::ARGV_INT, (char *)NULL, (char *)&int_val, "An integer value."},
        {"-long", vpParseArgv::ARGV_LONG, (char *)NULL, (char *)&long_val, "A long value."},
        {"-float", vpParseArgv::ARGV_FLOAT, (char *)NULL, (char *)&float_val, "A float value."},
        {"-double", vpParseArgv::ARGV_DOUBLE, (char *)NULL, (char *)&double_val, "A double value."},
        {"-string", vpParseArgv::ARGV_STRING, (char *)NULL, (char *)&string_val, "A chain value."},
        {"-h", vpParseArgv::ARGV_HELP, (char *)NULL, (char *)NULL, "Print the help."},
        {(char *)NULL, vpParseArgv::ARGV_END, (char *)NULL, (char *)NULL, (char *)NULL}};

    // Read the command line options
    if (vpParseArgv::parse(&argc, argv, argTable, vpParseArgv::ARGV_NO_DEFAULTS)) {
      return (-1);
    }

    cout << "Your parameters: " << endl;
    cout << "  Bool    value: " << bool_val << endl;
    cout << "  Integer value: " << int_val << endl;
    cout << "  Long    value: " << long_val << endl;
    cout << "  Float   value: " << float_val << endl;
    cout << "  Double  value: " << double_val << endl;
    if (string_val != NULL)
      cout << "  String  value: " << string_val << endl;
    else
      cout << "  String  value: \"\"" << endl << endl;

    cout << "Call  " << argv[0] << " -h to see how to change these parameters." << endl;

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e.getStringMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
