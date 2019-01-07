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
  \file parse-argv1.cpp

  \brief Parsing command line arguments.
*/

/*!
  \example parse-argv1.cpp

  Example of command line parsing.
*/

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpDebug.h>
#include <visp3/io/vpParseArgv.h>
// List of allowed command line options
#define GETOPTARGS "d:f:i:h"

void usage(const char *name, const char *badparam, int i_val, float f_val, double d_val);
bool getOptions(int argc, const char **argv, int &i_val, float &f_val, double &d_val);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param i_val : An integer.
  \param f_val : A float.
  \param d_val : A double.

*/
void usage(const char *name, const char *badparam, int i_val, float f_val, double d_val)
{
  fprintf(stdout, "\n\
Parsing command line arguments example.\n\
\n\
SYNOPSIS\n\
  %s [-i <integer>] [-f <float>] [-d <double> [-h]\n\
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <integer>                                         %d\n\
     An integer value.\n\
\n\
  -f <float>                                           %f\n\
     A float value.\n\
\n\
  -d <double>                                          %g\n\
     A double value.\n\
\n\
  -h\n\
     Print the help.\n\n", i_val, f_val, d_val);

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param i_val : An integer.
  \param f_val : A float.
  \param d_val : A double.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, int &i_val, float &f_val, double &d_val)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'd':
      d_val = atof(optarg_);
      break;
    case 'f':
      f_val = (float)atof(optarg_);
      break;
    case 'i':
      i_val = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, i_val, f_val, d_val);
      return false;
      break;

    default:
      usage(argv[0], optarg_, i_val, f_val, d_val);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, i_val, f_val, d_val);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    using ::std::cout;
    using ::std::endl;

    int i_val = 3;
    float f_val = 3.14f;
    double d_val = 3.1415;

    // Read the command line options
    if (getOptions(argc, argv, i_val, f_val, d_val) == false) {
      return (-1);
    }

    cout << "Your parameters: " << endl;
    cout << "  Integer value: " << i_val << endl;
    cout << "  Float   value: " << f_val << endl;
    cout << "  Double  value: " << d_val << endl << endl;
    cout << "Call  " << argv[0] << " -h to see how to change these parameters." << endl;

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
