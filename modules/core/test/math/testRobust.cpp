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
 * Test some vpMath functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRobust.cpp

  Test some vpMath functionalities.
*/

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRobust.h>
#include <visp3/io/vpParseArgv.h>
// List of allowed command line options
#define GETOPTARGS "cdho:"

void usage(const char *name, const char *badparam, std::string ofilename);
bool getOptions(int argc, const char **argv, std::string &ofilename);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ofilename : Output filename.
*/
void usage(const char *name, const char *badparam, std::string ofilename)
{
  fprintf(stdout, "\n\
Test some vpMath functionalities. Compute weights and print\n\
them in an output file.\n\
\n\
Using gnuplot the content of the output file can be printed by:\n\
set style data line\n\
set ylabel \"weight\"\n\
set yr [0:1.19]\n\
set xlabel \"Normalized residuals\"\n\
plot '%s' title \"Tukey Estimator\" lw 2, 1 title \"Least-Squares\" lw 2\n\
\n\
\n\
SYNOPSIS\n\
  %s [-o <output filename>] [-h]\n", ofilename.c_str(), name);

  fprintf(stdout, "\n\
OPTIONS:                                              Default\n\
  -o <output filename>                                %s\n\
     Name and path of the file containing computed \n\
     weights.\n\
\n\
  -h\n\
     Print the help.\n", ofilename.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ofilename : Output filename.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ofilename)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'o':
      ofilename = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ofilename);
      return false;
      break;

    case 'c':
    case 'd':
      break;
    default:
      usage(argv[0], optarg_, ofilename);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ofilename);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    std::string ofilename;
    std::string username;

// Set the default output filename
#if defined(_WIN32)
    ofilename = "C:/temp";
#else
    ofilename = "/tmp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Append to the output filename string, the login name of the user
    ofilename = ofilename + "/" + username;

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(ofilename) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(ofilename);
      } catch (...) {
        usage(argv[0], NULL, ofilename);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << ofilename << std::endl;
        std::cerr << "  Check your -o " << ofilename << " option " << std::endl;
        exit(-1);
      }
    }

    // Append to the output filename string, the name of the file
    ofilename = ofilename + "/w.dat";

    // Read the command line options
    if (getOptions(argc, argv, ofilename) == false) {
      exit(-1);
    }

    double sig = 1;

    double w;
    std::ofstream f;
    std::cout << "Create file: " << ofilename << std::endl;
    f.open(ofilename.c_str());
    if (f.fail()) {
      usage(argv[0], NULL, ofilename);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot create the file: " << ofilename << std::endl;
      std::cerr << "  Check your -o " << ofilename << " option " << std::endl;
      exit(-1);
    }
    double x = -10;
    while (x < 10) {
      if (fabs(x / sig) <= (4.6851)) {
        w = vpMath::sqr(1 - vpMath::sqr(x / (sig * 4.6851)));
      } else {
        w = 0;
      }
      f << x << "  " << w << std::endl;
      x += 0.01;
    }
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
