/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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


#include <visp/vpRobust.h>
#include <string>
#include <fstream>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
// List of allowed command line options
#define GETOPTARGS	"ho:"

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
     Print the help.\n",
	  ofilename.c_str());

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
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'o': ofilename = optarg; break;
    case 'h': usage(argv[0], NULL, ofilename); return false; break;

    default:
      usage(argv[0], optarg, ofilename);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ofilename);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}



int
main(int argc, const char ** argv)
{
  try {
    std::string ofilename;
    std::string username;

    // Set the default output filename
#ifdef WIN32
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
      }
      catch (...) {
        usage(argv[0], NULL, ofilename);
        std::cerr << std::endl
                  << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << ofilename << std::endl;
        std::cerr << "  Check your -o " << ofilename << " option " << std::endl;
        exit(-1);
      }
    }

    // Append to the output filename string, the name of the file
    ofilename = ofilename + "/w.dat";

    // Read the command line options
    if (getOptions(argc, argv, ofilename) == false) {
      exit (-1);
    }

    double sig = 1 ;

    double w ;
    std::ofstream f;
    std::cout << "Create file: " << ofilename << std::endl;
    f.open(ofilename.c_str());
    if (f == NULL) {
      usage(argv[0], NULL, ofilename);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Cannot create the file: " << ofilename << std::endl;
      std::cerr << "  Check your -o " << ofilename << " option " << std::endl;
      exit(-1);

    }
    double x = -10 ;
    while (x<10)
    {
      if (fabs(x/sig)<=(4.6851))
      {
        w = vpMath::sqr(1-vpMath::sqr(x/(sig*4.6851)));
      }
      else
      {
        w = 0;
      }
      f << x <<"  "<<w <<std::endl ;
      x+= 0.01 ;
    }
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

