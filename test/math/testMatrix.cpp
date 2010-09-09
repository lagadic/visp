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
 * See the file LICENSE.GPL at the root directory of this source
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
 * Test some vpMatrix functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrix.cpp

  Test some vpMatrix functionalities.
*/


#include <stdlib.h>
#include <stdio.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test some vpMatrix functionalities.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  vpTRACE("------------------------");
  vpTRACE("--- TEST PRETTY PRINT---");
  vpTRACE("------------------------");
  vpMatrix M ;
  M.eye(4);

  vpTRACE("call std::cout << M;");
  std::cout << M << std::endl;

  vpTRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);

  vpTRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[1][0]=1.235;
  M[1][1]=12.345;
  M[1][2]=.12345;
  vpTRACE("call std::cout << M;");
  std::cout << M;
  vpTRACE("call M.print (std::cout, 6);");
  M.print (std::cout, 6);

  vpTRACE("------------------------");
  M[0][0]=-1.235;
  M[1][0]=-12.235;

  vpTRACE("call std::cout << M;");
  std::cout << M;

  vpTRACE("call M.print (std::cout, 10);");
  M.print (std::cout, 10);

  vpTRACE("call M.print (std::cout, 2);");
  M.print (std::cout, 2);

  vpTRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[0][2]=-0.0000000876;
  vpTRACE("call std::cout << M;");
  std::cout << M;

  vpTRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);
  vpTRACE("call M.print (std::cout, 10, \"M\");");
  M.print (std::cout, 10, "M");
  vpTRACE("call M.print (std::cout, 20, \"M\");");
  M.print (std::cout, 20, "M");


  vpTRACE("------------------------");
  vpTRACE("--- TEST RESIZE --------");
  vpTRACE("------------------------");
  vpCTRACE  << "5x5" << std::endl;
  M.resize(5,5,false);
  vpCTRACE << std::endl<< M;
  vpCTRACE  << "3x2" << std::endl;
  M.resize(3,2,false);
  vpCTRACE <<std::endl<< M;
  vpCTRACE  << "2x2" << std::endl;
  M.resize(2,2,false);
  vpCTRACE << std::endl<<M;
  vpTRACE("------------------------");


  vpVelocityTwistMatrix vMe;
  vpMatrix A(1,6),B;

  A=1.0;
  //vMe=1.0;
  B=A*vMe;

  vpTRACE("------------------------");
  vpTRACE("--- TEST vpRowVector * vpColVector");
  vpTRACE("------------------------");
  vpRowVector r(3);
  r[0] = 2;
  r[1] = 3;
  r[2] = 4;

  vpColVector c(3);
  c[0] = 1;
  c[1] = 2;
  c[2] = -1;

  double rc = r * c;

  r.print(std::cout, 2, "r");
  c.print(std::cout, 2, "c");
  std::cout << "r * c = " << rc << std::endl;

  vpTRACE("------------------------");
  vpTRACE("--- TEST vpRowVector * vpMatrix");
  vpTRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);

  M[1][0] = 1.5;
  M[2][0] = 2.3;

  vpRowVector rM = r * M;

  r.print(std::cout, 2, "r");
  M.print(std::cout, 10, "M");
  std::cout << "r * M = " << rM << std::endl;

}
