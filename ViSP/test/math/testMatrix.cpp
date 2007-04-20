/****************************************************************************
 *
 * $Id: testMatrix.cpp,v 1.2 2007-04-20 14:22:25 asaunier Exp $
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
 * This file is part of the ViSP toolkit
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



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
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

}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv)
{
  char *optarg;
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
main(int argc, char ** argv)
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


  vpTwistMatrix vMe;
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


}
