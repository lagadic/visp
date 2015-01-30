/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpParseArgv.h>
#include <visp/vpGEMM.h>


#include <stdlib.h>
#include <stdio.h>

// List of allowed command line options
#define GETOPTARGS	"h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv);

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
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  try {
    // Read the command line options
    if (getOptions(argc, argv) == false) {
      exit (-1);
    }
    {
      vpMatrix M(4,5);
      int val = 0;
      for(unsigned int i=0; i<M.getRows(); i++) {
        for(unsigned int j=0; j<M.getCols(); j++) {
          M[i][j] = val++;
        }
      }
      std::cout <<"M ";
      M.print (std::cout, 4);

      vpMatrix N;
      N.init(M, 0, 1, 2, 3);
      std::cout <<"N ";
      N.print (std::cout, 4);
    }
    {

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST PRETTY PRINT---" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix M ;
      M.eye(4);

      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 4);" << std::endl;
      M.print (std::cout, 4);

      std::cout << "------------------------" << std::endl;
      M.resize(3,3) ;
      M.eye(3);
      M[1][0]=1.235;
      M[1][1]=12.345;
      M[1][2]=.12345;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M;
      std::cout << "call M.print (std::cout, 6);" << std::endl;
      M.print (std::cout, 6);
      std::cout << std::endl;

      std::cout << "------------------------" << std::endl;
      M[0][0]=-1.235;
      M[1][0]=-12.235;

      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 10);" << std::endl;
      M.print (std::cout, 10);
      std::cout << std::endl;

      std::cout << "call M.print (std::cout, 2);" << std::endl;
      M.print (std::cout, 2);
      std::cout << std::endl;

      std::cout << "------------------------" << std::endl;
      M.resize(3,3) ;
      M.eye(3);
      M[0][2]=-0.0000000876;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 4);" << std::endl;
      M.print (std::cout, 4);
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 10, \"M\");" << std::endl;
      M.print (std::cout, 10, "M");
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 20, \"M\");" << std::endl;
      M.print (std::cout, 20, "M");
      std::cout << std::endl;


      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST RESIZE --------" << std::endl;
      std::cout << "------------------------" << std::endl;
      std::cout <<  "5x5" << std::endl;
      M.resize(5,5,false);
      std::cout << M << std::endl;
      std::cout << "3x2" << std::endl;
      M.resize(3,2,false);
      std::cout << M << std::endl;
      std::cout << "2x2" << std::endl;
      M.resize(2,2,false);
      std::cout << M << std::endl;
      std::cout << "------------------------" << std::endl;

      vpVelocityTwistMatrix vMe;
      vpMatrix A(1,6),B;

      A=1.0;
      //vMe=1.0;
      B=A*vMe;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpRowVector * vpColVector" << std::endl;
      std::cout << "------------------------" << std::endl;
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

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpRowVector * vpMatrix" << std::endl;
      std::cout << "------------------------" << std::endl;
      M.resize(3,3) ;
      M.eye(3);

      M[1][0] = 1.5;
      M[2][0] = 2.3;

      vpRowVector rM = r * M;

      r.print(std::cout, 2, "r");
      M.print(std::cout, 10, "M");
      std::cout << "r * M = " << rM << std::endl;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpGEMM " << std::endl;
      std::cout << "------------------------" << std::endl;
      M.resize(3,3) ;
      M.eye(3);
      vpMatrix N(3, 3);
      N[0][0] = 2;
      N[1][0] = 1.2;
      N[1][2] = 0.6;
      N[2][2] = 0.25;

      vpMatrix C(3, 3);
      C.eye(3);

      vpMatrix D;

      //realise the operation D = 2 * M^T * N + 3 C
      vpGEMM(M, N, 2, C, 3, D, VP_GEMM_A_T);
      std::cout << D << std::endl;
      return 0;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

