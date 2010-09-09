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
 * Test various svd decompositions.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example testSvd.cpp
  \brief Test various svd decompositions.
*/


#include <stdlib.h>
#include <stdio.h>

#include <visp/vpTime.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test various svd decompositions.\n\
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

  int i,j ;
  vpMatrix L(60000,6), Ls ;
  for (i=0 ; i < L.getRows() ; i++)
    for  (j=0 ; j < L.getCols() ; j++)
      L[i][j] = 2*i+j + cos((double)(i+j))+((double)(i)) ;
  //  std::cout << L << std::endl ;
  Ls = L ;
  std::cout << "--------------------------------------"<<std::endl ;

  vpColVector W(L.getCols()) ;
  vpMatrix V(L.getCols(), L.getCols()) ;

  double t = vpTime::measureTimeMs() ;
  L.svdNr(W,V) ;
  t = vpTime::measureTimeMs() -t ;

  std::cout <<"svdNr Numerical recipes \n time " <<t << std::endl;
    std::cout << W.t() ;
  std::cout << "--------------------------------------"<<std::endl ;

#ifdef VISP_HAVE_GSL
  L = Ls ;
  t = vpTime::measureTimeMs() ;
  L.svdGsl(W,V) ;
  t = vpTime::measureTimeMs() -t ;
  std::cout <<"svdGsl_mod \n time " <<t << std::endl;
    std::cout << W.t() ;

  std::cout << "--------------------------------------"<<std::endl ;
#endif

  L = Ls ;
  t = vpTime::measureTimeMs() ;
  L.svdFlake(W,V) ;
  t = vpTime::measureTimeMs() -t ;
  std::cout <<"svdFlake\n time " <<t << std::endl;
    std::cout << W.t() ;



}

