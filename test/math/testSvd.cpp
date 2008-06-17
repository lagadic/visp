/****************************************************************************
 *
 * $Id: testSvd.cpp,v 1.5 2008-06-17 08:08:29 asaunier Exp $
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

