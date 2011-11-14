/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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



#include <visp/vpTime.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpParseArgv.h>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
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
#define abs(x) ((x) < 0 ? - (x) : (x))
#ifdef VISP_HAVE_GSL


bool testRandom(double epsilon)
{
    vpMatrix L0(6,6);
    vpMatrix L1(6,6);
    
    for (unsigned int i=0 ; i < L0.getRows() ; i++)
	for  (unsigned int j=0 ; j < L0.getCols() ; j++)
	    L1[i][j] = L0[i][j] = (double)rand()/(double)RAND_MAX;
	    
	
    
    vpColVector W0(L0.getCols()) ;
    vpMatrix V0(L0.getCols(), L0.getCols()) ;
    vpColVector W1(L1.getCols()) ;
    vpMatrix V1(L1.getCols(), L1.getCols()) ;

    L0.svdNr(W0,V0);
    L1.svdGsl(W1,V1);

    vpColVector _W0 = vpColVector::sort(W0);
    vpColVector _W1 = vpColVector::sort(W1);

    vpColVector diff = _W0-_W1;
    double error=-1.0;
    
    for(unsigned int i=0;i<6;i++)
	error=std::max(abs(diff[i]),error);
    
    return error<epsilon;
    
}

#endif

bool testSvdOpenCvGSLCoherence(double epsilon){
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100) && defined (VISP_HAVE_GSL) // Require opencv >= 2.1.0
    vpMatrix A;
    vpMatrix vA;
    vpColVector wA;
    vpMatrix B;
    vpMatrix vB;
    vpColVector wB;
    A.resize(6,6);
    B.resize(6,6);
    vA.resize(6,6);
    vB.resize(6,6);
    wA.resize(6);
    wB.resize(6);

    for (unsigned int i=0 ; i < A.getRows() ; i++)
        for  (unsigned int j=0 ; j < A.getCols() ; j++)
            B[i][j] = A[i][j] =  (double)rand()/(double)RAND_MAX;

    A.svdOpenCV(wA,vA);
    B.svdGsl(wB,vB);

    bool error=false;
    for (unsigned int i=0 ; i < A.getRows() ; i++){
        error = error | (abs(wA[i]-wB[i])>epsilon);
    }

    return !error;
#else
    (void)epsilon;
    return true;
#endif
}

int
main(int argc, const char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  unsigned int i,j ;
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
  std::cout << "TESTING RANDOM MATRICES:" ;

  bool ret = true;
  for(int i=0;i<2000;i++)
      ret = ret & testRandom(0.00001);  
  if(ret)
      std:: cout << "Success"<< std:: endl;
  else
      std:: cout << "Fail"<< std:: endl;

  std::cout << "--------------------------------------"<<std::endl ;
#endif

  std::cout << "--------------------------------------"<<std::endl ;
  std::cout << "TESTING OPENCV-GSL coherence:" ;

  bool ret2 = true;
  for(int i=0;i<1;i++)
      ret2 = ret2 & testSvdOpenCvGSLCoherence(0.00001);
  if(ret2)
      std:: cout << "Success"<< std:: endl;
  else
      std:: cout << "Fail"<< std:: endl;

  std::cout << "--------------------------------------"<<std::endl ;

  L = Ls ;
  t = vpTime::measureTimeMs() ;
  L.svdFlake(W,V) ;
  t = vpTime::measureTimeMs() -t ;
  std::cout <<"svdFlake\n time " <<t << std::endl;
    std::cout << W.t() ;



}

