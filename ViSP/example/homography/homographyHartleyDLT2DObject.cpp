/****************************************************************************
 *
 * $Id: homographyHartleyDLT2DObject.cpp,v 1.5 2008-06-17 08:08:25 asaunier Exp $
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
 * Example of the HartleyDLT homography estimation algorithm.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/
/*!
  \file homographyHartleyDLT2DObject.cpp

  \brief Example of the HartleyDLT homography estimation algorithm using
  vpHomography class.

*/
/*!
  \example homographyHartleyDLT2DObject.cpp

  Example of the HartleyDLT homography estimation algorithm using vpHomography
  class.

*/


#include <stdlib.h>
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomography.h>
#include <visp/vpDebug.h>
#include <visp/vpThetaUVector.h>

#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

#define L 0.1
#define nbpt 5

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.


*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test the HartleyDLT homography estimation algorithm.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.


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

  int i ;

  vpPoint P[nbpt]  ;  //  Point to be tracked
  double xa[nbpt], ya[nbpt] ;
  double xb[nbpt], yb[nbpt] ;

  vpPoint aP[nbpt]  ;  //  Point to be tracked
  vpPoint bP[nbpt]  ;  //  Point to be tracked

  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(2*L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,3*L, 0 ) ;
  P[4].setWorldCoordinates(0,0, 0 ) ;
  /*
    P[5].setWorldCoordinates(10,20, 0 ) ;
    P[6].setWorldCoordinates(-10,12, 0 ) ;
  */
  vpHomogeneousMatrix bMo(0,0,1, 0,0,0) ;
  vpHomogeneousMatrix aMb(1,0,0.0,vpMath::rad(10),0,vpMath::rad(40)) ;
  vpHomogeneousMatrix aMo =aMb*bMo ;
  for(i=0 ; i < nbpt ; i++)
  {
    P[i].project(aMo) ;
    aP[i] = P[i] ;
    xa[i] = P[i].get_x() ;
    ya[i] = P[i].get_y() ;
  }

  for(i=0 ; i < nbpt ; i++)
  {
    P[i].project(bMo) ;
    bP[i] = P[i] ;
    xb[i] = P[i].get_x() ;
    yb[i] = P[i].get_y() ;
  }
  std::cout << "-------------------------------" <<std::endl ;
  std::cout << "aMb "<<std::endl <<aMb << std::endl ;
  std::cout << "-------------------------------" <<std::endl ;
  vpHomography aHb ;

  vpHomography::HartleyDLT(nbpt,xb,yb,xa,ya,aHb) ;

  vpTRACE("aHb computed using the DLT algorithm") ;
  aHb /= aHb[2][2] ;
  std::cout << std::endl << aHb<< std::endl ;

  vpRotationMatrix aRb  ;
  vpTranslationVector aTb ;
  vpColVector n ;

  std::cout << "-------------------------------" <<std::endl ;
  vpTRACE("extract R, T and n ") ;
  aHb.computeDisplacement(aRb, aTb, n) ;
  std::cout << "Rotation: aRb" <<std::endl ;
  std::cout << aRb << std::endl ;
  std::cout << "Translation: aTb" <<std::endl;
  std::cout << (aTb).t() <<std::endl   ;
  std::cout << "Normal to the plane: n" <<std::endl;
  std::cout << (n).t() <<std::endl ;


  std::cout << "-------------------------------" <<std::endl ;
  vpTRACE("Compare with built homoraphy H = R + t/d ") ;
  vpPlane bp(0,0,1,1) ;
  vpHomography aHb_built(aMb,bp) ;
  vpTRACE( "aHb built from the displacement ") ;
  std::cout <<  std::endl <<aHb_built/aHb_built[2][2] << std::endl ;

  aHb_built.computeDisplacement(aRb, aTb, n) ;
  std::cout << "Rotation: aRb" <<std::endl ;
  std::cout << aRb << std::endl ;
  std::cout << "Translation: aTb" <<std::endl;
  std::cout << (aTb).t() <<std::endl ;
  std::cout << "Normal to the plane: n" <<std::endl;
  std::cout << (n).t() <<std::endl ;

  std::cout << "-------------------------------" <<std::endl ;
  vpTRACE("test if ap = aHb bp") ;

  for(i=0 ; i < nbpt ; i++)
  {
    std::cout << "Point "<< i<< std::endl ;
    vpPoint p ;
    std::cout << "(" ;
    std::cout << aP[i].get_x()/aP[i].get_w()<<", "<< aP[i].get_y()/aP[i].get_w() ;
    std::cout <<") =  (" ;
    p = aHb*bP[i] ;
    std::cout << p.get_x() /p.get_w()<<",  "<< p.get_y()/ p.get_w() <<")"<<std::endl ;
  }



}
