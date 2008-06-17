/****************************************************************************
 *
 * $Id: testRotation.cpp,v 1.5 2008-06-17 08:08:29 asaunier Exp $
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
 * Tests transformation from various representations of rotation.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file testRotation.cpp
  \brief Tests transformation within various representations of rotation.
*/


#include <stdlib.h>
#include <stdio.h>
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Tests transformation within various representations of rotation.\n\
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

  vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

  std::cout << "Initialization " <<std::endl ;
  std::cout << tu << std::endl ;


  std::cout << "From vpThetaUVector to vpRotationMatrix " << std::endl ;
  vpRotationMatrix R(tu)  ;

  std::cout << "Matrix R" ;
  if (R.isARotationMatrix()==1) std::cout <<" is a rotation matrix " << std::endl ;
  else std::cout <<" is not a rotation matrix " << std::endl ;

  std::cout << R << std::endl ;

  std::cout << "From vpRotationMatrix to vpRxyzVector " << std::endl ;
  vpRxyzVector RxyzBuildFromR(R) ;
  std::cout <<  RxyzBuildFromR <<std::endl ;



  std::cout << "From vpRxyzVector to vpThetaUVector " << std::endl ;
  std::cout << "  use From vpRxyzVector to vpRotationMatrix " << std::endl ;
  std::cout << "  use From vpRotationMatrix to vpThetaUVector " << std::endl ;


  vpThetaUVector tuBuildFromEu ;
  tuBuildFromEu.buildFrom(RxyzBuildFromR) ;

  std::cout << std::endl ;
  std::cout <<  "result : should equivalent to the first one " << std::endl ;
  std::cout << tuBuildFromEu << std::endl ;


  vpRzyzVector rzyz(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  std::cout << "Initialization vpRzyzVector " <<std::endl ;
  std::cout << rzyz << std::endl ;
  std::cout << "From vpRzyzVector to vpRotationMatrix  " << std::endl ;
  R.buildFrom(rzyz) ;
  std::cout << "From vpRotationMatrix to vpRzyzVector " << std::endl ;
  vpRzyzVector rzyz_final ;
  rzyz_final.buildFrom(R) ;
  std::cout << rzyz_final << std::endl ;


  vpRzyxVector rzyx(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  std::cout << "Initialization vpRzyxVector " <<std::endl ;
  std::cout << rzyx << std::endl ;
  std::cout << "From vpRzyxVector to vpRotationMatrix  " << std::endl ;
  R.buildFrom(rzyx) ;
  std::cout << R << std::endl ;
  std::cout << "From vpRotationMatrix to vpRzyxVector " << std::endl ;
  vpRzyxVector rzyx_final ;
  rzyx_final.buildFrom(R) ;
  std::cout << rzyx_final << std::endl ;


}
