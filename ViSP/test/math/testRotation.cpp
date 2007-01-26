/****************************************************************************
 *
 * $Id: testRotation.cpp,v 1.1 2007-01-26 16:26:32 asaunier Exp $
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


#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
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
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
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

  vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

  cout << "Initialization " <<endl ;
  cout << tu << endl ;


  cout << "From vpThetaUVector to vpRotationMatrix " << endl ;
  vpRotationMatrix R(tu)  ;

  cout << "Matrix R" ;
  if (R.isARotationMatrix()==1) cout <<" is a rotation matrix " << endl ;
  else cout <<" is not a rotation matrix " << endl ;

  cout << R << endl ;

  cout << "From vpRotationMatrix to vpRxyzVector " << endl ;
  vpRxyzVector RxyzBuildFromR(R) ;
  cout <<  RxyzBuildFromR <<endl ;



  cout << "From vpRxyzVector to vpThetaUVector " << endl ;
  cout << "  use From vpRxyzVector to vpRotationMatrix " << endl ;
  cout << "  use From vpRotationMatrix to vpThetaUVector " << endl ;


  vpThetaUVector tuBuildFromEu ;
  tuBuildFromEu.buildFrom(RxyzBuildFromR) ;

  cout << endl ;
  cout <<  "result : should equivalent to the first one " << endl ;
  cout << tuBuildFromEu << endl ;


  vpRzyzVector rzyz(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  cout << "Initialization vpRzyzVector " <<endl ;
  cout << rzyz << endl ;
  cout << "From vpRzyzVector to vpRotationMatrix  " << endl ;
  R.buildFrom(rzyz) ;
  cout << "From vpRotationMatrix to vpRzyzVector " << endl ;
  vpRzyzVector rzyz_final ;
  rzyz_final.buildFrom(R) ;
  cout << rzyz_final << endl ;


  vpRzyxVector rzyx(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  cout << "Initialization vpRzyxVector " <<endl ;
  cout << rzyx << endl ;
  cout << "From vpRzyxVector to vpRotationMatrix  " << endl ;
  R.buildFrom(rzyx) ;
  cout << R << endl ;
  cout << "From vpRotationMatrix to vpRzyxVector " << endl ;
  vpRzyxVector rzyx_final ;
  rzyx_final.buildFrom(R) ;
  cout << rzyx_final << endl ;


}
