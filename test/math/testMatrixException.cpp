/****************************************************************************
 *
 * $Id: testMatrixException.cpp,v 1.3 2007-04-27 16:40:16 fspindle Exp $
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
 * Test matrix exceptions.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrixException.cpp
  \brief Tests matrix exception
*/


#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>

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

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
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

  vpMatrix M ;
  vpMatrix M1(2,3) ;
  vpMatrix M2(3,3) ;
  vpMatrix M3(2,2) ;

  vpTRACE("test matrix size in multiply") ;

  try
  {
    M = M1*M3 ;
  }
  catch (vpMatrixException me)
  {
    vpCTRACE ;
    std::cout << me << std::endl ;
  }


  vpTRACE("test matrix size in addition") ;

  try
  {
    M = M1+M3 ;
  }
  catch (vpMatrixException me)
  {
    vpCTRACE ;
    std::cout << me << std::endl ;
  }


  vpCTRACE ;
}
