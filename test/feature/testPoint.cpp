/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Performs various tests on the point class.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



/*!
  \file testPoint.cpp
  \brief Performs various tests on the the point class.
*/

// List of allowed command line options
#define GETOPTARGS	"h"

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureException.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>

/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Performs various tests on the point class.\n\
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

  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;

  vpPoint point ;
  vpTRACE("set point coordinates in the world  frame ") ;
  point.setWorldCoordinates(0,0,0) ;


  std::cout <<"------------------------------------------------------"<<std::endl ;
  vpTRACE("test the projection ") ;
  point.track(cMo) ;

  vpTRACE("coordinates in the world frame ") ;
  std::cout << point.oP.t() << std::endl ;
  vpTRACE("coordinates in the camera frame  ") ;
  std::cout << point.cP.t() << std::endl ;

  vpTRACE("2D coordinates ") ;
  std::cout<< point.get_x() << "  " << point.get_y() << std::endl ;

  std::cout <<"------------------------------------------------------"<<std::endl ;
  vpTRACE("test the interaction matrix ") ;

  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,point) ;

  vpMatrix L ;
  L = p.interaction() ;
  std::cout << L << std::endl ;

  vpTRACE("test the interaction matrix select") ;
  vpTRACE("\t only X") ;
  L = p.interaction(vpFeaturePoint::selectX()) ;
  std::cout << L << std::endl ;

  vpTRACE("\t only Y") ;
  L = p.interaction(vpFeaturePoint::selectY()) ;
  std::cout << L << std::endl ;

  vpTRACE("\t X & Y") ;
  L = p.interaction(vpFeaturePoint::selectX() |
		    vpFeaturePoint::selectY()) ;
  std::cout << L << std::endl ;

  vpTRACE("\t selectAll") ;
  L = p.interaction(vpFeaturePoint::selectAll() ) ;
  std::cout << L << std::endl ;

  std::cout <<"------------------------------------------------------"<<std::endl ;
  vpTRACE("test the error ") ;

  try{
  vpFeaturePoint pd ;
  pd.set_x(0) ;
  pd.set_y(0) ;

  pd.print() ; std::cout << std::endl ;
  vpColVector e ;
  e = p.error(pd) ;
  std::cout << e << std::endl ;

  vpTRACE("test the interaction matrix select") ;
  vpTRACE("\t only X") ;
  e = p.error(pd,vpFeaturePoint::selectX()) ;
  std::cout << e << std::endl ;

  vpTRACE("\t only Y") ;
  e = p.error(pd,vpFeaturePoint::selectY()) ;
  std::cout << e << std::endl ;

  vpTRACE("\t X & Y") ;
  e = p.error(pd,vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  std::cout << e << std::endl ;

  vpTRACE("\t selectAll") ;
  e = p.error(pd,vpFeaturePoint::selectAll() ) ;
  std::cout << e << std::endl ;
  }
  catch(vpFeatureException me){ std::cout << me << std::endl ; }
  catch(vpException me){ std::cout << me << std::endl ; }
  std::cout <<"------------------------------------------------------"<<std::endl ;
  vpTRACE("test the  dimension") ;
  int dim ;
  dim = p.getDimension() ;
  std::cout << "Dimension = " << dim << std::endl ;

  vpTRACE("test the dimension with  select") ;
  vpTRACE("\t only X") ;
  dim = p.getDimension(vpFeaturePoint::selectX()) ;
  std::cout << "Dimension = " << dim << std::endl ;

  vpTRACE("\t only Y") ;
  dim = p.getDimension(vpFeaturePoint::selectY()) ;
  std::cout << "Dimension = " << dim << std::endl ;

  vpTRACE("\t X & Y") ;
  dim = p.getDimension(vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  std::cout << "Dimension = " << dim << std::endl ;

  vpTRACE("\t selectAll") ;
  dim = p.getDimension(vpFeaturePoint::selectAll() ) ;
  std::cout << "Dimension = " << dim << std::endl ;

}
