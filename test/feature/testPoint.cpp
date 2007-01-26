/****************************************************************************
 *
 * $Id: testPoint.cpp,v 1.1 2007-01-26 16:29:48 asaunier Exp $
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
void usage(char *name, char *badparam)
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

  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;

  vpPoint point ;
  vpTRACE("set point coordinates in the world  frame ") ;
  point.setWorldCoordinates(0,0,0) ;


  cout <<"------------------------------------------------------"<<endl ;
  vpTRACE("test the projection ") ;
  point.track(cMo) ;

  vpTRACE("coordinates in the world frame ") ;
  cout << point.oP.t() << endl ;
  vpTRACE("coordinates in the camera frame  ") ;
  cout << point.cP.t() << endl ;

  vpTRACE("2D coordinates ") ;
  cout<< point.get_x() << "  " << point.get_y() << endl ;

  cout <<"------------------------------------------------------"<<endl ;
  vpTRACE("test the interaction matrix ") ;

  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,point) ;

  vpMatrix L ;
  L = p.interaction() ;
  cout << L << endl ;

  vpTRACE("test the interaction matrix select") ;
  vpTRACE("\t only X") ;
  L = p.interaction(vpFeaturePoint::selectX()) ;
  cout << L << endl ;

  vpTRACE("\t only Y") ;
  L = p.interaction(vpFeaturePoint::selectY()) ;
  cout << L << endl ;

  vpTRACE("\t X & Y") ;
  L = p.interaction(vpFeaturePoint::selectX() |
		    vpFeaturePoint::selectY()) ;
  cout << L << endl ;

  vpTRACE("\t selectAll") ;
  L = p.interaction(vpFeaturePoint::selectAll() ) ;
  cout << L << endl ;

  cout <<"------------------------------------------------------"<<endl ;
  vpTRACE("test the error ") ;

  try{
  vpFeaturePoint pd ;
  pd.set_x(0) ;
  pd.set_y(0) ;

  pd.print() ; cout << endl ;
  vpColVector e ;
  e = p.error(pd) ;
  cout << e << endl ;

  vpTRACE("test the interaction matrix select") ;
  vpTRACE("\t only X") ;
  e = p.error(pd,vpFeaturePoint::selectX()) ;
  cout << e << endl ;

  vpTRACE("\t only Y") ;
  e = p.error(pd,vpFeaturePoint::selectY()) ;
  cout << e << endl ;

  vpTRACE("\t X & Y") ;
  e = p.error(pd,vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  cout << e << endl ;

  vpTRACE("\t selectAll") ;
  e = p.error(pd,vpFeaturePoint::selectAll() ) ;
  cout << e << endl ;
  }
  catch(vpFeatureException me){ cout << me << endl ; }
  catch(vpException me){ cout << me << endl ; }
  cout <<"------------------------------------------------------"<<endl ;
  vpTRACE("test the  dimension") ;
  int dim ;
  dim = p.getDimension() ;
  cout << "Dimension = " << dim << endl ;

  vpTRACE("test the dimension with  select") ;
  vpTRACE("\t only X") ;
  dim = p.getDimension(vpFeaturePoint::selectX()) ;
  cout << "Dimension = " << dim << endl ;

  vpTRACE("\t only Y") ;
  dim = p.getDimension(vpFeaturePoint::selectY()) ;
  cout << "Dimension = " << dim << endl ;

  vpTRACE("\t X & Y") ;
  dim = p.getDimension(vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  cout << "Dimension = " << dim << endl ;

  vpTRACE("\t selectAll") ;
  dim = p.getDimension(vpFeaturePoint::selectAll() ) ;
  cout << "Dimension = " << dim << endl ;

}
