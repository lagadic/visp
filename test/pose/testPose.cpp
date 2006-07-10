/****************************************************************************
 *
 * $Id: testPose.cpp,v 1.7 2006-07-10 16:44:45 fspindle Exp $
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
 * Compute the pose of a 3D object using the Dementhon, Lagrange and
 * Non-Linear approach.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

#define L 0.035

// #ifdef HAVE_GSL
// extern int gsl_warnings_off;
// //#include <gsl/gsl_errno.h>
// //#include <gsl/gsl_message.h>
// gsl_warnings_off = 1;
// #endif

/*!
  \example testPose.cpp

  Compute the pose of a 3D object using the Dementhon, Lagrange and
  Non-Linear approach.

*/
/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Compute the pose of a 3D object using the Dementhon, Lagrange and\n\
Non-Linear approach.\n\
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


  vpPoint P[4]  ;  //  Point to be tracked
  vpPose pose ;
  pose.clearPoint() ;


  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;

  vpHomogeneousMatrix cMo_ref(0.1,0.2,1,vpMath::rad(10),0,vpMath::rad(10)) ;
  int i ;
  for(i=0 ; i < 4 ; i++)
    P[i].project(cMo_ref) ;


  for (i=0 ; i < 4 ; i++)
  {
    pose.addPoint(P[i]) ; // and added to the pose computation class
  }

  // Let's go ...
  vpHomogeneousMatrix cMo ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  vpTRACE("LAGRANGE pose : ") ;
  cout << cMo << endl ;
  cout << "Lagrange residual term: " << pose.computeResidual(cMo) <<endl ;


  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  vpTRACE( "LOWE pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  vpTRACE(  "DEMENTHON pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Dementhon residual term: " << pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<endl ;
  //  vpTRACE(  "Pose LOWE"  ) ;
  //  cout <<  cMo << endl ;
  //  cout << "residu Lowe " << pose.computeResidual(cMo) <<endl ;


  cout <<endl <<endl ;
  cout <<"------------------------------------------------------------"<<endl ;
  cout << "Virtual Visual servoing " << endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  vpTRACE("LAGRANGE pose : ") ;
  cout << cMo << endl ;
  cout << "Lagrange residual term: " << pose.computeResidual(cMo) <<endl ;


  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  vpTRACE( "VIRTUAL_VS pose :" ) ;
  cout <<  cMo << endl ;
  cout << "vvs residual term: " <<pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  vpTRACE(  "DEMENTHON pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Dementhon residual term: " << pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  cout << "vvs residual term: " <<pose.computeResidual(cMo) <<endl ;

}
