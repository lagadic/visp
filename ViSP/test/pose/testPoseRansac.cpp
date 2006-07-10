/****************************************************************************
 *
 * $Id: testPoseRansac.cpp,v 1.3 2006-07-10 16:44:45 fspindle Exp $
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
 * Compute the pose of a 3D object using the Dementhon method. Assuming that 
 * the correspondance between 2D points and 3D points is not done, we use 
 * the RANSAC algorithm to achieve this task
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

#define L 0.1


/*!
  \example testPoseRansac.cpp

  Compute the pose of a 3D object using the Dementhon method. Assuming
  that the correspondance between 2D points and 3D points is not done,
  we use the RANSAC algorithm to achieve this task.

*/
/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Compute the pose of a 3D object using the Dementhon method. Assuming\n\
that the correspondance between 2D points and 3D points is not done,\n\
we use the RANSAC algorithm to achieve this task.\n\
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


  {
  vpPoint P[5]  ;  //  Point to be tracked
  vpList<vpPoint> lp, lP ;

  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;
  P[4].setWorldCoordinates(-0,0, L ) ;

  for (int i=0 ; i < 5 ; i++) lP += P[i] ;

  vpHomogeneousMatrix cMo_ref(0,0.2,1,vpMath::rad(0),0,0) ;
  for(int i=0 ; i < 5 ; i++)
    P[i].project(cMo_ref) ;

  vpPoint p[10] ;

  for (int i=0 ; i < 5 ; i++)
  {
    p[2*i] = P[i] ;
  }

  p[1].set_x(0.02) ;
  p[1].set_y(0.05) ;
  p[3].set_x(0.02) ;
  p[3].set_y(-0.05) ;
  p[5].set_x(0.07) ;
  p[5].set_y(-0.05) ;
  p[7].set_x(0.24) ;
  p[7].set_y(0.07) ;
  p[9].set_x(-0.02) ;
  p[9].set_y(-0.05) ;

  for (int i=0 ; i < 10 ; i++) lp += p[i] ;

  int ninliers ;
  vpList<vpPoint> lPi ;

  vpHomogeneousMatrix cMo ;
  vpPose::ransac(lp,lP, 5, 1e-6, ninliers, lPi, cMo) ;

  lPi.front() ;
  while (!lPi.outside())
  {
    vpPoint Pi ;
    Pi = lPi.value() ; lPi.next() ;
    Pi.print() ;
  }

  cout << cMo << endl ;

  }

  {
  vpPoint P[5]  ;  //  Point to be tracked


  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;
  P[4].setWorldCoordinates(-0,0, L ) ;

  vpHomogeneousMatrix cMo_ref(0,0.2,1,vpMath::rad(0),0,0) ;
  int i ;
  for(i=0 ; i < 5 ; i++)
    P[i].project(cMo_ref) ;

  vpPoint p[10] ;

  for (i=0 ; i < 5 ; i++)
  {
    p[2*i] = P[i] ;
  }

  p[1].set_x(0.02) ;
  p[1].set_y(0.05) ;
  p[3].set_x(0.02) ;
  p[3].set_y(-0.05) ;
  p[5].set_x(0.07) ;
  p[5].set_y(-0.05) ;
  p[7].set_x(0.24) ;
  p[7].set_y(0.07) ;
  p[9].set_x(-0.02) ;
  p[9].set_y(-0.05) ;


  int ninliers ;
  vpList<vpPoint> lPi ;

  vpHomogeneousMatrix cMo ;
  vpPose::ransac(10,p,5,P, 5, 1e-6, ninliers, lPi, cMo) ;

  lPi.front() ;
  while (!lPi.outside())
  {
    vpPoint Pi ;
    Pi = lPi.value() ; lPi.next() ;
    Pi.print() ;
  }

  cout << cMo << endl ;

  }

}
