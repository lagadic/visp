/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>

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
void usage(const char *name, const char *badparam)
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
    return (-1);
  }


  {
    std::cout << "Pose computation using std::list" << std::endl;
    vpPoint P[5]  ;  //  Point to be tracked
    std::list<vpPoint> lp, lP ;

    P[0].setWorldCoordinates(-L,-L, 0 ) ;
    P[1].setWorldCoordinates(L,-L, 0 ) ;
    P[2].setWorldCoordinates(L,L, 0 ) ;
    P[3].setWorldCoordinates(-L,L, 0 ) ;
    P[4].setWorldCoordinates(-0,0, L ) ;

    for (int i=0 ; i < 5 ; i++) lP.push_back(P[i]) ;

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

    for (int i=0 ; i < 10 ; i++) lp.push_back(p[i]) ;

    unsigned int ninliers ;
    std::list<vpPoint> lPi ;

    vpHomogeneousMatrix cMo ;
    vpPose::ransac(lp,lP, 5, 1e-6, ninliers, lPi, cMo) ;

    for (std::list<vpPoint>::const_iterator it = lPi.begin(); it != lPi.end(); ++ it)
    {
      vpPoint Pi ;
      Pi = *it;
      Pi.print() ;
      std::cout << std::endl;
    }

    std::cout << "cMo :\n" << cMo << std::endl << std::endl;

  }

  {
    std::cout << "Pose computation using arrays" << std::endl;
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


    unsigned int ninliers ;
    std::list<vpPoint> lPi ;

    vpHomogeneousMatrix cMo ;
    vpPose::ransac(10,p,5,P, 5, 1e-6, ninliers, lPi, cMo, 5000) ;//maximum number of trial set to 5000 instead of 10000.

    for (std::list<vpPoint>::const_iterator it = lPi.begin(); it != lPi.end(); ++ it)
    {
      vpPoint Pi ;
      Pi = *it;
      Pi.print() ;
      std::cout << std::endl;
    }

    std::cout << "cMo :\n" << cMo << std::endl ;
  }
}
