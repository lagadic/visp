



/*
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/
#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>

#define L 0.1


/*!
  \example testPoseRansac.cpp

  Compute the pose of a 3D object using the Dementhon method
  assuming that the correspondance between 2D points and 3D points is not done
  we use the RANSAC algorithm to achieve this task

*/
int
main()
{

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
