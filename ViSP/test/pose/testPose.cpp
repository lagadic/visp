



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

#define L 0.035

// #if HAVE_LIBGSL
// extern int gsl_warnings_off;
// //#include <gsl/gsl_errno.h>
// //#include <gsl/gsl_message.h>
// gsl_warnings_off = 1;
// #endif

/*!
  \example testPose.cpp

  Compute the pose of a 3D object using the Dementhon, Lagrange and
  Non-Linear approach

*/
int
main()
{

  vpPoint P[4]  ;  //  Point to be tracked
  vpPose pose ;
  pose.clearPoint() ;


  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0.2 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;

  vpHomogeneousMatrix cMo_ref(0,0.2,1,vpMath::rad(0),0,0) ;
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
  TRACE("Pose  LAGRANGE") ;
  cout << cMo << endl ;
  cout << "residu Lagrange " << pose.computeResidual(cMo) <<endl ;


  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  TRACE( "Pose LOWE" ) ;
  cout <<  cMo << endl ;
  cout << "residu Lowe " <<pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  TRACE(  "Pose DEMENTHON" ) ;
  cout <<  cMo << endl ;
  cout << "residu Dementhon " << pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  //  TRACE(  "Pose LOWE"  ) ;
  //  cout <<  cMo << endl ;
  //  cout << "residu Lowe " << pose.computeResidual(cMo) <<endl ;


  vpHomogeneousMatrix cMo1 ;


  cMo1 = cMo_ref ;
  cMo1[2][3] += 0.1 ;
  cMo1[1][3] += 0.1 ;
  cout << pose.computeResidual(cMo1) <<endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo1) ;
  cout << "Pose   VVS" << endl << cMo1 << endl ;
  cout << pose.computeResidual(cMo1) <<endl ;

}
