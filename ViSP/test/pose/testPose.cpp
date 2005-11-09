




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
  TRACE("LAGRANGE pose : ") ;
  cout << cMo << endl ;
  cout << "Lagrange residual term: " << pose.computeResidual(cMo) <<endl ;


  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  TRACE( "LOWE pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  TRACE(  "DEMENTHON pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Dementhon residual term: " << pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<endl ;
  //  TRACE(  "Pose LOWE"  ) ;
  //  cout <<  cMo << endl ;
  //  cout << "residu Lowe " << pose.computeResidual(cMo) <<endl ;


  cout <<endl <<endl ;
  cout <<"------------------------------------------------------------"<<endl ;
  cout << "Virtual Visual servoing " << endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  TRACE("LAGRANGE pose : ") ;
  cout << cMo << endl ;
  cout << "Lagrange residual term: " << pose.computeResidual(cMo) <<endl ;


  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  TRACE( "VIRTUAL_VS pose :" ) ;
  cout <<  cMo << endl ;
  cout << "vvs residual term: " <<pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  TRACE(  "DEMENTHON pose :" ) ;
  cout <<  cMo << endl ;
  cout << "Dementhon residual term: " << pose.computeResidual(cMo) <<endl ;

  cout <<"------------------------------------------------------------"<<endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  cout << "vvs residual term: " <<pose.computeResidual(cMo) <<endl ;

}
