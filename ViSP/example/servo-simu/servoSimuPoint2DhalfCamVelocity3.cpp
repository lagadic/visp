/****************************************************************************
 *
 * $Id: servoSimuPoint2DhalfCamVelocity3.cpp,v 1.2 2007-01-30 17:19:08 asaunier Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Simulation of a 2 1/2 D visual servoing control law.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



/*!
  \file servoSimuPoint2DhalfCamVelocity3.cpp
  \brief  Simulation of a 2 1/2 D visual servoing control law
*/

/*!
  \example servoSimuPoint2DhalfCamVelocity3.cpp
  Simulation of a 2 1/2 D visual servoing control law
*/

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>

int
main()
{

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " simulation of a 2 1/2 D visual servoing " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  // In this example we will simulate a visual servoing task.
  // In simulation, we have to define the scene frane Ro and the
  // camera frame Rc.
  // The camera location is given by an homogenous matrix cMo that
  // describes the position of the camera in the scene frame.


  vpTRACE("sets the initial camera location " ) ;
  // we give the camera location as a size 6 vector (3 translations in meter
  // and 3 rotation (theta U representation) )
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  // this pose vector is then transformed in a 4x4 homogeneous matrix
  vpHomogeneousMatrix cMo(c_r_o) ;

  // We define a robot
  // The vpRobot Camera implements a simple moving that is juste defined
  // by its location cMo
  vpRobotCamera robot ;

  // the robot position is set to the defined cMo position
  robot.setPosition(cMo) ;

  // Now that the current camera position has been defined,
  // let us defined the defined camera location.
  // It is defined by cdMo
  vpTRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;


  //----------------------------------------------------------------------
  // A 2 1/2 D visual servoing can be defined by
  // - the position of a point x,y
  // - the difference between this point depth and a desire depth
  //   modeled by log Z/Zd to be regulated to 0
  // - the rotation that the camera has to realized cdMc

  // Let us now defined the current value of these features


  // since we simulate we have to define a 3D point that will
  // forward-projected to define the current position x,y of the
  // reference point

  //------------------------------------------------------------------
  // First feature (x,y)
  vpTRACE("1st feature (x,y)");

  // Let oP be this ... point,
  // a vpPoint class has three main member
  // .oP : 3D coordinates in scene frame
  // .cP : 3D coordinates in camera frame
  // .p : 2D

  vpPoint P ;
  // defined P coordinates in the scene frame : oP
  P.setWorldCoordinates(0,0,0) ;
  // computes  the point coordinates in the camera frame and its
  // 2D coordinates cP and then p
  P.track(cMo) ;


  // Nevertheless, a vpPoint is not a feature, this is just a "tracker"
  // from which the feature are built
  // a feature is juste defined by a vector s, a way to compute the
  // interaction matrix and the error, and if required a (or a vector of)
  // 3D information

  // for a point (x,y) Visp implements the vpFeaturePoint class.
  // we no defined a feature for x,y
  vpFeaturePoint p ;

  // and we initialized the vector s=(x,y) of p from the tracker P
  // Z coordinates in p is also initialized, it will be used to compute
  // the interaction matrix
  vpFeatureBuilder::create(p,P)  ;


  // We also defined (again by forward projection) the desired position
  // of this point according to the desired camera position
  vpPoint Pd ;
  Pd.setWorldCoordinates(0,0,0) ;
  Pd.track(cdMo) ;

  // and we define (x*,y*)
  vpFeaturePoint pd ;
  vpFeatureBuilder::create(pd,Pd)  ;


  //------------------------------------------------------------------
  // Second feature log (Z/Zd)
  //

  // This case in intersting since this visual feature has not
  // been predefined in VisP
  // In such case we have a generic feature class vpGenericFeature
  // We will have to defined
  // the vector s : .set_s(...)
  // the interaction matrix Ls : .setInteractionMatrix(...)

  // log(Z/Zd) is then a size 1 vector logZ
  vpGenericFeature logZ(1) ;

  // initialized to s = log(Z/Zd)
  // Let us note that here we use the point P and Pd, it's not necessary
  // to forward project twice (it's already done)
  logZ.set_s(log(P.get_Z()/Pd.get_Z())) ;

  // This visual has to be regulated to zero

  //------------------------------------------------------------------
  // 3rd feature ThetaU
  // The thetaU feature is defined, tu represents the rotation that the camera
  // has to realized.
  // the complete displacement is then defined by:
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;  // cdMo * oMc

  // from this displacement, we extract the rotation cdRc represented by
  // the angle theta and the rotation axis u
  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;

  // This visual has to be regulated to zero

  //------------------------------------------------------------------
  // Let us now the task itself
  vpServo task ;


  // we build the task by "stacking" the visual feature
  // previously defined
  task.addFeature(p,pd) ;
  task.addFeature(logZ) ;
  task.addFeature(tu) ;

  // addFeature(X,Xd) means X should be regulated to Xd
  // addFeature(X) means that X should be regulated to 0
  // some features such as vpFeatureThetaU MUST be regulated to zero
  // (otherwise, it will results in an error at exectution level)

  //  we choose to control the robot in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  // Interaction matrix is computed with the current value of s
  task.setInteractionMatrixType(vpServo::CURRENT) ;

  // set the gain
  task.setLambda(0.1) ;

  // and display some information related to the task
  task.print() ;


  //------------------------------------------------------------------
  // An now the closed loop
  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<200)
  {
    // We get the new camera position cMo
    robot.getPosition(cMo) ;

    // and recompute the position of the reference point (x,y) and Z
    P.track(cMo) ;

    // and update the visual feature p
    vpFeatureBuilder::create(p,P)  ;

    // the visual feature logZ
    logZ.set_s(log(P.get_Z()/Pd.get_Z())) ;

    // for this latter feature the interaction matrix has not to be defined
    // it is a 1x6 matrix defined by :
    vpMatrix LlogZ(1,6) ;
    LlogZ[0][0] = LlogZ[0][1] = LlogZ[0][5] = 0 ;
    LlogZ[0][2] = -1/p.get_Z() ;
    LlogZ[0][3] = -p.get_y() ;
    LlogZ[0][4] =  p.get_x() ;
    // and initialized
    logZ.setInteractionMatrix(LlogZ) ;

    // Update the displacement that the camera has to realized
    cdMc = cdMo*cMo.inverse() ;
    // and the corresponding rotation
    tu.buildFrom(cdMc) ;

    // Let us note that that task is mainly defined by a list of pointers
    // toward visual feature
    // therefore by updating the visual feature we also update the
    // the vector  of visual feature s

    //  compute the control law
    vpColVector v ;
    vpTRACE(" ") ;

    cout << logZ.getInteractionMatrix() ;
    // compute v = -lambda L^+(s-sd)
    v = task.computeControlLaw() ;
    vpTRACE(" ") ;

    // and send the computed velocity expressed in the camera frame
    // to the robot controller
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
    // Note that for vpRobotCamera, camera position cMo, is updated using the
    // exponential map.

  }

  vpTRACE("Display task information " ) ;
  task.print() ;
  vpTRACE("Final camera location " ) ;
  cout << cMo << endl ;
}

