

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServo3D
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServo3D.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *   test 3D visual servoing
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServo3D.cpp
  \brief   test 3D visual servoing
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>

int
main()
{
  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " Simulation " << endl ;
  cout << " task :  3D visual servoing " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  ctrace ; cout << endl ;
  vpHomogeneousMatrix cMo(c_r_o) ;
  ctrace ; cout << endl ;
  robot.setPosition(cMo) ;
  ctrace ; cout << endl ;

  TRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;



  TRACE("compute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;

  vpFeatureTranslation t ;
  vpFeatureThetaU tu ;
  t.buildFrom(cdMc) ;
  tu.buildFrom(cdMc) ;

  TRACE("sets the desired rotation (always zero !) ") ;
  TRACE("since s is the rotation that the camera has to realize ") ;

  vpFeatureTranslation td ;
  vpFeatureThetaU tud ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  task.addFeature(t,td) ;
  task.addFeature(tu,tud) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;
  while(iter++<200)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;

    if (iter==1) TRACE("\t\t new rotation to realize ") ;
    cdMc = cdMo*cMo.inverse() ;
    t.buildFrom(cdMc) ;
    tu.buildFrom(cdMc) ;


    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    if (iter==1) task.print() ;

    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;


    cout << task.error.sumSquare() <<endl ; ;
  }

  TRACE("Display task information " ) ;
  task.print() ;
}

