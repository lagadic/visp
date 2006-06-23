

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
 *  $Id: testServo3D.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
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


  vpTRACE("sets the initial camera location " ) ;
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  vpCTRACE ; cout << endl ;
  vpHomogeneousMatrix cMo(c_r_o) ;
  vpCTRACE ; cout << endl ;
  robot.setPosition(cMo) ;
  vpCTRACE ; cout << endl ;

  vpTRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;



  vpTRACE("compute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;

  vpFeatureTranslation t ;
  vpFeatureThetaU tu ;
  t.buildFrom(cdMc) ;
  tu.buildFrom(cdMc) ;

  vpTRACE("sets the desired rotation (always zero !) ") ;
  vpTRACE("since s is the rotation that the camera has to realize ") ;

  vpFeatureTranslation td ;
  vpFeatureThetaU tud ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  task.addFeature(t,td) ;
  task.addFeature(tu,tud) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<200)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) vpTRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;

    if (iter==1) vpTRACE("\t\t new rotation to realize ") ;
    cdMc = cdMo*cMo.inverse() ;
    t.buildFrom(cdMc) ;
    tu.buildFrom(cdMc) ;


    if (iter==1) vpTRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    if (iter==1) task.print() ;

    if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;


    cout << task.error.sumSquare() <<endl ; ;
  }

  vpTRACE("Display task information " ) ;
  task.print() ;
}

