

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServo2D0.5.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: test2Servo2D0.5.cpp,v 1.2 2005-09-07 14:05:16 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *   using theta U visual feature
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServo2D0.5.cpp
  \brief  test 2 1/2 D visual servoing (x,y,log Z, theta U)
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
  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " task :  2 1/2 D visual servoing  (x,y,log Z, theta U)" << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  CTRACE ; cout << endl ;
  vpHomogeneousMatrix cMo(c_r_o) ;
  CTRACE ; cout << endl ;
  robot.setPosition(cMo) ;
  CTRACE ; cout << endl ;

  TRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;


  //------------------------------------------------------------------
  TRACE("\tsets the point coordinates in the world frame "  ) ;
  vpPoint point ;
  point.setWorldCoordinates(0,0,0) ;
  TRACE("\tproject : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
  point.track(cMo) ;

  vpPoint pointd ;
  pointd.setWorldCoordinates(0,0,0) ;
  pointd.track(cdMo) ;

  //------------------------------------------------------------------
  TRACE("1st feature (x,y)");
  vpFeaturePoint p,pd ;

  vpFeatureBuilder::create(p,point)  ;
  vpFeatureBuilder::create(pd,pointd)  ;


  //------------------------------------------------------------------
  TRACE("2nd feature (logZ)") ;
  TRACE("\tnot necessary to project twice (reuse p)") ;
  vpGenericFeature logZ(1) ;
  logZ.set_s(log(point.get_Z()/pointd.get_Z())) ;


  //------------------------------------------------------------------
  TRACE("3rd feature ThetaU") ;
  TRACE("\tcompute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;

  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;

  TRACE("\tsets the desired rotation (always zero !) ") ;
  TRACE("\tsince s is the rotation that the camera has to realize ") ;


  //------------------------------------------------------------------

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;
  task.addFeature(p,pd) ;
  task.addFeature(logZ) ;
  task.addFeature(tu) ;

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

    if (iter==1) TRACE("\t\t update the feature ") ;
    point.track(cMo) ;
    vpFeatureBuilder::create(p,point)  ;

    cdMc = cdMo*cMo.inverse() ;
    tu.buildFrom(cdMc) ;

    if (iter==1) TRACE("\t\t there is no feature for logZ, we explicitely "
		       "build the related interaction matrix") ;
    logZ.set_s(log(point.get_Z()/pointd.get_Z())) ;
    vpMatrix LlogZ(1,6) ;
    LlogZ[0][0] = LlogZ[0][1] = LlogZ[0][5] = 0 ;
    LlogZ[0][2] = -1/p.get_Z() ;
    LlogZ[0][3] = -p.get_y() ;
    LlogZ[0][4] =  p.get_x() ;

    logZ.setInteractionMatrix(LlogZ) ;


    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;

    if (iter==1) task.print() ;

    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;


    cout << task.error.sumSquare() <<endl ; ;
  }

  TRACE("Display task information " ) ;
  task.print() ;
  TRACE("Final camera location " ) ;
  cout << cMo << endl ;
}

