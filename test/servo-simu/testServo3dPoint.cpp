

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServo3dPoint.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServo3dPoint.cpp,v 1.2 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *   servo a 3D point
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServo3dPoint.cpp
  \brief  servo a 3D point, tests the control law, eye-in-hand control, velocity computed in the camera frame
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint3D.h>
#include <visp/vpPoint.h>
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
  cout << " task : servo a 3D point " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;
  robot.setPosition(cMo) ;


  vpTRACE("sets the point coordinates in the world frame "  ) ;
  vpPoint point ;
  point.setWorldCoordinates(0,0,0) ;


  vpTRACE("project : computes  the point coordinates in the camera frame "  ) ;
  point.track(cMo) ;

  cout << point.cP.t() ;

  vpFeaturePoint3D p ;
  p.buildFrom(point) ;

  vpTRACE("sets the desired position of the point ") ;
  vpFeaturePoint3D pd ;
  pd.set_XYZ(0,0,1) ;



  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  vpTRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

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
    if (iter==1) vpTRACE("\t\t new point position ") ;
    point.track(cMo) ;
    p.buildFrom(point) ;
    //   cout << p.cP.t() ;
    //   cout << (p.get_s()).t() ;


    if (iter==1) vpTRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    //  vpTRACE("computeControlLaw" ) ;

    if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    //   vpTRACE("\t\t || s - s* || ") ;
    cout << task.error.sumSquare() <<endl ; ;
  }

  vpTRACE("Display task information " ) ;
  task.print() ;
}

