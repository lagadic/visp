

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: moveAfma4.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: moveAfma4.cpp,v 1.1 2005-09-19 13:30:27 fspindle Exp $
 *
 * Description
 * ============
 *  Tests the control law
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example moveAfma4.cpp

  Example of a real robot control, the Afma4 robot (cylindrical robot, with 4
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef HAVE_ROBOT_AFMA4

#include <visp/vpRobotAfma4.h>
#include <visp/vpDebug.h>

int gsl_warnings_off;

int
main()
{
  vpRobotAfma4 robot ;

  vpColVector qd(4) ;
  vpColVector q(4) ;

  //
  // Position control in articular
  //
  qd[0] = vpMath::rad(10);
  qd[1] = -0.1;
  qd[2] = vpMath::rad(20);
  qd[3] = vpMath::rad(-10);

  TRACE("Position control: in articular...") ;
  CTRACE << "  position to reach: " << qd.t() << endl ;
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  robot.setPosition(vpRobot::ARTICULAR_FRAME, qd) ;
  sleep(1) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, q) ;
  CTRACE << "  measured position: " << q.t() ;
  sleep(1) ;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  //
  // Velocity control in articular
  //
  TRACE("Velocity control: in articular...") ;
  TRACE("Velocity control: rotation arround vertical axis...") ;
  q = 0 ;
  q[0] = vpMath::rad(2) ; // rotation arround vertical axis
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;

  TRACE("Velocity control: vertical translation...") ;
  q = 0 ;
  q[1] = 0.2 ; // Vertical translation
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
  sleep(5) ;

  TRACE("Velocity control: vertical translation...") ;
  q = 0 ;
  q[1] = -0.2 ; // Vertical translation
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
  sleep(5) ;
  TRACE("Velocity control: pan rotation...") ;
  q = 0 ;
  q[2] = vpMath::rad(3) ; // pan
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
  sleep(5) ;

  TRACE("Velocity control: tilt rotation...") ;
  q = 0 ;
  q[3] = vpMath::rad(2) ; // tilt
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
  sleep(5) ;

  //
  // Velocity control in camera frame
  //
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
  TRACE("Velocity control: in camera frame...") ;
  TRACE("Velocity control: rx rotation...") ;
  q.resize(2) ;
  q = 0.0;
  q[0] = vpMath::rad(2) ; // rotation arround vertical axis
  robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
  sleep(5) ;

  TRACE("Velocity control: ry rotation...") ;
  q.resize(2) ;
  q = 0.0;
  q[1] = vpMath::rad(2) ; // rotation arround vertical axis
  robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
  sleep(5) ;

  //
  // Camera control : zoom / focus / iris
  //

  int zoom, focus, iris;

  TRACE("Camera control...") ;
  TRACE("Actual camera settings...") ;
  zoom = robot.getZoom() ;
  iris = robot.getIris() ;
  focus = robot.getFocus() ;
  CTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << endl;

  TRACE("Set camera settings...") ;
  zoom = 3000;
  focus = 400;
  iris = 900;
  CTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << endl;
  robot.setZoom(zoom);
  robot.setIris(iris);
  robot.setFocus(focus);

  TRACE("Actual camera settings...") ;
  zoom = robot.getZoom() ;
  iris = robot.getIris() ;
  focus = robot.getFocus() ;
  CTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << endl;

  TRACE("Set camera auto iris on...") ;
  robot.setAutoIris(true);
  sleep(2);
  iris = robot.getIris() ;
  CTRACE << "Actual iris: " << iris << endl;

  TRACE("Set camera auto iris off...") ;
  robot.setAutoIris(false);
  robot.setIris(400);
  iris = robot.getIris() ;
  CTRACE << "Actual iris: " << iris << endl;



}
#else
int
main()
{
  ERROR_TRACE("You do not have an afma4 robot connected to your computer...");
}

#endif
