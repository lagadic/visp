

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: movePtu46.cpp,v $
 * Author:    Fabien SPindler
 *
 * Version control
 * ===============
 *
 *  $Id: movePtu46.cpp,v 1.1 2006-01-16 09:57:08 fspindle Exp $
 *
 * Description
 * ============
 *  Tests the control law
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example movePtu46.cpp

  Example of a real robot control, the ptu-46 robot (pan-tilt turret). The
  robot is controlled first in position, then in velocity.

*/
#include <unistd.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef HAVE_ROBOT_PTUEVI

#include <visp/vpRobotPtu46.h>

int
main()
{
  vpRobotPtu46 robot ;
  vpColVector q(2) ;

  ERROR_TRACE(" ") ;

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;

  q = 0;
  CTRACE << "Set position in the articular frame: " << q.t();
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  q[0] = vpMath::rad(10);
  q[1] = vpMath::rad(20);
  CTRACE << "Set position in the articular frame: " << q.t();
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  vpColVector qm(2) ;
  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame " << qm.t() ;

  vpColVector qdot(2) ;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
#if 0
  qdot = 0 ;
  qdot[0] = vpMath::rad(10) ;
  qdot[1] = vpMath::rad(10) ;
  CTRACE << "Set camera frame velocity " << qdot.t() ;

  robot.setVelocity(vpRobot::CAMERA_FRAME, qdot) ;
  sleep(2) ;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-10) ;
  qdot[1] = vpMath::rad(-10) ;

  CTRACE << "Set camera frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, qdot) ;
  sleep(2) ;
#endif

  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(10) ;
  CTRACE << "Set articular frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-5);
  //qdot[1] = vpMath::rad(-5);

  CTRACE << "Set articular frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;




}
#else
int
main()
{
  ERROR_TRACE("You do not have a ptu-46 robot connected to your computer...");
}

#endif
