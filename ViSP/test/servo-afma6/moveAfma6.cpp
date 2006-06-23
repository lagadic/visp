

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: moveAfma6.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: moveAfma6.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *  Tests the control law
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example moveAfma6.cpp

  Example of a real robot control, the Afma6 robot (cartesian robot, with 6
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA6

#include <visp/vpRobotAfma6.h>

int
main()
{
  vpRobotAfma6 robot ;

  vpERROR_TRACE(" ") ;


  robot.setPosition(vpRobot::REFERENCE_FRAME,
		    -0.1,0.2,0.1,
		    vpMath::rad(10),vpMath::rad(20),vpMath::rad(30)) ;

  vpColVector q(6) ;
  robot.getPosition(vpRobot::REFERENCE_FRAME, q) ;
  cout << "Position in the reference frame " << q.t() ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, q) ;
  cout << "Position in the articular frame " << q.t() ;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
  vpERROR_TRACE(" ") ;
  q =0 ;
  q[2] = 0.01 ;


  robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
  sleep(5) ;

  q = 0 ;
  q[1] = 0.01 ;

  robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
  sleep(5) ;

  q = 0 ;
  q[5] = 0.01 ;

  robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
  sleep(5) ;




}
#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot connected to your computer...");
}

#endif
