

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: moveBiclops.cpp,v $
 * Author:    Fabien SPindler
 *
 * Version control
 * ===============
 *
 *  $Id: moveBiclops.cpp,v 1.1 2006-02-21 11:17:04 fspindle Exp $
 *
 * Description
 * ============
 *  Tests the control law
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example moveBiclops.cpp

  Example of a real robot control, the biclops robot (pan-tilt turret) by
  Traclabs. The robot is controlled first in position, then in velocity.

  See http://www.traclabs.com/tracbiclops.htm for more details.
*/
#include <unistd.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpColVector.h>

#ifdef HAVE_ROBOT_BICLOPS_PT

#include <visp/vpRobotBiclops.h>

int
main()
{
  vpRobotBiclops robot ;
  vpColVector q     (vpBiclops::ndof) ; // desired position
  vpColVector qdot  (vpBiclops::ndof) ; // desired velocity
  vpColVector qm    (vpBiclops::ndof) ; // measured position
  vpColVector qm_dot(vpBiclops::ndof) ; // measured velocity

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;

  q = 0;
  q[0] = vpMath::rad(-10);
  q[1] = vpMath::rad(-20);
  CTRACE << "Set position in the articular frame: "
	 << " pan: " << vpMath::deg(q[0]) << " deg"
	 << " tilt: " << vpMath::deg(q[1]) << " deg" << endl ;
  robot.setPositioningVelocity(30.) ;
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  CTRACE << "---------------------------------------- " << endl;

  q[0] = vpMath::rad(10);
  q[1] = vpMath::rad(20);
  CTRACE << "Set position in the articular frame: "
	 << " pan: " << vpMath::deg(q[0]) << " deg"
	 << " tilt: " << vpMath::deg(q[1]) << " deg" << endl ;
  robot.setPositioningVelocity(10) ;
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  CTRACE << "---------------------------------------- " << endl;

  CTRACE << "Set STATE_VELOCITY_CONTROL" << endl;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  CTRACE << "---------------------------------------- " << endl;
  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(25) ;
  CTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(5) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


  CTRACE << "---------------------------------------- " << endl;
  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = -vpMath::rad(25) ;
  CTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(3) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


  CTRACE << "---------------------------------------- " << endl;


  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(10) ;
  CTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  CTRACE << "---------------------------------------- " << endl;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-5);
  //qdot[1] = vpMath::rad(-5);

  CTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  CTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


}
#else
int
main()
{
  ERROR_TRACE("You do not have a biclops robot connected to your computer...");
}

#endif
