

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
 *  $Id: moveBiclops.cpp,v 1.4 2006-08-23 10:43:57 brenier Exp $
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

#include <visp/vpParseArgv.h>
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpColVector.h>
#include <visp/vpTime.h>

#ifdef VISP_HAVE_BICLOPS

#include <visp/vpRobotBiclops.h>

// List of allowed command line options
#define GETOPTARGS	"c:h"

/*

  Print the program options.

  \param conf : Biclops configuration file.

 */
void usage(char *name, char *badparam, string conf)
{
  fprintf(stdout, "\n\
Move the biclops robot\n\
\n\
SYNOPSIS\n\
  %s [-c <Biclops configuration file>] [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c <Biclops configuration file>                      %s\n\
     Sets the biclops robot configuration file.\n\n",
	  conf.c_str());

}

/*!

  Set the program options.

  \param conf : Biclops configuration file.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, string& conf)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': conf = optarg; break;
    case 'h': usage(argv[0], NULL, conf); return false; break;

    default:
      usage(argv[0], optarg, conf); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, conf);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}

int
main(int argc, char ** argv)
{
  string opt_conf = "/usr/share/BiclopsDefault.cfg";

  // Read the command line options
  if (getOptions(argc, argv, opt_conf) == false) {
    exit (-1);
  }

  vpRobotBiclops robot(opt_conf.c_str());
  
  vpColVector q     (vpBiclops::ndof) ; // desired position
  vpColVector qdot  (vpBiclops::ndof) ; // desired velocity
  vpColVector qm    (vpBiclops::ndof) ; // measured position
  vpColVector qm_dot(vpBiclops::ndof) ; // measured velocity

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;

  q = 0;
  q[0] = vpMath::rad(-10);
  q[1] = vpMath::rad(-20);
  vpCTRACE << "Set position in the articular frame: "
	 << " pan: " << vpMath::deg(q[0]) << " deg"
	 << " tilt: " << vpMath::deg(q[1]) << " deg" << endl ;
  robot.setPositioningVelocity(30.) ;
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  vpCTRACE << "---------------------------------------- " << endl;

  q[0] = vpMath::rad(10);
  q[1] = vpMath::rad(20);
  vpCTRACE << "Set position in the articular frame: "
	 << " pan: " << vpMath::deg(q[0]) << " deg"
	 << " tilt: " << vpMath::deg(q[1]) << " deg" << endl ;
  robot.setPositioningVelocity(10) ;
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  vpCTRACE << "---------------------------------------- " << endl;

  vpCTRACE << "Set STATE_VELOCITY_CONTROL" << endl;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  vpCTRACE << "---------------------------------------- " << endl;
  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(25) ;
  vpCTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
 
  //waits 5000ms
  vpTime::wait(5000.0);

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


  vpCTRACE << "---------------------------------------- " << endl;
  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = -vpMath::rad(25) ;
  vpCTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  
  //waits 3000 ms
  vpTime::wait(3000.0);

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


  vpCTRACE << "---------------------------------------- " << endl;


  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(10) ;
  vpCTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;

  //waits 2000 ms
  vpTime::wait(2000.0);

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;

  vpCTRACE << "---------------------------------------- " << endl;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-5);
  //qdot[1] = vpMath::rad(-5);

  vpCTRACE << "Set articular frame velocity "
	 << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
	 << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << endl ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;

  //waits 2000 ms
  vpTime::wait(2000.0);

  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0]) << " deg"
	 << " tilt: " << vpMath::deg(qm[1]) << " deg" << endl ;
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Velocity in the articular frame: "
	 << " pan: " << vpMath::deg(qm[0])
	 << " tilt: " << vpMath::deg(qm[1]) << endl ;


}
#else
int
main()
{
  vpERROR_TRACE("You do not have a biclops robot connected to your computer...");
}

#endif
