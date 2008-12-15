/****************************************************************************
 *
 * $Id: moveAfma4.cpp,v 1.10 2008-12-15 17:19:22 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
 * Test for Afma 4 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file moveAfma4.cpp

  \brief Example of a real robot control, the Afma4 robot (cylindrical
  robot, with 4 degrees of freedom). The robot is controlled first in
  position, then in velocity.

*/
/*!
  \example moveAfma4.cpp

  Example of a real robot control, the Afma4 robot (cylindrical robot,
  with 4 degrees of freedom). The robot is controlled first in
  position, then in velocity.

*/
#include <stdlib.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA4

#include <visp/vpParseArgv.h>
#include <visp/vpRobotAfma4.h>

// List of allowed command line options
#define GETOPTARGS	"mh"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Example of a positionning control followed by a velocity control \n\
of the Afma4 robot.\n						   \
\n\
SYNOPSIS\n\
  %s [-m] [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -m\n\
     Turn off the control of the robot. This option is\n\
     essentially useful for security reasons during nightly\n\
     tests.\n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }

}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param control: Indicates if the control has to be applied to the robot.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &control)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'm': control = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

int
main(int argc, const char ** argv)
{
  try
    {
      bool control = true; // Turn on the robot control by applying positions
			   // and velocities to the robot.
      // Read the command line options
      if (getOptions(argc, argv, control) == false) {
	exit (-1);
      }

      vpRobotAfma4 robot ;

      vpColVector qd(robot.njoint) ;
      vpColVector q(robot.njoint) ;

      //
      // Position control in articular
      //
      qd[0] = vpMath::rad(10);
      qd[1] = -0.1;
      qd[2] = vpMath::rad(20);
      qd[3] = vpMath::rad(-10);

      std::cout << "Position control: in articular..." << std::endl;
      std::cout << "  position to reach: " << qd.t() << std::endl;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
      if (control)
	robot.setPosition(vpRobot::ARTICULAR_FRAME, qd) ;
      sleep(1) ;

 
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q) ;
      std::cout << "  measured position: " << q.t() ;
      sleep(1) ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      //
      // Velocity control in articular
      //
      std::cout << "Velocity control: in articular..." << std::endl;

      q = 0 ;
      q[0] = vpMath::rad(2) ; // rotation arround vertical axis
      std::cout << "  rotation arround vertical axis: " << q[0] << std::endl; 
      if (control)
	robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;
     
      q = 0 ;
      q[1] = 0.2 ; // Vertical translation
      std::cout << "  vertical translation: " << q[1] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      q = 0 ;
      q[1] = -0.2 ; // Vertical translation
      std::cout << "  vertical translation: " << q[1] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;
      q = 0 ;
      q[2] = vpMath::rad(3) ; // pan
      std::cout << "  pan rotation: " << q[2] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      q = 0 ;
      q[3] = vpMath::rad(2) ; // tilt
      std::cout << "  tilt rotation: " << q[3] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      //
      // Velocity control in camera frame
      //
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
      std::cout << "Velocity control: in camera frame..." << std::endl;
      q.resize(2) ;
      q = 0.0;
      q[0] = vpMath::rad(2) ; // rotation arround vertical axis
      std::cout << "  rx rotation: " << q[0] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      q.resize(2) ;
      q = 0.0;
      q[1] = vpMath::rad(2) ; // rotation arround vertical axis
      std::cout << "  ry rotation: " << q[1] << std::endl;
      if (control)
	robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      std::cout << "The end" << std::endl;
    }
  catch (...) {
    vpERROR_TRACE(" Test failed") ;
  }
  return 0;
}
#else
int
main()
{
  vpERROR_TRACE("You do not have an afma4 robot connected to your computer...");
  return 0;
}

#endif
