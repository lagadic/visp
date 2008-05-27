/****************************************************************************
 *
 * $Id: moveAfma6.cpp,v 1.10 2008-05-27 09:42:19 fspindle Exp $
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
 * Example of a real robot control, the Afma6 robot (cartesian robot, with 6
 * degrees of freedom). The robot is controlled first in position, then in
 * velocity.
 *
 * Authors:
 * Eric Marchand
 *
 ****************************************************************************/
/*!
  \file moveAfma6.cpp

  \brief Example of a real robot control, the Afma6 robot (cartesian robot, with 6
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/

/*!
  \example moveAfma6.cpp

  Example of a real robot control, the Afma6 robot (cartesian robot, with 6
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

#ifdef VISP_HAVE_AFMA6

#include <visp/vpRobotAfma6.h>

// List of allowed command line options
#define GETOPTARGS	"mh"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Example of a positionning control followed by a velocity control \n\
of the Afma6 robot.\n						   \
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
bool getOptions(int argc, char **argv, bool &control)
{
  char *optarg;
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
main(int argc, char ** argv)
{
  try
    {
      bool control = true; // Turn on the robot control by applying positions
			   // and velocities to the robot.
      // Read the command line options
      if (getOptions(argc, argv, control) == false) {
	exit (-1);
      }

      vpRobotAfma6 robot ;

      if (control)
	robot.setPosition(vpRobot::REFERENCE_FRAME,
			  -0.1,0.2,0.1,
			  vpMath::rad(10),vpMath::rad(20),vpMath::rad(30)) ;

      vpColVector q(6) ;
      robot.getPosition(vpRobot::REFERENCE_FRAME, q) ;
      std::cout << "Position in the reference frame " << q.t() ;

      robot.getPosition(vpRobot::ARTICULAR_FRAME, q) ;
      std::cout << "Position in the articular frame " << q.t() ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
      vpERROR_TRACE(" ") ;
      q =0 ;
      q[2] = 0.01 ;

      if (control)
	robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      q = 0 ;
      q[1] = 0.01 ;

      if (control)
	robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      q = 0 ;
      q[5] = 0.01 ;

      if (control)
	robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;
    }
  catch (...)
    {
      vpERROR_TRACE(" Test failed") ;
      return 0;
    }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot connected to your computer...");
  return 0;
}

#endif
