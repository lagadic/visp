/****************************************************************************
 *
 * $Id: moveAfma6.cpp,v 1.6 2007-01-29 10:37:11 asaunier Exp $
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
  \example moveAfma6.cpp

  \brief Example of a real robot control, the Afma6 robot (cartesian robot, with 6
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
  try
    {
      vpRobotAfma6 robot ;

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
}

#endif
