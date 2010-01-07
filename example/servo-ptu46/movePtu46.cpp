/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 *   Tests the control law
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file movePtu46.cpp

  \brief Example of a real robot control, the ptu-46 robot (pan-tilt turret). The
  robot is controlled first in position, then in velocity.

*/


/*!
  \example movePtu46.cpp

  Example of a real robot control, the ptu-46 robot (pan-tilt turret). The
  robot is controlled first in position, then in velocity.

*/
#ifdef UNIX
#  include <unistd.h>
#endif

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_PTU46

#include <visp/vpRobotPtu46.h>

int
main()
{
  try
  {
  vpRobotPtu46 robot ;
  vpColVector q(2) ;

  vpERROR_TRACE(" ") ;

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;

  q = 0;
  vpCTRACE << "Set position in the articular frame: " << q.t();
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  q[0] = vpMath::rad(10);
  q[1] = vpMath::rad(20);
  vpCTRACE << "Set position in the articular frame: " << q.t();
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q) ;

  vpColVector qm(2) ;
  robot.getPosition(vpRobot::ARTICULAR_FRAME, qm) ;
  vpCTRACE << "Position in the articular frame " << qm.t() ;

  vpColVector qdot(2) ;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
#if 0
  qdot = 0 ;
  qdot[0] = vpMath::rad(10) ;
  qdot[1] = vpMath::rad(10) ;
  vpCTRACE << "Set camera frame velocity " << qdot.t() ;

  robot.setVelocity(vpRobot::CAMERA_FRAME, qdot) ;
  sleep(2) ;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-10) ;
  qdot[1] = vpMath::rad(-10) ;

  vpCTRACE << "Set camera frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, qdot) ;
  sleep(2) ;
#endif

  qdot = 0 ;
  //  qdot[0] = vpMath::rad(0.1) ;
  qdot[1] = vpMath::rad(10) ;
  vpCTRACE << "Set articular frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;

  qdot = 0 ;
  qdot[0] = vpMath::rad(-5);
  //qdot[1] = vpMath::rad(-5);

  vpCTRACE << "Set articular frame velocity " << qdot.t() ;
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
  sleep(2) ;
  }
  catch (...)
  {
    std::cout << "Sorry PtU46 not available ..." << std::endl;
  }

  return 0;

}
#else
int
main()
{
  vpERROR_TRACE("You do not have a ptu-46 robot connected to your computer...");
  return 0; 
}

#endif
