/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

  \brief Example of a real robot control, the ptu-46 robot (pan-tilt turret).
  The robot is controlled first in position, then in velocity.

*/

/*!
  \example movePtu46.cpp

  Example of a real robot control, the ptu-46 robot (pan-tilt turret). The
  robot is controlled first in position, then in velocity.

*/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <unistd.h>
#endif

#ifdef VISP_HAVE_PTU46

#include <visp3/robot/vpRobotPtu46.h>

int main()
{
  try {
    vpRobotPtu46 robot;
    vpColVector q(2);

    vpERROR_TRACE(" ");

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    q = 0;
    vpCTRACE << "Set position in the articular frame: " << q.t();
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);

    q[0] = vpMath::rad(10);
    q[1] = vpMath::rad(20);
    vpCTRACE << "Set position in the articular frame: " << q.t();
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);

    vpColVector qm(2);
    robot.getPosition(vpRobot::ARTICULAR_FRAME, qm);
    vpCTRACE << "Position in the articular frame " << qm.t();

    vpColVector qdot(2);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
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

    qdot = 0;
    //  qdot[0] = vpMath::rad(0.1) ;
    qdot[1] = vpMath::rad(10);
    vpCTRACE << "Set articular frame velocity " << qdot.t();
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
    sleep(2);

    qdot = 0;
    qdot[0] = vpMath::rad(-5);
    // qdot[1] = vpMath::rad(-5);

    vpCTRACE << "Set articular frame velocity " << qdot.t();
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
    sleep(2);
  }
  catch (const vpException &e) {
    std::cout << "Sorry PtU46 not available. Got exception: " << e << std::endl;
    return EXIT_FAILURE
  }
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You do not have an PTU46 PT robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
