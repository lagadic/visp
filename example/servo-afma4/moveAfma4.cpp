/****************************************************************************
 *
 * $Id: moveAfma4.cpp,v 1.6 2007-04-20 14:22:15 asaunier Exp $
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

  \brief Example of a real robot control, the Afma4 robot (cylindrical robot, with 4
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/
/*!
  \example moveAfma4.cpp

  Example of a real robot control, the Afma4 robot (cylindrical robot, with 4
  degrees of freedom). The robot is controlled first in position, then in
  velocity.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA4

#include <visp/vpRobotAfma4.h>
#include <visp/vpDebug.h>

int gsl_warnings_off;

int
main()
{
  try
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

      vpTRACE("Position control: in articular...") ;
      vpCTRACE << "  position to reach: " << qd.t() << std::endl ;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
      robot.setPosition(vpRobot::ARTICULAR_FRAME, qd) ;
      sleep(1) ;

      robot.getPosition(vpRobot::ARTICULAR_FRAME, q) ;
      vpCTRACE << "  measured position: " << q.t() ;
      sleep(1) ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      //
      // Velocity control in articular
      //
      vpTRACE("Velocity control: in articular...") ;
      vpTRACE("Velocity control: rotation arround vertical axis...") ;
      q = 0 ;
      q[0] = vpMath::rad(2) ; // rotation arround vertical axis
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;

      vpTRACE("Velocity control: vertical translation...") ;
      q = 0 ;
      q[1] = 0.2 ; // Vertical translation
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      vpTRACE("Velocity control: vertical translation...") ;
      q = 0 ;
      q[1] = -0.2 ; // Vertical translation
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;
      vpTRACE("Velocity control: pan rotation...") ;
      q = 0 ;
      q[2] = vpMath::rad(3) ; // pan
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      vpTRACE("Velocity control: tilt rotation...") ;
      q = 0 ;
      q[3] = vpMath::rad(2) ; // tilt
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q) ;
      sleep(5) ;

      //
      // Velocity control in camera frame
      //
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
      vpTRACE("Velocity control: in camera frame...") ;
      vpTRACE("Velocity control: rx rotation...") ;
      q.resize(2) ;
      q = 0.0;
      q[0] = vpMath::rad(2) ; // rotation arround vertical axis
      robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      vpTRACE("Velocity control: ry rotation...") ;
      q.resize(2) ;
      q = 0.0;
      q[1] = vpMath::rad(2) ; // rotation arround vertical axis
      robot.setVelocity(vpRobot::CAMERA_FRAME, q) ;
      sleep(5) ;

      //
      // Camera control : zoom / focus / iris
      //

      int zoom, focus, iris;

      vpTRACE("Camera control...") ;
      vpTRACE("Actual camera settings...") ;
      zoom = robot.getZoom() ;
      iris = robot.getIris() ;
      focus = robot.getFocus() ;
      vpCTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << std::endl;

      vpTRACE("Set camera settings...") ;
      zoom = 3000;
      focus = 400;
      iris = 900;
      vpCTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << std::endl;
      robot.setZoom(zoom);
      robot.setIris(iris);
      robot.setFocus(focus);

      vpTRACE("Actual camera settings...") ;
      zoom = robot.getZoom() ;
      iris = robot.getIris() ;
      focus = robot.getFocus() ;
      vpCTRACE << "  Z: " << zoom << " F: " << focus << " I: " << iris << std::endl;

      vpTRACE("Set camera auto iris on...") ;
      robot.setAutoIris(true);
      sleep(2);
      iris = robot.getIris() ;
      vpCTRACE << "Actual iris: " << iris << std::endl;

      vpTRACE("Set camera auto iris off...") ;
      robot.setAutoIris(false);
      robot.setIris(400);
      iris = robot.getIris() ;
      vpCTRACE << "Actual iris: " << iris << std::endl;
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
  vpERROR_TRACE("You do not have an afma4 robot connected to your computer...");
}

#endif
