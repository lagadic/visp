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
 * Defines a robot just to show which function you must implement.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpRobotTemplate.cpp
  \brief class that defines a robot just to show which function you must
  implement
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotTemplate.h>

//! basic initialization
void vpRobotTemplate::init()
{
  vpTRACE(" Get the joint limits ");
  std::cout << "Not implemented ! " << std::endl;
}

//! constructor
vpRobotTemplate::vpRobotTemplate() { init(); }

//! constructor
vpRobotTemplate::~vpRobotTemplate() { std::cout << "Not implemented ! " << std::endl; }

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

get_eJe
get_fJe

*/

//! get the robot Jacobian expressed in the end-effector frame
void vpRobotTemplate::get_eJe(vpMatrix & /* _eJe */) { std::cout << "Not implemented ! " << std::endl; }

//! get the robot Jacobian expressed in the robot reference frame
void vpRobotTemplate::get_fJe(vpMatrix & /* _fJe */) { std::cout << "Not implemented ! " << std::endl; }

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

sendCameraVelocity
sendArticularVelocity


*/
//! send to the controller a velocity expressed in the camera frame
void vpRobotTemplate::sendCameraVelocity(const vpColVector & /* v */)
{
  std::cout << "Not implemented ! " << std::endl;
  std::cout << "To implement me you need : " << std::endl;
  std::cout << "\t to known the robot jacobian expressed in ";
  std::cout << "the end-effector frame (eJe) " << std::endl;
  std::cout << "\t the frame transformation  between camera frame ";
  std::cout << "and end-effector frame (cMe)" << std::endl;
}

//! send to the controller a velocity expressed in the articular frame
void vpRobotTemplate::sendArticularVelocity(const vpColVector & /* qdot */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! send to the controller a velocity (frame as to ve specified)
void vpRobotTemplate::setVelocity(const vpRobot::vpControlFrameType /* frame */, const vpColVector & /* vel */)
{
  std::cout << "Not implemented ! " << std::endl;
}

/*

THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

//! get a position expressed in the robot reference frame
void vpRobotTemplate::getPosition(vpPoseVector & /* q */) { std::cout << "Not implemented ! " << std::endl; }
//! get a position expressed in the articular frame
void vpRobotTemplate::getArticularPosition(vpColVector & /* q */) { std::cout << "Not implemented ! " << std::endl; }
//! get a displacement (frame as to ve specified)
void vpRobotTemplate::getPosition(const vpRobot::vpControlFrameType /*frame*/, vpColVector & /* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! set a displacement (frame as to ve specified)
void vpRobotTemplate::setPosition(const vpRobot::vpControlFrameType /*frame*/, const vpColVector & /* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a displacement (frame as to ve specified)
void vpRobotTemplate::getDisplacement(const vpRobot::vpControlFrameType /* frame */, vpColVector & /* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
