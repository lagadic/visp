/****************************************************************************
 *
 * $Id: vpRobotTemplate.cpp,v 1.6 2008-05-23 09:02:45 asaunier Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Defines a robot just to show which function you must implement.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpRobotTemplate.cpp
  \brief class that defines a robot just to show which function you must implement
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotTemplate.h>
#include <visp/vpDebug.h>

//! basic initialization
void vpRobotTemplate::init()
{
  vpTRACE(" Get the joint limits " ) ;
  std::cout << "Not implemented ! " << std::endl;
}

//! constructor
vpRobotTemplate::vpRobotTemplate()
{
  init() ;
}


//! constructor
vpRobotTemplate::~vpRobotTemplate()
{
  std::cout << "Not implemented ! " << std::endl;
}

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

get_eJe
get_fJe

*/

//! get the robot Jacobian expressed in the end-effector frame
void
vpRobotTemplate::get_eJe(vpMatrix &/* _eJe */)
{
  std::cout << "Not implemented ! " << std::endl;
}

//! get the robot Jacobian expressed in the robot reference frame
void
vpRobotTemplate::get_fJe(vpMatrix &/* _fJe */)
{
  std::cout << "Not implemented ! " << std::endl;
}


/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

sendCameraVelocity
sendArticularVelocity


*/
//! send to the controller a velocity expressed in the camera frame
void vpRobotTemplate::sendCameraVelocity(const vpColVector &/* v */)
{
  std::cout << "Not implemented ! " << std::endl;
  std::cout << "To implement me you need : " << std::endl ;
  std::cout << "\t to known the robot jacobian expressed in " ;
  std::cout << "the end-effector frame (eJe) " <<std::endl ;
  std::cout << "\t the frame transformation  between camera frame " ;
  std::cout << "and end-effector frame (cMe)" << std::endl ;
}

//! send to the controller a velocity expressed in the articular frame
void
vpRobotTemplate::sendArticularVelocity(const vpColVector &/* qdot */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! send to the controller a velocity (frame as to ve specified)
void
vpRobotTemplate::setVelocity(const vpRobot::vpControlFrameType /* frame */,
                             const vpColVector &/* vel */)
{
  std::cout << "Not implemented ! " << std::endl;
}


/*

THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

//! get a position expressed in the robot reference frame
void
vpRobotTemplate::getPosition(vpPoseVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a position expressed in the articular frame
void
vpRobotTemplate::getArticularPosition(vpColVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a displacement (frame as to ve specified)
void
vpRobotTemplate::getPosition( const vpRobot::vpControlFrameType/*frame*/,
                              vpColVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! set a displacement (frame as to ve specified)
void
vpRobotTemplate::setPosition( const vpRobot::vpControlFrameType/*frame*/,
                              const vpColVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a displacement expressed in the camera frame
void
vpRobotTemplate::getCameraDisplacement(vpColVector &/* v */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a displacement expressed  in the articular frame
void
vpRobotTemplate::getArticularDisplacement(vpColVector &/* qdot */)
{
  std::cout << "Not implemented ! " << std::endl;
}
//! get a displacement (frame as to ve specified)
void
vpRobotTemplate::getDisplacement(const vpRobot::vpControlFrameType /* frame */,
                                 vpColVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
