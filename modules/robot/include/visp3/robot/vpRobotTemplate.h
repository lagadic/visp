/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef vpRobotTemplate_H
#define vpRobotTemplate_H

/*!
  \file vpRobotTemplate.h
  \brief class that defines a robot just to show which function you must
  implement
*/

#include <visp3/robot/vpRobot.h>

/*!
  \class vpRobotTemplate
  \ingroup group_robot_real_template
  \brief class that defines a robot just to show which function you must
  implement
*/

class VISP_EXPORT vpRobotTemplate : public vpRobot
{

public:
  //! basic initialization
  void init();

  //! constructor
  vpRobotTemplate();
  //! destructor
  virtual ~vpRobotTemplate();

  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe);
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe);

  //! send to the controller a velocity expressed in the camera frame
  void sendCameraVelocity(const vpColVector &v);
  //! send to the controller a velocity expressed in the articular frame
  void sendArticularVelocity(const vpColVector &qdot);
  //! send to the controller a velocity (frame as to be specified)
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  //! get a position expressed in the robot reference frame
  void getPosition(vpPoseVector &q);
  //! get a position expressed in the articular frame
  void getArticularPosition(vpColVector &q);
  //! get a displacement (frame as to be specified)
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
  //! set a displacement (frame as to be specified)
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);

  //! get a displacement (frame as to be specified)
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q);
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
