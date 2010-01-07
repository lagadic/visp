/****************************************************************************
 *
 * $Id: vpRobotTemplate.h,v 1.6 2008-05-27 11:55:48 fspindle Exp $
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


#ifndef vpRobotTemplate_H
#define vpRobotTemplate_H

/*!
  \file vpRobotTemplate.h
  \brief class that defines a robot just to show which function you must implement
*/

#include <visp/vpConfig.h>
#include <visp/vpRobot.h>

/*!
  \class vpRobotTemplate
  \brief class that defines a robot just to show which function you must implement
*/

class VISP_EXPORT vpRobotTemplate : public vpRobot
{

public:

  //! basic initialization
  void init() ;

  //! constructor
  vpRobotTemplate() ;
  //! destructor
  virtual ~vpRobotTemplate() ;


  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe) ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe) ;

  //! send to the controller a velocity expressed in the camera frame
  void sendCameraVelocity(const vpColVector &v)   ;
  //! send to the controller a velocity expressed in the articular frame
  void sendArticularVelocity(const vpColVector &qdot)  ;
  //! send to the controller a velocity (frame as to be specified)
  void setVelocity(const vpRobot::vpControlFrameType frame,
                   const  vpColVector &vel) ;

  //! get a position expressed in the robot reference frame
  void getPosition(vpPoseVector &q) ;
  //! get a position expressed in the articular frame
  void getArticularPosition(vpColVector &q) ;
  //! get a displacement (frame as to be specified)
  void getPosition(const vpRobot::vpControlFrameType frame,
                   vpColVector &q) ;
  //! set a displacement (frame as to be specified)
  void setPosition(const vpRobot::vpControlFrameType frame,
                   const vpColVector &q) ;

  //! get a displacement expressed in the camera frame
  void getCameraDisplacement(vpColVector &v) ;
  //! get a displacement expressed  in the articular frame
  void getArticularDisplacement(vpColVector &qdot) ;
  //! get a displacement (frame as to be specified)
  void getDisplacement(const vpRobot::vpControlFrameType frame,
                       vpColVector &q) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
