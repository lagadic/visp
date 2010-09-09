/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Generic virtual robot.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRobot_H
#define vpRobot_H

/*!
  \file vpRobot.h
  \brief class that defines a generic virtual robot
*/

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>


/*!
  \class vpRobot
  \brief class that defines a generic virtual robot
*/
class VISP_EXPORT vpRobot
{
public:
  /*!
    Robot control states.
  */
  typedef enum 
    {
      STATE_STOP,  /*!< Stops robot motion especially in velocity and
		     acceleration control. */
      STATE_VELOCITY_CONTROL, //!< Initialize the velocity controller.
      STATE_POSITION_CONTROL, //!< Initialize the position controller.
      STATE_ACCELERATION_CONTROL //!< Initialize the acceleration controller.
    } vpRobotStateType ;

    /** \brief valeur utilisee par default pour l'etat du robot a
     * la construction. */
  static const vpRobot::vpRobotStateType
  defaultEtatRobot = vpRobot::STATE_STOP;

  /*!
    Robot control frames. 
  */
  typedef enum 
    {
      
      REFERENCE_FRAME, /*!< Corresponds to a fixed reference frame
	attached to the robot structure. */
      ARTICULAR_FRAME, /*!< Corresponds to the joint space. */
      CAMERA_FRAME,    /*!< Corresponds to a frame attached to the
	camera mounted on the robot end-effector. */
      MIXT_FRAME /*!< Corresponds to a "virtual" frame where
	translations are expressed in the reference frame, and
	rotations in the camera frame.*/      
    } vpControlFrameType ;

    static const vpRobot::vpControlFrameType
    defaultFrameRobot = vpRobot::CAMERA_FRAME ;

private:  /* Membres privees */
    vpRobot::vpRobotStateType   stateRobot;
    vpRobot::vpControlFrameType   frameRobot;
public:
  virtual vpRobotStateType
  setRobotState (const vpRobot::vpRobotStateType newState);
  virtual
  vpRobotStateType     getRobotState (void) { return stateRobot ; }

  vpControlFrameType   setRobotFrame (vpRobot::vpControlFrameType newFrame);
  vpControlFrameType   getRobotFrame (void) { return frameRobot ; }


protected:
  double maxTranslationVelocity;
  static const double maxTranslationVelocityDefault;// = 0.2;
  double maxRotationVelocity;
  static const double maxRotationVelocityDefault;// = 0.7;


protected:
  //! number of degrees of freedom
  int nDof ;
  //! robot Jacobian expressed in the end-effector frame
  vpMatrix eJe ;
  //! is the robot Jacobian expressed in the end-effector frame available
  int eJeAvailable ;
  //! robot Jacobian expressed in the robot reference frame available
  vpMatrix fJe ;
  //! is the robot Jacobian expressed in the robot reference frame available
  int fJeAvailable ;

public:
  virtual void init() = 0 ;

  vpRobot (void);
  virtual ~vpRobot() { ; }

  //---------- Jacobian -----------------------------
  //! get the robot Jacobian expressed in the end-effector frame
  virtual void get_eJe(vpMatrix &_eJe)  = 0 ;
  //! get the robot Jacobian expressed in the robot reference frame
  virtual void get_fJe(vpMatrix &_fJe)  = 0 ;


  //! set to the controller a velocity (frame as to ve specified)
  virtual void setVelocity(const vpRobot::vpControlFrameType frame,
			   const vpColVector &vel) = 0 ;


  void setMaxTranslationVelocity (const double maxVt);
  double getMaxTranslationVelocity (void) const ;
  void setMaxRotationVelocity (const double maxVr);
  double getMaxRotationVelocity (void) const;

  //---------- POSITION -----------------------------

  //! get a displacement (frame as to ve specified)
  virtual void getPosition(const vpRobot::vpControlFrameType frame,
			   vpColVector &q)   = 0 ;

  //! get a displacement (frame as to ve specified)
  vpColVector getPosition (const vpRobot::vpControlFrameType frame);

  //! set a displacement (frame as to ve specified)
  virtual void setPosition(const vpRobot::vpControlFrameType frame,
			   const vpColVector &q)   = 0 ;


  //! get a displacement expressed in the camera frame
  virtual void getCameraDisplacement(vpColVector &v) = 0 ;
  //! get a displacement expressed  in the articular frame
  virtual void getArticularDisplacement(vpColVector  &qdot) = 0 ;
  //! get a displacement (frame as to ve specified)
  virtual void getDisplacement(const vpRobot::vpControlFrameType frame,
			       vpColVector &q) = 0 ;

  /*
    Joint limits stuff
  */
private:
  int areJointLimitsAvailable ;
  double *qmin;
  double *qmax ;
public:
  //  virtual void getJointLimits() ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
