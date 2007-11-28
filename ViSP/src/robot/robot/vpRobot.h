/****************************************************************************
 *
 * $Id: vpRobot.h,v 1.7 2007-11-28 11:32:04 fspindle Exp $
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
  enum RobotStateType
    {
      STATE_STOP,
      STATE_VELOCITY_CONTROL,
      STATE_POSITION_CONTROL,
      STATE_ACCELERATION_CONTROL
    }  ;

    /** \brief valeur utilisee par default pour l'etat du robot a
     * la construction. */
  static const vpRobot::RobotStateType
  defaultEtatRobot = vpRobot::STATE_STOP;

  enum ControlFrameType
    {
      REFERENCE_FRAME,
      ARTICULAR_FRAME,
      CAMERA_FRAME,
      MIXT_FRAME
    }  ;

    static const vpRobot::ControlFrameType
    defaultFrameRobot = vpRobot::CAMERA_FRAME ;

private:  /* Membres privees */
    vpRobot::RobotStateType   stateRobot;
    vpRobot::ControlFrameType   frameRobot;
public:
  virtual RobotStateType
  setRobotState (const vpRobot::RobotStateType newState);
  virtual
  RobotStateType     getRobotState (void) { return stateRobot ; }

  ControlFrameType   setRobotFrame (vpRobot::ControlFrameType newFrame);
  ControlFrameType   getRobotFrame (void) { return frameRobot ; }


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
  virtual void setVelocity(const vpRobot::ControlFrameType frame,
			   const vpColVector &vel) = 0 ;


  void setMaxTranslationVelocity (const double maxVt);
  double getMaxTranslationVelocity (void) const ;
  void setMaxRotationVelocity (const double maxVr);
  double getMaxRotationVelocity (void) const;

  //---------- POSITION -----------------------------

  //! get a displacement (frame as to ve specified)
  virtual void getPosition(const vpRobot::ControlFrameType frame,
			   vpColVector &q)   = 0 ;

  //! get a displacement (frame as to ve specified)
  vpColVector getPosition (const vpRobot::ControlFrameType frame);

  //! set a displacement (frame as to ve specified)
  virtual void setPosition(const vpRobot::ControlFrameType frame,
			   const vpColVector &q)   = 0 ;


  //! get a displacement expressed in the camera frame
  virtual void getCameraDisplacement(vpColVector &v) = 0 ;
  //! get a displacement expressed  in the articular frame
  virtual void getArticularDisplacement(vpColVector  &qdot) = 0 ;
  //! get a displacement (frame as to ve specified)
  virtual void getDisplacement(const vpRobot::ControlFrameType frame,
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
