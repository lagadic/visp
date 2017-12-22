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

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>

/*!
  \class vpRobot
  \ingroup group_robot_real_gantry group_robot_real_cylindrical
  group_robot_real_arm \ingroup group_robot_real_ptu group_robot_real_unicycle
  group_robot_real_template \brief Class that defines a generic virtual robot.
*/
class VISP_EXPORT vpRobot
{
public:
  /*!
    Robot control states.
  */
  typedef enum {
    STATE_STOP,                /*!< Stops robot motion especially in velocity and
                       acceleration control. */
    STATE_VELOCITY_CONTROL,    //!< Initialize the velocity controller.
    STATE_POSITION_CONTROL,    //!< Initialize the position controller.
    STATE_ACCELERATION_CONTROL //!< Initialize the acceleration controller.
  } vpRobotStateType;

  /*!
    Robot control frames.
  */
  typedef enum {
    REFERENCE_FRAME, /*!< Corresponds to a fixed reference frame
  attached to the robot structure. */
    ARTICULAR_FRAME, /*!< Corresponds to the joint space. */
    CAMERA_FRAME,    /*!< Corresponds to a frame attached to the
  camera mounted on the robot end-effector. */
    MIXT_FRAME       /*!< Corresponds to a "virtual" frame where
        translations are expressed in the reference frame, and
        rotations in the camera frame.*/
  } vpControlFrameType;

private: /* Membres privees */
  vpRobot::vpRobotStateType stateRobot;
  vpRobot::vpControlFrameType frameRobot;

protected:
  double maxTranslationVelocity;
  static const double maxTranslationVelocityDefault; // = 0.2;
  double maxRotationVelocity;
  static const double maxRotationVelocityDefault; // = 0.7;

  //! number of degrees of freedom
  int nDof;
  //! robot Jacobian expressed in the end-effector frame
  vpMatrix eJe;
  //! is the robot Jacobian expressed in the end-effector frame available
  int eJeAvailable;
  //! robot Jacobian expressed in the robot reference frame available
  vpMatrix fJe;
  //! is the robot Jacobian expressed in the robot reference frame available
  int fJeAvailable;

  int areJointLimitsAvailable;
  double *qmin;
  double *qmax;

  bool verbose_;

public:
  vpRobot(void);
  vpRobot(const vpRobot &robot);
  virtual ~vpRobot();

  /** @name Inherited functionalities from vpRobot */
  //@{

  //---------- Jacobian -----------------------------
  //! Get the robot Jacobian expressed in the end-effector frame
  virtual void get_eJe(vpMatrix &_eJe) = 0;
  //! Get the robot Jacobian expressed in the robot reference (or world)
  //! frame.
  virtual void get_fJe(vpMatrix &_fJe) = 0;

  //! Get a displacement (frame as to ve specified) between two successive
  //! position control.
  virtual void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) = 0;

  double getMaxTranslationVelocity(void) const;
  double getMaxRotationVelocity(void) const;
  //! Get the robot position (frame has to be specified).
  virtual void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) = 0;

  // Return the robot position (frame has to be specified).
  vpColVector getPosition(const vpRobot::vpControlFrameType frame);
  virtual vpRobotStateType getRobotState(void) const { return stateRobot; }

  virtual void init() = 0;

  vpRobot &operator=(const vpRobot &robot);

  void setMaxRotationVelocity(const double maxVr);
  void setMaxTranslationVelocity(const double maxVt);
  //! Set a displacement (frame has to be specified) in position control.
  virtual void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) = 0;
  virtual vpRobotStateType setRobotState(const vpRobot::vpRobotStateType newState);

  //! Set the velocity (frame has to be specified) that will be applied to the
  //! velocity controller.
  virtual void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) = 0;
  inline void setVerbose(bool verbose) { verbose_ = verbose; };

  //@}

  /** @name Static Public Member Functions inherited from vpRobot */
  //@{
  static vpColVector saturateVelocities(const vpColVector &v_in, const vpColVector &v_max, bool verbose = false);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpRobot */
  //@{
  vpControlFrameType setRobotFrame(vpRobot::vpControlFrameType newFrame);
  vpControlFrameType getRobotFrame(void) const { return frameRobot; }
  //@}
};

#endif
