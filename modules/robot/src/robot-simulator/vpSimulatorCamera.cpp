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
 * Defines the simplest robot : a free flying camera.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpSimulatorCamera.cpp
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpSimulatorCamera.h>

/*!
  Default constructor that sets the transformation between
  world frame and camera frame to identity.

*/
vpSimulatorCamera::vpSimulatorCamera() : wMc_() { init(); }

/*!
  Robot initialisation.

  Robot jacobian expressed in the end-effector frame \f$ {^e}{\bf J}_e \f$
  is set to identity (see get_eJe()).

*/
void vpSimulatorCamera::init()
{
  nDof = 6;
  eJe.eye(6, 6);
  eJeAvailable = true;
  fJeAvailable = false;
  areJointLimitsAvailable = false;
  qmin = NULL;
  qmax = NULL;

  setMaxTranslationVelocity(1.);           // vx, vy and vz max set to 1 m/s
  setMaxRotationVelocity(vpMath::rad(90)); // wx, wy and wz max set to 90 deg/s
}

/*!
  Destructor.

*/
vpSimulatorCamera::~vpSimulatorCamera() {}

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation. Here this transformation is equal to
  identity since camera frame and end-effector frame are at the same location.

*/
void vpSimulatorCamera::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpVelocityTwistMatrix cVe_;
  cVe = cVe_;
}

/*!
  Get the robot jacobian expressed in the end-effector frame.
  For that simple robot the Jacobian is the identity.

  \param eJe_ : A 6 by 6 matrix representing the robot jacobian \f$ {^e}{\bf
  J}_e\f$ expressed in the end-effector frame. Yhis matrix is equal to
  identity.
*/
void vpSimulatorCamera::get_eJe(vpMatrix &eJe_) { eJe_ = this->eJe; }

/*!
  Get the camera position in the world frame.

*/
void vpSimulatorCamera::getPosition(vpHomogeneousMatrix &wMc) const { wMc = this->wMc_; }
/*!
  Return the camera position in the world frame.

*/
vpHomogeneousMatrix vpSimulatorCamera::getPosition() const { return (this->wMc_); }

/*
  Get the current position of the camera.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesien frame,
  - joint (articular) coordinates of each axes
  - in a reference or fixed cartesien frame attached to the robot base
  - in a mixt cartesien frame (translation in reference frame, and rotation in
  camera frame)

  \param position : Measured position of the robot:
  - in camera cartesien frame, a 6 dimension vector, set to 0.

  - in articular, a 6 dimension vector corresponding to the articular
  position of each dof, first the 3 translations, then the 3
  articular rotation positions represented by a vpRxyzVector.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector).
*/
void vpSimulatorCamera::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  q.resize(6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    q = 0;
    break;

  case vpRobot::ARTICULAR_FRAME:
  case vpRobot::REFERENCE_FRAME: {
    // Convert wMc_ to a position
    // From fMc extract the pose
    vpRotationMatrix wRc;
    this->wMc_.extract(wRc);
    vpRxyzVector rxyz;
    rxyz.buildFrom(wRc);

    for (unsigned int i = 0; i < 3; i++) {
      q[i] = this->wMc_[i][3]; // translation x,y,z
      q[i + 3] = rxyz[i];      // Euler rotation x,y,z
    }

    break;
  }
  case vpRobot::MIXT_FRAME:
    std::cout << "MIXT_FRAME is not implemented in vpSimulatorCamera::getPosition()" << std::endl;
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    std::cout << "END_EFFECTOR_FRAME is not implemented in vpSimulatorCamera::getPosition()" << std::endl;
    break;
  }
}

/*!
  Send to the controller a velocity.

  \param frame : Control frame type. Only articular (vpRobot::ARTICULAR_FRAME)
  and camera frame (vpRobot::CAMERA_FRAME) are implemented.

  \param v : Velocity twist to apply to the robot.

  - In the camera frame, this velocity is represented by a twist vector of
  dimension 6 \f$ {\bf v} = [v_x v_y v_z w_x w_y w_z]^t \f$ where \f$ v_x,
  v_y, v_z  \f$ are the translation velocities in m/s and \f$ w_x, w_y, w_z
  \f$ the rotation velocities in rad/s applied in the camera frame.

  - In articular, the behavior is the same as in camera frame.

  Internally, the exponential map (vpExponentialMap) is used to update the
  camera position from its velocity after applying the velocity during a
  sampling time. This sampling time can be set using setSamplingTime().

  \sa setSamplingTime()

*/
void vpSimulatorCamera::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
  }

  switch (frame) {
  case vpRobot::ARTICULAR_FRAME:
  case vpRobot::CAMERA_FRAME: {
    vpColVector v_max(6);

    for (unsigned int i = 0; i < 3; i++)
      v_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      v_max[i] = getMaxRotationVelocity();

    vpColVector v_sat = vpRobot::saturateVelocities(v, v_max, true);

    wMc_ = wMc_ * vpExponentialMap::direct(v_sat, delta_t_);
    setRobotFrame(frame);
    break;
  }
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the reference frame:"
                                                              "functionality not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the mixt frame:"
                                                              "functionality not implemented");

    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the end-effector frame:"
                                                              "functionality not implemented");

    break;
  }
}

/*!
  Set the robot position in the world frame.

  \param wMc : Transformation from world frame to camera frame.
*/
void vpSimulatorCamera::setPosition(const vpHomogeneousMatrix &wMc)
{
  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  this->wMc_ = wMc;
}
