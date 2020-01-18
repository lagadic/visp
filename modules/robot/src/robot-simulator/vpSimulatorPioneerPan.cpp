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
 * Pioneer mobile robot equipped with a pan head simulator without display.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpSimulatorPioneerPan.cpp
  \brief class that defines the Pioneer mobile robot simulator equipped
  with a camera able to move in pan.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpSimulatorPioneerPan.h>

/*!
  Constructor.

  Initialise the robot by a call to init().

  Sampling time is set to 40 ms. To change it you should call
  setSamplingTime().

*/
vpSimulatorPioneerPan::vpSimulatorPioneerPan() : wMc_(), wMm_(), xm_(0), ym_(0), theta_(0), q_pan_() { init(); }

/*!
  Robot initialisation.

  Sampling time is set to 40 ms. To change it you should call
  setSamplingTime().

  Robot jacobian expressed in the end-effector frame \f$ {^e}{\bf J}_e \f$
  is set to identity (see get_eJe()).

*/
void vpSimulatorPioneerPan::init()
{
  xm_ = 0;
  ym_ = 0;
  theta_ = 0;
  q_pan_ = 0;

  nDof = 3;
  eJeAvailable = true;
  fJeAvailable = false;
  areJointLimitsAvailable = false;
  qmin = NULL;
  qmax = NULL;

  wMc_ = wMm_ * mMp_ * pMe_ * cMe_.inverse();
}

/*!
  Destructor.

*/
vpSimulatorPioneerPan::~vpSimulatorPioneerPan() {}

/*!
  Get the robot jacobian expressed in the end-effector frame.
  The jacobian expression is given in vpPioneerPan class.

  \param _eJe : A 6 by 3 matrix representing the robot jacobian \f$ {^e}{\bf
  J}_e\f$ expressed in the end-effector frame.
*/
void vpSimulatorPioneerPan::get_eJe(vpMatrix &_eJe) { _eJe = vpUnicycle::get_eJe(); }

/*!
  Send to the controller a velocity.

  \param frame : Control frame type. Only vpRobot::ARTICULAR_FRAME is
  implemented.

  \param v : Velocity vector \f$(v_x, \omega_z, \dot q)\f$ to apply to the
  robot, where \f$v_x\f$ is the linear translational velocity in m/s and
  \f$\omega_z\f$ is the rotational velocity in rad/s arround the vertical axis
  of the mobile base, and, \f$\dot q\f$ is the pan velocity in rad/s of the
  camera.

  Depending on the velocity specified as input, the robot position is updated
  using the sampling time that can be modified using setSamplingTime().

  \sa setSamplingTime()

*/
void vpSimulatorPioneerPan::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  switch (frame) {
  case vpRobot::ARTICULAR_FRAME: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    }

    setRobotFrame(frame);

    // v is a 3 dimension vector that contains vx, wz, qpan
    if (v.size() != 3) {
      vpERROR_TRACE("Bad dimension of the control vector");
      throw vpRobotException(vpRobotException::dimensionError, "Bad dimension of the control vector");
    }

    vpColVector v_max(3);

    v_max[0] = getMaxTranslationVelocity();
    v_max[1] = getMaxRotationVelocity();
    v_max[2] = getMaxRotationVelocity();

    vpColVector v_sat = vpRobot::saturateVelocities(v, v_max, true);

    xm_ += delta_t_ * v_sat[0] * cos(theta_);
    ym_ += delta_t_ * v_sat[0] * sin(theta_);
    theta_ += delta_t_ * v_sat[1];
    q_pan_ += delta_t_ * v_sat[2];

    vpRotationMatrix wRm(0, 0, theta_);
    vpTranslationVector wtm(xm_, ym_, 0);
    wMm_.buildFrom(wtm, wRm);

    // Update the end effector pose
    set_pMe(q_pan_);

    // Update the camera pose
    wMc_ = wMm_ * mMp_ * pMe_ * cMe_.inverse();

    // Update the jacobian
    set_eJe(q_pan_);

    break;
  }
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the camera frame:"
                                                              "functionality not implemented");
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the reference frame:"
                                                              "functionality not implemented");
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the mixt frame:"
                                                              "functionality not implemented");
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot set a velocity in the end-effector frame:"
                                                              "functionality not implemented");
  }
}

/*!
  Get the robot position in the world frame.

*/
void vpSimulatorPioneerPan::getPosition(vpHomogeneousMatrix &wMc) const { wMc = this->wMc_; }

/*
  Get the current position of the robot.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesien frame,
  - joint (articular) coordinates of each axes (not implemented)
  - in a reference or fixed cartesien frame attached to the robot base
  - in a mixt cartesien frame (translation in reference frame, and rotation in
  camera frame)

  \param position : Measured position of the robot:
  - in camera cartesien frame, a 6 dimension vector, set to 0.

  - in articular, this functionality is not implemented.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector).
*/
void vpSimulatorPioneerPan::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  q.resize(6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    q = 0;
    break;

  case vpRobot::ARTICULAR_FRAME:
    std::cout << "ARTICULAR_FRAME is not implemented in "
                 "vpSimulatorPioneer::getPosition()"
              << std::endl;
    break;
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
