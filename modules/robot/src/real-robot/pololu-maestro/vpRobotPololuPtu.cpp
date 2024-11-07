/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Common features for Pololu Maestro PanTiltUnit.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_POLOLU) && defined(VISP_HAVE_THREADS)

#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotPololuPtu.h>

BEGIN_VISP_NAMESPACE
vpRobotPololuPtu::vpRobotPololuPtu(const std::string &device, int baudrate, bool verbose)
  : m_verbose(verbose)
{
  nDof = 2;
  m_pan.connect(device, baudrate, 0);
  m_pan.setPwmRange(4095, 7905);
  m_pan.setAngularRange(static_cast<float>(vpMath::rad(-45)), static_cast<float>(vpMath::rad(45)));
  m_pan.setVerbose(verbose);

  m_tilt.connect(device, baudrate, 1);
  m_tilt.setPwmRange(4095, 7905);
  m_tilt.setAngularRange(static_cast<float>(vpMath::rad(-45)), static_cast<float>(vpMath::rad(45)));
  m_tilt.setVerbose(verbose);
}

vpRobotPololuPtu::~vpRobotPololuPtu()
{
  m_pan.stopVelocityCmd();
  m_tilt.stopVelocityCmd();
}

void vpRobotPololuPtu::get_eJe(vpMatrix &eJe)
{
  vpColVector q(nDof);
  getPosition(vpRobot::JOINT_STATE, q);

  get_eJe(q, eJe);
}

void vpRobotPololuPtu::get_fJe(vpMatrix &fJe)
{
  vpColVector q(nDof);
  getPosition(vpRobot::JOINT_STATE, q);

  get_fJe(q, fJe);
}

void vpRobotPololuPtu::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{
  eJe.resize(6, nDof);

  if (q.size() != static_cast<unsigned int>(nDof)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Pololu PTU joint position vector"));
  }

  double s2 = sin(q[1]);
  double c2 = cos(q[1]);

  eJe = 0;

  eJe[3][0] = -c2;
  eJe[4][1] = -1;
  eJe[5][0] = s2;
}

void vpRobotPololuPtu::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{
  if (q.size() != static_cast<unsigned int>(nDof)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Pololu PTU joint position vector"));
  }

  fJe.resize(6, nDof);

  double s1 = sin(q[0]);
  double c1 = cos(q[0]);

  fJe = 0;

  fJe[3][1] = s1;
  fJe[4][1] = -c1;
  fJe[5][0] = 1;
}

float vpRobotPololuPtu::getAngularVelocityResolution() const
{
  return m_pan.speedToRadS(1);
}

void vpRobotPololuPtu::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  if (q.size() != static_cast<unsigned int>(nDof)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Pololu PTU joint position vector"));
  }

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    std::cout << "Warning: Robot is not in position-based control. Modification of the robot state" << std::endl;
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in camera frame: "
                                                              "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in reference frame: "
                                                              "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in mixt frame: "
                                                              "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::JOINT_STATE:
    break;
  }

  float pos_vel = m_positioning_velocity_percentage * static_cast<float>(getMaxRotationVelocity());
  m_pan.setAngularPosition(static_cast<float>(q[0]), pos_vel);
  m_tilt.setAngularPosition(static_cast<float>(q[1]), pos_vel);
}

void vpRobotPololuPtu::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot)
{
  if (q_dot.size() != static_cast<unsigned int>(nDof)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Pololu PTU joint position vector"));
  }

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot "
                           "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the camera frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::JOINT_STATE: {
    if (q_dot.getRows() != 2) {
      throw vpRobotException(vpRobotException::wrongStateError, "Bad dimension for speed vector "
                                                                "in joint state");
    }
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the reference frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the mixt frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the end-effector frame:"
                                                              "functionality not implemented");
  }
  default: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot ");
  }
  }

  bool norm = false; // Flag to indicate when velocities need to be normalized

  // Saturate joint speed
  double max = getMaxRotationVelocity();
  vpColVector q_dot_sat(nDof);

  // Init q_dot_saturated
  q_dot_sat = q_dot;

  for (unsigned int i = 0; i < static_cast<unsigned int>(nDof); ++i) { // q1 and q2
    if (fabs(q_dot[i]) > max) {
      norm = true;
      max = fabs(q_dot[i]);
      vpERROR_TRACE("Excess velocity: ROTATION "
                    "(axe nr.%d).",
                    i);
    }
  }
  // Rotations velocities normalization
  if (norm == true) {
    max = getMaxRotationVelocity() / max;
    q_dot_sat = q_dot * max;
  }

  std::cout << "Send velocity: " << q_dot_sat.t() << std::endl;

  m_pan.setAngularVelocity(static_cast<float>(q_dot_sat[0]));
  m_tilt.setAngularVelocity(static_cast<float>(q_dot_sat[1]));
}

vpRobot::vpRobotStateType vpRobotPololuPtu::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    if (vpRobot::STATE_STOP != getRobotState()) {
      stopVelocity();
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Switch from velocity to position control." << std::endl;
      stopVelocity();
    }

    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

void vpRobotPololuPtu::stopVelocity()
{
  m_pan.stopVelocityCmd();
  m_tilt.stopVelocityCmd();
}

void vpRobotPololuPtu::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in camera frame: "
                                                              "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in reference frame: "
                                                              "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in mixt frame: "
                                                              "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::JOINT_STATE:
    break;
  }

  q.resize(nDof);
  q[0] = m_pan.getAngularPosition();
  q[1] = m_tilt.getAngularPosition();
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotPololuPtu.cpp.o) has no symbols
void dummy_vpRobotPololuPtu() { };
#endif
