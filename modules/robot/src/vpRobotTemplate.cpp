/****************************************************************************
 *
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
 * Defines a robot just to show which function you must implement.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#include <visp3/robot/vpRobotException.h>

/*!
  \file vpRobotTemplate.cpp
  Defines a robot just to show which function you must implement.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotTemplate.h>

BEGIN_VISP_NAMESPACE
/*!
  Basic initialization.
 */
void vpRobotTemplate::init()
{
  // If you want to control the robot in Cartesian in a tool frame, set the corresponding transformation in m_eMc
  // that is set to identity by default in the constructor.

  maxRotationVelocity = maxRotationVelocityDefault;
  maxTranslationVelocity = maxTranslationVelocityDefault;

  // Set here the robot degrees of freedom number
  nDof = 6; // If your arm has 6 dof
}

/*!
  Default constructor.
 */
vpRobotTemplate::vpRobotTemplate() : m_eMc() { init(); }

/*!
  Destructor.
 */
vpRobotTemplate::~vpRobotTemplate() { std::cout << "Not implemented ! " << std::endl; }

/*

  At least one of these function has to be implemented to control the robot with a
  Cartesian velocity:
  - get_eJe()
  - get_fJe()

*/

/*!
  Get the robot Jacobian expressed in the end-effector frame.

  \param[out] eJe_ : End-effector frame Jacobian.
*/
void vpRobotTemplate::get_eJe(vpMatrix &eJe_)
{
  (void)eJe_;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get the robot Jacobian expressed in the robot reference frame.

  \param[out] fJe_ : Base (or reference) frame Jacobian.
*/
void vpRobotTemplate::get_fJe(vpMatrix &fJe_)
{
  (void)fJe_;
  std::cout << "Not implemented ! " << std::endl;
}

/*

  At least one of these function has to be implemented to control the robot:
  - setCartVelocity()
  - setJointVelocity()

*/

/*!
  Send to the controller a 6-dim velocity twist vector expressed in a Cartesian frame.

  \param[in] frame : Cartesian control frame (either tool frame or end-effector) in which the velocity \e v is
  expressed. Units are m/s for translation and rad/s for rotation velocities.

  \param[in] v : 6-dim vector that contains the 6 components of the velocity twist to send to the robot.
  Units are m/s and rad/s.
*/
void vpRobotTemplate::setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  if (v.size() != 6) {
    throw(vpException(vpException::fatalError,
                      "Cannot send a velocity twist vector in tool frame that is not 6-dim (%d)", v.size()));
  }

  vpColVector v_e; // This is the velocity that the robot is able to apply in the end-effector frame
  switch (frame) {
  case vpRobot::TOOL_FRAME: {
    // We have to transform the requested velocity in the end-effector frame.
    // Knowing that the constant transformation between the tool frame and the end-effector frame obtained
    // by extrinsic calibration is set in m_eMc we can compute the velocity twist matrix eVc that transform
    // a velocity twist from tool (or camera) frame into end-effector frame
    vpVelocityTwistMatrix eVc(m_eMc);
    v_e = eVc * v;
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::REFERENCE_FRAME: {
    v_e = v;
    break;
  }
  case vpRobot::JOINT_STATE:
  case vpRobot::MIXT_FRAME:
    // Out of the scope
    break;
  }

  // Implement your stuff here to send the end-effector velocity twist v_e
  // - If the SDK allows to send cartesian velocities in the end-effector, it's done. Just wrap data in v_e
  // - If the SDK allows to send cartesian velocities in the reference (or base) frame you have to implement
  //   the robot Jacobian in set_fJe() and call:
  //   vpColVector v = get_fJe().inverse() * v_e;
  //   At this point you have to wrap data in v that is the 6-dim velocity to apply to the robot
  // - If the SDK allows to send only joint velocities you have to implement the robot Jacobian in set_eJe()
  //   and call:
  //   vpColVector qdot = get_eJe().inverse() * v_e;
  //   setJointVelocity(qdot);
  // - If the SDK allows to send only a cartesian position trajectory of the end-effector position in the base frame
  //   called fMe (for fix frame to end-effector homogeneous transformation) you can transform the cartesian
  //   velocity in the end-effector into a displacement eMe using the exponetial map:
  //   double delta_t = 0.010; // in sec
  //   vpHomogenesousMatrix eMe = vpExponentialMap::direct(v_e, delta_t);
  //   vpHomogenesousMatrix fMe = getPosition(vpRobot::REFERENCE_FRAME);
  //   the new position to reach is than given by fMe * eMe
  //   vpColVector fpe(vpPoseVector(fMe * eMe));
  //   setPosition(vpRobot::REFERENCE_FRAME, fpe);

  std::cout << "Not implemented ! " << std::endl;
  std::cout << "To implement me you need : " << std::endl;
  std::cout << "\t to known the robot jacobian expressed in ";
  std::cout << "the end-effector frame (eJe) " << std::endl;
  std::cout << "\t the frame transformation  between tool or camera frame ";
  std::cout << "and end-effector frame (cMe)" << std::endl;
}

/*!
  Send a joint velocity to the controller.
  \param[in] qdot : Joint velocities vector. Units are rad/s for a robot arm.
 */
void vpRobotTemplate::setJointVelocity(const vpColVector &qdot)
{
  (void)qdot;

  // Implement your stuff here to send the joint velocities qdot

  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Send to the controller a velocity in a given frame.

  \param[in] frame : Control frame in which the velocity \e vel is expressed.
  Velocities could be joint velocities, or cartesian velocities. Units are m/s for translation and
  rad/s for rotation velocities.

  \param[in] vel : Vector that contains the velocity to apply to the robot.
 */
void vpRobotTemplate::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot. "
                           "Call setRobotState(vpRobot::STATE_VELOCITY_CONTROL) once before "
                           "entering your control loop.");
  }

  vpColVector vel_sat(6);

  // Velocity saturation
  switch (frame) {
  // Saturation in cartesian space
  case vpRobot::TOOL_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    if (vel.size() != 6) {
      throw(vpException(vpException::dimensionError,
                        "Cannot apply a Cartesian velocity that is not a 6-dim vector (%d)", vel.size()));
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    setCartVelocity(frame, vel_sat);
    break;
  }
  // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    if (vel.size() != static_cast<size_t>(nDof)) {
      throw(vpException(vpException::dimensionError, "Cannot apply a joint velocity that is not a %-dim vector (%d)",
                        nDof, vel.size()));
    }
    vpColVector vel_max(vel.size());

    // Since the robot has only rotation axis all the joint max velocities are set to getMaxRotationVelocity()
    vel_max = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    setJointVelocity(vel_sat);
  }
  }
}

/*

  THESE FUNCTIONS ARE NOT MANDATORY BUT ARE USUALLY USEFUL

*/

/*!
  Get robot joint positions.

  \param[in] q : Joint velocities in rad/s.
 */
void vpRobotTemplate::getJointPosition(vpColVector &q)
{
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get robot position.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Position of the arm.
 */
void vpRobotTemplate::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  if (frame == JOINT_STATE) {
    getJointPosition(q);
  }
  else {
    std::cout << "Not implemented ! " << std::endl;
  }
}

/*!
  Set a position to reach.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[in] q : Position to reach.
 */
void vpRobotTemplate::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  (void)frame;
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Displacement in meter and rad.
 */
void vpRobotTemplate::getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  (void)frame;
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}
END_VISP_NAMESPACE
