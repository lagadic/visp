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
 * Interface for the Franka robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/core/vpIoTools.h>

#include "vpJointPosTrajGenerator_impl.h"
#include "vpJointVelTrajGenerator_impl.h"

/*!

  Default constructor.

*/
vpRobotFranka::vpRobotFranka()
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadIsRunning(false),
    m_controlThreadStopAsked(false), m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex(), m_dq_des(), m_eMc(), m_log_folder()
{
  init();
}

/*!
 * Establishes a connection with the robot.
 * \param[in] franka_address IP/hostname of the robot.
 * \param[in] realtime_config If set to kEnforce, an exception will be thrown if realtime priority cannot
 * be set when required. Setting realtime_config to kIgnore disables this behavior.
 */
vpRobotFranka::vpRobotFranka(const std::string &franka_address, franka::RealtimeConfig realtime_config)
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadIsRunning(false),
    m_controlThreadStopAsked(false), m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex(), m_dq_des(), m_v_cart_des(), m_eMc(),m_log_folder()
{
  init();
  connect(franka_address, realtime_config);
}

/*!
 * Initialize internal vars, such as min, max joint positions, max velocity and acceleration.
 */
void vpRobotFranka::init()
{
  nDof = 7;

  m_q_min   = std::array<double, 7> {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  m_q_max   = std::array<double, 7> {12.8973, 1.7628, 2.8973, 0.0175, 2.8973, 3.7525, 2.8973};
  m_dq_max  = std::array<double, 7> {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
  m_ddq_max = std::array<double, 7> {14.25, 7.125, 11.875, 11.875, 14.25, 19.0, 19.0};
}

/*!

  Destructor.

*/
vpRobotFranka::~vpRobotFranka()
{
  setRobotState(vpRobot::STATE_STOP);

  if (m_handler)
    delete m_handler;
}

/*!
 * Establishes a connection with the robot and set default behavior.
 * \param[in] franka_address IP/hostname of the robot.
 * \param[in] realtime_config If set to kEnforce, an exception will be thrown if realtime priority cannot
 * be set when required. Setting realtime_config to kIgnore disables this behavior.
 */
void vpRobotFranka::connect(const std::string &franka_address, franka::RealtimeConfig realtime_config)
{
  init();
  if(franka_address.empty()) {
    throw(vpException(vpException::fatalError, "Cannot connect Franka robot: IP/hostname is not set"));
  }
  if (m_handler)
    delete m_handler;

  m_handler = new franka::Robot(franka_address, realtime_config);

  std::array<double, 7> lower_torque_thresholds_nominal{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
  std::array<double, 7> upper_torque_thresholds_nominal{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 7> lower_torque_thresholds_acceleration{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
  std::array<double, 7> upper_torque_thresholds_acceleration{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  m_handler->setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

  m_handler->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  m_handler->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
//  m_handler->setFilters(100, 100, 100, 100, 100);
  m_handler->setFilters(10, 10, 10, 10, 10);
}

/*!
 * Get robot position.
 * \param[in] frame : Type of position to retrieve. Admissible values are:
 * - vpRobot::JOINT_STATE to get the 7 joint positions.
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian position of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian position of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] position : Robot position. When joint position is asked this vector is 7-dim. Otherwise for a cartesian
 * position this vector is 6-dim. Its content is similar to a vpPoseVector, with first the 3 tranlations in meter
 * and then the 3 orientations in radian as a \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 *
 * If you want to get a cartesian position, use rather
 * getPosition(const vpRobot::vpControlFrameType, vpPoseVector &)
 */
void vpRobotFranka::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();
  vpColVector q(nDof);
  for (int i=0; i < nDof; i++)
    q[i] = robot_state.q_d[i];

  switch(frame) {
  case JOINT_STATE: {
    position = q;
    break;
  }
  case END_EFFECTOR_FRAME: {
    position.resize(6);
    vpHomogeneousMatrix fMe = get_fMe(q);
    vpPoseVector fPe(fMe);
    for (size_t i=0; i < 6; i++) {
      position[i] = fPe[i];
    }
    break;
  }
  case TOOL_FRAME: { // same a CAMERA_FRAME
    position.resize(6);
    vpHomogeneousMatrix fMc = get_fMc(q);
    vpPoseVector fPc(fMc);
    for (size_t i=0; i < 6; i++) {
      position[i] = fPc[i];
    }
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
    break;
  }
  }
}

/*!
 * Given the joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * \param[in] q : Joint position as a 7-dim vector.
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotFranka::get_fMe(const vpColVector &q)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }
  if (q.size() != (unsigned int)nDof) {
    throw(vpException(vpException::fatalError, "Joint position is not a %d-dim vector", q.size()));
  }

  franka::Model model = m_handler->loadModel(); // TODO see if this function cost time

  std::array< double, 7 > q_array;
  for (size_t i = 0; i < (size_t)nDof; i++)
    q_array[i] = q[i];

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 16> pose_array = model.pose(franka::Frame::kEndEffector, q_array, robot_state.F_T_EE, robot_state.EE_T_K);
  vpHomogeneousMatrix fMe;
  for (unsigned int i=0; i< 4; i++) {
    for (unsigned int j=0; j< 4; j++) {
      fMe[i][j] = pose_array[j*4 + i];
    }
  }

  return fMe;
}

/*!
 * Given the joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_c\f$ that gives the position of the camera frame (or in general of
 * any tool attached to the robot) in the robot base frame.
 *
 * By default, the transformation \f$^{e}{\bf M}_c\f$ that corresponds to the transformation between
 * the end-effector and the camera (or tool) frame is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * To change the position of the camera (or tool) frame , use set_eMc().
 *
 * \param[in] q : Joint position as a 7-dim vector.
 * \return Position of the camera frame (or tool frame) in the robot base frame.
 */
vpHomogeneousMatrix vpRobotFranka::get_fMc(const vpColVector &q)
{
  vpHomogeneousMatrix fMe = get_fMe(q);
  return (fMe * m_eMc);
}

/*!
 * Get robot cartesian position.
 * \param[in] frame : Type of cartesian position to retrieve. Admissible values are:
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian position of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian position of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] pose : Robot cartesian position. This vector is 6-dim. Its content is similar to a
 * vpPoseVector, with first the 3 tranlations in meter and then the 3 orientations in radian as a
 *  \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 */
void vpRobotFranka::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 16> pose_array = robot_state.O_T_EE;
  vpHomogeneousMatrix fMe;
  for (unsigned int i=0; i< 4; i++) {
    for (unsigned int j=0; j< 4; j++) {
      fMe[i][j] = pose_array[j*4 + i];
    }
  }

  switch(frame) {
  case END_EFFECTOR_FRAME: {
    pose.buildFrom(fMe);
    break;
  }
  case TOOL_FRAME: {
    pose.buildFrom(fMe * m_eMc);
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: not implemented"));
    break;
  }
  }
}

/*!
 * Gets the Jacobian represented as a 6x7 matrix in row-major format and computed from the robot current joint position.
 * \param[out] eJe : Body Jacobian expressed in the end-effector frame.
 */
void vpRobotFranka::get_eJe(vpMatrix &eJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  franka::Model model = m_handler->loadModel();

  std::array<double, 42> jacobian = model.bodyJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  eJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      eJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable

}

/*!
 * Gets the Jacobian relative to the base frame represented as a 6x7 matrix in row-major format and computed
 * from the robot current joint position.
 * \param[out] fJe : Zero Jacobian expressed in the base frame.
 */
void vpRobotFranka::get_fJe(vpMatrix &fJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  franka::Model model = m_handler->loadModel(); // TODO see if this function cost time

  std::array<double, 42> jacobian = model.zeroJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  fJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      fJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable
}

/*!
 * Set the folder or directory used to record logs at 1Kz when setVelocity() is used.
 * By default the log folder is empty.
 *
 * When the log folder is empty, logs are not created.
 *
 * \param[in] folder : A path to a folder that will contain a basket of log files. If the folder doesn't exist
 * it will be created recursively.
 */
void vpRobotFranka::setLogFolder(const std::string &folder)
{
  if (!folder.empty()) {
    if (vpIoTools::checkDirectory(folder) == false) {
      try {
        vpIoTools::makeDirectory(folder);
        m_log_folder = folder;
      }
      catch(const vpException &e) {
        std::string error;
        error = "Unable to create Franka log folder: " + folder;
        throw(vpException(vpException::fatalError, error));
      }
    }
    else {
       m_log_folder = folder;
    }
  }
}

/*!
 * Set robot position. This function is blocking; it returns when the desired position is reached.
 * \param[in] frame : The only possible value is vpRobot::JOINT_STATE. Other values are not implemented.
 * \param[in] position : This is a 7-dim vector that corresponds to the robot joint positions expressed in rad.
 */
void vpRobotFranka::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot set Franka robot position: robot is not connected"));
  }
  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    std::cout << "Robot was not in position-based control. "
                 "Modification of the robot state" << std::endl;
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  if (frame == vpRobot::JOINT_STATE) {
    double speed_factor = m_positionningVelocity / 100.;

    std::array<double, 7> q_goal;
    for (size_t i = 0; i < 7; i++) {
      q_goal[i] = position[i];
    }

    vpJointPosTrajGenerator joint_pos_traj_generator(speed_factor, q_goal);
    m_handler->control(joint_pos_traj_generator);
  }
  else {
    throw (vpException(vpRobotException::functionNotImplementedError,
        "Cannot move the robot to a cartesian position. Only joint positionning is implemented"));
  }
}

/*!

  Set the maximal velocity percentage to use for a position control.

  \param[in] velocity : Percentage of the maximal velocity. Values should
  be in ]0:100].

*/
void vpRobotFranka::setPositioningVelocity(const double velocity)
{
  m_positionningVelocity = velocity;
}

/*!

  Change the robot state.

  \param[in] newState : New requested robot state.
*/
vpRobot::vpRobotStateType vpRobotFranka::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      // Stop the robot
//      std::cout << "DBG: ask to stop the thread setting m_controlThreadStopAsked = false" << std::endl;
      m_controlThreadStopAsked = true;
      if(m_controlThread.joinable()) {
//        std::cout << "DBG: Stop joint vel thread to stop the robot" << std::endl;
        m_controlThread.join();
//        std::cout << "DBG: control thread joined" << std::endl;
      }
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the robot
//      std::cout << "DBG: ask to stop the thread setting m_controlThreadStopAsked = false" << std::endl;
      m_controlThreadStopAsked = true;
      if(m_controlThread.joinable()) {
//        std::cout << "DBG: Stop joint vel thread to swith to position control" << std::endl;
        m_controlThread.join();
//        std::cout << "DBG: control thread joined" << std::endl;
      }
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
//    std::cout << "DBG: Start joint vel thread" << std::endl;

    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

/*!
  Apply a velocity to the robot.

  \param[in] frame : Control frame in which the velocity is expressed. Velocities
  could be expressed as joint velocities, cartesian velocity twist expressed in
  the robot reference frame, in the end-effector frame or in the camera or tool
  frame.

  \param[in] vel : Velocity vector. Translation velocities are expressed
  in m/s while rotation velocities in rad/s. The size of this vector
  is always 6 for a cartsian velocity skew, and 7 for joint velocities.

  - When joint velocities have to be applied, frame should be set to vpRobot::JOINT_STATE,
    and \f$ vel = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
    \dot{q}_5, \dot{q}_6]^t, \dot{q}_7]^T \f$ correspond to joint velocities in rad/s.

  - When cartesian velocities have to be applied in the reference frame (or in a frame
    also called fixed frame in ViSP), frame should be set to vpRobot::REFERENCE_FRAME,
    \f$ vel = [^{f} v_x, ^{f} v_y, ^{f} v_z, ^{f}
    \omega_x, ^{f} \omega_y, ^{f} \omega_z]^T \f$ is a velocity twist vector corresponding
    to the velocity of the origin of the camera frame (or tool frame)
    expressed in the reference frame, with translations velocities \f$ ^{f} v_x,
    ^{f} v_y, ^{f} v_z \f$ in m/s and rotation velocities \f$ ^{f}\omega_x, ^{f}
    \omega_y, ^{f} \omega_z \f$ in rad/s.

  - When cartesian velocities have to be applied in the end-effector frame,
    frame should be set to vpRobot::END_EFFECTOR_FRAME,
    \f$ vel = [^{e} v_x, ^{e} v_y, ^{e} v_z, ^{e} \omega_x, ^{e} \omega_y,
    ^{e} \omega_z]^T \f$ is a velocity twist vector corresponding
    to the velocity of the origin of the end-effector frame
    expressed in the end-effector frame, with translations velocities \f$ ^{e} v_x,
    ^{e} v_y, ^{e} v_z \f$ in m/s and rotation velocities \f$ ^{e}\omega_x, ^{e}
    \omega_y, ^{e} \omega_z \f$ in rad/s.

  - When cartesian velocities have to be applied in the camera frame or more
    generally in a tool frame, frame should be set to vpRobot::CAMERA_FRAME or vpRobot::TOOL_FRAME,
    \f$ vel = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c} \omega_x, ^{c} \omega_y,
    ^{c} \omega_z]^T \f$ is a velocity twist vector corresponding
    to the velocity of the origin of the camera (or tool frame) frame
    expressed in the camera (or tool frame), with translations velocities \f$ ^{c} v_x,
    ^{c} v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
    \omega_y, ^{c} \omega_z \f$ in rad/s.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \warning Velocities could be saturated if one of them exceed the
  maximal autorized speed (see vpRobot::maxTranslationVelocity and
  vpRobot::maxRotationVelocity). To change these values use
  setMaxTranslationVelocity() and setMaxRotationVelocity().
*/
void vpRobotFranka::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot. "
                           "Use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first.");
  }

  switch (frame) {
  // Saturation in joint space
  case JOINT_STATE: {
    if (vel.size() != (unsigned int)nDof) {
      throw vpRobotException(vpRobotException::wrongStateError,
                             "Joint velocity vector (%d) is not of size 7", vel.size());
    }

    vpColVector vel_max(nDof, getMaxRotationVelocity());

    vpColVector vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    for (size_t i = 0; i < m_dq_des.size(); i++) { // TODO create a function to convert
      m_dq_des[i] = vel_sat[i];
    }

    break;
  }

    // Saturation in cartesian space
  case vpRobot::TOOL_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME: {
    if (vel.size() != 6) {
      throw vpRobotException(vpRobotException::wrongStateError,
                             "Cartesian velocity vector (%d) is not of size 6", vel.size());
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    m_v_cart_des = vpRobot::saturateVelocities(vel, vel_max, true);

    break;
  }

  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Velocity controller not supported");
    break;
  }
  }

  if(! m_controlThreadIsRunning) {
    m_controlThreadIsRunning = true;
//    std::cout << "DBG: Start control thread... ++++++++++++++++++++" << std::endl;
    m_controlThread = std::thread(&vpJointVelTrajGenerator::control_thread, vpJointVelTrajGenerator(),
                                  std::ref(m_handler), std::ref(m_controlThreadStopAsked), m_log_folder,
                                  frame, m_eMc, std::ref(m_v_cart_des), std::ref(m_dq_des),
                                  std::cref(m_q_min), std::cref(m_q_max), std::cref(m_dq_max), std::cref(m_ddq_max),
                                  std::ref(m_robot_state), std::ref(m_mutex));
  }
}

franka::RobotState vpRobotFranka::getRobotInternalState()
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot state: robot is not connected"));
  }
  franka::RobotState robot_state;

  if (! m_controlThreadIsRunning) {
//    std::cout << "DBG: get robot state using readOnce()" << std::endl;
    robot_state = m_handler->readOnce();

    std::lock_guard<std::mutex> lock(m_mutex);
    m_robot_state = robot_state;
  }
  else { // robot_state is updated in the velocity control thread
    std::lock_guard<std::mutex> lock(m_mutex);
    robot_state = m_robot_state;
  }

  return robot_state;
}

/*!
  Gets minimal joint values.
  \return A 7-dimension vector that contains the minimal joint values for the 7 dof.
  All the values are expressed in radians.
 */
vpColVector vpRobotFranka::getJointMin() const
{
  vpColVector q_min(m_q_min.size());
  for (size_t i = 0; i < m_q_min.size(); i ++)
    q_min[i] = m_q_min[i];

  return q_min;
}
/*!
  Gets maximum joint values.
  \return A 7-dimension vector that contains the maximum joint values for the 7 dof.
  All the values are expressed in radians.
 */
vpColVector vpRobotFranka::getJointMax() const
{
  vpColVector q_max(m_q_max.size());
  for (size_t i = 0; i < m_q_max.size(); i ++)
    q_max[i] = m_q_max[i];

  return q_max;
}

/*!
 * Return the \f$ ^{e}{\bf M}_c\f$ homogeneous transformation that gives the position
 * of the camera frame (or in general of any tool frame) in the robot end-effector frame.
 *
 * By default, this transformation is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * To change the position of the camera (or tool) frame , use set_eMc().

 */
vpHomogeneousMatrix vpRobotFranka::get_eMc() const
{
  return m_eMc;
}

/*!
 * Set the \f$ ^{e}{\bf M}_c\f$ homogeneous transformation that gives the position
 * of the camera frame (or in general of any tool frame) in the robot end-effector frame.
 *
 * By default, this transformation is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * This transformation has to be set before controlling the robot cartesian velocity in
 * the camera frame or getting the position of the robot in the camera frame.
 *
 * \param[in] eMc : End-effector to camera frame transformation.
 */
void vpRobotFranka::set_eMc(const vpHomogeneousMatrix &eMc)
{
  m_eMc = eMc;
}


#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotFranka.cpp.o) has
// no symbols
void dummy_vpRobotFranka(){};
#endif

