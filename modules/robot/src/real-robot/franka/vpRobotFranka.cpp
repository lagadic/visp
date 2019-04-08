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
  : vpRobot(), m_handler(NULL), m_gripper(NULL), m_model(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadIsRunning(false),
    m_controlThreadStopAsked(false), m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex(), m_dq_des(), m_eMc(), m_log_folder(), m_franka_address()
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
  : vpRobot(), m_handler(NULL), m_gripper(NULL), m_model(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadIsRunning(false),
    m_controlThreadStopAsked(false), m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex(), m_dq_des(), m_v_cart_des(), m_eMc(),m_log_folder(), m_franka_address()
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
  m_q_max   = std::array<double, 7> {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  m_dq_max  = std::array<double, 7> {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
  m_ddq_max = std::array<double, 7> {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
}

/*!

  Destructor.

*/
vpRobotFranka::~vpRobotFranka()
{
  setRobotState(vpRobot::STATE_STOP);

  if (m_handler)
    delete m_handler;

  if (m_gripper) {
    std::cout << "Grasped object, will release it now." << std::endl;
    m_gripper->stop();
    delete m_gripper;
  }

  if (m_model) {
    delete m_model;
  }
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

  m_franka_address = franka_address;
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
#if (VISP_HAVE_FRANKA_VERSION < 0x000500)
  //  m_handler->setFilters(100, 100, 100, 100, 100);
  m_handler->setFilters(10, 10, 10, 10, 10);
#else
  // use franka::lowpassFilter() instead throw Franka::robot::control() with cutoff_frequency parameter
#endif
  if (m_model) {
    delete m_model;
  }
  m_model = new franka::Model(m_handler->loadModel());
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
  vpColVector q(7);
  for (int i=0; i < 7; i++)
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
  }
  }
}

/*!
 * Get robot force torque.
 * \param[in] frame : Type of forces and torques to retrieve. Admissible values are:
 * - vpRobot::JOINT_STATE to get the 7-dim measured link-side joint torque sensor signals. Unit: \f$[Nm]\f$.
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness
 *   frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
 * - vpRobot::CAMERA_FRAME or more generally a tool frame vpRobot::TOOL_FRAME to retrieve the external wrench (force, torque) applied on the tool frame.
 *   Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
 * \param[out] force : Measured forced and torques.
 *
 * If you want to get a cartesian position, use rather
 * getPosition(const vpRobot::vpControlFrameType, vpPoseVector &)
 */
void vpRobotFranka::getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  switch(frame) {
  case JOINT_STATE: {
    force.resize(7);
    for (int i=0; i < 7; i++)
      force[i] = robot_state.tau_J[i];

    break;
  }
  case END_EFFECTOR_FRAME: {
    force.resize(6);
    for (int i=0; i < 7; i++)
      force[i] = robot_state.K_F_ext_hat_K[i];
    break;
  }
  case TOOL_FRAME: {
    // end-effector frame
    vpColVector eFe(6);
    for (int i=0; i < 7; i++)
      eFe[i] = robot_state.K_F_ext_hat_K[i];

    // Transform in tool frame
    vpHomogeneousMatrix cMe = get_eMc().inverse();
    vpForceTwistMatrix cWe( cMe  );
    force = cWe * eFe;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
  }
  }
}

/*!
 * Get robot velocity.
 * \param[in] frame : Type of velocity to retrieve. Admissible values are:
 * - vpRobot::JOINT_STATE to get the 7 joint positions.
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian velocity of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian velocity of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] d_position : Robot velocity. When joints velocity is asked this vector is 7-dim. Otherwise for a cartesian
 * velocity this vector is 6-dim. Its content is similar to a vpPoseVector, with first the 3 tranlations in meter
 * and then the 3 orientations in radian as a \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 *
 * \warning For the moment, cartesian velocities measurement in vpRobot::END_EFFECTOR_FRAME, vpRobot::CAMERA_FRAME, vpRobot::TOOL_FRAME
 * are not implemented.
 */
void vpRobotFranka::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_position)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot velocity: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  switch(frame) {

  case JOINT_STATE: {
    d_position.resize(7);
    for (int i=0; i < 7; i++) {
      d_position[i]=robot_state.dq[i];
    }
    break;
  }

  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian velocity: not implemented"));
  }
  }
}

/*!
 * Get the Coriolis force vector (state-space equation) calculated from the current robot state: \f$ c= C \times
 * dq\f$, in \f$[Nm]\f$.
 * \param[out] coriolis : Coriolis force vector.
 */
void vpRobotFranka::getCoriolis(vpColVector &coriolis)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  std::array<double, 7> coriolis_;

  franka::RobotState robot_state = getRobotInternalState();

  coriolis_ = m_model->coriolis(robot_state);

  coriolis.resize(7);
  for (int i=0; i < 7; i++) {
    coriolis[i] = coriolis_[i];
  }
}

/*!
 * Get the gravity vector calculated form the current robot state. Unit: \f$[Nm]\f$.
 * \param[out] gravity : Gravity vector
 */
void vpRobotFranka::getGravity(vpColVector &gravity)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  std::array<double, 7> gravity_;

  franka::RobotState robot_state = getRobotInternalState();

  gravity_ = m_model->gravity(robot_state);

  gravity.resize(7);
  for (int i=0; i < 7; i++) {
    gravity[i] = gravity_[i];
  }
}

/*!
 * Get the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
 * \param[out] mass : 7x7 mass matrix, row-major.
 */
void vpRobotFranka::getMass(vpMatrix &mass)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  std::array<double, 49> mass_;

  franka::RobotState robot_state = getRobotInternalState();

  mass_ = m_model->mass(robot_state); // column-major

  mass.resize(7, 7); // row-major
  for (size_t i = 0; i < 7; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      mass[i][j] = mass_[j*7 + i];
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
  if (q.size() != 7) {
    throw(vpException(vpException::fatalError, "Joint position vector [%u] is not a 7-dim vector", q.size()));
  }

  std::array< double, 7 > q_array;
  for (size_t i = 0; i < 7; i++)
    q_array[i] = q[i];

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 16> pose_array = m_model->pose(franka::Frame::kEndEffector, q_array, robot_state.F_T_EE, robot_state.EE_T_K);
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
  }
  }
}

/*!
 * Gets the robot Jacobian in the end-effector frame relative to the end-effector frame represented as a 6x7 matrix in row-major
 * format and computed from the robot current joint position.
 * \param[out] eJe : Body Jacobian expressed in the end-effector frame.
 */
void vpRobotFranka::get_eJe(vpMatrix &eJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot eJe jacobian: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 42> jacobian = m_model->bodyJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  eJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      eJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable

}

/*!
 * Gets the robot Jacobian in the end-effector frame relative to the end-effector frame represented as a 6x7 matrix in row-major
 * format and computed from the robot current joint position.
 * \param[in] q : 7-dim vector corresponding to the robot joint position [rad].
 * \param[out] eJe : Body Jacobian expressed in the end-effector frame.
 */
void vpRobotFranka::get_eJe(const vpColVector &q, vpMatrix &eJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot eJe jacobian: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  std::array< double, 7 > q_array;
  for (size_t i = 0; i < 7; i++)
    q_array[i] = q[i];

  std::array<double, 42> jacobian = m_model->bodyJacobian(franka::Frame::kEndEffector, q_array, robot_state.F_T_EE, robot_state.EE_T_K); // column-major
  eJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      eJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable

}

/*!
 * Gets the robot Jacobian in the end-effector frame relative to the base frame represented as a 6x7 matrix in row-major format and computed
 * from the robot current joint position.
 * \param[out] fJe : Zero Jacobian expressed in the base frame.
 */
void vpRobotFranka::get_fJe(vpMatrix &fJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot fJe jacobian: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 42> jacobian = m_model->zeroJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  fJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      fJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable
}

/*!
 * Gets the robot Jacobian in the end-effector frame relative to the base frame represented as a 6x7 matrix in row-major format and computed
 * from the robot joint position given as input.
 * \param[in] q : 7-dim vector corresponding to the robot joint position [rad].
 * \param[out] fJe : Zero Jacobian expressed in the base frame.
 */
void vpRobotFranka::get_fJe(const vpColVector &q, vpMatrix &fJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot fJe jacobian: robot is not connected"));
  }
  if (q.size() != 7) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot fJe jacobian with an input joint position vector [%u] that is not a 7-dim vector", q.size()));
  }

  franka::RobotState robot_state = getRobotInternalState();

  std::array< double, 7 > q_array;
  for (size_t i = 0; i < 7; i++)
    q_array[i] = q[i];

  std::array<double, 42> jacobian = m_model->zeroJacobian(franka::Frame::kEndEffector, q_array, robot_state.F_T_EE, robot_state.EE_T_K); // column-major
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

    int nbAttempts = 10;
    for (int attempt = 1; attempt <= nbAttempts; attempt++) {
      try {
        m_handler->control(joint_pos_traj_generator);
        break;
      } catch (const franka::ControlException &e) {
        std::cerr << "Warning: communication error: " << e.what() << "\nRetry attempt: " << attempt << std::endl;
        m_handler->automaticErrorRecovery();
        if (attempt == nbAttempts)
          throw;
      }
    }
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
      m_controlThreadStopAsked = true;
      if(m_controlThread.joinable()) {
        m_controlThread.join();
        m_controlThreadStopAsked = false;
        m_controlThreadIsRunning = false;
      }
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the robot
      m_controlThreadStopAsked = true;
      if(m_controlThread.joinable()) {
        m_controlThread.join();
        m_controlThreadStopAsked = false;
        m_controlThreadIsRunning = false;
      }
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
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
    if (vel.size() != 7) {
      throw vpRobotException(vpRobotException::wrongStateError,
                             "Joint velocity vector (%d) is not of size 7", vel.size());
    }

    vpColVector vel_max(7, getMaxRotationVelocity());

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
  }
  }

  if(! m_controlThreadIsRunning) {
    m_controlThreadIsRunning = true;
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

/*!

  Moves the robot to the joint position specified in the filename. The
  positioning velocity is set to 10% of the robot maximal velocity.

  \param filename : File containing a joint position to reach.
  \param velocity_percentage : Velocity percentage. Values in range [1, 100].

*/
void vpRobotFranka::move(const std::string &filename, double velocity_percentage)
{
  vpColVector q;

  this->readPosFile(filename, q);
  this->setRobotState(vpRobot::STATE_POSITION_CONTROL);
  this->setPositioningVelocity(velocity_percentage);
  this->setPosition(vpRobot::JOINT_STATE, q);
}

/*!

  Read joint positions in a specific Franka position file.

  This position file has to start with a header. The seven joint positions
  are given after the "R:" keyword and are expressed in degres to be more
  representative for the user. Theses values are then converted in
  radians in \e q. The character "#" starting a line indicates a
  comment.

  A typical content of such a file is given below:

  \code
#PANDA - Joint position file
#
# R: q1 q2 q3 q4 q5 q6 q7
# with joint positions q1 to q7 expressed in degrees
#

R: 0.1 0.3 -0.25 -80.5 80 0 0
  \endcode

  \param[in] filename : Name of the position file to read.

  \param[out] q : 7-dim joint positions: q1 q2 q3 q4 q5 q6 q7 with values expressed in radians.

  \return true if the positions were successfully readen in the file. false, if
  an error occurs.

  The code below shows how to read a position from a file and move the robot to
  this position.
  \code
vpRobotFranka robot;
vpColVector q;        // Joint position
robot.readPosFile("myposition.pos", q); // Set the joint position from the file
robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

robot.setPositioningVelocity(5); // Positioning velocity set to 5%
robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
  \endcode

  \sa savePosFile(), move()
*/

bool vpRobotFranka::readPosFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#PANDA - Joint position file");
  bool pos_found = false;
  int lineNum = 0;
  size_t njoints = 7;

  q.resize(njoints);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Afma6 position file
        std::cout << "Error: this position file " << filename << " is not for Afma6 robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < njoints + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < njoints + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < njoints; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  for (unsigned int i = 0; i < njoints; i++) {
    q[i] = vpMath::rad(q[i]);
  }

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Panda robot in " << filename << std::endl;
    return false;
  }

  return true;
}

/*!

  Save joint positions in a specific Panda position file.

  This position file starts with a header on the first line. After
  convertion of the rotations in degrees, the joint position \e q is
  written on a line starting with the keyword "R: ". See readPosFile()
  documentation for an example of such a file.

  \param filename : Name of the position file to create.

  \param q : Joint positions vector to save in the
  filename with values expressed in radians.

  \warning The joint rotations written in the file are converted
  in degrees to be more representative for the user.

  \return true if the positions were successfully saved in the file. false, if
  an error occurs.

  \sa readPosFile()
*/
bool vpRobotFranka::savePosFile(const std::string &filename, const vpColVector &q)
{

  FILE *fd;
  fd = fopen(filename.c_str(), "w");
  if (fd == NULL)
    return false;

  fprintf(fd,
        "#PANDA - Joint position file\n"
        "#\n"
        "# R: q1 q2 q3 q4 q5 q6 q7\n"
        "# with joint positions q1 to q7 expressed in degrees\n"
        "#\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf %lf\n",  vpMath::deg(q[0]), vpMath::deg(q[1]), vpMath::deg(q[2]),
      vpMath::deg(q[3]), vpMath::deg(q[4]), vpMath::deg(q[5]), vpMath::deg(q[6]));

  fclose(fd);
  return (true);
}

/*!

  Stop the robot when it is controlled in velocity and set the robot state to vpRobot::STATE_STOP.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.
*/
void vpRobotFranka::stopMotion()
{
  if (getRobotState() == vpRobot::STATE_VELOCITY_CONTROL) {
    vpColVector q(7, 0);
    setVelocity(vpRobot::JOINT_STATE, q);
  }
  setRobotState(vpRobot::STATE_STOP);
}

/*!

  Performing a gripper homing.

  \sa gripperGrasp(), gripperMove(), gripperRelease()
*/
void vpRobotFranka::gripperHoming()
{
  if (m_franka_address.empty()) {
    throw (vpException(vpException::fatalError, "Cannot perform franka gripper homing without ip address"));
  }
  if (m_gripper == NULL)
    m_gripper = new franka::Gripper(m_franka_address);

  m_gripper->homing();
}

/*!

  Moves the gripper fingers to a specified width.
  @param[in] width : Intended opening width. [m]

  \return EXIT_SUCCESS if the success, EXIT_FAILURE otherwise.

  \sa gripperHoming(), gripperGrasp(), gripperRelease()
*/
int vpRobotFranka::gripperMove(double width)
{
  if (m_franka_address.empty()) {
    throw (vpException(vpException::fatalError, "Cannot open franka gripper without ip address"));
  }
  if (m_gripper == NULL)
    m_gripper = new franka::Gripper(m_franka_address);

  // Check for the maximum grasping width.
  franka::GripperState gripper_state = m_gripper->readOnce();

  if (gripper_state.max_width < width) {
    std::cout << "Finger width request is too large for the current fingers on the gripper."
              << "Maximum possible width is " << gripper_state.max_width << std::endl;
    return EXIT_FAILURE;
  }

  m_gripper->move(width, 0.1);
  return EXIT_SUCCESS;
}

/*!

  Closes the gripper.

  \return EXIT_SUCCESS if the success, EXIT_FAILURE otherwise.

  \sa gripperHoming(), gripperGrasp(), gripperRelease(), gripperOpen()
*/
int vpRobotFranka::gripperClose()
{
  return gripperMove(0);
}

/*!

  Closes the gripper.

  \return EXIT_SUCCESS if the success, EXIT_FAILURE otherwise.

  \sa gripperHoming(), gripperGrasp(), gripperRelease(), gripperOpen()
*/
int vpRobotFranka::gripperOpen()
{
  if (m_franka_address.empty()) {
    throw (vpException(vpException::fatalError, "Cannot open franka gripper without ip address"));
  }
  if (m_gripper == NULL)
    m_gripper = new franka::Gripper(m_franka_address);

  // Check for the maximum grasping width.
  franka::GripperState gripper_state = m_gripper->readOnce();

  m_gripper->move(gripper_state.max_width, 0.1);
  return EXIT_SUCCESS;
}

/*!

  Release an object that is grasped.

  \sa gripperHoming(), gripperMove(), gripperRelease()
*/
void vpRobotFranka::gripperRelease()
{
  if (m_franka_address.empty()) {
    throw (vpException(vpException::fatalError, "Cannot release franka gripper without ip address"));
  }
  if (m_gripper == NULL)
    m_gripper = new franka::Gripper(m_franka_address);

  m_gripper->stop();
}

/*!

  Grasp an object that has a given width.

  An object is considered grasped if the distance \e d between the gripper fingers satisfies
  \e grasping_width - 0.005 < d < \e grasping_width + 0.005.

  \param[in] grasping_width : Size of the object to grasp. [m]
  \param[in] force : Grasping force. [N]

  \return EXIT_SUCCESS if grasping succeed, EXIT_FAILURE otherwise.

  \sa gripperHoming(), gripperOpen(), gripperRelease()
*/
int vpRobotFranka::gripperGrasp(double grasping_width, double force)
{
  if (m_gripper == NULL)
    m_gripper = new franka::Gripper(m_franka_address);

  // Check for the maximum grasping width.
  franka::GripperState gripper_state = m_gripper->readOnce();
  std::cout << "Gripper max witdh: " << gripper_state.max_width << std::endl;
  if (gripper_state.max_width < grasping_width) {
    std::cout << "Object is too large for the current fingers on the gripper."
              << "Maximum possible width is " << gripper_state.max_width << std::endl;
    return EXIT_FAILURE;
  }

  // Grasp the object.
  if (!m_gripper->grasp(grasping_width, 0.1, force)) {
    std::cout << "Failed to grasp object." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotFranka.cpp.o) has
// no symbols
void dummy_vpRobotFranka(){};
#endif

