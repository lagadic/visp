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
 * Interface for Universal Robots.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_UR_RTDE)

#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotUniversalRobots.h>

BEGIN_VISP_NAMESPACE
/*!
 * Default constructor.
 * - set eMc transformation to eye()
 * - set default positioning velocity to 20% of the max joint speed
 * - set max joint speed to 180 deg/s
 * - set max joint acceleration to 800 deg/s^2
 */
  vpRobotUniversalRobots::vpRobotUniversalRobots() : m_rtde_receive(), m_rtde_control(), m_db_client(), m_eMc()
{
  init();
}

/*!
 * Destructor that shut down the connexion with the robot.
 */
vpRobotUniversalRobots::~vpRobotUniversalRobots() { setRobotState(vpRobot::STATE_STOP); }

/*!
 * Establishes a connection with the robot and
 * - set eMc transformation to eye()
 * - set default positioning velocity to 20% of the max joint speed
 * - set max joint speed to 180 deg/s
 * - set max linear speed to 0.5 m/s
 * \param[in] ur_address IP/hostname of the robot.
 */
vpRobotUniversalRobots::vpRobotUniversalRobots(const std::string &ur_address)
  : m_rtde_receive(nullptr), m_rtde_control(nullptr), m_eMc()
{
  init();
  connect(ur_address);
}

/*!
 * Establishes a connection with the robot and set default behavior.
 * \param[in] ur_address IP/hostname of the robot.
 *
 * \exception vpException::fatalError : When connexion cannot be established.
 */
void vpRobotUniversalRobots::connect(const std::string &ur_address)
{
  if (!m_rtde_receive) {
    m_rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(ur_address);
  }
  if (!m_rtde_control) {
    m_rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(ur_address);
  }
  if (!m_db_client) {
    m_db_client = std::make_shared<ur_rtde::DashboardClient>(ur_address);
  }
  if (!m_rtde_receive->isConnected()) {
    m_rtde_receive->reconnect();
    if (!m_rtde_receive->isConnected()) {
      throw(vpException(vpException::fatalError, "Cannot connect UR robot to receive interface"));
    }
  }
  if (!m_rtde_control->isConnected()) {
    m_rtde_control->reconnect();
    if (!m_rtde_control->isConnected()) {
      throw(vpException(vpException::fatalError, "Cannot connect UR robot to control interface"));
    }
  }
  if (!m_db_client->isConnected()) {
    m_db_client->connect();
    if (!m_db_client->isConnected()) {
      throw(vpException(vpException::fatalError, "Cannot connect UR robot to dashboard client"));
    }
  }
}

/*!
 * Disconnect the robot interfaces.
 */
void vpRobotUniversalRobots::disconnect()
{
  if (m_rtde_receive && m_rtde_receive->isConnected()) {
    m_rtde_receive->disconnect();
  }
  if (m_rtde_control && m_rtde_control->isConnected()) {
    m_rtde_control->disconnect();
  }
  if (m_db_client && m_db_client->isConnected()) {
    m_db_client->disconnect();
  }
}

/*!
 * Initialize internal vars.
 */
void vpRobotUniversalRobots::init()
{
  nDof = 6;
  m_positioningVelocity = 20.;
  m_max_joint_speed = vpMath::rad(180.);        // deg/s
  m_max_joint_acceleration = vpMath::rad(800.); // deg/s^2
  m_max_linear_speed = 0.5;                     // m/s
  m_max_linear_acceleration = 2.5;              // m/s^2
  m_vel_control_frame = vpRobot::JOINT_STATE;
}

/*!
 * Given the current joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * As described
 * [here](https://docs.pickit3d.com/en/2.4/faq/robot-programming/how-to-define-the-tcp-on-a-universal-robots.html) the
 * end-effector position could be modified setting the Tool Center Point (TCP). When TCP translations and rotations are
 * set to 0, the end-effector corresponds to the robot flange position.
 *
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotUniversalRobots::get_fMe()
{
  vpPoseVector fPe;
  getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
  return vpHomogeneousMatrix(fPe);
}

/*!
 * Given a joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * As described
 * [here](https://docs.pickit3d.com/en/2.4/faq/robot-programming/how-to-define-the-tcp-on-a-universal-robots.html) the
 * end-effector position could be modified setting the Tool Center Point (TCP). When TCP translations and rotations are
 * set to 0, the end-effector corresponds to the robot flange position.
 *
 * \param[in] q : Joint position as a 6-dim vector
 *
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotUniversalRobots::get_fMe(const vpColVector &q)
{
  if (!m_rtde_control || !m_rtde_receive) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot forward kinematics: robot is not connected"));
  }
  if (q.size() != 6) {
    throw(vpException(vpException::fatalError,
                      "Cannot get UR robot forward kinematics: joint position vector is not 6-dim (%d)", q.size()));
  }
  // Robot modes:
  // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
  if (m_rtde_receive->getRobotMode() != 7) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot forward kinematics: brakes are not released"));
  }

  const std::vector<double> q_std = q.toStdVector();
  std::vector<double> tcp_pose = m_rtde_control->getForwardKinematics(q_std);

  vpPoseVector f_P_e;
  for (size_t i = 0; i < 6; i++) {
    f_P_e[i] = tcp_pose[i];
  }
  vpHomogeneousMatrix fMe(f_P_e);
  return fMe;
}

/*!
 * Given the current joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_c\f$ that gives the position of the camera frame (or in general of
 * any tool attached to the robot) in the robot base frame.
 *
 * By default, the transformation \f$^{e}{\bf M}_c\f$ that corresponds to the transformation between
 * the end-effector and the camera (or tool) frame is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * To change the position of the camera (or tool) frame, use set_eMc().
 *
 * \return Position of the camera frame (or tool frame) in the robot base frame.
 */
vpHomogeneousMatrix vpRobotUniversalRobots::get_fMc()
{
  vpPoseVector fPc;
  getPosition(vpRobot::CAMERA_FRAME, fPc);
  return vpHomogeneousMatrix(fPc);
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
void vpRobotUniversalRobots::set_eMc(const vpHomogeneousMatrix &eMc) { m_eMc = eMc; }

/*!
 * Return the \f$ ^{e}{\bf M}_c\f$ homogeneous transformation that gives the position
 * of the camera frame (or in general of any tool frame) in the robot end-effector frame.
 *
 * By default, this transformation is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * To change the position of the camera (or tool) frame on the end-effector frame, use set_eMc().
 *
 */
vpHomogeneousMatrix vpRobotUniversalRobots::get_eMc() const { return m_eMc; }

/*!
 * Get robot force torque.
 * \param[in] frame : Type of forces and torques to retrieve. Admissible values are:
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the external wrench (force, torque) acting on stiffness frame, expressed
 * relative to the stiffness frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
 * - vpRobot::CAMERA_FRAME or more generally a tool frame vpRobot::TOOL_FRAME to retrieve the external wrench (force,
 * torque) applied on the tool frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
 * \param[out] force : Measured forced and torques.
 */
void vpRobotUniversalRobots::getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force)
{
  if (!m_rtde_receive) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot force/torque: robot is not connected"));
  }

  std::vector<double> eFe = m_rtde_receive->getActualTCPForce();

  switch (frame) {
  case JOINT_STATE: {
    throw(vpException(vpException::fatalError, "Cannot get UR force/torque in joint space"));
    break;
  }
  case END_EFFECTOR_FRAME: {
    force = eFe;
    break;
  }
  case TOOL_FRAME: {
    // Transform in tool frame
    vpHomogeneousMatrix cMe = get_eMc().inverse();
    vpForceTwistMatrix cWe(cMe);
    force = cWe * eFe;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get UR force/torque: frame not supported"));
  }
  }
}

/*!
 * Return PolyScope version.
 */
std::string vpRobotUniversalRobots::getPolyScopeVersion()
{
  if (!m_db_client) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot PolyScope verson: robot is not connected"));
  }
  return m_db_client->polyscopeVersion();
}

/*!
 * Get robot position.
 * \param[in] frame : Type of position to retrieve. Admissible values are:
 * - vpRobot::JOINT_STATE to get the 6 joint positions.
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian position of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian position of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] position : Robot position. When joint position is asked this vector is 6-dim. Otherwise for a cartesian
 * position this vector is also 6-dim. Its content is similar to a vpPoseVector, with first the 3 tranlations in meter
 * and then the 3 orientations in radian as a \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 *
 * If you want to get a cartesian position, use rather
 * getPosition(const vpRobot::vpControlFrameType, vpPoseVector &)
 */
void vpRobotUniversalRobots::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  if (!m_rtde_receive) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot position: robot is not connected"));
  }
  // Robot modes:
  // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
  if (m_rtde_receive->getRobotMode() < 4) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot position: robot is not powered on"));
  }

  switch (frame) {
  case JOINT_STATE: {
    position = m_rtde_receive->getActualQ();
    break;
  }
  case END_EFFECTOR_FRAME: {
    std::vector<double> tcp_pose = m_rtde_receive->getActualTCPPose();
    position.resize(6);
    for (size_t i = 0; i < tcp_pose.size(); i++) {
      position[i] = tcp_pose[i];
    }

    break;
  }
  case TOOL_FRAME: { // same a CAMERA_FRAME
    std::vector<double> tcp_pose = m_rtde_receive->getActualTCPPose();
    vpPoseVector fPe;
    for (size_t i = 0; i < tcp_pose.size(); i++) {
      fPe[i] = tcp_pose[i];
    }

    vpHomogeneousMatrix fMe(fPe);
    vpHomogeneousMatrix fMc = fMe * m_eMc;
    vpPoseVector fPc(fMc);
    position.resize(6);
    for (size_t i = 0; i < 6; i++) {
      position[i] = fPc[i];
    }
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get UR cartesian position: wrong method"));
  }
  }
}

/*!
 * Get robot cartesian position.
 * \param[in] frame : Type of cartesian position to retrieve. Admissible values are:
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian position of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian position of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] pose : Robot cartesian position. This vector is 6-dim with first the 3 tranlations in meter and then the
 * 3 orientations in radian as a \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 */
void vpRobotUniversalRobots::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose)
{
  if (!m_rtde_receive) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot position: robot is not connected"));
  }
  if (frame == JOINT_STATE) {
    throw(vpException(vpException::fatalError, "Cannot get UR joint position as a pose vector"));
  }

  switch (frame) {
  case END_EFFECTOR_FRAME:
  case TOOL_FRAME: {
    vpColVector position;
    getPosition(frame, position);
    for (size_t i = 0; i < 6; i++) {
      pose[i] = position[i];
    }
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: not implemented"));
  }
  }
}

/*!
 * Get and return robot model as a string like "UR5", "UR10"...
 */
std::string vpRobotUniversalRobots::getRobotModel() const
{
  if (!m_db_client) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot model: robot is not connected"));
  }
  return m_db_client->getRobotModel();
}
/*!
 * Get and return robot mode. Available robot modes are described
 * [here](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv).
 */
int vpRobotUniversalRobots::getRobotMode() const
{
  if (!m_rtde_receive) {
    throw(vpException(vpException::fatalError, "Cannot get UR robot mode: robot is not connected"));
  }
  // Robot modes:
  // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
  return (m_rtde_receive->getRobotMode());
}

/*!
 * Set the maximal velocity percentage to use for a position control.
 *
 * \param[in] velocity : Percentage of the maximal velocity. Values should be in ]0:100].
 */
void vpRobotUniversalRobots::setPositioningVelocity(double velocity) { m_positioningVelocity = velocity; }

/*!
 * Set robot cartesian position. This function is blocking; it returns when the desired position is reached.
 * \param[in] pose : A 6-dim vector vector corresponding to the position to reach. All the positions are expressed in
 * meters for the translations and radians for the rotations.
 *
 * \param[in] frame : Frame in which the position is expressed.
 * - In the camera frame (or the tool frame which is the same), the 3 first vector values correspond to the
 * translation between the robot base and the camera (or tool frame), while the 3 last vector values to the ThetaU
 * rotations represented by a vpThetaUVector.
 * - In the end-effector frame (or TCP frame), the 3 first vector values correspond to the translation between the
 * robot base and the end-effector, while the 3 last vector values to the ThetaU rotations represented by a
 * vpThetaUVector.
 */
void vpRobotUniversalRobots::setPosition(const vpRobot::vpControlFrameType frame, const vpPoseVector &pose)
{
  if (frame == vpRobot::JOINT_STATE) {
    throw(vpException(vpException::fatalError, "Cannot set UR robot cartesian position: joint state is specified"));
  }
  vpColVector position(pose);
  setPosition(frame, position);
}

/*!
 * Set robot position. This function is blocking; it returns when the desired position is reached.
 * \param[in] position : A 6-dim vector vector corresponding to the position to reach. All the positions are expressed
 * in meters for the translations and radians for the rotations.
 *
 * \param[in] frame : Frame in which the position is expressed.
 * - In the joint space, positions are the six joint positions.
 * - In the camera frame (or the tool frame which is the same), the 3 first vector values correspond to the
 * translation between the robot base and the camera (or tool frame), while the 3 last vector values to the ThetaU
 * rotations represented by a vpThetaUVector.
 * - In the end-effector frame (or TCP frame), the 3 first vector values correspond to the translation between the
 * robot base and the end-effector, while the 3 last vector values to the ThetaU rotations represented by a
 * vpThetaUVector.
 */
void vpRobotUniversalRobots::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
  if (!m_rtde_control) {
    throw(vpException(vpException::fatalError, "Cannot set UR robot position: robot is not connected"));
  }

  // Robot modes:
  // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde20RTDEReceiveInterface12getRobotModeEv
  if (m_rtde_receive->getRobotMode() != 7) {
    throw(vpException(vpException::fatalError, "Cannot set UR robot position: brakes are not released"));
  }

  if (position.size() != 6) {
    throw(vpException(vpException::fatalError,
                      "Cannot set UR robot position: position vector is not a 6-dim vector (%d)", position.size()));
  }

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    std::cout << "Robot is not in position-based control. "
      "Modification of the robot state"
      << std::endl;
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  if (frame == vpRobot::JOINT_STATE) {
    double speed_factor = m_positioningVelocity / 100.;
    std::vector<double> new_q = position.toStdVector();
    m_rtde_control->moveJ(new_q, m_max_joint_speed * speed_factor);
  }
  else if (frame == vpRobot::END_EFFECTOR_FRAME) {
    double speed_factor = m_positioningVelocity / 100.;
    std::vector<double> new_pose = position.toStdVector();
    // Move synchronously to ensure a the blocking behaviour
    m_rtde_control->moveL(new_pose, m_max_linear_speed * speed_factor);
  }
  else if (frame == vpRobot::CAMERA_FRAME) {
    double speed_factor = m_positioningVelocity / 100.;

    vpTranslationVector f_t_c(position.extract(0, 3));
    vpThetaUVector f_tu_c(position.extract(3, 3));
    vpHomogeneousMatrix fMc(f_t_c, f_tu_c);
    vpHomogeneousMatrix fMe = fMc * m_eMc.inverse();
    vpPoseVector fPe(fMe);
    std::vector<double> new_pose = fPe.toStdVector();
    // Move synchronously to ensure a the blocking behaviour
    m_rtde_control->moveL(new_pose, m_max_linear_speed * speed_factor);
  }
  else {
    throw(vpException(vpRobotException::functionNotImplementedError,
                      "Cannot move the robot to a cartesian position. Only joint positioning is implemented"));
  }
}

/*!
 * Apply a velocity to the robot.
 *
 * \param[in] frame : Control frame in which the velocity is expressed. Velocities
 * could be expressed in joint state, robot base frame, end-effector frame or camera frame.
 *
 * \param[in] vel : Velocity vector. Translation velocities are expressed
 * in m/s while rotation velocities in rad/s. The size of this vector
 * is always 6.
 *
 * - In joint state, \f$ vel = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
 * \dot{q}_5, \dot{q}_6]^t \f$ correspond to joint velocities in rad/s.
 *
 * - When cartesian velocities have to be applied in the reference frame corresponding to
 * the robot base frame, `frame` should be set to vpRobot::REFERENCE_FRAME,
 * \f$ vel = [^{f} v_x, ^{f} v_y, ^{f} v_z, ^{f}
 * \omega_x, ^{f} \omega_y, ^{f} \omega_z]^T \f$ is a velocity twist vector corresponding
 * to the velocity of the origin of the end-effector frame
 * expressed in the reference frame, with translations velocities \f$ ^{f} v_x,
 * ^{f} v_y, ^{f} v_z \f$ in m/s and rotation velocities \f$ ^{f}\omega_x, ^{f}
 * \omega_y, ^{f} \omega_z \f$ in rad/s.
 *
 * - When cartesian velocities have to be applied in the end-effector frame,
 * frame should be set to vpRobot::END_EFFECTOR_FRAME,
 * \f$ vel = [^{e} v_x, ^{e} v_y, ^{e} v_z, ^{e} \omega_x, ^{e} \omega_y,
 * ^{e} \omega_z]^T \f$ is a velocity twist vector corresponding
 * to the velocity of the origin of the end-effector frame
 * expressed in the end-effector frame, with translations velocities \f$ ^{e} v_x,
 * ^{e} v_y, ^{e} v_z \f$ in m/s and rotation velocities \f$ ^{e}\omega_x, ^{e}
 * \omega_y, ^{e} \omega_z \f$ in rad/s.
 *
 * - When cartesian velocities have to be applied in the camera frame or more
 * generally in a tool frame, frame should be set to vpRobot::CAMERA_FRAME or
 * vpRobot::TOOL_FRAME (which are equivalent in ViSP),
 * \f$ vel = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c} \omega_x, ^{c} \omega_y,
 * ^{c} \omega_z]^T \f$ is a velocity twist vector corresponding
 * to the velocity of the origin of the camera (or tool frame) frame
 * expressed in the camera (or tool frame), with translations velocities \f$ ^{c} v_x,
 * ^{c} v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
 * \omega_y, ^{c} \omega_z \f$ in rad/s.
 * The position of the camera frame could be set using set_eMc().
 *
 * \exception vpRobotException::wrongStateError : If a the robot is not
 * configured to handle a velocity. The robot can handle a velocity only if the
 * velocity control mode is set. For that, call setRobotState(
 * vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().
 *
 * \warning Velocities could be saturated if one of them exceed the
 * maximal authorized speed (see vpRobot::maxTranslationVelocity and
 * vpRobot::maxRotationVelocity). To change these values use
 * setMaxTranslationVelocity() and setMaxRotationVelocity().
 *
 * \code
 * #include <visp3/core/vpColVector.h>
 * #include <visp3/core/vpMath.h>
 * #include <visp3/robot/vpRobotUniversalRobots.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_UR_RTDE)
 *   vpRobotUniversalRobots robot;
 *
 *   vpColVector qd(6);
 *   // Set a joint velocity
 *   qd[0] = 0.1;             // Joint 1 velocity in rad/s
 *   qd[1] = vpMath::rad(15); // Joint 2 velocity in rad/s
 *   qd[2] = 0;               // Joint 3 velocity in rad/s
 *   qd[3] = M_PI/8;          // Joint 4 velocity in rad/s
 *   qd[4] = 0;               // Joint 5 velocity in rad/s
 *   qd[5] = 0;               // Joint 6 velocity in rad/s
 *
 *   // Initialize the controller to velocity control
 *   robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
 *
 *   while (1) {
 *     // Apply a velocity in the joint space
 *     robot.setVelocity(vpRobot::JOINT_STATE, qvel);
 *
 *     // Compute new velocities qvel...
 *   }
 *
 *   // Stop the robot
 *   robot.setRobotState(vpRobot::STATE_STOP);
 * #endif
 * }
 * \endcode
 */
void vpRobotUniversalRobots::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot: robot is not in velocity control state "
                           "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  vpColVector vel_sat(6);
  m_vel_control_frame = frame;

  // Velocity saturation
  switch (frame) {
  // saturation in cartesian space
  case vpRobot::CAMERA_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    break;
  }
  // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    vpColVector vel_max(6);
    vel_max = getMaxRotationVelocity();
    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
  }
  }

  if (frame == vpRobot::JOINT_STATE) {
    double dt = 1.0 / 1000; // 2ms
    double acceleration = 0.5;
    m_rtde_control->speedJ(vel_sat.toStdVector(), acceleration, dt);
  }
  else if (frame == vpRobot::REFERENCE_FRAME) {
    double dt = 1.0 / 1000; // 2ms
    double acceleration = 0.25;
    m_rtde_control->speedL(vel_sat.toStdVector(), acceleration, dt);
  }
  else if (frame == vpRobot::END_EFFECTOR_FRAME) {
    double dt = 1.0 / 1000; // 2ms
    double acceleration = 0.25;
    vpVelocityTwistMatrix fVe(get_fMe(), false);
    m_rtde_control->speedL((fVe * vel_sat).toStdVector(), acceleration, dt);
  }
  else if (frame == vpRobot::CAMERA_FRAME) {
    double dt = 1.0 / 1000; // 2ms
    double acceleration = 0.25;
    vpColVector w_v_e = vpVelocityTwistMatrix(get_fMe(), false) * vpVelocityTwistMatrix(m_eMc) * vel_sat;
    m_rtde_control->speedL(w_v_e.toStdVector(), acceleration, dt);
  }
  else {
    throw(vpException(vpRobotException::functionNotImplementedError,
                      "Cannot move the robot in velocity in the specified frame: not implemented"));
  }
}

/*!
 *
 * Stop the robot when it is controlled in velocity and set the robot state to vpRobot::STATE_STOP.
 *
 * \exception vpRobotException::lowLevelError : If the low level
 * controller returns an error during robot stopping.
 */
void vpRobotUniversalRobots::stopMotion()
{
  if (!m_rtde_control) {
    throw(vpException(vpException::fatalError, "Cannot stop UR robot: robot is not connected"));
  }

  setRobotState(vpRobot::STATE_STOP);
}

/*!
 * Moves the robot to the joint position specified in the filename. The positioning velocity is set to 10% of the
 * robot maximal velocity.
 *
 * \param[in] filename : File containing a joint position to reach.
 * \param[in] velocity_percentage : Velocity percentage. Values in range [1, 100].
 *
 */
void vpRobotUniversalRobots::move(const std::string &filename, double velocity_percentage)
{
  vpColVector q;

  readPosFile(filename, q);
  setRobotState(vpRobot::STATE_POSITION_CONTROL);
  setPositioningVelocity(velocity_percentage);
  setPosition(vpRobot::JOINT_STATE, q);
}

/*!
 * Read joint positions in a specific Franka position file.
 *
 * This position file has to start with a header. The seven joint positions are given after the "R:" keyword and are
 * expressed in degres to be more representative for the user. Theses values are then converted in radians in \e q.
 * The character "#" starting a line indicates a comment.
 *
 * A typical content of such a file is given below:
 *
 * \code
#UR - Joint position file
#
# R: q1 q2 q3 q4 q5 q6
# with joint positions q1 to q6 expressed in degrees
#
R: 0.1 0.3 -0.25 -80.5 80 0
 * \endcode
 *
 * \param[in] filename : Name of the position file to read.
 *
 * \param[out] q : 6-dim joint positions: q1 q2 q3 q4 q5 q6 with values expressed in radians.
 *
 * \return true if the positions were successfully read from the file, false otherwise.
 *
 * The code below shows how to read a position from a file and move the robot to this position.
 * \code
vpRobotUniversalRobots robot;
vpColVector q;        // Joint position
robot.readPosFile("myposition.pos", q); // Set the joint position from the file
robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

robot.setPositioningVelocity(5); // Positioning velocity set to 5%
robot.setPosition(vpRobot::JOINT_STATE, q); // Move to the joint position
  \endcode
 *
 * \sa savePosFile(), move()
 */
bool vpRobotUniversalRobots::readPosFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#UR - Joint position file");
  bool pos_found = false;
  int lineNum = 0;
  size_t njoints = static_cast<size_t>(nDof);

  q.resize(njoints);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Afma6 position file
        std::cout << "Error: this position file " << filename << " is not for Universal Robots" << std::endl;
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
    std::cout << "Error: unable to find a position for UR robot in " << filename << std::endl;
    return false;
  }

  return true;
}

/*!
 * Save joint positions in a specific Panda position file.
 *
 * This position file starts with a header on the first line. After convertion of the rotations in degrees, the joint
 * position \e q is written on a line starting with the keyword "R: ". See readPosFile() documentation for an example
 * of such a file.
 *
 * \param[in] filename : Name of the position file to create.
 *
 * \param[in] q : Joint positions vector to save in the filename with values expressed in radians.
 *
 * \warning The joint rotations written in the file are converted in degrees to be more representative for the user.
 *
 * \return true if the positions were successfully saved in the file. false, if an error occurs.
 *
 * \sa readPosFile()
 */
bool vpRobotUniversalRobots::savePosFile(const std::string &filename, const vpColVector &q)
{

  FILE *fd;
  fd = fopen(filename.c_str(), "w");
  if (fd == nullptr)
    return false;

  fprintf(fd, "#UR - Joint position file\n"
              "#\n"
              "# R: q1 q2 q3 q4 q5 q6\n"
              "# with joint positions q1 to q6 expressed in degrees\n"
              "#\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf\n", vpMath::deg(q[0]), vpMath::deg(q[1]), vpMath::deg(q[2]),
          vpMath::deg(q[3]), vpMath::deg(q[4]), vpMath::deg(q[5]));

  fclose(fd);
  return (true);
}

/*!
 * Change the robot state.
 *
 * \param[in] newState : New requested robot state.
 */
vpRobot::vpRobotStateType vpRobotUniversalRobots::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      if (!m_rtde_control) {
        throw(vpException(vpException::fatalError, "Cannot stop UR robot: robot is not connected"));
      }
      m_rtde_control->speedStop();
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      if (!m_rtde_control) {
        throw(vpException(vpException::fatalError, "Cannot stop UR robot: robot is not connected"));
      }
      m_rtde_control->speedStop();
    }
    else {
   // std::cout << "Change the control mode from stop to position control" << std::endl;
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
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotUniversalRobots.cpp.o) has no symbols
void dummy_vpRobotUniversalRobots() { };
#endif
