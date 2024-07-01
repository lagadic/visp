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
 * Interface for Flir Ptu Cpi robot.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIR_PTU_SDK

#include <signal.h>
#include <stdexcept>

extern "C" {
#include <cpi.h>
}

/*!
  \file vpRobotFlirPtu.cpp
  \brief Interface for Flir Ptu Cpi robot.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotFlirPtu.h>

BEGIN_VISP_NAMESPACE
/*!

  Emergency stops the robot if the program is interrupted by a SIGINT
  (CTRL C), SIGSEGV (segmentation fault), SIGBUS (bus error), SIGKILL
  or SIGQUIT signal.

*/
void vpRobotFlirPtu::emergencyStop(int signo)
{
  std::stringstream msg;
  msg << "Stop the FLIR PTU by signal (" << signo << "): " << (char)7;
  switch (signo) {
  case SIGINT:
    msg << "SIGINT (stop by ^C) ";
    break;
  case SIGSEGV:
    msg << "SIGSEGV (stop due to a segmentation fault) ";
    break;
#ifndef _WIN32
  case SIGBUS:
    msg << "SIGBUS (stop due to a bus error) ";
    break;
  case SIGKILL:
    msg << "SIGKILL (stop by CTRL \\) ";
    break;
  case SIGQUIT:
    msg << "SIGQUIT ";
    break;
#endif
  default:
    msg << signo << std::endl;
  }

  throw vpRobotException(vpRobotException::signalException, msg.str());
}

/*!
  Basic initialization.
 */
void vpRobotFlirPtu::init()
{
  // If you want to control the robot in Cartesian in a tool frame, set the corresponding transformation in m_eMc
  // that is set to identity by default in the constructor.

  maxRotationVelocity = maxRotationVelocityDefault;
  maxTranslationVelocity = maxTranslationVelocityDefault;

  // Set here the robot degrees of freedom number
  nDof = 2; // Flir Ptu has 2 dof
}

/*!
  Default constructor.
 */
vpRobotFlirPtu::vpRobotFlirPtu()
  : m_eMc(), m_cer(nullptr), m_status(0), m_pos_max_tics(2), m_pos_min_tics(2), m_vel_max_tics(2), m_res(2),
  m_connected(false), m_njoints(2), m_positioning_velocity(20.)
{
  signal(SIGINT, vpRobotFlirPtu::emergencyStop);
  signal(SIGSEGV, vpRobotFlirPtu::emergencyStop);
#ifndef _WIN32
  signal(SIGBUS, vpRobotFlirPtu::emergencyStop);
  signal(SIGKILL, vpRobotFlirPtu::emergencyStop);
  signal(SIGQUIT, vpRobotFlirPtu::emergencyStop);
#endif

  init();
}

/*!
  Destructor.
 */
vpRobotFlirPtu::~vpRobotFlirPtu()
{
  stopMotion();
  disconnect();
}

/*
  At least one of these function has to be implemented to control the robot with a
  Cartesian velocity:
  - get_eJe()
  - get_fJe()
*/

/*!
  Get the robot Jacobian expressed in the end-effector frame.
  \return End-effector frame Jacobian.

  \sa get_fJe()
*/
vpMatrix vpRobotFlirPtu::get_eJe()
{
  vpMatrix eJe;
  vpColVector q;
  getPosition(vpRobot::JOINT_STATE, q);
  eJe.resize(6, 2);
  eJe = 0;
  eJe[3][0] = -sin(q[1]);
  eJe[4][1] = -1;
  eJe[5][0] = -cos(q[1]);
  return eJe;
}

/*!
  Get the robot Jacobian expressed in the end-effector frame.
  \param[out] eJe : End-effector frame Jacobian.

  \sa get_fJe()
*/
void vpRobotFlirPtu::get_eJe(vpMatrix &eJe) { eJe = get_eJe(); }

/*!
  Get the robot Jacobian expressed in the robot reference frame.
  \return Base (or reference) frame Jacobian.

  \sa get_eJe()
*/
vpMatrix vpRobotFlirPtu::get_fJe()
{
  vpMatrix fJe;
  vpColVector q;
  getPosition(vpRobot::JOINT_STATE, q);
  fJe.resize(6, 2);
  fJe = 0;
  fJe[3][1] = -sin(q[1]);
  fJe[4][1] = -cos(q[1]);
  fJe[5][0] = -1;

  return fJe;
}

/*!
  Get the robot Jacobian expressed in the robot reference frame.
  \param[out] fJe : Base (or reference) frame Jacobian.

  \sa get_fJe()
*/
void vpRobotFlirPtu::get_fJe(vpMatrix &fJe) { fJe = get_fJe(); }

/*!
  Get the robot geometric model corresponding to the homogeneous transformation between base (or reference) frame and
  end-effector frame. \return Homogeneous transformation between base (or reference) frame and end-effector frame.
*/
vpMatrix vpRobotFlirPtu::get_fMe()
{
  vpHomogeneousMatrix fMe;
  vpColVector q;
  getPosition(vpRobot::JOINT_STATE, q);
  double c1 = cos(q[0]);
  double c2 = cos(q[1]);
  double s1 = sin(q[0]);
  double s2 = sin(q[1]);

  fMe[0][0] = c1 * c2;
  fMe[0][1] = s1;
  fMe[0][2] = -c1 * s2;

  fMe[1][0] = -s1 * c2;
  fMe[1][1] = c1;
  fMe[1][2] = s1 * s2;

  fMe[2][0] = s2;
  fMe[2][1] = 0;
  fMe[2][2] = c2;

  return fMe;
}

/*!

  Return the velocity twist transformation matrix from camera frame to end-effector
  frame.  This transformation allows to transform a velocity twist expressed
  in the end-effector frame into the camera frame thanks to the homogeneous matrix
  eMc set using set_eMc().

  \return Velocity twist transformation.

  \sa set_eMc(), get_eMc()

*/
vpVelocityTwistMatrix vpRobotFlirPtu::get_cVe() const
{
  vpVelocityTwistMatrix cVe;
  cVe.build(m_eMc.inverse());

  return cVe;
}

/*
  At least one of these function has to be implemented to control the robot:
  - setCartVelocity()
  - setJointVelocity()
*/

/*!
  Send to the controller a 6-dim velocity skew vector expressed in a Cartesian frame.
  \param[in] frame : Cartesian control frame (either tool frame or end-effector) in which the velocity \e v is
  expressed. Units are m/s for translation and rad/s for rotation velocities. \param[in] v : 6-dim vector that contains
  the 6 components of the velocity skew to send to the robot. Units are m/s and rad/s.
*/
void vpRobotFlirPtu::setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  if (v.size() != 6) {
    throw(vpException(vpException::fatalError,
                      "Cannot send a velocity-skew vector in tool frame that is not 6-dim (%d)", v.size()));
  }

  vpColVector v_e; // This is the velocity that the robot is able to apply in the end-effector frame
  switch (frame) {
  case vpRobot::TOOL_FRAME: {
    // We have to transform the requested velocity in the end-effector frame.
    // Knowing that the constant transformation between the tool frame and the end-effector frame obtained
    // by extrinsic calibration is set in m_eMc we can compute the velocity twist matrix eVc that transform
    // a velocity skew from tool (or camera) frame into end-effector frame
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

  // Implement your stuff here to send the end-effector velocity skew v_e
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
void vpRobotFlirPtu::setJointVelocity(const vpColVector &qdot)
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  std::vector<int> vel_tics(2);

  for (int i = 0; i < 2; i++) {
    vel_tics[i] = rad2tics(i, qdot[i]);
    if (std::fabs(vel_tics[i]) > m_vel_max_tics[i]) {
      disconnect();
      throw(vpException(vpException::fatalError, "Cannot set joint %d velocity %f (deg/s). Out of limits [-%f, %f].", i,
                        vpMath::deg(qdot[i]), -tics2deg(i, m_vel_max_tics[i]), tics2deg(i, m_vel_max_tics[i])));
    }
  }

  if (cpi_ptcmd(m_cer, &m_status, OP_PAN_DESIRED_SPEED_SET, vel_tics[0]) ||
      cpi_ptcmd(m_cer, &m_status, OP_TILT_DESIRED_SPEED_SET, vel_tics[1])) {
    throw(vpException(vpException::fatalError, "Unable to set velocity."));
  }
}

/*!
  Send to the controller a velocity in a given frame.
  \param[in] frame : Control frame in which the velocity \e vel is expressed.
  Velocities could be joint velocities, or cartesian velocities. Units are m/s for translation and
  rad/s for rotation velocities.
  \param[in] vel : Vector that contains the velocity to apply to the robot.
 */
void vpRobotFlirPtu::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
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
      throw(vpException(vpException::dimensionError, "Cannot apply a joint velocity that is not a %d-dim vector (%d)",
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
  \param[in] q : Joint position as a 2-dim vector [pan, tilt] with values in radians.
 */
void vpRobotFlirPtu::getJointPosition(vpColVector &q)
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  std::vector<int> pos_tics(2);

  if (cpi_ptcmd(m_cer, &m_status, OP_PAN_CURRENT_POS_GET, &pos_tics[0])) {
    disconnect();
    throw(vpException(vpException::fatalError, "Unable to query pan position."));
  }
  if (cpi_ptcmd(m_cer, &m_status, OP_TILT_CURRENT_POS_GET, &pos_tics[1])) {
    disconnect();
    throw(vpException(vpException::fatalError, "Unable to query pan position."));
  }

  q.resize(2);
  for (int i = 0; i < 2; i++) {
    q[i] = tics2rad(i, pos_tics[i]);
  }
}

/*!
  Get robot position.
  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Position of the arm.
 */
void vpRobotFlirPtu::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
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
void vpRobotFlirPtu::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  if (frame != vpRobot::JOINT_STATE) {
    std::cout << "FLIR PTU positioning is not implemented in this frame" << std::endl;
    return;
  }

  if (q.size() != 2) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU has only %d joints. Cannot set a position that is %d-dim.",
                      m_njoints, q.size()));
  }
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  double vmin = 0.01, vmax = 100.;
  if (m_positioning_velocity < vmin || m_positioning_velocity > vmax) {
    disconnect();
    throw(
        vpException(vpException::fatalError, "FLIR PTU Positioning velocity %f is not in range [%f, %f]", vmin, vmax));
  }

  std::vector<int> pos_tics(2);

  for (int i = 0; i < 2; i++) {
    pos_tics[i] = rad2tics(i, q[i]);
    if (pos_tics[i] < m_pos_min_tics[i] || pos_tics[i] > m_pos_max_tics[i]) {
      disconnect();
      throw(vpException(vpException::fatalError, "Cannot set joint %d position %f (deg). Out of limits [%f, %f].", i,
                        vpMath::deg(q[i]), tics2deg(i, m_pos_min_tics[i]), tics2deg(i, m_pos_max_tics[i])));
    }
  }

  // Set desired speed wrt max pan/tilt speed
  if (cpi_ptcmd(m_cer, &m_status, OP_PAN_DESIRED_SPEED_SET, (int)(m_vel_max_tics[0] * m_positioning_velocity / 100.)) ||
      cpi_ptcmd(m_cer, &m_status, OP_TILT_DESIRED_SPEED_SET,
                (int)(m_vel_max_tics[1] * m_positioning_velocity / 100.))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Setting FLIR pan/tilt positioning velocity failed"));
  }

  if (cpi_ptcmd(m_cer, &m_status, OP_PAN_DESIRED_POS_SET, pos_tics[0]) ||
      cpi_ptcmd(m_cer, &m_status, OP_TILT_DESIRED_POS_SET, pos_tics[1])) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU failed to go to position %d, %d (deg).", vpMath::deg(q[0]),
                      vpMath::deg(q[1])));
  }

  if (cpi_block_until(m_cer, nullptr, nullptr, OP_PAN_CURRENT_POS_GET, pos_tics[0]) ||
      cpi_block_until(m_cer, nullptr, nullptr, OP_TILT_CURRENT_POS_GET, pos_tics[1])) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU failed to wait until position %d, %d reached (deg)",
                      vpMath::deg(q[0]), vpMath::deg(q[1])));
  }
}

/*!
  Get a displacement.
  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Displacement in meter and rad.
 */
void vpRobotFlirPtu::getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  (void)frame;
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Connect to FLIR PTU.
  \param[in] portname : Connect to serial/socket.
  \param[in] baudrate : Use baud rate (default: 9600).
  \sa disconnect()
 */
void vpRobotFlirPtu::connect(const std::string &portname, int baudrate)
{
  char errstr[128];

  if (m_connected) {
    disconnect();
  }

  if (portname.empty()) {
    disconnect();
    throw(vpException(vpException::fatalError, "Port name is required to connect to FLIR PTU."));
  }

  if ((m_cer = (struct cerial *)malloc(sizeof(struct cerial))) == nullptr) {
    disconnect();
    throw(vpException(vpException::fatalError, "Out of memory during FLIR PTU connection."));
  }

  // Open a port
  if (ceropen(m_cer, portname.c_str(), 0)) {
#if _WIN32
    throw(vpException(vpException::fatalError, "Failed to open %s: %s.", portname.c_str(),
                      cerstrerror(m_cer, errstr, sizeof(errstr))));
#else
    throw(vpException(vpException::fatalError, "Failed to open %s: %s.\nRun `sudo chmod a+rw %s`", portname.c_str(),
                      cerstrerror(m_cer, errstr, sizeof(errstr)), portname.c_str()));
#endif
  }

  // Set baudrate
  // ignore errors since not all devices are serial ports
  cerioctl(m_cer, CERIAL_IOCTL_BAUDRATE_SET, &baudrate);

  // Flush any characters already buffered
  cerioctl(m_cer, CERIAL_IOCTL_FLUSH_INPUT, nullptr);

  // Set two second timeout */
  int timeout = 2000;
  if (cerioctl(m_cer, CERIAL_IOCTL_TIMEOUT_SET, &timeout)) {
    disconnect();
    throw(vpException(vpException::fatalError, "cerial: timeout ioctl not supported."));
  }

  // Sync and lock
  int trial = 0;
  do {
    trial++;
  } while (trial <= 3 && (cpi_resync(m_cer) || cpi_ptcmd(m_cer, &m_status, OP_NOOP)));
  if (trial > 3) {
    disconnect();
    throw(vpException(vpException::fatalError, "Cannot communicate with FLIR PTU."));
  }

  // Immediately execute commands (slave mode should be opt-in)
  int rc;
  if ((rc = cpi_ptcmd(m_cer, &m_status, OP_EXEC_MODE_SET, (cpi_enum)CPI_IMMEDIATE_MODE))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Set Immediate Mode failed: %s", cpi_strerror(rc)));
  }

  m_connected = true;

  getLimits();
}

/*!
  Close connection to PTU.
  \sa connect()
 */
void vpRobotFlirPtu::disconnect()
{
  if (m_cer != nullptr) {
    cerclose(m_cer);
    free(m_cer);
    m_cer = nullptr;
    m_connected = false;
  }
}

/*!
 Read min/max position and speed.
*/
void vpRobotFlirPtu::getLimits()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  int status;

  if ((status = cpi_ptcmd(m_cer, &m_status, OP_PAN_MAX_POSITION, &m_pos_max_tics[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_PAN_MIN_POSITION, &m_pos_min_tics[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_TILT_MAX_POSITION, &m_pos_max_tics[1])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_TILT_MIN_POSITION, &m_pos_min_tics[1])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_PAN_UPPER_SPEED_LIMIT_GET, &m_vel_max_tics[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_TILT_UPPER_SPEED_LIMIT_GET, &m_vel_max_tics[1]))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Failed to get limits (%d) %s.", status, cpi_strerror(status)));
  }

  // Get the ptu resolution so we can convert the angles to ptu positions
  if ((status = cpi_ptcmd(m_cer, &m_status, OP_PAN_RESOLUTION, &m_res[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_TILT_RESOLUTION, &m_res[1]))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Failed to get resolution (%d) %s.", status, cpi_strerror(status)));
  }

  for (size_t i = 0; i < 2; i++) {
    m_res[i] /= 3600.; // Resolutions are in arc-seconds, but we want degrees
  }
}

/*!
  Return pan axis min and max position limits in radians as a 2-dim vector, with first value the pan min position and
  second value, the pan max position.

  \sa getTiltPosLimits(), getPanTiltVelMax(), setPanPosLimits()
*/
vpColVector vpRobotFlirPtu::getPanPosLimits()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }
  vpColVector pan_pos_limits(2);
  pan_pos_limits[0] = tics2rad(0, m_pos_min_tics[0]);
  pan_pos_limits[1] = tics2rad(0, m_pos_max_tics[0]);

  return pan_pos_limits;
}

/*!
  Return tilt axis min and max position limits in radians as a 2-dim vector, with first value the tilt min position and
  second value, the tilt max position.

  \sa getPanPosLimits(), getPanTiltVelMax(), setTiltPosLimits()
 */
vpColVector vpRobotFlirPtu::getTiltPosLimits()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }
  vpColVector tilt_pos_limits(2);
  tilt_pos_limits[0] = tics2rad(0, m_pos_min_tics[1]);
  tilt_pos_limits[1] = tics2rad(0, m_pos_max_tics[1]);

  return tilt_pos_limits;
}

/*!
  Return pan/tilt axis max velocity in rad/s as a 2-dim vector, with first value the pan max velocity and second value,
  the max tilt velocity.

  \sa getPanPosLimits(), getTiltPosLimits()
 */
vpColVector vpRobotFlirPtu::getPanTiltVelMax()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }
  vpColVector vel_max(2);
  for (int i = 0; i < 2; i++) {
    vel_max[i] = tics2rad(i, m_vel_max_tics[i]);
  }
  return vel_max;
}

/*!
   Modify pan position limit.
   \param pan_limits : 2-dim vector that contains pan min and max limits in rad.

   \sa getPanPosLimits()
 */
void vpRobotFlirPtu::setPanPosLimits(const vpColVector &pan_limits)
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }
  if (pan_limits.size() != 2) {
    disconnect();
    throw(vpException(vpException::fatalError, "Cannot set max position that is not a 2-dim vector."));
  }
  std::vector<int> pan_limits_tics(2);
  for (int i = 0; i < 2; i++) {
    pan_limits_tics[i] = rad2tics(i, pan_limits[i]);
  }

  int status;
  if ((status = cpi_ptcmd(m_cer, &m_status, OP_PAN_USER_MIN_POS_SET, pan_limits_tics[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_PAN_USER_MAX_POS_SET, pan_limits_tics[1]))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Failed to set pan position limits (%d) %s.", status,
                      cpi_strerror(status)));
  }
}

/*!
   Modify tilt position limit.
   \param tilt_limits : 2-dim vector that contains tilt min and max limits in rad.

   \sa getPanPosLimits()
 */
void vpRobotFlirPtu::setTiltPosLimits(const vpColVector &tilt_limits)
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }
  if (tilt_limits.size() != 2) {
    disconnect();
    throw(vpException(vpException::fatalError, "Cannot set max position that is not a 2-dim vector."));
  }
  std::vector<int> tilt_limits_tics(2);
  for (int i = 0; i < 2; i++) {
    tilt_limits_tics[i] = rad2tics(i, tilt_limits[i]);
  }

  int status;
  if ((status = cpi_ptcmd(m_cer, &m_status, OP_TILT_USER_MIN_POS_SET, tilt_limits_tics[0])) ||
      (status = cpi_ptcmd(m_cer, &m_status, OP_TILT_USER_MAX_POS_SET, tilt_limits_tics[1]))) {
    disconnect();
    throw(vpException(vpException::fatalError, "Failed to set tilt position limits (%d) %s.", status,
                      cpi_strerror(status)));
  }
}

/*!

  Set the velocity used for a position control.

  \param velocity : Velocity in % of the maximum velocity between [0, 100]. Default value is 20.
*/
void vpRobotFlirPtu::setPositioningVelocity(double velocity) { m_positioning_velocity = velocity; }

/*!

  Change the robot state.

  \param newState : New requested robot state if the robot is connected. If the robot is not connected, we return the
  current state.
*/
vpRobot::vpRobotStateType vpRobotFlirPtu::setRobotState(vpRobot::vpRobotStateType newState)
{
  if (!m_connected) {
    return getRobotState();
  }

  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      stopMotion();

      // Set the PTU to pure velocity mode
      if (cpi_ptcmd(m_cer, &m_status, OP_SPEED_CONTROL_MODE_SET, (cpi_enum)CPI_CONTROL_INDEPENDENT)) {
        throw(vpException(vpException::fatalError, "Unable to set control mode independent."));
      }
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      stopMotion();

      // Set the PTU to pure velocity mode
      if (cpi_ptcmd(m_cer, &m_status, OP_SPEED_CONTROL_MODE_SET, (cpi_enum)CPI_CONTROL_INDEPENDENT)) {
        throw(vpException(vpException::fatalError, "Unable to set control mode independent."));
      }

    }
    else {
   // std::cout << "Change the control mode from stop to position
   // control.\n";
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";

      // Set the PTU to pure velocity mode
      if (cpi_ptcmd(m_cer, &m_status, OP_SPEED_CONTROL_MODE_SET, (cpi_enum)CPI_CONTROL_PURE_VELOCITY)) {
        throw(vpException(vpException::fatalError, "Unable to set velocity control mode."));
      }
    }
    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

/*!
  Reset PTU axis.
 */
void vpRobotFlirPtu::reset()
{
  if (!m_connected) {
    return;
  }

  if (cpi_ptcmd(m_cer, &m_status, OP_RESET, nullptr)) {
    throw(vpException(vpException::fatalError, "Unable to reset PTU."));
  }
}

/*!
  Stop PTU motion in velocity control mode.
 */
void vpRobotFlirPtu::stopMotion()
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    return;
  }

  if (!m_connected) {
    return;
  }

  if (cpi_ptcmd(m_cer, &m_status, OP_HALT, nullptr)) {
    throw(vpException(vpException::fatalError, "Unable to stop PTU."));
  }
}

/*!
   When connected to the PTU by serial, get the PTU network IP address.

   \sa getNetworkGateway()
 */
std::string vpRobotFlirPtu::getNetworkIP()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  char str[64];
  if (cpi_ptcmd(m_cer, &m_status, OP_NET_IP_GET, (int)sizeof(str), nullptr, &str)) {
    throw(vpException(vpException::fatalError, "Unable to get Network IP."));
  }

  return (std::string(str));
}

/*!
   When connected to the PTU by serial, get the PTU network gateway.

   \sa getNetworkIP()
 */
std::string vpRobotFlirPtu::getNetworkGateway()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  char str[64];
  if (cpi_ptcmd(m_cer, &m_status, OP_NET_GATEWAY_GET, (int)sizeof(str), nullptr, &str)) {
    throw(vpException(vpException::fatalError, "Unable to get Network Gateway."));
  }

  return (std::string(str));
}

/*!
   When connected to the PTU, get the PTU network hostname.

   \sa getNetworkIP()
 */
std::string vpRobotFlirPtu::getNetworkHostName()
{
  if (!m_connected) {
    disconnect();
    throw(vpException(vpException::fatalError, "FLIR PTU is not connected."));
  }

  char str[64];
  if (cpi_ptcmd(m_cer, &m_status, OP_NET_HOSTNAME_GET, (int)sizeof(str), nullptr, &str)) {
    throw(vpException(vpException::fatalError, "Unable to get Network hostname."));
  }

  return (std::string(str));
}

/*!
   Converts the angle from radian to robot position units using axis i resolution.
   \param rad : Angle in radian.
   \param i : Axis to consider, with 0 for pan and 1 for tilt axis.
   \returns The position in robot units using axis resolution.

   \sa pos2rad()
 */
int vpRobotFlirPtu::rad2tics(int axis, double rad) { return (static_cast<int>(vpMath::deg(rad) / m_res[axis])); }

/*!
   Converts the angle from robot position units into degrees using axis i resolution.
   \param tics : Position in robot units.
   \param i : Axis, with 0 for pan and 1 for tilt axis.
   \returns The angle in degrees using axis resolution.

   \sa pos2rad()
 */
double vpRobotFlirPtu::tics2deg(int axis, int tics) { return (tics * m_res[axis]); }

/*!
   Converts the angle from robot position units into radian using axis i resolution.
   \param tics : Position in robot units.
   \param i : Axis, with 0 for pan and 1 for tilt axis.
   \returns The angle in radian using axis resolution.

   \sa pos2rad()
 */
double vpRobotFlirPtu::tics2rad(int axis, int tics) { return vpMath::rad(tics2deg(axis, tics)); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotFlirPtu.cpp.o) has
// no symbols
void dummy_vpRobotFlirPtu() { };
#endif
