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
 * Interface for the Biclops robot.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_BICLOPS) && defined(VISP_HAVE_THREADS)

#include <cmath> // std::fabs
#include <errno.h>
#include <limits> // numeric_limits
#include <signal.h>
#include <string.h>
#include <mutex>

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/robot/vpRobotException.h>

#include "private/vpRobotBiclopsController_impl.h"

//#define VP_DEBUG         // Activate the debug mode
//#define VP_DEBUG_MODE 12 // Activate debug level up to 12
#include <visp3/core/vpDebug.h>

BEGIN_VISP_NAMESPACE
/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

const double vpRobotBiclops::defaultPositioningVelocity = 10.0;

static std::mutex m_mutex_end_thread;
static std::mutex m_mutex_shm;
static std::mutex m_mutex_measure;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

vpRobotBiclops::vpRobotBiclops()
  : vpBiclops(), vpRobot(), m_control_thread(), m_positioningVelocity(defaultPositioningVelocity),
  m_q_previous()
{
  vpDEBUG_TRACE(12, "Begin default constructor.");

  m_controller = new vpRobotBiclopsController;
  setConfigFile("/usr/share/BiclopsDefault.cfg");
}

vpRobotBiclops::vpRobotBiclops(const std::string &filename)
  : vpBiclops(), vpRobot(), m_control_thread(), m_positioningVelocity(defaultPositioningVelocity),
  m_q_previous()
{
  vpDEBUG_TRACE(12, "Begin default constructor.");

  m_controller = new vpRobotBiclopsController;
  setConfigFile(filename);

  init();
}

vpRobotBiclops::~vpRobotBiclops()
{
  vpDEBUG_TRACE(12, "Start vpRobotBiclops::~vpRobotBiclops()");
  setRobotState(vpRobot::STATE_STOP);

  vpDEBUG_TRACE(12, "Unlock mutex vpEndThread_mutex");
  m_mutex_end_thread.unlock();

  /* wait the end of the control thread */
  vpDEBUG_TRACE(12, "Wait end of control thread");

  if (m_control_thread.joinable()) {
    m_control_thread.join();
  }

  delete m_controller;
  vpDEBUG_TRACE(12, "Stop vpRobotBiclops::~vpRobotBiclops()");
}

void vpRobotBiclops::setConfigFile(const std::string &filename) { m_configfile = filename; }

void vpRobotBiclops::init()
{
  // test if the config file exists
  FILE *fd = fopen(m_configfile.c_str(), "r");
  if (fd == nullptr) {
    vpCERROR << "Cannot open Biclops config file: " << m_configfile << std::endl;
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with biclops");
  }
  fclose(fd);

  // Initialize the controller
  m_controller->init(m_configfile);

  try {
    setRobotState(vpRobot::STATE_STOP);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }

  // Initialize previous articular position to manage getDisplacement()
  m_q_previous.resize(vpBiclops::ndof);
  m_q_previous = 0;

  return;
}

void vpRobotBiclops::vpRobotBiclopsSpeedControlLoop(void *arg)
{
  vpRobotBiclopsController *controller = static_cast<vpRobotBiclopsController *>(arg);

  //   PMDAxisControl *m_panAxis  = controller->getPanAxis();
  //   PMDAxisControl *m_tiltAxis = controller->getTiltAxis();
  vpRobotBiclopsController::shmType shm;

  vpDEBUG_TRACE(10, "Start control loop");
  vpColVector mes_q;
  vpColVector mes_q_dot;
  vpColVector softLimit(vpBiclops::ndof);
  vpColVector q_dot(vpBiclops::ndof);
  bool *new_q_dot = new bool[vpBiclops::ndof];
  bool *change_dir = new bool[vpBiclops::ndof];   // change of direction
  bool *force_halt = new bool[vpBiclops::ndof];   // force an axis to halt
  bool *enable_limit = new bool[vpBiclops::ndof]; // enable soft limit
  vpColVector prev_q_dot(vpBiclops::ndof);        // previous desired speed
  double secure = vpMath::rad(2);                 // add a security angle before joint limit

  // Set the soft limits
  softLimit[0] = vpBiclops::panJointLimit - secure;
  softLimit[1] = vpBiclops::tiltJointLimit - secure;
  vpDEBUG_TRACE(12, "soft limit pan: %f tilt: %f", vpMath::deg(softLimit[0]), vpMath::deg(softLimit[1]));

  // Initialization
  vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
  m_mutex_shm.lock();

  shm = controller->readShm();

  vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
  m_mutex_shm.unlock();

  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    prev_q_dot[i] = shm.q_dot[i];
    new_q_dot[i] = false;
    change_dir[i] = false;
    force_halt[i] = false;
    enable_limit[i] = true;
  }

  // Initialize actual position and velocity
  mes_q = controller->getActualPosition();
  mes_q_dot = controller->getActualVelocity();

  vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
  m_mutex_shm.lock();

  shm = controller->readShm();
  // Updates the shm
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    shm.actual_q[i] = mes_q[i];
    shm.actual_q_dot[i] = mes_q_dot[i];
  }
  // Update current positions
  controller->writeShm(shm);

  vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
  m_mutex_shm.unlock();

  vpDEBUG_TRACE(11, "unlock mutex vpMeasure_mutex");
  m_mutex_measure.unlock(); // A position is available

  while (!controller->isStopRequested()) {

    // Get actual position and velocity
    mes_q = controller->getActualPosition();
    mes_q_dot = controller->getActualVelocity();

    vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
    m_mutex_shm.lock();

    shm = controller->readShm();

    // Updates the shm
    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      shm.actual_q[i] = mes_q[i];
      shm.actual_q_dot[i] = mes_q_dot[i];
    }

    vpDEBUG_TRACE(12, "mes pan: %f tilt: %f", vpMath::deg(mes_q[0]), vpMath::deg(mes_q[1]));
    vpDEBUG_TRACE(13, "mes pan vel: %f tilt vel: %f", vpMath::deg(mes_q_dot[0]), vpMath::deg(mes_q_dot[1]));
    vpDEBUG_TRACE(12, "desired  q_dot : %f %f", vpMath::deg(shm.q_dot[0]), vpMath::deg(shm.q_dot[1]));
    vpDEBUG_TRACE(13, "previous q_dot : %f %f", vpMath::deg(prev_q_dot[0]), vpMath::deg(prev_q_dot[1]));

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      // test if joint limits are reached
      if (mes_q[i] < -softLimit[i]) {
        vpDEBUG_TRACE(12, "Axe %d in low joint limit", i);
        shm.status[i] = vpRobotBiclopsController::STOP;
        shm.jointLimit[i] = true;
      }
      else if (mes_q[i] > softLimit[i]) {
        vpDEBUG_TRACE(12, "Axe %d in hight joint limit", i);
        shm.status[i] = vpRobotBiclopsController::STOP;
        shm.jointLimit[i] = true;
      }
      else {
        shm.status[i] = vpRobotBiclopsController::SPEED;
        shm.jointLimit[i] = false;
      }

      // Test if new a speed is demanded
      // if (shm.q_dot[i] != prev_q_dot[i])
      if (std::fabs(shm.q_dot[i] - prev_q_dot[i]) >
          std::fabs(vpMath::maximum(shm.q_dot[i], prev_q_dot[i])) * std::numeric_limits<double>::epsilon())
        new_q_dot[i] = true;
      else
        new_q_dot[i] = false;

      // Test if desired speed change of sign
      if ((shm.q_dot[i] * prev_q_dot[i]) < 0.)
        change_dir[i] = true;
      else
        change_dir[i] = false;
    }
    vpDEBUG_TRACE(13, "status      : %d %d", shm.status[0], shm.status[1]);
    vpDEBUG_TRACE(13, "joint       : %d %d", shm.jointLimit[0], shm.jointLimit[1]);
    vpDEBUG_TRACE(13, "new q_dot   : %d %d", new_q_dot[0], new_q_dot[1]);
    vpDEBUG_TRACE(13, "new dir     : %d %d", change_dir[0], change_dir[1]);
    vpDEBUG_TRACE(13, "force halt  : %d %d", force_halt[0], force_halt[1]);
    vpDEBUG_TRACE(13, "enable limit: %d %d", enable_limit[0], enable_limit[1]);

    bool updateVelocity = false;
    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      // Test if a new desired speed is to apply
      if (new_q_dot[i]) {
        // A new desired speed is to apply
        if (shm.status[i] == vpRobotBiclopsController::STOP) {
          // Axis in joint limit
          if (change_dir[i] == false) {
            // New desired speed without change of direction
            // We go in the joint limit
            if (enable_limit[i] == true) { // limit detection active
              // We have to stop this axis
              // Test if this axis was stopped before
              if (force_halt[i] == false) {
                q_dot[i] = 0.;
                force_halt[i] = true;  // indicate that it will be stopped
                updateVelocity = true; // We have to send this new speed
              }
            }
            else {
              // We have to apply the desired speed to go away the joint
              // Update the desired speed
              q_dot[i] = shm.q_dot[i];
              shm.status[i] = vpRobotBiclopsController::SPEED;
              force_halt[i] = false;
              updateVelocity = true; // We have to send this new speed
            }
          }
          else {
            // New desired speed and change of direction.
            if (enable_limit[i] == true) { // limit detection active
              // Update the desired speed to go away the joint limit
              q_dot[i] = shm.q_dot[i];
              shm.status[i] = vpRobotBiclopsController::SPEED;
              force_halt[i] = false;
              enable_limit[i] = false; // Disable joint limit detection
              updateVelocity = true;   // We have to send this new speed
            }
            else {
              // We have to stop this axis
              // Test if this axis was stopped before
              if (force_halt[i] == false) {
                q_dot[i] = 0.;
                force_halt[i] = true;   // indicate that it will be stopped
                enable_limit[i] = true; // Joint limit detection must be active
                updateVelocity = true;  // We have to send this new speed
              }
            }
          }
        }
        else {
          // Axis not in joint limit

          // Update the desired speed
          q_dot[i] = shm.q_dot[i];
          shm.status[i] = vpRobotBiclopsController::SPEED;
          enable_limit[i] = true; // Joint limit detection must be active
          updateVelocity = true;  // We have to send this new speed
        }
      }
      else {
        // No change of the desired speed. We have to stop the robot in case
        // of joint limit
        if (shm.status[i] == vpRobotBiclopsController::STOP) { // axis limit
          if (enable_limit[i] == true) {                       // limit detection active

            // Test if this axis was stopped before
            if (force_halt[i] == false) {
              // We have to stop this axis
              q_dot[i] = 0.;
              force_halt[i] = true;  // indicate that it will be stopped
              updateVelocity = true; // We have to send this new speed
            }
          }
        }
        else {
          // No need to stop the robot
          enable_limit[i] = true; // Normal situation, activate limit detection
        }
      }
    }
    // Update the actual positions
    controller->writeShm(shm);

    vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
    m_mutex_shm.unlock();

    if (updateVelocity) {
      vpDEBUG_TRACE(12, "apply q_dot : %f %f", vpMath::deg(q_dot[0]), vpMath::deg(q_dot[1]));

      // Apply the velocity
      controller->setVelocity(q_dot);
    }

    // Update the previous speed for next iteration
    for (unsigned int i = 0; i < vpBiclops::ndof; i++)
      prev_q_dot[i] = shm.q_dot[i];

    // wait 5 ms
    vpTime::wait(5.0);
  }
  controller->stopRequest(false);
  // Stop the robot
  vpDEBUG_TRACE(10, "End of the control thread: stop the robot");
  q_dot = 0;
  controller->setVelocity(q_dot);

  delete[] new_q_dot;
  delete[] change_dir;
  delete[] force_halt;
  delete[] enable_limit;
  vpDEBUG_TRACE(11, "unlock vpEndThread_mutex");
  m_mutex_end_thread.unlock();
}

vpRobot::vpRobotStateType vpRobotBiclops::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    if (vpRobot::STATE_STOP != getRobotState()) {
      stopMotion();
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      vpDEBUG_TRACE(12, "Speed to position control.");
      stopMotion();
    }

    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {

    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      vpDEBUG_TRACE(12, "Lock mutex vpEndThread_mutex");
      m_mutex_end_thread.lock();

      vpDEBUG_TRACE(12, "Create speed control thread");
      m_control_thread = std::thread(&vpRobotBiclops::vpRobotBiclopsSpeedControlLoop, m_controller);
      vpTime::wait(100.0);

      vpDEBUG_TRACE(12, "Speed control thread created");
    }
    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

void vpRobotBiclops::stopMotion(void)
{
  vpColVector q_dot(vpBiclops::ndof);
  q_dot = 0;
  m_controller->setVelocity(q_dot);
  // std::cout << "Request to stop the velocity controller thread...." << std::endl;
  m_controller->stopRequest(true);
}

void vpRobotBiclops::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  cMe = vpBiclops::get_cMe();

  cVe.build(cMe);
}

void vpRobotBiclops::get_cMe(vpHomogeneousMatrix &cMe) const { cMe = vpBiclops::get_cMe(); }

void vpRobotBiclops::get_eJe(vpMatrix &eJe)
{
  vpColVector q(2);
  getPosition(vpRobot::JOINT_STATE, q);

  vpBiclops::get_eJe(q, eJe);
}

void vpRobotBiclops::get_fJe(vpMatrix &fJe)
{
  vpColVector q(2);
  getPosition(vpRobot::JOINT_STATE, q);

  vpBiclops::get_fJe(q, fJe);
}

void vpRobotBiclops::setPositioningVelocity(double velocity)
{
  if (velocity < 0 || velocity > 100) {
    vpERROR_TRACE("Bad positioning velocity");
    throw vpRobotException(vpRobotException::constructionError, "Bad positioning velocity");
  }

  m_positioningVelocity = velocity;
}

double vpRobotBiclops::getPositioningVelocity(void) { return m_positioningVelocity; }

void vpRobotBiclops::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    vpERROR_TRACE("Robot was not in position-based control\n"
                  "Modification of the robot state");
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

  vpDEBUG_TRACE(12, "Lock mutex vpEndThread_mutex");
  m_mutex_end_thread.lock();
  m_controller->setPosition(q, m_positioningVelocity);
  vpDEBUG_TRACE(12, "Unlock mutex vpEndThread_mutex");
  m_mutex_end_thread.unlock();
  return;
}

void vpRobotBiclops::setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2)
{
  try {
    vpColVector q(2);
    q[0] = q1;
    q[1] = q2;

    setPosition(frame, q);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

void vpRobotBiclops::setPosition(const std::string &filename)
{
  vpColVector q;
  if (readPositionFile(filename.c_str(), q) == false) {
    vpERROR_TRACE("Cannot get Biclops position from file");
    throw vpRobotException(vpRobotException::readingParametersError, "Cannot get Biclops position from file");
  }
  setPosition(vpRobot::JOINT_STATE, q);
}

void vpRobotBiclops::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
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

  vpRobot::vpRobotStateType state;
  state = vpRobot::getRobotState();

  switch (state) {
  case STATE_STOP:
  case STATE_POSITION_CONTROL:
    q = m_controller->getPosition();

    break;
  case STATE_VELOCITY_CONTROL:
  case STATE_ACCELERATION_CONTROL:
  default:
    q.resize(vpBiclops::ndof);

    vpDEBUG_TRACE(12, "Lock mutex vpMeasure_mutex");
    m_mutex_measure.lock(); // Wait until a position is available

    vpRobotBiclopsController::shmType shm;

    vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
    m_mutex_shm.lock();

    shm = m_controller->readShm();

    vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
    m_mutex_shm.unlock();

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      q[i] = shm.actual_q[i];
    }

    vpCDEBUG(11) << "++++++++ Measure actuals: " << q.t();

    vpDEBUG_TRACE(12, "unlock mutex vpMeasure_mutex");
    m_mutex_measure.unlock(); // A position is available

    break;
  }
}

void vpRobotBiclops::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
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

  vpDEBUG_TRACE(12, "Velocity limitation.");
  bool norm = false; // Flag to indicate when velocities need to be normalized

  // Saturate joint speed
  double max = vpBiclops::speedLimit;
  vpColVector q_dot_sat(vpBiclops::ndof);

  // init q_dot_saturated
  q_dot_sat = q_dot;

  for (unsigned int i = 0; i < vpBiclops::ndof; ++i) // q1 and q2
  {
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
    max = vpBiclops::speedLimit / max;
    q_dot_sat = q_dot * max;
  }

  vpCDEBUG(12) << "send velocity: " << q_dot_sat.t() << std::endl;

  vpRobotBiclopsController::shmType shm;

  vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
  m_mutex_shm.lock();

  shm = m_controller->readShm();

  for (unsigned int i = 0; i < vpBiclops::ndof; i++)
    shm.q_dot[i] = q_dot[i];

  m_controller->writeShm(shm);

  vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
  m_mutex_shm.unlock();

  return;
}

void vpRobotBiclops::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot)
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

  vpRobot::vpRobotStateType state;
  state = vpRobot::getRobotState();

  switch (state) {
  case STATE_STOP:
  case STATE_POSITION_CONTROL:
    q_dot = m_controller->getVelocity();

    break;
  case STATE_VELOCITY_CONTROL:
  case STATE_ACCELERATION_CONTROL:
  default:
    q_dot.resize(vpBiclops::ndof);

    vpDEBUG_TRACE(12, "Lock mutex vpMeasure_mutex");
    m_mutex_measure.lock(); // Wait until a position is available

    vpRobotBiclopsController::shmType shm;

    vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
    m_mutex_shm.lock();

    shm = m_controller->readShm();

    vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
    m_mutex_shm.unlock();

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      q_dot[i] = shm.actual_q_dot[i];
    }

    vpCDEBUG(11) << "++++++++ Velocity actuals: " << q_dot.t();

    vpDEBUG_TRACE(12, "unlock mutex vpMeasure_mutex");
    m_mutex_measure.unlock(); // A position is available

    break;
  }
}

vpColVector vpRobotBiclops::getVelocity(vpRobot::vpControlFrameType frame)
{
  vpColVector q_dot;
  getVelocity(frame, q_dot);

  return q_dot;
}

bool vpRobotBiclops::readPositionFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#PTU-EVI - Position");
  bool pos_found = false;
  int lineNum = 0;

  q.resize(vpBiclops::ndof);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Biclops position file
        std::cout << "Error: this position file " << filename << " is not for Biclops robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < vpBiclops::ndof + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < vpBiclops::ndof + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < vpBiclops::ndof; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  q.deg2rad();

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Biclops robot in " << filename << std::endl;
    return false;
  }

  return true;
}

void vpRobotBiclops::getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &d)
{
  vpColVector q_current; // current position

  getPosition(vpRobot::JOINT_STATE, q_current);

  switch (frame) {
  case vpRobot::JOINT_STATE:
    d.resize(vpBiclops::ndof);
    d = q_current - m_q_previous;
    break;

  case vpRobot::CAMERA_FRAME: {
    d.resize(6);
    vpHomogeneousMatrix fMc_current;
    vpHomogeneousMatrix fMc_previous;
    fMc_current = vpBiclops::get_fMc(q_current);
    fMc_previous = vpBiclops::get_fMc(m_q_previous);
    vpHomogeneousMatrix c_previousMc_current;
    // fMc_c = fMc_p * c_pMc_c
    // => c_pMc_c = (fMc_p)^-1 * fMc_c
    c_previousMc_current = fMc_previous.inverse() * fMc_current;

    // Compute the instantaneous velocity from this homogeneous matrix.
    d = vpExponentialMap::inverse(c_previousMc_current);
    break;
  }

  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the reference frame:"
                                                              "functionality not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the mixt frame:"
                                                              "functionality not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the end-effector frame:"
                                                              "functionality not implemented");
    break;
  }

  m_q_previous = q_current; // Update for next call of this method
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotBiclops.cpp.o) has no symbols
void dummy_vpRobotBiclops() { };
#endif
