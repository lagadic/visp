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
 * Interface for the Reflex Takktile 2 hand from Right Hand Robotics.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_TAKKTILE2

#include <reflex_driver2.h>

#include <visp3/core/vpMath.h>
#include <visp3/robot/vpReflexTakktile2.h>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpReflexTakktile2::Impl : public reflex_driver2::ReflexDriver
{
public:
  Impl() { }

  ~Impl() { }
};

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

vpReflexTakktile2::HandInfo::HandInfo()
{
  proximal.resize(NUM_FINGERS);
  distal_approx.resize(NUM_FINGERS);

  pressure.resize(NUM_FINGERS);
  contact.resize(NUM_FINGERS);
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    pressure[i].resize(NUM_SENSORS_PER_FINGER);
    contact[i].resize(NUM_SENSORS_PER_FINGER);
  }

  joint_angle.resize(NUM_DYNAMIXELS);
  raw_angle.resize(NUM_DYNAMIXELS);
  velocity.resize(NUM_DYNAMIXELS);
  load.resize(NUM_DYNAMIXELS);
  voltage.resize(NUM_DYNAMIXELS);
  temperature.resize(NUM_DYNAMIXELS);
  error_state.resize(NUM_DYNAMIXELS);
}

/*!
  Prints the Reflex Takktile2 hand info contents to a stream.
  \param os : A std::stream.
  \param hand : Hand info data.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpReflexTakktile2::HandInfo &hand)
{
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    os << "Finger " << i + 1 << ": " << std::endl;

    os << "\tProximal: " << hand.proximal[i] << std::endl;
    os << "\tDistal Approx: " << hand.distal_approx[i] << std::endl;

    os << "\tPressures: ";
    for (size_t j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      os << hand.pressure[i][j] << ", ";
    }
    os << std::endl;

    os << "\tContact: ";
    for (size_t j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      os << hand.contact[i][j] << ", ";
    }
    os << std::endl;

    os << "\tJoint Angle: " << hand.joint_angle[i] << " rad" << std::endl;
    os << "\tJoint Angle: " << vpMath::deg(static_cast<double>(hand.joint_angle[i])) << " deg" << std::endl;
    os << "\tVelocity: " << hand.velocity[i] << " rad/s" << std::endl;
    os << "\tVelocity: " << vpMath::deg(static_cast<double>(hand.velocity[i])) << " deg/s" << std::endl;
    os << "\tError State: " << hand.error_state[i] << std::endl;
  }

  os << "Preshape: " << std::endl;
  os << "\tJoint Angle: " << hand.joint_angle[3] << std::endl;
  os << "\tVelocity: " << hand.velocity[3] << std::endl;
  os << "\tError State: " << hand.error_state[3] << std::endl;

  return os;
}

/*!
 * Default constructor.
 */
vpReflexTakktile2::vpReflexTakktile2()
  : m_network_interface(), m_finger_file_name(), m_tactile_file_name(), m_motor_file_name(), m_hand_info(),
  m_impl(new Impl())
{ }

/*!
 * Destructor.
 */
vpReflexTakktile2::~vpReflexTakktile2() { delete m_impl; }

/*!
 * Calibrates the tactile sensors and fingers.
 */
void vpReflexTakktile2::calibrate() { m_impl->calibrate(); }

/*!
 * Puts the dynamixels motors into idle mode.
 */
void vpReflexTakktile2::disableTorque() { m_impl->disable_torque(); }

/*!
 * Returns hand info data.
 */
vpReflexTakktile2::HandInfo vpReflexTakktile2::getHandInfo()
{
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    m_hand_info.proximal[i] = m_impl->hand_info.proximal[i];
    m_hand_info.distal_approx[i] = m_impl->hand_info.distal_approx[i];
    for (size_t j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      m_hand_info.pressure[i][j] = m_impl->hand_info.pressure[i][j];
      m_hand_info.contact[i][j] = m_impl->hand_info.contact[i][j];
    }
  }
  for (size_t i = 0; i < NUM_DYNAMIXELS; i++) {
    m_hand_info.joint_angle[i] = m_impl->hand_info.joint_angle[i];
    m_hand_info.raw_angle[i] = m_impl->hand_info.raw_angle[i];
    m_hand_info.velocity[i] = m_impl->hand_info.velocity[i];
    m_hand_info.load[i] = m_impl->hand_info.load[i];
    m_hand_info.voltage[i] = m_impl->hand_info.voltage[i];
    m_hand_info.temperature[i] = m_impl->hand_info.temperature[i];
    m_hand_info.error_state[i] = m_impl->hand_info.error_state[i];
  }

  return m_hand_info;
}

/*!
 * \return Number of servos.
 */
int vpReflexTakktile2::getNumServos() const { return static_cast<int>(NUM_SERVOS); }

/*!
 * \return Number of fingers.
 */
int vpReflexTakktile2::getNumFingers() const { return static_cast<int>(NUM_FINGERS); }

/*!
 * \return Number of sensors per finger.
 */
int vpReflexTakktile2::getNumSensorsPerFinger() const { return static_cast<int>(NUM_SENSORS_PER_FINGER); }

/*!
 * \return 4-dim vector corresponding to the position in [rad] of finger 1, 2 and 3 and preshape
 * respectively.
 */
vpColVector vpReflexTakktile2::getPosition() const
{
  vpColVector position(NUM_SERVOS);
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    position[i] = static_cast<double>(m_impl->hand_info.joint_angle[i]);
  }
  return position;
}

/*!
 * \return 4-dim vector corresponding to the velocity in [rad/s] of finger 1, 2 and 3 and preshape
 * respectively.
 */
vpColVector vpReflexTakktile2::getVelocity() const
{
  vpColVector velocity(NUM_SERVOS);
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    velocity[i] = static_cast<double>(m_impl->hand_info.velocity[i]);
  }
  return velocity;
}

/*!
 * Set hand position. This function commands the motors from radians, using the zero references from
 * `yaml/finger_calibrate.yaml` file to translate into the raw Dynamixel values.
 *
 * \param targets : 4-dim vector with angular positions in [rad] for finger 1, 2, 3 and preshape respectively.
 *
 * \sa setPositioningVelocity()
 */
void vpReflexTakktile2::setPosition(const vpColVector &targets)
{
  if (targets.size() != NUM_SERVOS) {
    vpException(vpException::dimensionError, "Wrong Takktile 2 position vector dimension (%d) instead of %d.",
                targets.size(), NUM_SERVOS);
  }
  float targets_[NUM_SERVOS];
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    targets_[i] = static_cast<float>(targets[i]);
  }
  m_impl->set_angle_position(targets_);
}

/*!
 * Sets the same tactile threshold level to all tactile sensors for reporting contact.
 * \param threshold : Value between 0 and 1000.
 *
 * \sa setTactileThreshold(std::vector<int> &)
 */
void vpReflexTakktile2::setTactileThreshold(int threshold) { m_impl->populate_tactile_thresholds(threshold); }

/*!
 * Sets the threshold level to each tactile sensor for reporting contact.
 * \param thresholds : This vector is of dim the number of fingers multiplied
 * by the number of tactile sensors per finger. Values are between 0 and 1000.
 *
 * \sa setTactileThreshold(int)
 */
void vpReflexTakktile2::setTactileThreshold(const std::vector<int> &thresholds)
{
  if (thresholds.size() != NUM_FINGERS * NUM_SENSORS_PER_FINGER) {
    vpException(vpException::dimensionError, "Wrong Takktile threshold vector dimension (%d) instead of %d.",
                thresholds.size(), NUM_FINGERS * NUM_SENSORS_PER_FINGER);
  }
  int thresholds_[NUM_FINGERS * NUM_SENSORS_PER_FINGER];
  for (size_t i = 0; i < NUM_FINGERS * NUM_SENSORS_PER_FINGER; i++) {
    thresholds_[i] = thresholds[i];
  }

  m_impl->set_tactile_thresholds(thresholds_);
}

/*!
 * Changes the positioning speed of the motor.
 * \param targets : 4-dim vector with angular velocities in [rad/s] for finger 1, 2, 3 and preshape respectively.
 *
 * \sa setPosition()
 */
void vpReflexTakktile2::setPositioningVelocity(const vpColVector &targets)
{
  if (targets.size() != NUM_SERVOS) {
    vpException(vpException::dimensionError, "Wrong Takktile 2 velocity vector dimension (%d) instead of %d.",
                targets.size(), NUM_SERVOS);
  }
  float targets_[NUM_SERVOS];
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    targets_[i] = static_cast<float>(targets[i]);
  }
  m_impl->set_motor_speed(targets_);
}

/*!
 * Moves the fingers in until any of them makes contact with an object.
 *
 * \param targets : 4-dim vector with angular velocities in [rad/s] for finger 1, 2, 3 and preshape respectively.
 */
void vpReflexTakktile2::setVelocityUntilAnyContact(const vpColVector &targets)
{
  if (targets.size() != NUM_SERVOS) {
    vpException(vpException::dimensionError, "Wrong Takktile 2 velocity vector dimension (%d) instead of %d.",
                targets.size(), NUM_SERVOS);
  }
  float targets_[NUM_SERVOS];
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    targets_[i] = static_cast<float>(targets[i]);
  }
  m_impl->move_until_any_contact(targets_);
}

/*!
 * Moves the fingers in until each finger makes contact with an object.
 *
 * \param targets : 4-dim vector with angular velocities in [rad/s] for finger 1, 2, 3 and preshape respectively.
 */
void vpReflexTakktile2::setVelocityUntilEachContact(const vpColVector &targets)
{
  if (targets.size() != NUM_SERVOS) {
    vpException(vpException::dimensionError, "Wrong Takktile 2 velocity vector dimension (%d) instead of %d.",
                targets.size(), NUM_SERVOS);
  }
  float targets_[NUM_SERVOS];
  for (unsigned int i = 0; i < NUM_SERVOS; i++) {
    targets_[i] = static_cast<float>(targets[i]);
  }
  m_impl->move_until_each_contact(targets_);
}

/*!
 * Open Ethernet connection to Reflex Takktile 2 hand.
 */
void vpReflexTakktile2::open()
{
  m_impl->open(m_network_interface, m_finger_file_name, m_tactile_file_name, m_motor_file_name);
  reflex_hand2::ReflexHandState *state = &m_impl->rh->rx_state_;
  m_impl->rh->setStateCallback(std::bind(&reflex_driver2::ReflexDriver::reflex_hand_state_cb, m_impl, state));
}

/*!
 * Tries to communicate with the Reflex Hand object.
 * \param milliseconds : Duration in [ms].
 */
void vpReflexTakktile2::wait(int milliseconds) { m_impl->wait(milliseconds); }
END_VISP_NAMESPACE
#endif
