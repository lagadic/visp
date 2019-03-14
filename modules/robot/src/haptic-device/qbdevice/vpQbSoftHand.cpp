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
 * Interface for the qb robotics devices.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_QBDEVICE

#include <regex>

#include <visp3/robot/vpQbSoftHand.h>

/*!
 * Default constructor that does nothing.
 * To connect to a device call init().
 */
vpQbSoftHand::vpQbSoftHand()
  : vpQbDevice()
{
}

/**
 * Close all the still open serial ports.
 * \sa close()
 */
vpQbSoftHand::~vpQbSoftHand()
{
}

/**
 * Retrieve the motor currents of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param[out] current The one-element device motor current vector, expressed in \em mA.
 */
void vpQbSoftHand::getCurrent(vpColVector &current, const int &id)
{
  if (! m_init_done) {
    init(id);
  }

  current.resize(1);
  if (!isInConnectedSet(id)) {
    throw(vpException(vpException::fatalError, "Cannot get current, Qb device is not connected"));
  }
  std::lock_guard<std::mutex> serial_lock(*m_serial_protectors.at(m_connected_devices.at(id)));

  std::vector<short int> currents(2);
  int failures = getCurrents(id, m_max_repeats, currents);  // blocks while reading

  if (! isReliable(failures, m_max_repeats)) {
    throw(vpException(vpException::fatalError, "Cannot get current, communication error with Qb device after %d attempts", m_max_repeats));
  }
  current[0] = static_cast<double>(currents[0]);
}

/**
 * Retrieve the motor position of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param[out] position The device one-element position vector, expressed in range [\p 0, \p 1] with 0 corresponding to an opened hand and 1 a closed hand.
 * \sa getMeasurements()
 */
void vpQbSoftHand::getPosition(vpColVector &position, const int &id)
{
  if (! m_init_done) {
    init(id);
  }

  position.resize(1);
  if (!isInConnectedSet(id)) {
    throw(vpException(vpException::fatalError, "Cannot get position, Qb device is not connected"));
  }
  std::lock_guard<std::mutex> serial_lock(*m_serial_protectors.at(m_connected_devices.at(id)));

  std::vector<short int> positions;
  int failures = getPositions(id, m_max_repeats, positions);  // blocks while reading

  position[0] = static_cast<double>(positions[0])/static_cast<double>(m_position_limits[1]);

  if (! isReliable(failures, m_max_repeats)) {
    throw(vpException(vpException::fatalError, "Cannot get position, communication error with Qb device after %d attempts", m_max_repeats));
  }
}

/**
 * Send the reference command to the motors of the device with given id in a non-blocking fashion.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param position The one-element position vector in range [\p 0, \p 1] with 0 corresponding to an opened hand and 1 a closed hand.
 */
void vpQbSoftHand::setPosition(const vpColVector &position, const int &id)
{
  if (! m_init_done) {
    init(id);
  }

  std::vector<short int> commands(2);
  if (position.size() != 1) {
    throw(vpException(vpException::fatalError, "Command vector size %d is not equal to 2", position.size()));
  }

  commands[0] = static_cast<short int>(position[0]*m_position_limits[1]);

  if(commands[0] < m_position_limits[0]) {
    commands[0] = m_position_limits[0];
  }
  else if (commands[0] > m_position_limits[1]) {
    commands[0] = m_position_limits[1];
  }

  if (!isInConnectedSet(id)) {
    throw(vpException(vpException::fatalError, "Cannot set position, Qb device is not connected"));
  }
  std::lock_guard<std::mutex> serial_lock(*m_serial_protectors.at(m_connected_devices.at(id)));

  //int failures = setCommandsAndWait(id, m_max_repeats, commands);  // FS: doesn't work
  int failures = setCommandsAsync(id, commands);

  if (! isReliable(failures, m_max_repeats)) {
    throw(vpException(vpException::fatalError, "Cannot set position, communication error with Qb device after %d attempts", m_max_repeats));
  }
}


/**
 * Send the reference command to the motors of the device with given id in a blocking fashion.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param position The one-element position vector in range [\p 0, \p 1] with 0 corresponding to an opened hand and 1 a closed hand.
 * \param speed_factor The speed factor in range [\p 0.01, \p 1] with 1 corresponding to the fastest mouvement.
 * \param stiffness Stiffness parameter in range [\p 0, \p 1] is the scale factor applied to the maximum allowed current in order to
 * limit the current into the motors. When set at zero, you can send any position command to the motors, but he will not move.
 * When set to 1, it means that the maximum current returned by getCurrentMax() could be reached during positionning.
 */
void vpQbSoftHand::setPosition(const vpColVector &position, double speed_factor, double stiffness, const int &id)
{
  vpColVector q_mes(1), q(1), current;
  getPosition(q_mes, id);
  double current_max = getCurrentMax();

  double max_delta_q = 1; // 0 opened, 1 closed
  double min_delta_t = 2.0; // number of [sec] to open or close with the max velocity
  double precision = 0.05;
  double delta_t = 40;      // [ms]
  double max_slope = max_delta_q / min_delta_t;
  double sign = (position[0] > q_mes[0]) ? 1.0 : -1.0;
  double vel = speed_factor;
  if (vel < 0.01) {
    vel = 0.01;
  }
  else if (vel > 1.) {
    vel = 1.0;
  }
  double current_factor = stiffness;
  if (current_factor < 0.0) {
    current_factor = 0.0;
  }
  else if (current_factor > 1.) {
    current_factor = 1.0;
  }
  double slope = sign * max_slope * vel;

  unsigned int i = 0;
  int current_failures = 0;
  do {
    double t0 = vpTime::measureTimeMs();
    q[0] = q_mes[0] + slope * delta_t/1000.0 * i;
    if(q[0] < m_position_limits[0]) {
      q[0] = m_position_limits[0];
    }
    else if (q[0] > m_position_limits[1]) {
      q[0] = m_position_limits[1];
    }
    setPosition(q, id);
    getCurrent(current, id);
    i ++;

    if (std::fabs(current[0]) > current_factor*current_max) {
      current_failures ++;
    }
    else {
      current_failures = 0;
    }

    vpTime::wait(t0, delta_t);
  } while (! vpMath::equal(q[0], position[0], precision) && ! (current_failures > 1));
}
#endif

