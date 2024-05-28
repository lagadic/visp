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
 * ATI Force torque interface.
 *
*****************************************************************************/

#include <stdint.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpForceTorqueAtiNetFTSensor.h>

// Make vpForceTorqueAtiNetFTSensor available only if inet_ntop() used to
// communicate by UDP with the sensor through vpUDPClient is available; inet_ntop()
// is not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef struct response_struct
{
  uint32_t rdt_sequence;
  uint32_t ft_sequence;
  uint32_t status;
  int32_t FTData[6];
} RESPONSE;
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_VISP_NAMESPACE
/*!
 * Default constructor that set counts per force to 1000000, counts per torque to 1000000000 and scaling factor to 1.
 * Note that counts per force, counts per torque and scaling factor are used to transform force / torque in user units
 * (N and Nm). These default values could be changed using setCountsPerForce(), setCountsPerTorque() and
 * setScalingFactor().
 */
vpForceTorqueAtiNetFTSensor::vpForceTorqueAtiNetFTSensor()
  : vpUDPClient(), m_counts_per_force(1000000), m_counts_per_torque(1000000000), m_scaling_factor(1), m_ft_bias(6, 0),
  m_data_count(0), m_data_count_prev(0), m_ft(6, 0), m_is_streaming_started(false)
{ }

/*!
 * Constructor that initializes an Eternet UDP connection to a given hostname and port.
 * \param hostname : Device hostname or IP address.
 * \param port : Ethernet port.
 */
vpForceTorqueAtiNetFTSensor::vpForceTorqueAtiNetFTSensor(const std::string &hostname, int port)
  : vpUDPClient(hostname, port), m_counts_per_force(1000000), m_counts_per_torque(1000000000), m_scaling_factor(1),
  m_ft_bias(6, 0), m_data_count(0), m_data_count_prev(0), m_ft(6, 0), m_is_streaming_started(false)
{ }

/*!
 * Start high-speed real-time Net F/T streaming.
 * \return True if streaming was started, false otherwise.
 */
bool vpForceTorqueAtiNetFTSensor::startStreaming()
{
  if (!m_is_init) {
    throw(vpException(vpException::notInitialized, "Cannot start streaming: UDP client is not initialized"));
  }

  if (m_is_streaming_started) {
    throw(vpException(vpException::notInitialized, "Streaming is already started"));
  }

  // Since UDP packet could be lost, retry startup 10 times before giving up
  for (unsigned int i = 0; i < 10; ++i) {
    int len = 8;
    unsigned char request[8];                 // The request data sent to the Net F/T
    *(uint16_t *)&request[0] = htons(0x1234); // Standard header
    *(uint16_t *)&request[2] = htons(0x0002); // Start high-speed streaming (see table 10.1 in Net F/T user manual)
    *(uint32_t *)&request[4] = htonl(0);      // Infinite sample (see section 10.1 in Net F/T user manual

    // Send start stream
    if (send(request, len) != len) {
      throw(vpException(vpException::notInitialized, "UDP client is not initialized"));
    }
    std::cout << "wait: " << i << std::endl;

    m_is_streaming_started = true;
    if (waitForNewData()) {
      return true;
    }
  }
  m_is_streaming_started = false;
  return false;
}

/*!
 * Stop high-speed real-time Net F/T streaming.
 */
void vpForceTorqueAtiNetFTSensor::stopStreaming()
{
  if (!m_is_init) {
    throw(vpException(vpException::notInitialized, "Cannot stop streaming: UDP client is not initialized"));
  }

  if (!m_is_streaming_started) {
    throw(vpException(vpException::notInitialized, "Cannot stop streaming: streaming was not started"));
  }

  int len = 8;
  unsigned char request[8];                 // The request data sent to the Net F/T
  *(uint16_t *)&request[0] = htons(0x1234); // Standard header
  *(uint16_t *)&request[2] = htons(0x0000); // Stop streaming (see table 10.1 in Net F/T user manual)
  *(uint32_t *)&request[4] = htonl(0);      // Infinite sample (see section 10.1 in Net F/T user manual

  // Send start stream
  if (send(request, len) != len) {
    throw(vpException(vpException::notInitialized, "Cannot stop streaming"));
  }

  m_is_streaming_started = false;
}

/*!
 * Destructor that stops Net F/T streaming and closes the Ethernet connection with the device.
 *
 * \sa stopStreaming()
 */
vpForceTorqueAtiNetFTSensor::~vpForceTorqueAtiNetFTSensor()
{
  if (m_is_streaming_started) {
    stopStreaming();
  }
}

/*!
 * Bias F/T sensor. Bias value is a mean over a given number of counts.
 * \warning This function is blocking. Between 2 successive counts we wait for 5 ms.
 * \param n_counts : Number of counts used to bias.
 *
 * \sa unbias()
 */
void vpForceTorqueAtiNetFTSensor::bias(unsigned int n_counts)
{
  if (!m_is_init) {
    throw(vpException(vpException::notInitialized, "Cannot bias: UDP client is not initialized"));
  }

  if (!m_is_streaming_started) {
    throw(vpException(vpException::notInitialized, "Cannot bias: streaming was not started"));
  }

  vpColVector ft_bias_tmp(6, 0);
  m_ft_bias = 0;

  if (n_counts == 0) {
    m_ft_bias = getForceTorque();
  }
  else {
    for (unsigned int i = 0; i < n_counts; i++) {
      ft_bias_tmp += getForceTorque();
      waitForNewData();
    }
    m_ft_bias = ft_bias_tmp / n_counts;
  }
}

/*!
 * Unbias F/T sensor.
 *
 * \sa bias()
 */
void vpForceTorqueAtiNetFTSensor::unbias() { m_ft_bias = 0; }

/*!
 * Return force / torque measurements in user units, respectively N and Nm.
 *
 * To obtain the force and torque values in user units (N and Nm), each received force value is internally multiplied by
 * the scaling factor and divided by the counts per force and each received torque value is internally multiplied by the
 * scaling factor and divided by the counts per torque.
 *
 * \return A 6-dim vector that contains the 3 forces and 3 torques [Fx, Fy, Fz, Tx, Ty, Tz] with forces in N
 * and torques in Nm.
 */
vpColVector vpForceTorqueAtiNetFTSensor::getForceTorque() const
{
  if (!m_is_init) {
    throw(vpException(vpException::notInitialized, "Cannot get F/T: UDP client is not initialized"));
  }

  if (!m_is_streaming_started) {
    throw(vpException(vpException::notInitialized, "Cannot get F/T: streaming was not started"));
  }

  if (m_data_count_prev == m_data_count) {
    throw(vpException(vpException::notInitialized, "Cannot get F/T: no new data available"));
  }

  return m_ft;
}

/*!
 * Wait for new data.
 * \param timeout : Timeout in ms.
 * \return True if a new data was received, false otherwise.
 */
bool vpForceTorqueAtiNetFTSensor::waitForNewData(unsigned int timeout)
{
  if (!m_is_init) {
    throw(vpException(vpException::notInitialized, "Cannot wait for new data: UDP client is not initialized"));
  }

  if (!m_is_streaming_started) {
    throw(vpException(vpException::notInitialized, "Cannot wait for new data: streaming was not started"));
  }

  double t = vpTime::measureTimeMs();
  m_data_count_prev = m_data_count;
  while (vpTime::measureTimeMs() - t < static_cast<double>(timeout)) {
    unsigned char response[36];
    RESPONSE resp;
    if (receive((void *)response, 36)) {
      resp.rdt_sequence = ntohl(*(uint32_t *)&response[0]);
      resp.ft_sequence = ntohl(*(uint32_t *)&response[4]);
      resp.status = ntohl(*(uint32_t *)&response[8]);
      for (int i = 0; i < 6; i++) {
        resp.FTData[i] = ntohl(*(int32_t *)&response[12 + i * 4]);
      }
      // Output the response data.
      if (resp.status) {
        throw(vpException(vpException::notInitialized, "Cannot wait for new data: status 0x%08x is not 0x00000000",
                          resp.status));
      }
      double force_factor = static_cast<double>(m_scaling_factor) / static_cast<double>(m_counts_per_force);
      double torque_factor = static_cast<double>(m_scaling_factor) / static_cast<double>(m_counts_per_torque);
      for (int i = 0; i < 3; i++) {
        m_ft[i] = resp.FTData[i] * force_factor;
      }
      for (int i = 3; i < 6; i++) {
        m_ft[i] = resp.FTData[i] * torque_factor;
      }
      // Consider bias
      m_ft -= m_ft_bias;

      m_data_count++;
    }

    if (m_data_count != m_data_count_prev) {
      return true;
    }
    vpTime::sleepMs(1);
  }

  return false;
}
END_VISP_NAMESPACE
#endif
