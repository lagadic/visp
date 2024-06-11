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
 * Wrapper over IIT force-torque sensor.
 *
 * Authors:
 * Alexander Oliva
 *
*****************************************************************************/

/*!
  \file vpForceTorqueIitSensor.cpp
  \brief Wrapper over IIT force-torque sensor.
*/

#include <visp3/sensor/vpForceTorqueIitSensor.h>

#if defined(VISP_HAVE_FT_IIT_SDK) && defined(VISP_HAVE_THREADS)

BEGIN_VISP_NAMESPACE
/*!
  Default constructor.

  Establish communication with sensor(-s) and start data acquisition thread.
*/
vpForceTorqueIitSensor::vpForceTorqueIitSensor()
  : m_ftLib(), m_numSensorsInLib(0), m_ft(6, 0), m_ft_filt(6, 0), m_ftSensorsData(), m_acquisitionEnabled(false),
  m_dataValid(false), m_connected(false), m_acquisitionThread(), m_timeCur(), m_timePrev(), m_mutex(),
  m_warmupMilliseconds(500)
{
  // Get number of connected in library sensors
  m_numSensorsInLib = m_ftLib._getNumberOfConnectedSensors();

  /*
   * Initialize Communication with sensor(-s):
   *
   *      streaming is configured with "storeOption=0",
   *      which means that data will be stored in the library's main thread
   *      and will be given in an output file.
   *
   *      "storeDataFlag" is initialized as "false", so that the
   *      recording will not start with the streaming thread, but
   *      the moment for when the recording will start & stop.
   */
  if (m_ftLib._configureStreaming(false, 0) == 0) {
    // Start the main acquisition thread
    m_ftLib._startStreamingThread();

    m_connected = true;
  }
}

/*!
  Stops data streaming.
 */
void vpForceTorqueIitSensor::close()
{
  m_ftLib._stopStreamingThread();
  m_acquisitionEnabled = false;
  m_dataValid = false;
}

/*!
  Destructor that stops streaming
 */
vpForceTorqueIitSensor::~vpForceTorqueIitSensor() { this->close(); }

/*!
   Join acquisition thread.
 */
void vpForceTorqueIitSensor::join()
{
  if (m_acquisitionThread.joinable()) {
    m_acquisitionThread.join();
  }
}

/*!
  Acquisition loop. Warm up the sensor
 */
void vpForceTorqueIitSensor::acquisitionLoop()
{
  m_timePrev = m_timeCur = std::chrono::system_clock::now();

  // Main thread
  auto time_init = std::chrono::system_clock::now();
  while (m_acquisitionEnabled) {

    // Get time passed since the last acquired sample
    m_timeCur = std::chrono::system_clock::now();

    // Calculate delta time
    auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(m_timeCur - m_timePrev).count();

    if (elapsed_milliseconds >= 1) {
      /*
       * Once a new aquisition started,
       * reset loop timers to keep a relatively fixed sampling time
       */
      // Update previous time
      m_timePrev = m_timeCur;

      /*
       * get all connected sensors' data:
       * call to a mutex method allowing to access the ftsensors' data
       * in streaming mode (1 sample / packet / sensor)
       */
      m_ftSensorsData = m_ftLib._getFTSensorsData();

      // Warm up the sensor. At the beginning we experienced that values returned in m_ftSensorsData.ftSensor->ft
      // are completly wrong like the following:
      // 2.237378396e+11  207.3293304  14291.07715 1.479413346e+19  13.26593399  3380.078613
      auto warmup_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(m_timeCur - time_init).count();
      if (warmup_milliseconds > m_warmupMilliseconds) {
        m_dataValid = true;
      }
      else {
        continue;
      }

      const std::lock_guard<std::mutex> lock(m_mutex);
      for (unsigned int i = 0; i < 6; i++) {
        m_ft[i] = m_ftSensorsData.ftSensor->ft[i];
        m_ft_filt[i] = m_ftSensorsData.ftSensor->filt_ft[i];
      }
    }
  }
}

/*!
  Bias all sensors by TCP.
 */
void vpForceTorqueIitSensor::bias() { m_ftLib._biasAllFTSensorsTCP(); }

/*!
  Return true if communication established with at least one sensor.
  \param timeout_ms : Timeout in milliseconds.

  If you get the following error:
  \code
[ ERROR: ] Connection failed! Error Connection refused
  \endcode
  it means probably that you didn't set the right parameters in the configuration file `configurationSettings.ini`.
  See class description to get some hints.
 */
bool vpForceTorqueIitSensor::connected(int timeout_ms) const
{
  vpChrono chrono;
  chrono.start();
  while (!m_connected && chrono.getDurationMs() < timeout_ms) {
    vpTime::sleepMs(1);
  }

  return m_connected;
}

/*!
  Get force-torque data in SI units.

  \param[in] filtered : When true return filtered force-torque measurements,
  when false return raw data.
  If no filter is configured while getting filtered measurements, the SDK will
  return the raw data.
  To configure the filter, you must access the sensor through the web interface.
  The default ip address is `192.168.1.1` if in default mode.
  Once in the web interface select NETWORK SETTINGS and you can configure the
  Data Filtering Settings:
  - Filter Type: Low-Pass or High-Pass Butterworth
  - Filter Order: 1, 2 or 3
  - Cut-off Frequency: frequency in Hz

  \return A 6-dim vector \f$[F_x \; F_y \; F_z \; T_x \; T_y \; T_z]^T\f$ with forces \f$F_x\f$,
  \f$F_y\f$, \f$F_z\f$ in N, and torques \f$T_x\f$, \f$T_y\f$, \f$T_z\f$ in Nm.

  As shown in the next image, our sensor has IP  `192.168.100.10`. Filtering
  is configured as Low-Pass, with a 3 order filter and a 10 Hz cutt-off frequency.
  \image html vpForceTorqueIitSensor-ethernet.png
 */
vpColVector vpForceTorqueIitSensor::getForceTorque(bool filtered)
{
  const std::lock_guard<std::mutex> lock(m_mutex);
  if (filtered) {
    return m_ft_filt;
  }
  else {
    return m_ft;
  }
}

/*!
   Start acquisition thread and wait until data are available.
 */
void vpForceTorqueIitSensor::startStreaming()
{
  m_acquisitionEnabled = true;
  m_acquisitionThread = std::thread([this] { this->acquisitionLoop(); });

  while (!m_dataValid) {
    vpTime::wait(10);
  }
}

/*!
   Stop acquisition thread.
 */
void vpForceTorqueIitSensor::stopStreaming()
{
  m_acquisitionEnabled = false;
  if (m_acquisitionThread.joinable()) {
    m_acquisitionThread.join();
  }
}
END_VISP_NAMESPACE
#else
// Work around to avoid warning:
// libvisp_sensor.a(vpForceTorqueIitSensor.cpp.o) has no symbols
void dummy_vpForceTorqueIitSensor() { };
#endif
