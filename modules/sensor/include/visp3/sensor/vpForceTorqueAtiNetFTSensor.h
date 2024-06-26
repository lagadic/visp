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
 * ATI Force torque interface.
 */
#ifndef _vpForceTorqueAtiNetFTSensor_h_
#define _vpForceTorqueAtiNetFTSensor_h_

#include <visp3/core/vpConfig.h>

#include <ostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpUDPClient.h>

// Make vpForceTorqueAtiNetFTSensor available only if inet_ntop() used to
// communicate by UDP with the sensor through vpUDPClient is available; inet_ntop()
// is not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

BEGIN_VISP_NAMESPACE
/*!
 * \class vpForceTorqueAtiNetFTSensor
 *
 * \ingroup group_sensor_ft
 *
 * Interface for ATI force/torque sensor using [Net F/T](https://www.ati-ia.com/products/ft/ft_NetFT.aspx) over UDP.
 *
 * The Network Force/Torque (Net F/T) sensor system measures six components of force and torque (Fx, Fy, Fz, Tx, Ty, Tz).
 * The Net F/T provides an EtherNet/IP communication interface and is compatible with standard Ethernet. The Net
 * F/T system is available with any of ATI transducer models. The Net F/T's web browser interface allows for easy
 * configuration and set up via the Ethernet connection present on all NetBox models.
 *
 * This class was tested with ATI Nano 43 F/T sensor connected to a NetBox. To use this class, you don't need to install
 * any specific third-party.
 *
 * To use this class, connect an Ethernet cable to the NetBox. The default IP address of the Net F/T is: 192.168.1.1.
 * The default Ethernet port is 49152.
 * You can use your favorite web browser on http://192.168.1.1 to modify Net F/T sensor settings and select sensor
 * calibration configuration.
 *
 * The following example shows how to use this class to get F/T measurements.
 * \code
 * #include <iostream>
 *
 * #include <visp3/sensor/vpForceTorqueAtiNetFTSensor.h>
 *
 * int main(int argc, char **argv)
 * {
 *   vpForceTorqueAtiNetFTSensor ati_net_ft("192.168.1.1", 49152);
 *
 *   ati_net_ft.startStreaming();
 *   ati_net_ft.bias();
 *
 *   while (1) {
 *     double t = vpTime::measureTimeMs();
 *     if (ati_net_ft.waitForNewData()) {
 *       vpColVector ft = ati_net_ft.getForceTorque();
 *       std::cout << "F/T: " << ft.t() << std::endl;
 *     }
 *     std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
 *   }
 * }
 * \endcode
 *
 * It produces the following output:
 * \code
 * F/T: -0.00150018  0.0030764  -0.00791356  -8.22294e-06  4.18799e-05  1.078288e-05
 * Loop time: 0.03393554688 ms
 * ...
 * \endcode
 * where 3 first values are forces Fx, Fy, Fz in N and the 3 last are torques Tx, Ty, Tz in Nm.
*/
class VISP_EXPORT vpForceTorqueAtiNetFTSensor : public vpUDPClient
{
public:
  vpForceTorqueAtiNetFTSensor();
  vpForceTorqueAtiNetFTSensor(const std::string &hostname, int port);
  virtual ~vpForceTorqueAtiNetFTSensor() VP_OVERRIDE;

  void bias(unsigned int n_counts = 50);
  /*!
   * \return Counts per force used to tranform measured data in N.
   * \sa getCountsPerTorque(), getForceTorque()
   */
  inline unsigned long getCountsPerForce() const { return m_counts_per_force; }
  /*!
   * \return Counts per torque used to tranform measured data in Nm.
   * \sa getCountsPerForce(), getForceTorque()
   */
  inline unsigned long getCountsPerTorque() const { return m_counts_per_torque; }
  /*!
   * \return Data counter. Each call to waitForNewData() will increment data counter when a new data is received.
   */
  inline unsigned long getDataCounter() const { return m_data_count; }
  /*!
   * \return Scaling factor to transform measured data in user units (N and Nm).
   * \sa getCountsPerForce(), getCountsPerTorque(), getForceTorque()
   */
  inline unsigned long getScalingFactor() const { return m_scaling_factor; }
  vpColVector getForceTorque() const;
  /*!
   * Set counts per force value. Default value is 1000000.
   * \param counts : Counts per force.
   * \sa setCountsPerTorque(), setScalingFactor()
   */
  inline void setCountsPerForce(unsigned long counts) { m_counts_per_force = counts; }
  /*!
   * Set counts per torque value. Default value is 1000000000.
   * \param counts : Counts per torque.
   * \sa setCountsPerForce(), setScalingFactor()
   */
  inline void setCountsPerTorque(unsigned long counts) { m_counts_per_torque = counts; }
  /*!
   * Set scaling factor. Default value is 1.
   * \param scaling_factor : scaling factor.
   * \sa setCountsPerForce(), setCountsPerTorque()
   */
  inline void setScalingFactor(unsigned long scaling_factor) { m_scaling_factor = scaling_factor; }
  bool startStreaming();
  void stopStreaming();

  void unbias();
  bool waitForNewData(unsigned int timeout = 50);

protected:
  unsigned long m_counts_per_force;
  unsigned long m_counts_per_torque;
  unsigned long m_scaling_factor;
  vpColVector m_ft_bias;
  unsigned long m_data_count;
  unsigned long m_data_count_prev;
  vpColVector m_ft;
  bool m_is_streaming_started;
};
END_VISP_NAMESPACE
#endif
#endif
