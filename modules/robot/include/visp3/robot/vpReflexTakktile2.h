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

#ifndef _vpReflexTakktile2_h_
#define _vpReflexTakktile2_h_

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_TAKKTILE2

#include <string>
#include <vector>

#include <visp3/core/vpColVector.h>

BEGIN_VISP_NAMESPACE
/*!

  \class vpReflexTakktile2

  \ingroup group_robot_haptic

  Right Hand Robotics Reflex Takktile 2 hand controller.

  See https://www.labs.righthandrobotics.com/docs for more details.

  This class was tested with the [Reflex Takktile2 hand](https://www.labs.righthandrobotics.com/reflexhand).

  To communicate by Ethernet with the hand we recommend to setup computer network with a static configuration:
  \verbatim
  IP: 10.1.1.10
  Gateway: 254.0.0.0
  \endverbatim

*/
class VISP_EXPORT vpReflexTakktile2
{
public:
  class HandInfo
  {
  public:
    std::vector<float> proximal;
    std::vector<float> distal_approx;
    std::vector<std::vector<int> > pressure;
    std::vector<std::vector<bool> > contact;

    std::vector<float> joint_angle;
    std::vector<float> raw_angle;
    std::vector<float> velocity;
    std::vector<float> load;
    std::vector<float> voltage;

    std::vector<uint32_t> temperature;
    std::vector<std::string> error_state;

    HandInfo();
    ~HandInfo() { }

    friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const HandInfo &hand);
  };

  vpReflexTakktile2();
  virtual ~vpReflexTakktile2();

  void calibrate();

  void disableTorque();

  HandInfo getHandInfo();

  int getNumFingers() const;
  int getNumSensorsPerFinger() const;
  int getNumServos() const;

  vpColVector getPosition() const;
  vpColVector getVelocity() const;

  void open();

  /*!
   * Set yaml file used to calibrate fingers.
   * \param finger_file_name : Finger calibration file.
   */
  void setFingerConfigFile(const std::string &finger_file_name) { m_finger_file_name = finger_file_name; }

  /*!
   * Set yaml file used to define motor constants.
   * \param motor_file_name : Motor constant file.
   */
  void setMotorConfigFile(const std::string &motor_file_name) { m_motor_file_name = motor_file_name; }

  /*!
   * Set network interface name.
   * \param network_interface : Network interface name used to communicate with the Right Hand.
   * Use `ifconfig` command to know the name of this interface.
   */
  void setNetworkInterface(const std::string &network_interface = "eth0") { m_network_interface = network_interface; }

  void setPosition(const vpColVector &targets);

  /*!
   * Set yaml file used to calibrate tactile sensors.
   * \param tactile_file_name : Tactile calibration file.
   */
  void setTactileConfigFile(const std::string &tactile_file_name) { m_tactile_file_name = tactile_file_name; }

  void setTactileThreshold(int threshold);
  void setTactileThreshold(const std::vector<int> &thresholds);

  void setPositioningVelocity(const vpColVector &targets);
  void setVelocityUntilAnyContact(const vpColVector &targets);
  void setVelocityUntilEachContact(const vpColVector &targets);

  void wait(int milliseconds);

protected:
  std::string m_network_interface;
  std::string m_finger_file_name;
  std::string m_tactile_file_name;
  std::string m_motor_file_name;
  HandInfo m_hand_info;

private:
  // Implementation
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
