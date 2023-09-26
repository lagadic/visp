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
 * Common features for Pololu Maestro PanTiltUnit.
 */

#ifndef _vpPTUPololuMaestro_h_
#define _vpPTUPololuMaestro_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <RPMSerialInterface.h>

#include <visp3/robot/vpServoPololuMaestro.h>

using namespace std;

/*!
 * \class vpPTUPololuMaestro
 * \ingroup group_robot_real_arm
 *
 * \brief Interface for the Pololu Maestro Pan Tilt Unit using servo motors.
 *
 * See https://www.pololu.com/category/102/maestro-usb-servo-controllers for more details.
 *
 * This class handle the vpServoPololuMaestro class in a higher level and allows to control
 * the Pan Tilt Unit using position or velocity commands.
 *
 */
class VISP_EXPORT vpPTUPololuMaestro
{
public:
  typedef enum { pan, tilt, yaw } Axe;

  /*!
  * Default constructor.
  */
  vpPTUPololuMaestro();


  /*!
  * Value constructor.
  *
  * \param baudrate : Baudrate used for the serial communication.
  *
  * \param device : Name of the serial interface used for communication.
  */
  vpPTUPololuMaestro(const std::string &device, int baudrate);

  /*!
  * Destructor.
  */
  virtual ~vpPTUPololuMaestro();

  /*!
  * Get ranges for a given axe.
  *
  * \param minAngle : Pointer to minimum range of the servo.
  *
  * \param maxAngle : Pointer to maximum range of the servo.
  *
  * \param rangeAngle : Pointer to the range of the servo.
  *
  * \param axe : One of the axe define in vpPTUPololuMaestro.h Axe enum.
  *
  */
  void getRange(float &minAngle, float &maxAngle, float &rangeAngle, Axe axe = pan);

  /*!
  *   Get the current position of a servo motor in degree.
  *
  * \param angle : Angle, in degree.
  *
  * \param axe : One of the axe define in vpPTUPololuMaestro.h Axe enum.
  *
  */
  void getPositionAngle(float &angle, Axe axe);


  /*!
  * Initiate the serial connection with the Pololu board.
  *
  * \param device : Name of the serial interface used for communication.
  *
  * \param baudrate : Baudrate used for the serial communication.
  *
  */
  void setConnection(std::string device, int baudrate);

  /*!
  * Set position in degree and the maximum speed of displacement for a given axe.
  *
  * \param angle : Angle to reach, in degree.
  *
  * \param speed : Maximum speed for movement in units of (0.25 μs)/(10 ms).
  *                You can use the vpServoPololuMaestro::degSToSpeed method for conversion.
  *
  * \param axe : One of the axe define in vpPTUPololuMaestro.h Axe enum.
  *
  */
  void setPositionAngle(float angle, unsigned short speed = 0, Axe axe = pan);

  /*!
  *  Set and start the velocity command for a given axe.
  *
  * \param speed : Speed for movement in units of (0.25 μs)/(10 ms).
  *                You can use the vpServoPololuMaestro::degSToSpeed method for conversion.
  *
  * \param axe : One of the axe define in vpPTUPololuMaestro.h Axe enum.
  *
  */
  void setVelocityCmd(short speed, Axe axe = pan);


  /*!
  *  Stop the velocity command for a given axe.
  *
  * \param axe : One of the axe define in vpPTUPololuMaestro.h Axe enum.
  *
  */
  void stopVelocityCmd(Axe axe);

private:
  // Serial connection parameters.
  int m_baudrate;
  std::string m_device;
  RPM::SerialInterface *m_serialInterface;

  vpServoPololuMaestro m_pan;
  vpServoPololuMaestro m_tilt;

  bool m_verbose;
};

#endif
#endif
