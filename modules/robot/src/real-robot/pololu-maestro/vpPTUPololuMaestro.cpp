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

#include <visp3/core/vpConfig.h>


#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO


#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpPTUPololuMaestro.h>

vpPTUPololuMaestro::vpPTUPololuMaestro()
  : m_baudrate(9600), m_device("/dev/ttyACM0"), m_serialInterface(NULL), m_verbose(false)
{
  setConnection(this->m_device, this->m_baudrate);

  m_pan = vpServoPololuMaestro(this->m_serialInterface, 0);
  m_tilt = vpServoPololuMaestro(this->m_serialInterface, 1);
}

vpPTUPololuMaestro::vpPTUPololuMaestro(const std::string &device, int baudrate)
  : m_baudrate(baudrate), m_device(device), m_serialInterface(NULL), m_verbose(false)
{
  setConnection(this->m_device, this->m_baudrate);

  m_pan = vpServoPololuMaestro(this->m_serialInterface, 0);
  m_tilt = vpServoPololuMaestro(this->m_serialInterface, 1);
}

vpPTUPololuMaestro::~vpPTUPololuMaestro() { }

void vpPTUPololuMaestro::setConnection(std::string device, int baudrate)
{
  if (m_serialInterface != NULL) {
    throw(vpRobotException(vpRobotException::constructionError, "Pololu board already connected"));
  }
  std::string error_msg;
  this->m_serialInterface = RPM::SerialInterface::createSerialInterface(device, baudrate, &error_msg);

  if (!m_serialInterface->isOpen()) {
    if (m_verbose) {
      //std::cout << error_msg << std::endl;
      //std::cout << m_serialInterface->isOpen() << "\n";
    }
    throw(vpRobotException(vpRobotException::constructionError, "Cannot connect to pololu board with device: %s and baudrate: %d", device.c_str(), baudrate));
  }
  else if (m_serialInterface->isOpen() && m_verbose) {
    std::cout << "Serial " << device << " is started!\n";
  }
}

void vpPTUPololuMaestro::setPositionAngle(float angle, unsigned short speed, Axe axe)
{
  switch (axe) {
  case pan:
    m_pan.setPositionAngle(angle, speed);
    break;
  case tilt:
    m_tilt.setPositionAngle(angle, speed);
    break;
  default:
    throw(vpRobotException(vpRobotException::notImplementedError, "No corresponding axe."));
    break;
  }
}

void vpPTUPololuMaestro::getRange(float &minAngle, float &maxAngle, float &rangeAngle, Axe axe)
{
  switch (axe) {
  case pan:
    m_pan.getRangeAngle(minAngle, maxAngle, rangeAngle);
    break;
  case tilt:
    m_tilt.getRangeAngle(minAngle, maxAngle, rangeAngle);
    break;
  default:
    throw(vpRobotException(vpRobotException::notImplementedError, "No corresponding axe."));
    minAngle = 0;
    maxAngle = 0;
    rangeAngle = 0;
    break;
  }
}

void vpPTUPololuMaestro::setVelocityCmd(short speed, Axe axe)
{
  if (m_verbose) {
    std::cout << "Start vel cmd" << std::endl;
  }
  switch (axe) {
  case pan:
    m_pan.setVelocityCmd(speed);
    break;
  case tilt:
    m_tilt.setVelocityCmd(speed);
    break;
  default:
    throw(vpRobotException(vpRobotException::notImplementedError, "No corresponding axe."));
    break;
  }
}

void vpPTUPololuMaestro::stopVelocityCmd(Axe axe)
{
  switch (axe) {
  case pan:
    m_pan.stopVelCmd();
    break;
  case tilt:
    m_tilt.stopVelCmd();
    break;
  default:
    throw(vpRobotException(vpRobotException::notImplementedError, "No corresponding axe."));
    break;
  }
}

void vpPTUPololuMaestro::getPositionAngle(float &angle, Axe axe)
{
  switch (axe) {
  case pan:
    angle = m_pan.getPositionAngle();
    break;
  case tilt:
    angle = m_tilt.getPositionAngle();
    break;
  default:
    throw(vpRobotException(vpRobotException::notImplementedError, "No corresponding axe."));
    break;
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpPTUPololuMaestro.cpp.o) has no symbols
void dummy_vpPTUPololuMaestro() { };
#endif
