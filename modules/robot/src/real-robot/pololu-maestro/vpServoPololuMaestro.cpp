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
 * Common features for Pololu Maestro Servo Motor.
 */

#include <visp3/core/vpConfig.h>


#ifdef VISP_HAVE_POLOLU

#include <chrono>
#include <thread>

#include <RPMSerialInterface.h>

#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpServoPololuMaestro.h>

std::chrono::milliseconds millis(1);

RPMSerialInterface *vpServoPololuMaestro::m_interface = nullptr;
int vpServoPololuMaestro::m_nb_servo = 0;

vpServoPololuMaestro::vpServoPololuMaestro(const std::string &device, int baudrate, int channel, bool verbose)
  : m_channel(channel), m_FlagVelCmdRunning(false), m_position(0), m_speed(0), m_velocityDirection(0), m_verbose(verbose)
{
  std::string error_msg;
  m_nb_servo++;

  if (!m_interface) {
    m_interface = RPMSerialInterface::createSerialInterface(device, baudrate, &error_msg);

    if (!m_interface->isOpen()) {
      throw(vpRobotException(vpRobotException::constructionError,
                             "Cannot connect to pololu board with device: %s and baudrate: %d", device.c_str(), baudrate));
    }
  }

  std::thread t(&vpServoPololuMaestro::VelocityCmdThread, this);
  t.detach();

  if (m_verbose) {
    std::cout << "Default servo created on channel: " << m_channel << std::endl;
  }
}

vpServoPololuMaestro::~vpServoPololuMaestro()
{
  if (m_nb_servo == 1) {
    delete m_interface;
  }
  m_nb_servo--;
}

int vpServoPololuMaestro::angle2PWM(float angle)
{
  return ((angle + abs(m_minAngle)) / m_rangeAngle) * m_rangePWM + m_minPWM;
}

bool vpServoPololuMaestro::checkConnection()
{
  if (!m_interface->isOpen()) {
    if (m_verbose) {
      std::cout << "Serial Communication Failed!\n";
    }
    return true;
  }
  return false;
}

short vpServoPololuMaestro::degSToSpeed(float speedDegS)
{
  return (speedDegS / 100) * (m_rangePWM / m_rangeAngle);
}

unsigned short vpServoPololuMaestro::getPosition()
{
  m_interface->getPositionCP(m_channel, m_position);
  return m_position;
}

float vpServoPololuMaestro::getPositionAngle()
{
  m_interface->getPositionCP(m_channel, m_position);
  float positionAngle = PWM2Angle(m_position);
  return positionAngle;
}

void vpServoPololuMaestro::getRangeAngle(float &minAngle, float &maxAngle, float &rangeAngle)
{
  minAngle = m_minAngle;
  maxAngle = m_maxAngle;
  rangeAngle = m_rangeAngle;
}

void vpServoPololuMaestro::getRangePWM(int &minPWM, int &maxPWM, int &rangePWM)
{
  minPWM = m_minPWM;
  maxPWM = m_maxPWM;
  rangePWM = m_rangePWM;
}

unsigned short vpServoPololuMaestro::getSpeed() { return m_speed; }

float vpServoPololuMaestro::PWM2Angle(int PWM) { return (PWM * (m_rangeAngle / m_rangePWM) + m_minAngle); }

void vpServoPololuMaestro::setPositionAngle(float targetAngle, unsigned short speed)
{
  if ((m_minAngle <= targetAngle) && (targetAngle <= m_maxAngle)) {
    int targetPWM = angle2PWM(targetAngle);
    setPositionPWM(targetPWM, speed);
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given position: %d is outside of the servo range. You can check the range using the method getRangeAngle()", targetAngle));
  }
}

void vpServoPololuMaestro::setPositionPWM(int targetPWM, unsigned short speed)
{
  if ((m_minPWM <= targetPWM) && (targetPWM <= m_maxPWM)) {
    setSpeed(speed);
    // std::cout << "position (PWM):"<<targetPWM<<" desired speed :"<<speed<<" \n";
    // std::cout << "current speed :"<<getSpeed()<<" \n";
    m_interface->setTargetCP(m_channel, targetPWM);
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given position: %d is outside of the servo range. You can check the range using the method getRangePWM()", targetPWM));
  }
}

void vpServoPololuMaestro::setSpeed(unsigned short speed)
{
  if (speed <= 1000) {
    m_interface->setSpeedCP(m_channel, speed);
    m_speed = speed;
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given speed : %d is outside of the servo speed range. range is from 0 to 1000", speed));
  }
}

void vpServoPololuMaestro::setSpeedDegS(float speedDegS)
{
  unsigned short speed =
    (unsigned short)abs(degSToSpeed(speedDegS)); // making sure the speed is positive and convert it tu ushort

  setSpeed(speed);
}

void vpServoPololuMaestro::setVelocityCmd(short velocityCmdSpeed)
{
  if (velocityCmdSpeed <= 1000) {
    m_speed = abs(velocityCmdSpeed);
    if (velocityCmdSpeed >= 0) {
      m_velocityDirection = m_maxPWM;
    }
    else {
      m_velocityDirection = m_minPWM;
    }
    m_FlagVelCmdRunning = true;
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "given velocityCmdSpeed : %d is outside of the servo speed range. range is from 0 to 1000", velocityCmdSpeed));
  }
}

float vpServoPololuMaestro::speedToDeGS(short speed) { return (speed * 100) * (m_rangeAngle / m_rangePWM); }

void vpServoPololuMaestro::stopVelCmd()
{
  if (m_verbose) {
    std::cout << "Stoping vel cmd \n";
  }
  m_FlagVelCmdRunning = false;

  std::this_thread::sleep_for(10 * millis);
  setPositionPWM(getPosition());
}

void vpServoPololuMaestro::VelocityCmdThread()
{
  int antispamCounter = 0;
  while (true) {
    if (m_FlagVelCmdRunning) {
      getPosition();
      if (int(m_position) != (m_velocityDirection)) {
        setPositionPWM(m_velocityDirection, m_speed);
      }
      else {
        if (m_verbose) {
          std::cout << "Edge of range reach" << std::endl;
        }
      }
    }
    else {
      if (antispamCounter == 100) {
        std::cout << "waiting flag is: " << m_FlagVelCmdRunning << std::endl;
        antispamCounter = 0;
      }
      else {
        antispamCounter++;
      }
    }
    std::this_thread::sleep_for(10 * millis);
  }

  throw(vpRobotException(vpRobotException::lowLevelError, "Vel cmd stopped unexpectedly "));
}


#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpServoPololuMaestro.cpp.o) has no symbols
void dummy_vpServoPololuMaestro() { };
#endif
