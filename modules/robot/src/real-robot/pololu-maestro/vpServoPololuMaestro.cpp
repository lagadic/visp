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


#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include <chrono>
#include <thread>

#include <RPMSerialInterface.h>

#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpServoPololuMaestro.h>

std::chrono::milliseconds millis(1);


vpServoPololuMaestro::vpServoPololuMaestro(const std::string &device, int baudrate)
  : m_interface(NULL), m_channel(0), m_FlagVelCmdRunning(false), m_position(0), m_speed(0), m_velocityDirection(0), m_verbose(false)
{
  std::string error_msg;
  this->m_interface = RPM::SerialInterface::createSerialInterface(device, baudrate, &error_msg);

  if (!m_interface->isOpen()) {
    throw(vpRobotException(vpRobotException::constructionError,
                           "Cannot connect to pololu board with device: %s and baudrate: %d", device.c_str(), baudrate));
  }

  std::thread t(&vpServoPololuMaestro::VelocityCmdThread, this);
  t.detach();

  if (m_verbose) {
    std::cout << "Default servo created on channel: " << this->m_channel << std::endl;
  }
}


vpServoPololuMaestro::vpServoPololuMaestro(RPM::SerialInterface *interface, int channel, bool verbose)
  : m_interface(interface), m_channel(channel), m_FlagVelCmdRunning(false), m_position(0), m_speed(0), m_velocityDirection(0), m_verbose(verbose)
{
  std::thread t(&vpServoPololuMaestro::VelocityCmdThread, this);
  t.detach();

  if (m_verbose) {
    std::cout << "Value servo created on channel: " << this->m_channel << std::endl;
  }
}


vpServoPololuMaestro::~vpServoPololuMaestro() { }


int vpServoPololuMaestro::angle2PWM(float angle)
{
  return ((angle + abs(m_minAngle)) / m_rangeAngle) * m_rangePWM + m_minPWM;
}

bool vpServoPololuMaestro::checkConnection()
{
  if (!this->m_interface->isOpen()) {
    if (m_verbose) {
      std::cout << "Serial Communication Failed!\n";
    }
    return 1;
  }
  return 0;
}

short vpServoPololuMaestro::degSToSpeed(float speedDegS)
{
  return (speedDegS / 100) * (this->m_rangePWM / this->m_rangeAngle);
}

unsigned short vpServoPololuMaestro::getPosition()
{
  m_interface->getPositionCP(this->m_channel, this->m_position);
  return this->m_position;
}

float vpServoPololuMaestro::getPositionAngle()
{
  m_interface->getPositionCP(this->m_channel, this->m_position);
  float positionAngle = this->PWM2Angle(this->m_position);
  return positionAngle;
}

void vpServoPololuMaestro::getRangeAngle(float &minAngle, float &maxAngle, float &rangeAngle)
{
  minAngle = this->m_minAngle;
  maxAngle = this->m_maxAngle;
  rangeAngle = this->m_rangeAngle;
}

void vpServoPololuMaestro::getRangePWM(int &minPWM, int &maxPWM, int &rangePWM)
{
  minPWM = this->m_minPWM;
  maxPWM = this->m_maxPWM;
  rangePWM = this->m_rangePWM;
}

unsigned short vpServoPololuMaestro::getSpeed() { return this->m_speed; }


float vpServoPololuMaestro::PWM2Angle(int PWM) { return (PWM * (m_rangeAngle / m_rangePWM) + m_minAngle); }

void vpServoPololuMaestro::setPositionAngle(float targetAngle, unsigned short speed)
{
  if ((this->m_minAngle <= targetAngle) && (targetAngle <= this->m_maxAngle)) {
    int targetPWM = angle2PWM(targetAngle);
    setPositionPWM(targetPWM, speed);
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "given position: %d is outside of the servo range. You can check the range using the method getRangeAngle()", targetAngle));

  }
}

void vpServoPololuMaestro::setPositionPWM(int targetPWM, unsigned short speed)
{
  if ((this->m_minPWM <= targetPWM) && (targetPWM <= this->m_maxPWM)) {
    this->setSpeed(speed);
    // std::cout << "position (PWM):"<<targetPWM<<" desired speed :"<<speed<<" \n";
    // std::cout << "current speed :"<<getSpeed()<<" \n";
    m_interface->setTargetCP(this->m_channel, targetPWM);
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "given position: %d is outside of the servo range. You can check the range using the method getRangePWM()", targetPWM));
  }
}

void vpServoPololuMaestro::setSpeed(unsigned short speed)
{
  if (speed <= 1000) {
    m_interface->setSpeedCP(this->m_channel, speed);
    this->m_speed = speed;
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "given speed : %d is outside of the servo speed range. range is from 0 to 1000", speed));
  }
}


void vpServoPololuMaestro::setSpeedDegS(float speedDegS)
{
  unsigned short speed =
    (unsigned short)abs(this->degSToSpeed(speedDegS)); // making sure the speed is positive and convert it tu ushort

  this->setSpeed(speed);
}

void vpServoPololuMaestro::setVelocityCmd(short velocityCmdSpeed)
{
  if (velocityCmdSpeed <= 1000) {
    this->m_speed = abs(velocityCmdSpeed);
    if (velocityCmdSpeed >= 0) {
      this->m_velocityDirection = m_maxPWM;
    }
    else {
      this->m_velocityDirection = m_minPWM;
    }
    this->m_FlagVelCmdRunning = true;
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "given velocityCmdSpeed : %d is outside of the servo speed range. range is from 0 to 1000", velocityCmdSpeed));
  }
}

float vpServoPololuMaestro::speedToDeGS(short speed) { return (speed * 100) * (this->m_rangeAngle / this->m_rangePWM); }


void vpServoPololuMaestro::stopVelCmd()
{
  if (m_verbose) {
    std::cout << "stoping vel cmd \n";
  }
  this->m_FlagVelCmdRunning = false;

  std::this_thread::sleep_for(10 * millis);
  this->setPositionPWM(this->getPosition());
}


void vpServoPololuMaestro::VelocityCmdThread()
{
  int antispamCounter = 0;
  while (true) {
    if (this->m_FlagVelCmdRunning) {
      this->getPosition();
      if (int(this->m_position) != (this->m_velocityDirection)) {
        this->setPositionPWM(this->m_velocityDirection, this->m_speed);
      }
      else {
        if (m_verbose) {
          std::cout << "edge of range reach" << std::endl;
        }
      }
    }
    else {
      if (antispamCounter == 100) {
        std::cout << "waiting"
          << "flag is : " << this->m_FlagVelCmdRunning << std::endl;
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
