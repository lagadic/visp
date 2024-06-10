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


#if defined(VISP_HAVE_POLOLU) && defined(VISP_HAVE_THREADS)

#include <chrono>
#include <thread>

#include <RPMSerialInterface.h>

#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpPololu.h>

std::chrono::milliseconds millis(1);

BEGIN_VISP_NAMESPACE
RPMSerialInterface *vpPololu::m_interface = nullptr;
int vpPololu::m_nb_servo = 0;

vpPololu::vpPololu(bool verbose)
  : m_channel(0), m_apply_velocity_cmd(false), m_stop_velocity_cmd_thread(false),
  m_vel_speed(1), m_vel_target_position(0), m_vel_speed_prev(1), m_vel_target_position_prev(0), m_mutex_velocity_cmd(), m_verbose(verbose)
{ }

vpPololu::vpPololu(const std::string &device, int baudrate, int channel, bool verbose)
  : m_channel(channel), m_apply_velocity_cmd(false), m_stop_velocity_cmd_thread(false),
  m_vel_speed(1), m_vel_target_position(0), m_vel_speed_prev(1), m_vel_target_position_prev(0), m_mutex_velocity_cmd(), m_verbose(verbose)
{
  connect(device, baudrate, channel);
}

void vpPololu::connect(const std::string &device, int baudrate, int channel)
{
  std::string error_msg;
  m_channel = channel;
  m_nb_servo++;

  if (!m_interface) {
    if (m_verbose) {
      std::cout << "Creating serial interface '" << device << "' at " << baudrate << " bauds" << std::endl;
    }
    m_interface = RPMSerialInterface::createSerialInterface(device, baudrate, &error_msg);

    if (m_interface == nullptr) {
      throw(vpRobotException(vpRobotException::constructionError,
                             "Cannot connect to pololu board with serial port %s at baudrate %d.\n%s",
                             device.c_str(), baudrate, error_msg.c_str()));

    }
  }

  std::thread t(&vpPololu::VelocityCmdThread, this);
  t.detach();
}


vpPololu::~vpPololu()
{
  m_stop_velocity_cmd_thread = true;
  if (m_nb_servo == 1) {
    if (m_verbose) {
      std::cout << "Deleting serial interface" << std::endl;
    }
    delete m_interface;
  }
  m_nb_servo--;
}

void vpPololu::calibrate(unsigned short &pwm_min, unsigned short &pwm_max)
{
  bool ret = false;
  ret = m_interface->setTargetMSSCP(m_channel, 0);
  if (ret == false) {
    throw(vpException(vpException::fatalError, "Unable to set servo normalized target at min position"));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  pwm_min = getPwmPosition();

  ret = m_interface->setTargetMSSCP(m_channel, 254);
  if (ret == false) {
    throw(vpException(vpException::fatalError, "Unable to set servo normalized target at max position"));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  pwm_max = getPwmPosition();
}

unsigned short vpPololu::radToPwm(float angle) const
{
  float a = m_range_pwm / m_range_angle;
  float b = m_min_pwm - m_min_angle * a;

  return static_cast<unsigned short>(a * angle + b);
}

bool vpPololu::connected() const
{
  if (!m_interface->isOpen()) {
    if (m_verbose) {
      std::cout << "Serial Communication Failed!\n";
    }
    return false;
  }
  return true;
}

short vpPololu::radSToSpeed(float speed_rad_s) const
{
  return static_cast<short>((speed_rad_s / 100.f) * (m_range_pwm / m_range_angle));
}

unsigned short vpPololu::getPwmPosition() const
{
  unsigned short position_pwm;
  bool ret = false;
  int nb_attempt = 10;
  int wait_ms = 2;
  for (int i = 0; i < nb_attempt && ret == false; i++) {
    ret = m_interface->getPositionCP(m_channel, position_pwm);
    if (ret == false) {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
      if (m_verbose) {
        std::cout << "Failed to get position, new attempt: " << i << std::endl;
      }
    }
  }
  if (ret == false) {
    throw(vpException(vpException::fatalError, "Unable to get servo position"));
  }
  return position_pwm;
}

float vpPololu::getAngularPosition() const
{
  unsigned short position_pwm = getPwmPosition();
  float position_angle = pwmToRad(position_pwm);

  return position_angle;
}

void vpPololu::getRangeAngles(float &minAngle, float &maxAngle) const
{
  minAngle = m_min_angle;
  maxAngle = m_max_angle;
}

void vpPololu::getRangePwm(unsigned short &min, unsigned short &max)
{
  min = m_min_pwm;
  max = m_max_pwm;
}

float vpPololu::pwmToRad(unsigned short pwm) const
{
  float a = m_range_angle / m_range_pwm;
  float b = m_min_angle - m_min_pwm * a;

  return (a * pwm + b);
}

void vpPololu::setAngularPosition(float pos_rad, float vel_rad_s)
{
  if ((m_min_angle <= pos_rad) && (pos_rad <= m_max_angle)) {
    unsigned short pos_pwm = radToPwm(pos_rad);
    unsigned short pos_speed = static_cast<unsigned short>(std::fabs(radSToSpeed(vel_rad_s)));
    // Handle the case where pos_speed = 0 which corresponds to the pwm max speed
    if (pos_speed == 0) {
      pos_speed = 1;
    }
    setPwmPosition(pos_pwm, pos_speed);
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given position: %d is outside of the servo range. You can check the range using the method getRangeAngles()", pos_rad));
  }
}

void vpPololu::setPwmPosition(unsigned short pos_pwm, unsigned short speed_pwm)
{
  bool ret = false;
  if ((m_min_pwm <= pos_pwm) && (pos_pwm <= m_max_pwm)) {
    if (speed_pwm <= 1000) {
      ret = m_interface->setSpeedCP(m_channel, speed_pwm);
      if (ret == false) {
        throw(vpException(vpException::fatalError, "Unable to set servo speed"));
      }
    }
    else {
      throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                             "Given speed (pwm): %d is outside of the servo speed range. range is from 0 to 1000", speed_pwm));
    }
    if (m_verbose) {
      std::cout << "Channel " << m_channel << " set position (pwm): " << pos_pwm << " with speed: " << speed_pwm << std::endl;
    }
    ret = m_interface->setTargetCP(m_channel, pos_pwm);
    if (ret == false) {
      throw(vpException(vpException::fatalError, "Unable to set servo target position"));
    }
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given position (pwm): %d is outside servo range. You can check the range using the method getRangePwm()", pos_pwm));
  }
}

void vpPololu::setAngularVelocity(float vel_rad_s)
{
  short pwm_vel = radSToSpeed(vel_rad_s);

  // Handle the case where vel_rad_s != 0 but pwm_vel == 0. In that case we set a minimal speed
  if (pwm_vel == 0) {
    if (vel_rad_s > 0) {
      pwm_vel = 1;
    }
    else if (vel_rad_s < 0) {
      pwm_vel = -1;
    }
  }
  setPwmVelocity(pwm_vel);
}

void vpPololu::setPwmVelocity(short pwm_vel)
{
  unsigned short vel_speed;
  unsigned short vel_target_position;
  if (pwm_vel <= 1000) {
    vel_speed = static_cast<unsigned short>(std::abs(pwm_vel));
    if (pwm_vel > 0) {
      vel_target_position = m_max_pwm;
    }
    else if (pwm_vel < 0) {
      vel_target_position = m_min_pwm;
    }
    else { // pwm_vel = 0
      vel_target_position = getPwmPosition(); // Stay at current position
      vel_speed = 1; // Set to min speed to keep current position
    }
    m_mutex_velocity_cmd.lock();
    m_vel_speed = vel_speed;
    m_vel_target_position = vel_target_position;
    m_apply_velocity_cmd = true;
    m_mutex_velocity_cmd.unlock();
  }
  else {
    throw(vpRobotException(vpRobotException::positionOutOfRangeError,
                           "Given pwm velocity %d is outside of the servo velocity range. Range is from 0 to 1000", pwm_vel));
  }
}

float vpPololu::speedToRadS(short speed_pwm) const
{
  return (speed_pwm * 100) * (m_range_angle / m_range_pwm);
}

void vpPololu::stopVelocityCmd()
{
  if (m_verbose) {
    std::cout << "Stoping vel cmd channel: " << m_channel << std::endl;
  }

  m_mutex_velocity_cmd.lock();
  m_apply_velocity_cmd = false;
  m_mutex_velocity_cmd.unlock();

  //std::this_thread::sleep_for(10 * millis);
  unsigned short pos_pwm = getPwmPosition();
  if (m_verbose) {
    std::cout << "Stoping channel " << m_channel << " at position " << pos_pwm << std::endl;
  }
  setPwmPosition(pos_pwm, 0); // 0 to be as fast as possible to reach pos_pwm
}

void vpPololu::VelocityCmdThread()
{
  if (m_verbose) {
    std::cout << "Create Velocity command thread" << std::endl;
  }
  unsigned short vel_speed;
  unsigned short vel_target_position;
  bool apply_velocity_cmd;

  while (!m_stop_velocity_cmd_thread) {
    m_mutex_velocity_cmd.lock();
    vel_speed = m_vel_speed;
    vel_target_position = m_vel_target_position;
    apply_velocity_cmd = m_apply_velocity_cmd;
    m_mutex_velocity_cmd.unlock();
    if (apply_velocity_cmd) {
      //unsigned short position = getPwmPosition();
      //if (position != m_vel_target_position) {
      if (m_vel_speed_prev != vel_speed || m_vel_target_position_prev != vel_target_position) {
        setPwmPosition(vel_target_position, vel_speed);
      }

      m_vel_speed_prev = vel_speed;
      m_vel_target_position_prev = vel_target_position;
    }

    std::this_thread::sleep_for(20 * millis);
  }
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpPololu.cpp.o) has no symbols
void dummy_vpPololu() { };
#endif
