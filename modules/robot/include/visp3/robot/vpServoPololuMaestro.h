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

#ifndef _vpServoPololuMaestro_h_
#define _vpServoPololuMaestro_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_POLOLU

#include <iostream>
#include <string>

class RPMSerialInterface;

/*!
 * \class vpServoPololuMaestro
 * \ingroup group_robot_real_arm
 *
 * \brief Interface for the Pololu Board controlled servo.
 *
 * See https://www.pololu.com/category/102/maestro-usb-servo-controllers for more details.
 *
 * This class give a position and velocity control for the servo motors plugged into the Board.
 *
 */
class VISP_EXPORT vpServoPololuMaestro
{
public:
  /*!
   * Default constructor.
   * The VelocityCmdThread is created in the constructor and will run independently of the rest of the class.
   * You can set velocity commands using setVelCmd() method.
   *
   * \param[in] device : Serial device name to dial with Pololu board.
   * \param[in] baudrate : Baudrate used to dial with Pololu board.
   * \param[in] channel : Channel to which the servo is connected to the Pololu board.
   * \param[in] verbose : When true enable verbose mode.
   *
   */
  vpServoPololuMaestro(const std::string &device = "/dev/ttyACM0", int baudrate = 9600, int channel = 0, bool verbose = false);

  /*!
   * Destructor.
   */
  virtual ~vpServoPololuMaestro();

  /*!
   * Convert angles to PWM for servo commands.
   *
   * \param angle : Angle, in degree to be converted.
   *
   * \return Corresponding PWM value for the angle.
   */
  int angle2PWM(float angle);

  /*!
   * Check if the serial connection is still up.
   *
   * \return TRUE is the connection has an error.
   */
  bool checkConnection();

  /*!
   *  Convert deg/s speed into Pololu's speed.
   *
   * \param speedDegS : Speed converted to deg/s.
   *
   * \return speed : Speed in units of (0.25 μs)/(10 ms).
   *
   */
  short degSToSpeed(float speedDegS);

  /*!
   * Return position in PWM.
   *
   * \return position: Return current position in PWM.
   */
  unsigned short getPosition();

  /*!
   * Return position in PWM.
   *
   * \return positionAngle : Return current position in deg;
   */
  float getPositionAngle();

  /*!
   * Get min, max and range for angle cmd.
   *
   * \param minAngle : Min range value for angle control.
   *
   * \param maxAngle : Max range value for angle control.
   *
   * \param rangeAngle : Range value for angle control.
   *
   */
  void getRangeAngle(float &minAngle, float &maxAngle, float &rangeAngle);

  /*!
   * Get min, max and range for PWM cmd.
   *
   * \param minPWM : Min range value for PWM control.
   *
   * \param maxPWM : Max range value for PWM control.
   *
   * \param rangePWM : Range value for PWM control.
   *
   */
  void getRangePWM(int &minPWM, int &maxPWM, int &rangePWM);

  /*!
   *  Return the current speed parameter.
   *
   * \return speed : Speed to use for movement in units of (0.25 μs)/(10 ms). No speed (0) will use maximum speed.
   *
   */
  unsigned short getSpeed();

  /*!
   * Convert PWM to angles  for servo commands.
   *
   * \param PWM : PWM value.
   *
   * \return Corresponding angle value for the PWM.
   */
  float PWM2Angle(int PWM);

  /*!
   * Set the position to reach in angle.
   *
   * \param targetAngle : Position in angle to reach.
   *
   * \param speed : OPTIONAL : Speed to use for movement in units of (0.25 μs)/(10 ms). Default is '0' and will use
   * maximum speed.
   *
   * \return error : Return 1 if an error is encountered.
   *
   */
  void setPositionAngle(float targetAngle, unsigned short speed = 0);

  /*!
   * Set the position to reach in PWM.
   *
   * \param targetPWM : Position in PWM to reach.
   *
   * \param speed : OPTIONAL : Speed to use for movement in units of (0.25 μs)/(10 ms). Default is 0, maximum speed.
   *
   * \exception When PWM out of range.
   */
  void setPositionPWM(int targetPWM, unsigned short speed = 0);

  /*!
   *  Set the speed of the motor movements.
   *
   * \param speed : Speed to use for movement in units of (0.25 μs)/(10 ms). No speed (0) will use maximum speed.
   *
   * \return error : Return 1 if an error is encountered.
   */
  void setSpeed(unsigned short speed);

  /*!
   *  Set the speed of the motor movements in deg/s.
   *
   * \param speedRadS : Speed to use for movement in deg/s. No speed (0) will use maximum speed
   *
   * \return error : return 1 if an error is encountered
   *
   */
  void  setSpeedDegS(float speedRadS);

  /*!
   *  Set the speed of the motor movements and start the velocity command thread. The motor will move to the edge of the
   * range at the given speed.
   *
   * \param velocityCmdSpeed : Speed to use for movement in units of (0.25 μs)/(10 ms). No speed (0) will use maximum speed.
   *
   * \return error : Return 1 if an error is encountered.
   */
  void setVelocityCmd(short velocityCmdSpeed);

  /*!
   *  Convert Pololu's speed to deg/s speed.
   *
   * \param speed : Speed in units of (0.25 μs)/(10 ms).
   *
   * \return speedDegS : Speed converted to deg/s
   *
   */
  float speedToDeGS(short speed);

  /*!
   *  Stop the velocity command thread.
   *
   */
  void stopVelCmd();

private:
  static RPMSerialInterface *m_interface; // Only one interface should be used even when controlling multiple servos
  static int m_nb_servo; // Object counter to handel serial interface destruction

  int m_channel;
  bool m_FlagVelCmdRunning;

  unsigned short m_position;
  unsigned short m_speed;
  int m_velocityDirection;

  // ranges
  int m_minPWM = 2800;
  int m_maxPWM = 8800;
  int m_rangePWM = m_maxPWM - m_minPWM;
  float m_minAngle = -40;
  float m_maxAngle = 40;
  float m_rangeAngle = abs(m_minAngle) + abs(m_maxAngle);

  bool m_verbose;

  /*!
   * Thread use for Velocity control. This thread is launch in the constructor of the object and, unless crashes, will
   * run until the process is ended. If the m_FlagVelCmdRunning is set to TRUE, by invoking the setVelocityCmd method, the
   * motor will go to the edge of the motor range using the speed set in setVelocityCmd. The velocity command can be
   * stopped invoking the stopVelCmd() method.
   */
  void VelocityCmdThread();
};

#endif
#endif
