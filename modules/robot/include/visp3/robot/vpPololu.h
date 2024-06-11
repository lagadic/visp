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

#ifndef _vpPololu_h_
#define _vpPololu_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_POLOLU) && defined(VISP_HAVE_THREADS)

#include <iostream>
#include <string>
#include <mutex>

class RPMSerialInterface;

BEGIN_VISP_NAMESPACE
/*!
 * \class vpPololu
 * \ingroup group_robot_real_arm
 *
 * \brief Interface for the Pololu Maestro USB Servo Controllers.
 *
 * See https://www.pololu.com/category/102/maestro-usb-servo-controllers for more details.
 *
 * This class give a position and velocity control for the servo motors plugged into the board on a given channel.
 * If you want to control two or more servo motors, you need to instanciate a new object for each additional
 * servo motor. An example is given in the vpRobotPololuPtu class that allows to control a pan-tilt unit with
 * two servo motors.
 *
 * It implements a velocity controller that runs in a separate thread.
 *
 * - A servomotor can be position-controlled by giving it position commands in pwm units using setPwmPosition()
 *   or in angles expressed in radians using setAngularPosition().
 * - It can be also velocity-controller by giving it velocity commands in pwm units using setPwmVelocity() or in ras/s
 *   using setAngularVelocity().
 *
 * The conversion between pwm units and radians positions is done using radToPwm() or pwmToRad().
 * For velocity control conversion from pwm units and rad/s is done using speedToRadS() and radSToSpeed().
 *
 * Each servo has a pwm position range that could be retrieved using calibrate() and set using set using setPwmRange().
 *
 * It is the user responsability to set the corresponding angular range using setAngularRange().
*/
class VISP_EXPORT vpPololu
{
public:
  /*!
   * Default constructor.
   *
   * \param[in] verbose : When true enable verbose mode.
   *
   * You need to call connect() to setup the serial link with the Pololu board.
   */
  vpPololu(bool verbose = false);

  /*!
   * Constructor that enables the serial link with the Pololu board calling internally connect().
   * The velocity controller thread is created in the constructor and will run independently of the rest of the class.
   * You can set velocity commands using setPwmVelocity() or setAngularVelocity().
   *
   * \param[in] device : Serial device name to dial with Pololu board.
   * \param[in] baudrate : Baudrate used to dial with Pololu board. Note that this parameter is only used on Windows.
   * \param[in] channel : Channel to which the servo is connected to the Pololu board.
   * \param[in] verbose : When true enable verbose mode.
   */
  vpPololu(const std::string &device, int baudrate = 38400, int channel = 0, bool verbose = false);

  /*!
   * Destructor.
   */
  virtual ~vpPololu();

  /*!
   * Move servo motor to minimal pwm position and then to maximal pwm position
   * to retrieve min and max pwm values.
   *
   * \param[out] pwm_min : Min position (pwm).
   * \param[out] pwm_max : Max position (pwm).
   */
  void calibrate(unsigned short &pwm_min, unsigned short &pwm_max);

  /*!
   * Open a connection with the Pololu board.
   *
   * \param[in] device : Serial device name to dial with Pololu board.
   * \param[in] baudrate : Baudrate used to dial with Pololu board. Note that this parameter is only used on Windows.
   * \param[in] channel : Channel to which the servo is connected to the Pololu board.
   */
  void connect(const std::string &device, int baudrate, int channel);

  /*!
   * Check if the serial connection is still up.
   *
   * \return true is the connection is enabled, false if the board is not connected.
   */
  bool connected() const;

  /*!
   * Return angular position in rad.
   *
   * \return Current position in rad.
   */
  float getAngularPosition() const;

  /*!
   * Return PWM position.
   *
   * \return Current PWM position.
   */
  unsigned short getPwmPosition() const;

  /*!
   * Get min, max and range for angle cmd.
   *
   * \param[out] minAngle : Min range value for angle control.
   *
   * \param[out] maxAngle : Max range value for angle control.
   *
   * \sa setAngularRange()
   */
  void getRangeAngles(float &minAngle, float &maxAngle) const;

  /*!
   * Get min, max range for PWM cmd.
   *
   * \param[out] min : Min value for PWM control.
   *
   * \param[out] max: Max value for PWM control.
   *
   * \sa setPwmRange()
   */
  void getRangePwm(unsigned short &min, unsigned short &max);

  /*!
   * Set the position to reach in angle.
   *
   * \param[in] pos_rad : Position to reach in radians.
   *
   * \param[in] vel_rad_s : Velocity to use for the positioning in rad/s. Default is '0' and will use
   * maximum speed.
   */
  void setAngularPosition(float pos_rad, float vel_rad_s = 0.f);

  /*!
   * Set min and max axis angles range in rad.
   *
   * \param[in] min_angle : Min value for angle (rad).
   *
   * \param[in] max_angle : Max value for angle (rad).
   *
   * \sa getRangeAngles()
   */
  inline void setAngularRange(float min_angle, float max_angle)
  {
    m_min_angle = min_angle;
    m_max_angle = max_angle;
    m_range_angle = m_max_angle - m_min_angle;
  }

  /*!
   * Set the angular velocity of the motor movements in rad/s.
   *
   * \param[in] vel_rad_s : Velocity to apply for movement in rad/s.
   */
  void setAngularVelocity(float vel_rad_s);

  /*!
   * Set the position to reach in PWM.
   *
   * \param[in] pos_pwm : Position in PWM to reach.
   *
   * \param[in] speed_pwm : Speed to use for movement in units of (0.25 us)/(10 ms). Default is 0, maximum speed.
   *
   * \exception When PWM out of range.
   */
  void setPwmPosition(unsigned short pos_pwm, unsigned short speed_pwm = 0);

  /*!
   * Set min, max PWM cmd.
   *
   * \param[in] min_pwm : Min value for PWM control.
   *
   * \param[in] max_pwm : Max value for PWM control.
   *
   * \sa getRangePwm()
   */
  inline void setPwmRange(unsigned short min_pwm, unsigned short max_pwm)
  {
    m_min_pwm = min_pwm;
    m_max_pwm = max_pwm;
    m_range_pwm = m_max_pwm - m_min_pwm;
  }

  /*!
   * Set the pwm velocity of the motor movements. The motor will move to the edge of the
   * range at the given speed.
   *
   * \param[in] pwm_vel : PWM velocity to use for movement in units of (0.25 us)/(10 ms). When set to 0, will use the
   * maximum speed.
   */
  void setPwmVelocity(short pwm_vel);

  /*!
   * Enable/disable verbose mode.
   *
   * \param[in] verbose : Set to true to enable verbose mode, false otherwise.
   */
  void setVerbose(bool verbose)
  {
    m_verbose = verbose;
  }

  /*!
   *  Stop the velocity command thread.
   */
  void stopVelocityCmd();

  /*!
   * @name Public Member Functions for Conversion
   */
  //@{
  /*!
   * Convert a PWM value to an angle in radians.
   *
   * \param[in] pwm : PWM value.
   *
   * \return Corresponding angle value in radian for the PWM.
   *
   * \sa radToPwm()
   */
  float pwmToRad(unsigned short pwm) const;

  /*!
   * Convert angles in radians to PWM for servo commands.
   *
   * \param angle : Angle in radian to convert.
   *
   * \return Corresponding PWM value for the angle.
   *
   * \sa pwmToRad()
   */
  unsigned short radToPwm(float angle) const;

  /*!
   * Convert deg/s speed into Pololu's speed.
   *
   * \param speed_rad_s : Speed converted to rad/s.
   *
   * \return Signed speed in units of (0.25 us)/(10 ms).
   *
   * \sa speedToRadS()
   */
  short radSToSpeed(float speed_rad_s) const;

  /*!
   * Convert Pololu's pwm velocity to rad/s velocity.
   *
   * \param[in] speed : Signed speed in units of (0.25 us)/(10 ms).
   *
   * \return Speed converted to rad/s.
   *
   * \sa radSToSpeed()
   */
  float speedToRadS(short speed) const;
  //@}

private:
  static RPMSerialInterface *m_interface; // Only one interface should be used even when controlling multiple servos
  static int m_nb_servo; // Object counter to handel serial interface destruction

  int m_channel;
  bool m_apply_velocity_cmd;
  bool m_stop_velocity_cmd_thread;

  unsigned short m_vel_speed;    //!< PWM speed to in velocity control
  unsigned short m_vel_target_position; //!< Min or max PWM target position to reach in velocity control

  unsigned short m_vel_speed_prev; //!< Previous PWM speed to in velocity control
  unsigned short m_vel_target_position_prev; //!< Previous Min or max PWM target position to reach in velocity control

  std::mutex m_mutex_velocity_cmd;

  // ranges
  unsigned short m_min_pwm = 4095;
  unsigned short m_max_pwm = 7905;
  unsigned short m_range_pwm = m_max_pwm - m_min_pwm;
  float m_min_angle = -40;
  float m_max_angle = 40;
  float m_range_angle = abs(m_min_angle) + abs(m_max_angle);

  bool m_verbose;

  /*!
   * Thread use for Velocity control. This thread is launch in the constructor of the object and, unless crashes, will
   * run until the process is ended. If the m_apply_velocity_cmd is set to TRUE, by invoking the setPwmVelocity method, the
   * motor will go to the edge of the motor range using the speed set in setPwmVelocity. The velocity command can be
   * stopped invoking the stopVelocityCmd() method.
   */
  void VelocityCmdThread();
};
END_VISP_NAMESPACE
#endif
#endif
