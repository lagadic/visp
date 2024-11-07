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

#ifndef _vpRobotPololuPtu_h_
#define _vpRobotPololuPtu_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_POLOLU) && defined(VISP_HAVE_THREADS)

#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpPololu.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpRobotPololuPtu
 * \ingroup group_robot_real_arm
 *
 * \brief Interface for the Pololu Maestro pan-tilt unit using two servo motors.
 *
 * See https://www.pololu.com/category/102/maestro-usb-servo-controllers for more details.
 *
 * This class handle the vpPololu class in a higher level and allows to control
 * the pan-tilt unit using position or velocity commands.
 *
 * The corresponding Denavit-Hartenberg representations of the PTU is the following:
 *
 * | Joint | \f$a_i\f$ | \f$d_i\f$ | \f$\alpha_i\f$ | \f$\theta_i\f$    |
 * | :---: | :-------: | :-------: | -------------: | ----------------: |
 * |     1 |         0 |         0 |   \f$ \pi/2\f$ |         \f$q_1\f$ |
 * |     2 |         0 |         0 |   \f$-\pi/2\f$ | \f$q_2 - \pi/2\f$ |
*/
class VISP_EXPORT vpRobotPololuPtu : public vpRobot
{
public:
  /*!
   * Default constructor.
   *
   * \param[in] device : Name of the serial interface used for communication.
   * \param[in] baudrate : Baudrate used for the serial communication. Note that this parameter is only used on Windows.
   * \param[in] verbose : When true, enable verbose mode.
   */
  vpRobotPololuPtu(const std::string &device = "/dev/ttyACM0", int baudrate = 9600, bool verbose = false);

  /*!
   * Destructor that stops the movements.
   */
  ~vpRobotPololuPtu() VP_OVERRIDE;

  /*!
   * Get the robot jacobian expressed in the end-effector frame.
   *
   * \warning End-effector frame is not the embedded camera frame. It corresponds to the frame
   * associated to the tilt axis (see also get_cMe).
   *
   * \param[out] eJe : Jacobian between end effector frame and end effector frame (on
   * tilt axis).
   */
  void get_eJe(vpMatrix &eJe) VP_OVERRIDE;

  /*!
   * Get the robot jacobian expressed in the end-effector frame.
   *
   * \warning End-effector frame is not the embedded camera frame. It corresponds to the frame
   * associated to the tilt axis (see also get_cMe).
   *
   * \param[in] q : Joint positions to consider [rad].
   *
   * \param[out] eJe : Jacobian between end effector frame and end effector frame (on
   * tilt axis).
   */
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const;

  /*!
   * Get the robot jacobian expressed in the robot reference frame.
   *
   * \param[out] fJe : Jacobian between reference frame (or fix frame) and end
   * effector frame (on tilt axis).
   */
  void get_fJe(vpMatrix &fJe) VP_OVERRIDE;

  /*!
   * Get the robot jacobian expressed in the robot reference frame.
   *
   * \param[in] q : Joint positions to consider [rad].
   *
   * \param[out] fJe : Jacobian between reference frame (or fix frame) and end
   * effector frame (on tilt axis).
   */
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const;

  /*!
   * Return the minimul angular velocity in rad/s that could be applied to move the motors.
   * It corresponds to 1 pwm converted in rad/s.
   */
  float getAngularVelocityResolution() const;

  /*!
   * Return the position of each joint.
   *
   * \param[in] frame : Control frame. This PTU can only be controlled in
   * joint state.
   *
   * \param[out] q : The position of the joints in radians.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame type
   * is given.
   */
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;

  /*!
   * Get the percentage of the maximum velocity applied to move the PTU in position.
   *
   * \return Positioning velocity percentage in [0, 100.0]. The
   * maximum positioning velocity is given vpRobot::getMaxRotationVelocity().
   *
   * \sa setPositioningVelocityPercentage()
   */
  float getPositioningVelocityPercentage() const
  {
    return m_positioning_velocity_percentage;
  }

  /*!
   * Move the robot to a given joint position.
   *
   * \warning This method is blocking. That mean that it waits the end of the
   * positioning.
   *
   * \param[in] frame : Control frame. This PTU can only be controlled in
   * joint state.
   *
   * \param[in] q : The joint position to set for each axis in radians.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame
   * type is given.
   */
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;

  /*!
   * Set the percentage of the maximum velocity applied to move the PTU in position.
   *
   * \param[in] positioning_velocity_percentage : Percentage between [0,100] of the maximum velocity. The
   * maximum positioning velocity is given vpRobot::getMaxRotationVelocity().
   *
   * \sa getPositioningVelocityPercentage()
   */
  void setPositioningVelocityPercentage(float positioning_velocity_percentage)
  {
    m_positioning_velocity_percentage = positioning_velocity_percentage;
  }

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
   * Send a velocity on each axis.
   *
   * \param[in] frame : Control frame. This Biclops head can only be controlled in
   * joint state. Be aware, the camera frame (vpRobot::CAMERA_FRAME), the reference
   * frame (vpRobot::REFERENCE_FRAME), end-effector frame (vpRobot::END_EFFECTOR_FRAME)
   * and the mixt frame (vpRobot::MIXT_FRAME) are not implemented.
   *
   * \param[in] q_dot : The desired joint velocities for each axis in rad/s. \f$ \dot
   * {r} = [\dot{q}_1, \dot{q}_2]^t \f$ with \f$ \dot{q}_1 \f$ the pan of the
   * camera and \f$ \dot{q}_2\f$ the tilt of the camera.
   *
   * \exception vpRobotException::wrongStateError : If a the robot is not
   * configured to handle a velocity. The robot can handle a velocity only if the
   * velocity control mode is set. For that, call setRobotState(
   * vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame type
   * (vpRobot::CAMERA_FRAME, vpRobot::REFERENCE_FRAME, vpRobot::END_EFFECTOR_FRAME
   * or vpRobot::MIXT_FRAME) is given.
   *
   * \warning Velocities could be saturated if one of them exceed the maximal
   * authorized speed (see vpRobot::maxRotationVelocity).
   */
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot) VP_OVERRIDE;

  /*!
   * Stop the velocity command.
   */
  void stopVelocity();

  /*!
   * Change the state of the robot either to stop them, or to set position or
   * speed control.
   */
  vpRobot::vpRobotStateType setRobotState(const vpRobot::vpRobotStateType newState) VP_OVERRIDE;

private:
  /*!
   * Initialize the robot.
   *
   * \exception vpRobotException::constructionError If the config file cannot be
   * opened.
   */
  void init() VP_OVERRIDE { };

  /*!
   * Get the robot displacement since the last call of this method.
   *
   * \warning The first call of this method gives not a good value for the
   * displacement.
   *
   * \param[in] frame The frame in which the measured displacement is expressed.
   *
   * \param[out] d The displacement:
   *
   * - In joint state, the dimension of q is 2  (the number of axis of the robot)
   *   with respectively d[0] (pan displacement), d[1] (tilt displacement).
   *
   * - In camera frame, the dimension of d is 6 (tx, ty, ty, tux, tuy, tuz).
   *   Translations are expressed in meters, rotations in radians with the theta U
   *   representation.
   *
   * \exception vpRobotException::wrongStateError If a not supported frame type
   * is given.
   */
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &d) VP_OVERRIDE
  {
    (void)frame;
    (void)d;
  };

  vpPololu m_pan;
  vpPololu m_tilt;
  float m_positioning_velocity_percentage;

  bool m_verbose;
};
END_VISP_NAMESPACE
#endif
#endif
