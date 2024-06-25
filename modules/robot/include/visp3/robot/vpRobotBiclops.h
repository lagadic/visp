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
 * Interface for the Biclops robot.
 */

#ifndef _vpRobotBiclops_h_
#define _vpRobotBiclops_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_BICLOPS) && defined(VISP_HAVE_THREADS)

/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>
#include <thread>
#include <stdio.h>

/* --- ViSP --- */
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobot.h>

/* ------------------------------------------------------------------------ */
/* --- CLASS -------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

BEGIN_VISP_NAMESPACE
/*!
 * \class vpRobotBiclops
 *
 * \ingroup group_robot_real_ptu
 *
 * \brief Interface for the Biclops, pan, tilt head control.
 *
 * Two different models are proposed and can be set using vpBiclops::DenavitHartenbergModel.
 * The vpBiclops::DH1 and vpBiclops::DH2 model differ in the orientation of the tilt axis.
 * The following image gives the location of the end-effector frame and a potential camera frame.
 *
 * \image html img-biclops-frames.jpg Biclops PT models
 *
 * See http://www.traclabs.com/biclopspt.html for more details.
 *
 * This class provide a position and a speed control interface for the Biclops
 * head. To manage the Biclops joint limits in speed control, a control loop is
 * running in a separate thread implemented in vpRobotBiclopsSpeedControlLoop().
 *
 * \warning Velocity control mode is not exported from the top-level Biclops
 * API class provided by Traclabs. That means that there is no protection in
 * this mode to prevent an axis from striking its hard limit. In position mode,
 * Traclabs put soft limits in that keep any command from driving to a position
 * too close to the hard limits. In velocity mode this protection does not
 * exist in the current API.
 *
 * \warning With the understanding that hitting the hard limits at full
 * speed/power can damage the unit, damage due to velocity mode commanding is
 * under user responsibility.
*/
class VISP_EXPORT vpRobotBiclops : public vpBiclops, public vpRobot
{
public:
  static const double defaultPositioningVelocity;

  /*!
   * Default constructor.
   *
   * Does nothing more than setting the default configuration file
   * to `/usr/share/BiclopsDefault.cfg`.
   *
   * As shown in the following example, the turret need to be initialized
   * using init() function.
   *
   * \code
   * #include <visp3/robot/vpRobotBiclops.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   * #ifdef VISP_HAVE_BICLOPS
   *   vpRobotBiclops robot; // Use the default config file in /usr/share/BiclopsDefault.cfg
   *
   *   // Initialize the head
   *   robot.init();
   *
   *   // Move the robot to a specified pan and tilt
   *   robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
   *   vpColVector q(2);
   *   q[0] = vpMath::rad(20); // pan
   *   q[1] = vpMath::rad(40); // tilt
   *   robot.setPosition(vpRobot::JOINT_STATE, q);
   * #endif
   *   return 0;
   * }
   * \endcode
   */
  vpRobotBiclops();

  /*!
   * Constructor that initialize the Biclops pan, tilt head by reading the
   * configuration file provided by Traclabs
   * and do the homing sequence.
   *
   * The following example shows how to use the constructor.
   *
   * \code
   * #include <visp3/robot/vpRobotBiclops.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   * #ifdef VISP_HAVE_BICLOPS
   *   // Specify the config file location and initialize the turret
   *   vpRobotBiclops robot("/usr/share/BiclopsDefault.cfg");
   *
   *   // Move the robot to a specified pan and tilt
   *   robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
   *
   *   vpColVector q(2);
   *   q[0] = vpMath::rad(-20); // pan
   *   q[1] = vpMath::rad(10); // tilt
   *   robot.setPosition(vpRobot::JOINT_STATE, q);
   * #endif
   *   return 0;
   * }
   * \endcode
   */
  VP_EXPLICIT vpRobotBiclops(const std::string &filename);

  /*!
   * Destructor.
   * Wait the end of the control thread.
   */
  virtual ~vpRobotBiclops();

  /*!
   * Set the Biclops config filename.
   * Check if the config file exists and initialize the head.
   *
   * \exception vpRobotException::constructionError If the config file cannot be
   * opened.
   */
  void init() VP_OVERRIDE;

  /*!
   * Get the homogeneous matrix corresponding to the transformation between the
   * camera frame and the end effector frame. The end effector frame is located
   * on the tilt axis.
   *
   * \param cMe : Homogeneous matrix between camera and end effector frame.
   */
  void get_cMe(vpHomogeneousMatrix &cMe) const;

  /*!
   * Get the twist matrix corresponding to the transformation between the
   * camera frame and the end effector frame. The end effector frame is located
   * on the tilt axis.
   *
   * \param cVe : Twist transformation between camera and end effector frame to
   * express a velocity skew from end effector frame in camera frame.
   */
  void get_cVe(vpVelocityTwistMatrix &cVe) const;

  /*!
   * Get the robot jacobian expressed in the end-effector frame.
   *
   * \warning Re is not the embedded camera frame. It corresponds to the frame
   * associated to the tilt axis (see also get_cMe).
   *
   * \param eJe : Jacobian between end effector frame and end effector frame (on
   * tilt axis).
   */
  void get_eJe(vpMatrix &eJe) VP_OVERRIDE;

  /*!
   * Get the robot jacobian expressed in the robot reference frame
   *
   * \param fJe : Jacobian between reference frame (or fix frame) and end
   * effector frame (on tilt axis).
   */
  void get_fJe(vpMatrix &fJe) VP_OVERRIDE;

  /*!
   * Get the robot displacement since the last call of this method.
   *
   * \warning The first call of this method gives not a good value for the
   * displacement.
   *
   * \param frame The frame in which the measured displacement is expressed.
   *
   * \param d The displacement:
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
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &d) VP_OVERRIDE;

  /*!
   * Return the position of each axis.
   * - In positioning control mode, call vpRobotBiclopsController::getPosition()
   * - In speed control mode, call vpRobotBiclopsController::getActualPosition()
   *
   * \param frame : Control frame. This Biclops head can only be controlled in
   * joint state.
   *
   * \param q : The position of the axis in radians.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame type
   * is given.
   */
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;

  /*!
   * Get the velocity in % used for a position control.
   *
   * \return Positioning velocity in [0, 100.0]. The
   * maximum positioning velocity is given vpBiclops::speedLimit.
   */
  double getPositioningVelocity(void);

  /*!
   * Get the joint velocity.
   *
   * \param frame : Control frame. This head can only be controlled in joint state.
   *
   * \param q_dot : The measured joint velocity in rad/s.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame type
   * is given.
   */
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot);

  /*!
   * Return the joint velocity.
   *
   * \param frame : Control frame. This head can only be controlled in joint state.
   *
   * \return The measured joint velocity in rad/s.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame type
   * is given.
   */
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);

  /*!
   * Get joint positions from the position file.
   *
   * \param filename : Position file.
   *
   * \param q : The joint positions read in the file.
   *
   * \code
   * # Example of Biclops position file
   * # The axis positions must be preceeded by R:
   * # First value : pan  joint position in degrees
   * # Second value: tilt joint position in degrees
   * R: 15.0 5.0
   * \endcode
   *
   * \return true if a position was found, false otherwise.
   */
  bool readPositionFile(const std::string &filename, vpColVector &q);

  /*!
   * Set the Biclops config filename.
   */
  void setConfigFile(const std::string &filename = "/usr/share/BiclopsDefault.cfg");

  /*!
   * Move the robot in position control.
   *
   * \warning This method is blocking. That mean that it waits the end of the
   * positioning.
   *
   * \param frame : Control frame. This Biclops head can only be controlled in
   * joint state.
   *
   * \param q : The joint position to set for each axis in radians.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame
   * type is given.
   */
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;

  /*!
   * Move the robot in position control.
   *
   * \warning This method is blocking. That mean that it wait the end of the
   * positioning.
   *
   * \param frame : Control frame. This Biclops head can only be controlled in
   * joint state.
   *
   * \param q1 : The pan joint position to set in radians.
   * \param q2 : The tilt joint position to set in radians.
   *
   * \exception vpRobotException::wrongStateError : If a not supported frame
   * type is given.
   */
  void setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2);

  /*!
   * Read the content of the position file and moves the head to joint
   * positions.
   *
   * \param filename : Position filename
   *
   * \exception vpRobotException::readingParametersError : If the joint
   * positions cannot be read from file.
   *
   * \sa readPositionFile()
   */
  void setPosition(const std::string &filename);

  /*!
   * Set the velocity used for a position control.
   *
   * \param velocity : Velocity in % of the maximum velocity between [0,100]. The
   * maximum velocity is given vpBiclops::speedLimit.
   */
  void setPositioningVelocity(double velocity);

  /*!
   * Change the state of the robot either to stop them, or to set position or
   * speed control.
   */
  vpRobot::vpRobotStateType setRobotState(const vpRobot::vpRobotStateType newState) VP_OVERRIDE;

  /*!
   * Send a velocity on each axis.
   *
   * \param frame : Control frame. This Biclops head can only be controlled in
   * joint state. Be aware, the camera frame (vpRobot::CAMERA_FRAME), the reference
   * frame (vpRobot::REFERENCE_FRAME), end-effector frame (vpRobot::END_EFFECTOR_FRAME)
   * and the mixt frame (vpRobot::MIXT_FRAME) are not implemented.
   *
   * \param q_dot : The desired joint velocities for each axis in rad/s. \f$ \dot
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
   * Halt all the axis.
   */
  void stopMotion();

  /*
   * Control loop to manage the Biclops joint limits in speed control.
   *
   * This control loop is running in a separate thread in order to detect each 5
   * ms joint limits during the speed control. If a joint limit is detected the
   * axis should be halted.
   *
   * \warning Velocity control mode is not exported from the top-level Biclops
   * API class provided by Traclabs. That means that there is no protection in
   * this mode to prevent an axis from striking its hard limit. In position mode,
   * Traclabs put soft limits in that keep any command from driving to a position
   * too close to the hard limits. In velocity mode this protection does not
   * exist in the current API.
   *
   * \warning With the understanding that hitting the hard limits at full
   * speed/power can damage the unit, damage due to velocity mode commanding is
   * under user responsibility.
   */
  static void vpRobotBiclopsSpeedControlLoop(void *arg);

private:
  std::thread m_control_thread;

  std::string m_configfile; // Biclops config file

  class vpRobotBiclopsController;
  vpRobotBiclopsController *m_controller;

  double m_positioningVelocity;
  vpColVector m_q_previous;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  /*! \brief No copy constructor allowed.   */
  //  vpRobotBiclops(const vpRobotBiclops &)
  //    : vpBiclops(), vpRobot(), m_control_thread(), m_controller(),
  //      m_positioningVelocity(0), m_q_previous()
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpRobotBiclops &operator=(const vpRobotBiclops &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif
};
END_VISP_NAMESPACE
#endif /* #ifndef _vpRobotBiclops_h_ */

#endif
