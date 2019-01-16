/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Interface for Pioneer robots based on Aria 3rd party library.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotPioneer.h>
// Warning: vpMath.h should be included after Aria.h to avoid the build issue:
// "/usr/include/Aria/ariaUtil.h:732:21: error: ‘isfinite’ was not declared
// in this scope"
// This error is due to cmath header included from vpMath.h that makes
// isfinite() ambiguous between ::isfinite() and std::isfinite()
#include <visp3/core/vpMath.h>

#ifdef VISP_HAVE_PIONEER

/*!
  Default constructor that initializes Aria.
  */
vpRobotPioneer::vpRobotPioneer() : vpPioneer(), ArRobot()
{
  isInitialized = false;

  Aria::init();
}

/*!
  Destructor.
  */
vpRobotPioneer::~vpRobotPioneer()
{
#if 0
  std::cout << "Ending robot thread..." << std::endl;
  stopRunning();

  // wait for the thread to stop
  waitForRunExit();
#endif
}

/*!
  Set the velocity (frame has to be specified) that will be applied to the
  robot.

  \param frame : Control frame. For the moment, only vpRobot::ARTICULAR_FRAME
  to control left and right wheel velocities and vpRobot::REFERENCE_FRAME to
  control translational and rotational velocities are implemented.

  \param vel : A two dimension vector that corresponds to the velocities to
  apply to the robot.
  - If the frame is vpRobot::ARTICULAR_FRAME, first value is the velocity of
  the left wheel and second value is the velocity of the right wheel in m/s.
  In that case sets the velocity of the wheels independently.
  - If the frame is vpRobot::REFERENCE_FRAME, first value is the translation
  velocity in m/s. Second value is the rotational velocity in rad/s along the
  vertical axis.

  Note that to secure the usage of the robot, velocities are saturated to the
  maximum allowed which can be obtained by getMaxTranslationVelocity() and
  getMaxRotationVelocity(). To change the default values, use
  setMaxTranslationVelocity() and setMaxRotationVelocity().

  \exception vpRobotException::dimensionError : Velocity vector is not a 2
  dimension vector. \exception vpRobotException::wrongStateError : If the
  specified control frame is not supported.
  */
void vpRobotPioneer::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  init();

  /*
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ()) {
    vpERROR_TRACE ("Cannot send a velocity to the robot "
       "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException (vpRobotException::wrongStateError,
          "Cannot send a velocity to the robot "
          "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  } */

  if (vel.size() != 2) {
    throw(vpRobotException(vpRobotException::dimensionError, "Velocity vector is not a 2 dimension vector"));
  }

  vpColVector vel_max(2);
  vpColVector vel_sat;

  if (frame == vpRobot::REFERENCE_FRAME) {
    vel_max[0] = getMaxTranslationVelocity();
    vel_max[1] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
    this->lock();
    this->setVel(vel_sat[0] * 1000.);         // convert velocity in mm/s
    this->setRotVel(vpMath::deg(vel_sat[1])); // convert velocity in deg/s
    this->unlock();
  } else if (frame == vpRobot::ARTICULAR_FRAME) {
    vel_max[0] = getMaxTranslationVelocity();
    vel_max[1] = getMaxTranslationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
    this->lock();
    // std::cout << "v: " << (vel*1000).t() << " mm/s" << std::endl;
    this->setVel2(vel_sat[0] * 1000.,
                  vel_sat[1] * 1000.); // convert velocity in mm/s
    this->unlock();
  } else {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send the robot velocity in the specified control frame");
  }
}

/*!
  Initialize the robot.
  - Sets the robot in asynchronous mode by starting a low level thread. The
  robot will be stopped if there is no connection to the robot at any given
  point.
  - Enables the motors on the robot, if it is connected.

  */
void vpRobotPioneer::init()
{
  if (!isInitialized) {
    // Start the robot processing cycle running in the background.
    // True parameter means that if the connection is lost, then the
    // run loop ends.
    this->runAsync(true);
    this->lock();
    this->enableMotors();
    this->unlock();

    isInitialized = true;
  }
}

/*!
  Gets the current translational velocity of the robot.

  \param frame : Control frame. For the moment, only vpRobot::ARTICULAR_FRAME
  to get left and right wheel velocities and vpRobot::REFERENCE_FRAME to get
  translational and rotational velocities are implemented.

  \param velocity : A two dimension vector that corresponds to the current
  velocities applied to the robot.
  - If the frame is vpRobot::ARTICULAR_FRAME, first value is the velocity of
  the left wheel and second value is the velocity of the right wheel in m/s.
  - If the frame is vpRobot::REFERENCE_FRAME, first value is the translation
  velocity in m/s. Second value is the rotational velocity in rad/s.

  \exception vpRobotException::dimensionError : Velocity vector is not a 2
  dimension vector. \exception vpRobotException::wrongStateError : If the
  specified control frame is not supported.

  \sa getVelocity(const vpRobot::vpControlFrameType)
  */
void vpRobotPioneer::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity)
{
  init();
  velocity.resize(2);

  if (frame == vpRobot::ARTICULAR_FRAME) {
    this->lock();
    velocity[0] = this->getLeftVel() / 1000.;
    velocity[1] = this->getRightVel() / 1000;
    this->unlock();
  } else if (frame == vpRobot::REFERENCE_FRAME) {
    this->lock();
    velocity[0] = this->getVel() / 1000.;
    velocity[1] = vpMath::rad(this->getRotVel());
    this->unlock();
  } else {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot get the robot volocity in the specified control frame");
  }
}

/*!
  Gets the current translational velocity of the robot.

  \param frame : Control frame. For the moment, only vpRobot::ARTICULAR_FRAME
  to get left and right wheel velocities and vpRobot::REFERENCE_FRAME to get
  translational and rotational velocities are implemented.

  \return A two dimension vector that corresponds to the current velocities
  applied to the robot.
  - If the frame is vpRobot::ARTICULAR_FRAME, first value is the velocity of
  the left wheel and second value is the velocity of the right wheel in m/s.
  - If the frame is vpRobot::REFERENCE_FRAME, first value is the translation
  velocity in m/s. Second value is the rotational velocity in rad/s.

  \exception vpRobotException::dimensionError : Velocity vector is not a 2
  dimension vector. \exception vpRobotException::wrongStateError : If the
  specified control frame is not supported.

  \sa getVelocity(const vpRobot::vpControlFrameType, vpColVector &)
  */
vpColVector vpRobotPioneer::getVelocity(const vpRobot::vpControlFrameType frame)
{
  vpColVector velocity;
  getVelocity(frame, velocity);
  return velocity;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotPioneer.cpp.o) has no
// symbols
void dummy_vpRobotPioneer(){};
#endif
