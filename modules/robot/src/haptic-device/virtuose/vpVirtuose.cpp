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
 * Description: Class which enables to project an image in the 3D space
 * and get the view of a virtual camera.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpVirtuose.cpp
  \brief Wrapper over Haption Virtuose SDK to control haptic devices.
*/

#include <visp3/core/vpException.h>
#include <visp3/robot/vpVirtuose.h>

#ifdef VISP_HAVE_VIRTUOSE

/*!
 * Default constructor.
 * Set command type to virtual mechanism by default (impedance mode).
 * Authorize indexing on all movements by default.
 */
vpVirtuose::vpVirtuose()
  : m_virtContext(NULL), m_ip("localhost#5000"), m_verbose(false), m_apiMajorVersion(0), m_apiMinorVersion(0),
    m_ctrlMajorVersion(0), m_ctrlMinorVersion(0), m_typeCommand(COMMAND_TYPE_IMPEDANCE), m_indexType(INDEXING_ALL),
    m_is_init(false), m_period(0.001f), m_njoints(6)
{
  virtAPIVersion(&m_apiMajorVersion, &m_apiMinorVersion);
  std::cout << "API version: " << m_apiMajorVersion << "." << m_apiMinorVersion << std::endl;
}

/*!
 * Default destructor that delete the VirtContext object.
 */
void vpVirtuose::close()
{
  if (m_virtContext != NULL) {
    virtClose(m_virtContext);
    m_virtContext = NULL;
  }
}

/*!
 * Default destructor that delete the VirtContext object.
 */
vpVirtuose::~vpVirtuose()
{
  close();
}

/*!
 * Add a force to be applied to the virtuose (impedance effort).
 * This function works in every mode.
 * \param force : Is 6 component dynamic tensor (three forces and three
 * torques) wrt virtuose end-effector and is expressed in the coordinates of
 * the base frame.
 */
void vpVirtuose::addForce(vpColVector &force)
{
  if (force.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply a force feedback (dim %d) to the haptic "
                      "device that is not 6-dimension",
                      force.size()));
  }

  init();

  float virtforce[6];
  for (unsigned int i = 0; i < 6; i++)
    virtforce[i] = (float)force[i];

  if (virtAddForce(m_virtContext, virtforce)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtAddForce: error code %d", err));
  }
}

/*!
 * Activate or desactivate force feedback.
 * \param enable : 1 to activate (system's default value), 0 to desactivate.
 */
void vpVirtuose::enableForceFeedback(int enable)
{
  init();

  if (virtEnableForceFeedback(m_virtContext, enable)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtEnableForceFeedback(): error code %d", err));
  }
}

/*!
 * Return the 6 joint values of the virtuose.
 */
vpColVector vpVirtuose::getArticularPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }


  float articular_position_[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  if (virtGetArticularPosition(m_virtContext, articular_position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetArticularPosition(): error code %d", err));
  }

  vpColVector articularPosition(m_njoints, 0);
  for (unsigned int i = 0; i < m_njoints; i++)
    articularPosition[i] = articular_position_[i];

  return articularPosition;
}

/*!
 * Return the 6 joint velocities of the virtuose.
 */
vpColVector vpVirtuose::getArticularVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  float articular_velocity_[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  if (virtGetArticularSpeed(m_virtContext, articular_velocity_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetArticularSpeed: error code %d", err));
  }

  vpColVector articularVelocity(m_njoints, 0);

  for (unsigned int i = 0; i < m_njoints; i++)
    articularVelocity[i] = articular_velocity_[i];

  return articularVelocity;
}

/*!
 * Return the indexed position of the end-effector, expressed in the
 * coordinates of the environment reference frame. With respect to the
 * function getPosition(), getAvatarPosition() takes into account current
 * offsets (indexing) and motor scale factors. \sa getPosition(),
 * getPhysicalPosition()
 */
vpPoseVector vpVirtuose::getAvatarPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  float position_[7];
  vpPoseVector position;
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetAvatarPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetAvatarPosition: error code %d", err));
  } else {
    for (int i = 0; i < 3; i++)
      translation[i] = position_[i];
    for (int i = 0; i < 4; i++)
      quaternion[i] = position_[3 + i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);

    return position;
  }
}

/*!
 * Return the current position of the base frame
 * with respect to the observation reference frame.
 *
 * \sa setBaseFrame(), getObservationFrame()
 */
vpPoseVector vpVirtuose::getBaseFrame() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetBaseFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetBaseFrame: error code %d", err));
  } else {
    for (int i = 0; i < 3; i++)
      translation[i] = position_[i];
    for (int i = 0; i < 4; i++)
      quaternion[i] = position_[3 + i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);

    return position;
  }
}

/*!
 * Return the current command type.
 */
VirtCommandType vpVirtuose::getCommandType() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  VirtCommandType type;

  if (virtGetCommandType(m_virtContext, &type)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetCommandType: error code %d", err));
  }
  return type;
}

/*!
 * Return the status of DeadMan sensor : true if the sensor is ON (a user is
 * holding the handle) and false if the sensor is OFF (no user detected).
 */
bool vpVirtuose::getDeadMan() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  int deadman;
  if (virtGetDeadMan(m_virtContext, &deadman)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetDeadMan: error code %d", err));
  }
  return (deadman ? true : false);
}

/*!
 * Return the status of the emergency stop button : true if the system is
 * operational (button correctly plugged and not triggered) and false if the
 * system is not operational (button not plugged or triggered).
 */
bool vpVirtuose::getEmergencyStop() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  int emergencyStop;
  if (virtGetEmergencyStop(m_virtContext, &emergencyStop)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetEmergencyStop: error code %d", err));
  }
  return (emergencyStop ? true : false);
}

/*!
 * Return the 6-dimension force tensor to be applied to the object attached to
 * the Virtuose, allowing the dynamic simulation of the scene.
 */
vpColVector vpVirtuose::getForce() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector force(6, 0);
  float force_[6];
  if (virtGetForce(m_virtContext, force_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetForce: error code %d", err));
  }

  for (unsigned int i = 0; i < 6; i++)
    force[i] = force_[i];
  return force;
}

/*!
  Return the handler used to communicate with the device.
  This function could be used to access to a functionality that is not
  implemented in vpVirtuose class.

  The following sample code shows how to use this function to get the
  device joint positions. This functionality is already implemented
  in getArticularPosition().
  \code
#include <visp3/robot/vpVirtuose.h>

int main()
{
  vpVirtuose virtuose;

  virtuose.init();

  VirtContext handler = virtuose.getHandler();
  float q[6];
  if (virtGetArticularPosition(handler, q)) { // Use the handler to access to Haption API directly
    std::cout << "Cannot get articular position" << std::endl;
  }
  std::cout << "Joint position: ";
  for (unsigned int i=0; i<6; i++)
    std::cout << q[i] << " ";
  std::cout << std::endl;
}
  \endcode

  \sa getArticularPosition()

 */
VirtContext vpVirtuose::getHandler() { return m_virtContext; }

/*!
  Get device number of joints.

  \return The number of joints of the device. Sould be 6 for Virtuose, 9 for the glove fingers.
 */
unsigned int vpVirtuose::getJointsNumber() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  return m_njoints;
}


/*!
 * Return the cartesian current position of the observation reference frame
 * with respect to the environment reference frame.
 *
 * \sa setObservationFrame(), getBaseFrame()
 */
vpPoseVector vpVirtuose::getObservationFrame() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetObservationFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetObservationFrame: error code %d", err));
  } else {
    for (int i = 0; i < 3; i++)
      translation[i] = position_[i];
    for (int i = 0; i < 4; i++)
      quaternion[i] = position_[3 + i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return the cartesian physical position of the Virtuose expressed in the
 * coordinates of the base reference frame.
 *
 * \sa getAvatarPosition(), getPosition()
 */
vpPoseVector vpVirtuose::getPhysicalPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetPhysicalPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetPhysicalPosition: error code %d", err));
  } else {
    for (int i = 0; i < 3; i++)
      translation[i] = position_[i];
    for (int i = 0; i < 4; i++)
      quaternion[i] = position_[3 + i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return the physical cartesian velocity twist of the Virtuose expressed in
 * the coordinates of the base reference frame. \return A 6-dimension velocity
 * twist with 3 translation velocities and 3 rotation velocities. \sa
 * getVelocity()
 */
vpColVector vpVirtuose::getPhysicalVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector vel(6, 0);
  float speed[6];
  if (virtGetPhysicalSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetPhysicalSpeed: error code %s",
                      virtGetErrorMessage(err)));
  }
  for (unsigned int i = 0; i < 6; i++)
    vel[i] = speed[i];
  return vel;
}

/*!
 * Return the cartesian position of the virtuose (or the object attached to
 * it, if any) expressed in the coordinates of the environment reference
 * frame. \sa getAvatarPosition(), getPhysicalPosition()
 */
vpPoseVector vpVirtuose::getPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetPosition: error code %d", err));
  } else {
    for (int i = 0; i < 3; i++)
      translation[i] = position_[i];
    for (int i = 0; i < 4; i++)
      quaternion[i] = position_[3 + i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return status of the motors : true if motors are ON, false otherwise.
 */
bool vpVirtuose::getPower() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  int power;
  virtGetPowerOn(m_virtContext, &power);
  return (power ? true : false);
}

/*!
 * Return the cartesian velocity twist of the virtuose (or the object attached
 * to it, if any) expressed in the coordinates of the environment reference
 * frame. \return A 6-dimension velocity twist with 3 translation velocities
 * and 3 rotation velocities. \sa getPhysicalVelocity()
 */
vpColVector vpVirtuose::getVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector vel(6, 0);
  float speed[6];
  if (virtGetSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Cannot get haptic device velocity: %s", virtGetErrorMessage(err)));
  }
  for (unsigned int i = 0; i < 6; i++)
    vel[i] = speed[i];
  return vel;
}

/*!
 * Initialize virtuose device opening the connection to the device and setting
 * the default command type. If the device is already initialized, a call to
 * init() does nothing.
 */
void vpVirtuose::init()
{
  if (!m_is_init) {
    m_virtContext = virtOpen(m_ip.c_str());

    if (m_virtContext == NULL) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Cannot open haptic device: %s", virtGetErrorMessage(err)));
    }

    if (virtGetControlerVersion(m_virtContext, &m_ctrlMajorVersion, &m_ctrlMinorVersion)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Cannot get haptic device controller version: %s",
                        virtGetErrorMessage(err)));
    }

    if (m_verbose) {
      std::cout << "Controller version: " << m_ctrlMajorVersion << "." << m_ctrlMinorVersion << std::endl;
    }

    if (virtSetCommandType(m_virtContext, m_typeCommand)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(
          vpException(vpException::fatalError, "Cannot set haptic device command type: %s", virtGetErrorMessage(err)));
    }

    if (virtSetTimeStep(m_virtContext, m_period)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Error calling virtSetTimeStep: error code %d", err));
    }

    // Update number of joints
    float articular_position_[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if (virtGetArticularPosition(m_virtContext, articular_position_)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Error calling virtGetArticularPosition() in int(): error code %d", err));
    }

    m_njoints = 6; // At least 6 joints
    for (unsigned int i=m_njoints; i < 20; i++) {
      m_njoints = i;
      if (std::fabs(articular_position_[i]) <= std::numeric_limits<float>::epsilon()) {
        break;
      }
    }

    m_is_init = true;
  }
}

/*!
 * Send a command of articular force to the Virtuose.
 * setArticularForce() only works in mode COMMAND_TYPE_ARTICULAR_IMPEDANCE, to
 * be set with setCommandType().
 *
 * \param articularForce : Six dimension torque vector.
 */
void vpVirtuose::setArticularForce(const vpColVector &articularForce)
{
  init();

  if (articularForce.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply an articular force feedback (dim %d) to "
                      "the haptic device that is not 6-dimension",
                      articularForce.size()));
  }

  float articular_force[6];
  for (unsigned int i = 0; i < 6; i++)
    articular_force[i] = (float)articularForce[i];

  if (virtSetArticularForce(m_virtContext, articular_force)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetArticularForce: error code %d", err));
  }
}

/*!
 * Send a command of articular (joint) position to the virtuose.
 * This function works only in COMMAND_TYPE_ARTICULAR mode, to be set with
 * setCommandType().
 *
 * \param articularPosition : Six dimension joint position vector.
 */
void vpVirtuose::setArticularPosition(const vpColVector &articularPosition)
{
  init();

  if (articularPosition.size() != m_njoints) {
    throw(vpException(vpException::dimensionError,
                      "Cannot send an articular position command (dim %d) to "
                      "the haptic device that is not %d-dimension",
                      m_njoints, articularPosition.size()));
  }

  float *articular_position = new float[m_njoints];
  for (unsigned int i = 0; i < m_njoints; i++)
    articular_position[i] = (float)articularPosition[i];

  if (virtSetArticularPosition(m_virtContext, articular_position)) {
    int err = virtGetErrorCode(m_virtContext);
    delete [] articular_position;
    throw(vpException(vpException::fatalError, "Error calling virtSetArticularPosition: error code %d", err));
  }
  delete[] articular_position;
}

/*!
 * Send a command of articular (joint) velocity to the virtuose.
 * This function works only in COMMAND_TYPE_ARTICULAR mode, to be set with
 * setCommandType().
 *
 * \param articularVelocity : Six dimension joint velocity vector.
 */
void vpVirtuose::setArticularVelocity(const vpColVector &articularVelocity)
{
  init();

  if (articularVelocity.size() != m_njoints) {
    throw(vpException(vpException::dimensionError,
                      "Cannot send an articular velocity command (dim %d) to "
                      "the haptic device that is not %d-dimension",
                      m_njoints, articularVelocity.size()));
  }

  float *articular_velocity = new float [m_njoints];
  for (unsigned int i = 0; i < m_njoints; i++)
    articular_velocity[i] = (float)articularVelocity[i];

  if (virtSetArticularSpeed(m_virtContext, articular_velocity)) {
    int err = virtGetErrorCode(m_virtContext);
    delete [] articular_velocity;
    throw(vpException(vpException::fatalError, "Error calling virtSetArticularVelocity: error code %d", err));
  }
  delete[] articular_velocity;
}

/*!
 * Move the base frame with respect to the observation reference frame.
 * It can be called at any time.
 * \param position : Position of the base frame.
 *
 * \sa getBaseFrame(), setObservationFrame()
 */
void vpVirtuose::setBaseFrame(const vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i = 0; i < 3; i++)
    position_[i] = (float)translation[i];
  for (int i = 0; i < 4; i++)
    position_[3 + i] = (float)quaternion[i];

  if (virtSetBaseFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetBaseFrame: error code %d", err));
  }
}

/*!
 * Set the command type.
 * \param type : Possible values:
 * - COMMAND_TYPE_NONE : No possible movement.
 * - COMMAND_TYPE_IMPEDANCE : Force/position control.
 * - COMMAND_TYPE_VIRTMECH : Position/force control with virtual mechanism.
 * - COMMAND_TYPE_ARTICULAR : Joint control.
 * - COMMAND_TYPE_ARTICULAR_IMPEDANCE : Force/Position control in the joint
 * space.
 */
void vpVirtuose::setCommandType(const VirtCommandType &type)
{
  init();

  if (m_typeCommand != type) {
    m_typeCommand = type;

    if (virtSetCommandType(m_virtContext, m_typeCommand)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Error calling virtSetCommandType: error code %d", err));
    }
  }
}

/*!
 * Set the force to be applied by the Virtuose.
 * setForce() only works in COMMAND_TYPE_IMPEDANCE mode, to be set with
 * setCommandType(). \param force : Force vector that represents a dynamic
 * tensor with 6 components.
 */
void vpVirtuose::setForce(const vpColVector &force)
{
  init();

  if (force.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply a force feedback (dim %d) to the haptic "
                      "device that is not 6-dimension",
                      force.size()));
  }

  float virtforce[6];
  for (unsigned int i = 0; i < 6; i++)
    virtforce[i] = (float)force[i];

  if (virtSetForce(m_virtContext, virtforce)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetForce: error code %d", err));
  }
}

/*!
 * Set the force scale factor.
 * \param forceFactor : Force scale factor applied to the force torque tensor
 * set by setForce().
 */
void vpVirtuose::setForceFactor(const float &forceFactor)
{
  init();

  if (virtSetForceFactor(m_virtContext, forceFactor)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetForceFactor: error code %d", err));
  }
}

/*!
 * Set indexing (offset) mode.
 * \param type : Possible choices are:
 * - INDEXING_ALL : authorize indexing on all movements.
 * - INDEXING_TRANS : authorize indexing on translation, i.e., the orientation
 * of the object is always identical to the orientation of the device
 * end-effector.
 * - INDEXING_NONE : forbids indexing on all movements.
 * - The following values are also implemented:
 * INDEXING_ALL_FORCE_FEEDBACK_INHIBITION,
 * INDEXING_TRANS_FORCE_FEEDBACK_INHIBITION. These values correspond to the
 * same modes listed before, but the force feedback is inhibited during
 * indexing.
 */
void vpVirtuose::setIndexingMode(const VirtIndexingType &type)
{
  init();

  if (m_indexType != type) {
    m_indexType = type;

    if (virtSetIndexingMode(m_virtContext, m_indexType)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Error calling setIndexingMode: error code %d", err));
    }
  }
}

/*!
 * Move the observation frame with respect to the environment reference frame.
 * It can be called at any time.
 * \param position : Position of the observation frame.
 *
 * \sa getObservationFrame(), setBaseFrame()
 */
void vpVirtuose::setObservationFrame(const vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i = 0; i < 3; i++)
    position_[i] = (float)translation[i];
  for (int i = 0; i < 4; i++)
    position_[3 + i] = (float)quaternion[i];

  if (virtSetObservationFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetObservationFrame: error code %d", err));
  }
}

/*!
 * Register the periodic function.
 * setPeriodicFunction() defines a callback function to be called at a fixed
period of time, as timing for the simulation.
 * The callback function is synchronized with the Virtuose controller
(messages arrive at very constant time intervals from it)
 * and generates hardware interrupts to be taken into account by the operating
system.
 * In practice, this function is much more efficient for timing the simulation
than common software timers.
 * This function is started using startPeriodicFunction() and stopped using
stopPeriodicFunction().
 * \param CallBackVirt : Callback function.
 *
 * Example of the use of the periodic function:
 \code
#include <visp3/robot/vpVirtuose.h>

void CallBackVirtuose(VirtContext VC, void* ptr)
{
  (void) VC;
  vpVirtuose* p_virtuose=(vpVirtuose*)ptr;
  vpPoseVector position = p_virtuose->getPosition();
  return;
}

int main()
{
  vpVirtuose virtuose;
  float period = 0.001;
  virtuose.setTimeStep(period);
  virtuose.setPeriodicFunction(CallBackVirtuose, period, virtuose);
  virtuose.startPeriodicFunction();
  virtuose.stopPeriodicFunction();
}
 \endcode

 \sa startPeriodicFunction(), stopPeriodicFunction()
 */
void vpVirtuose::setPeriodicFunction(VirtPeriodicFunction CallBackVirt)
{
  init();

  if (virtSetPeriodicFunction(m_virtContext, CallBackVirt, &m_period, this)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetPeriodicFunction: error code %d", err));
  }
}

/*!
 * Modify the current value of the control position and send it to the
 * Virtuose. \param position : Position of the end-effector (or the attached
 * object, if any).
 */
void vpVirtuose::setPosition(vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i = 0; i < 3; i++)
    position_[i] = (float)translation[i];
  for (int i = 0; i < 4; i++)
    position_[3 + i] = (float)quaternion[i];

  if (virtSetPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetPosition: error code %d", err));
  }
}

/*!
 * Turn the motor power OFF.
 */
void vpVirtuose::setPowerOff()
{
  init();

  if (virtSetPowerOn(m_virtContext, 0)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetPowerOff: error code %d", err));
  }
}

/*!
 * Turn the motor power ON.
 */
void vpVirtuose::setPowerOn()
{
  init();

  if (virtSetPowerOn(m_virtContext, 1)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetPowerOn: error code %d", err));
  }
}

/*!
 * Set saturation values of the force feedback
 * \param forceLimit : Value expressed in N.
 * \param torqueLimit : Value expressed in Nm.
 */
void vpVirtuose::setSaturation(const float &forceLimit, const float &torqueLimit)
{
  init();

  if (virtSaturateTorque(m_virtContext, forceLimit, torqueLimit)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSaturateTorque: error code %d", err));
  }
}

/*!
 * Set the the simulation time step.
 * The function must be called before the selection of the type of control
 * mode. \param timeStep : Simulation time step (seconds).
 */
void vpVirtuose::setTimeStep(const float &timeStep)
{
  init();

  if (m_period != timeStep) {
    m_period = timeStep;

    if (virtSetTimeStep(m_virtContext, m_period)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Error calling virtSetTimeStep: error code %d", err));
    }
  }
}

/*!
 *  Modify the current value of the control velocity and send it to the
 * Virtuose. \param velocity : Velocity twist vector, where translations
 * velocities are expressed in m/s and rotation velocities in rad/s.
 */
void vpVirtuose::setVelocity(vpColVector &velocity)
{
  init();

  if (velocity.size() != 6) {
    throw(vpException(vpException::dimensionError, "Cannot set a velocity vector (dim %d) that is not 6-dimension",
                      velocity.size()));
  }

  float speed[6];
  for (unsigned int i = 0; i < 6; i++)
    speed[i] = (float)velocity[i];

  if (virtSetSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtSetSpeed: error code %d", err));
  }
}

/*!
 * Set the speed factor.
 * \param velocityFactor : Scale factor applied to the velocities set using
 * setVelocity().
 */
void vpVirtuose::setVelocityFactor(const float &velocityFactor)
{
  init();

  if (virtSetSpeedFactor(m_virtContext, velocityFactor)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling setVelocityFactor: error code %d", err));
  }
}

/*!
 * Start the callback function set with setPeriodicFunction().
 *
 * \sa stopPeriodicFunction(), setPeriodicFunction()
 */
void vpVirtuose::startPeriodicFunction()
{
  init();

  if (virtStartLoop(m_virtContext)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling startLoop: error code %d", err));
  } else
    std::cout << "Haptic loop open." << std::endl;
}

/*!
 * Stop the callback function set with setPeriodicFunction().
 *
 * \sa startPeriodicFunction(), setPeriodicFunction()
 */
void vpVirtuose::stopPeriodicFunction()
{
  init();

  if (virtStopLoop(m_virtContext)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling stopLoop: error code %d", err));
  } else
    std::cout << "Haptic loop closed." << std::endl;
}

#else
// Work around to avoid warning
void dummy_vpVirtuose(){};
#endif
