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
 * Interface for Kinova Jaco robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_JACOSDK

#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotException.h>

/*!
  \file vpRobotKinova.cpp
  \brief Interface for Kinova Jaco2 robot.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotKinova.h>

/*!
  Basic initialization.
 */
void vpRobotKinova::init()
{
  // If you want to control the robot in Cartesian in a tool frame, set the corresponding transformation in m_eMc
  // that is set to identity by default in the constructor.

  maxRotationVelocity = maxRotationVelocityDefault;
  maxTranslationVelocity = maxTranslationVelocityDefault;

  // Set here the robot degrees of freedom number
  nDof = 6; // Jaco2 has 6 dof
}

/*!
  Default constructor.
 */
vpRobotKinova::vpRobotKinova()
  : m_eMc(), m_plugin_location("./"), m_commandLayer_handle(), m_verbose(false), m_plugin_loaded(false)
{
  init();
}

/*!
  Destructor.
 */
vpRobotKinova::~vpRobotKinova()
{
  closePlugin();
}

/*

  At least one of these function has to be implemented to control the robot with a
  Cartesian velocity:
  - get_eJe()
  - get_fJe()

*/

/*!
  Get the robot Jacobian expressed in the end-effector frame.

  \param[out] eJe : End-effector frame Jacobian.
*/
void vpRobotKinova::get_eJe(vpMatrix &eJe)
{
  (void) eJe;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get the robot Jacobian expressed in the robot reference frame.

  \param[out] fJe : Base (or reference) frame Jacobian.
*/
void vpRobotKinova::get_fJe(vpMatrix &fJe)
{
  (void) fJe;
  std::cout << "Not implemented ! " << std::endl;
}

/*

  At least one of these function has to be implemented to control the robot:
  - setCartVelocity()
  - setJointVelocity()

*/

/*!
  Send to the controller a 6-dim velocity twist vector expressed in a Cartesian frame.

  \param[in] frame : Cartesian control frame (either tool frame or end-effector) in which the velocity \e v is expressed.
  Units are m/s for translation and rad/s for rotation velocities.

  \param[in] v : 6-dim vector that contains the 6 components of the velocity twist to send to the robot.
  Units are m/s and rad/s.
*/
void vpRobotKinova::setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  if (v.size() != 6) {
    throw(vpException(vpException::fatalError, "Cannot send a velocity twist vector in tool frame that is not 6-dim (%d)", v.size()));
  }

  vpColVector v_e; // This is the velocity that the robot is able to apply in the end-effector frame
  switch(frame) {
  case vpRobot::TOOL_FRAME: {
    // We have to transform the requested velocity in the end-effector frame.
    // Knowing that the constant transformation between the tool frame and the end-effector frame obtained
    // by extrinsic calibration is set in m_eMc we can compute the velocity twist matrix eVc that transform
    // a velocity twist from tool (or camera) frame into end-effector frame
    vpVelocityTwistMatrix eVc(m_eMc);
    v_e = eVc * v;
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::REFERENCE_FRAME: {
    v_e = v;
    break;
  }
  case vpRobot::JOINT_STATE:
  case vpRobot::MIXT_FRAME:
    // Out of the scope
    break;
  }

  // Implement your stuff here to send the end-effector velocity twist v_e
  // - If the SDK allows to send cartesian velocities in the end-effector, it's done. Just wrap data in v_e
  // - If the SDK allows to send cartesian velocities in the reference (or base) frame you have to implement
  //   the robot Jacobian in set_fJe() and call:
  //   vpColVector v = get_fJe().inverse() * v_e;
  //   At this point you have to wrap data in v that is the 6-dim velocity to apply to the robot
  // - If the SDK allows to send only joint velocities you have to implement the robot Jacobian in set_eJe()
  //   and call:
  //   vpColVector qdot = get_eJe().inverse() * v_e;
  //   setJointVelocity(qdot);
  // - If the SDK allows to send only a cartesian position trajectory of the end-effector position in the base frame
  //   called fMe (for fix frame to end-effector homogeneous transformation) you can transform the cartesian
  //   velocity in the end-effector into a displacement eMe using the exponetial map:
  //   double delta_t = 0.010; // in sec
  //   vpHomogenesousMatrix eMe = vpExponentialMap::direct(v_e, delta_t);
  //   vpHomogenesousMatrix fMe = getPosition(vpRobot::REFERENCE_FRAME);
  //   the new position to reach is than given by fMe * eMe
  //   vpColVector fpe(vpPoseVector(fMe * eMe));
  //   setPosition(vpRobot::REFERENCE_FRAME, fpe);

  std::cout << "Not implemented ! " << std::endl;
  std::cout << "To implement me you need : " << std::endl;
  std::cout << "\t to known the robot jacobian expressed in ";
  std::cout << "the end-effector frame (eJe) " << std::endl;
  std::cout << "\t the frame transformation  between tool or camera frame ";
  std::cout << "and end-effector frame (cMe)" << std::endl;
}

/*!
  Send a joint velocity to the controller.
  \param[in] qdot : Joint velocities vector. Units are rad/s for a robot arm.
 */
void vpRobotKinova::setJointVelocity(const vpColVector &qdot)
{
  (void) qdot;

  // Implement your stuff here to send the joint velocities qdot

  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Send to the controller a velocity in a given frame.

  \param[in] frame : Control frame in which the velocity \e vel is expressed.
  Velocities could be joint velocities, or cartesian velocities. Units are m/s for translation and
  rad/s for rotation velocities.

  \param[in] vel : Vector that contains the velocity to apply to the robot.
 */
void vpRobotKinova::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
"Cannot send a velocity to the robot. "
"Call setRobotState(vpRobot::STATE_VELOCITY_CONTROL) once before "
"entering your control loop.");
  }

  vpColVector vel_sat(6);

  // Velocity saturation
  switch (frame) {
    // Saturation in cartesian space
  case vpRobot::TOOL_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    if (vel.size() != 6) {
      throw(vpException(vpException::dimensionError, "Cannot apply a Cartesian velocity that is not a 6-dim vector (%d)", vel.size()));
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    setCartVelocity(frame, vel_sat);
    break;
  }
                            // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    if (vel.size() != static_cast<size_t>(nDof)) {
      throw(vpException(vpException::dimensionError, "Cannot apply a joint velocity that is not a %-dim vector (%d)", nDof, vel.size()));
    }
    vpColVector vel_max(vel.size());

    // Since the robot has only rotation axis all the joint max velocities are set to getMaxRotationVelocity()
    vel_max = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    setJointVelocity(vel_sat);
  }
  }
}

/*

  THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

/*!
  Get robot joint positions.

  \warning We consider here that the robot has only 6 dof, but from the Jaco SDK it could be 7. Should be improved.

  \param[in] q : Joint position in rad.
 */
void vpRobotKinova::getJointPosition(vpColVector &q)
{
  AngularPosition currentCommand;

  // We get the actual angular command of the robot. Values are in deg
  KinovaGetAngularCommand(currentCommand);

  q.resize(6);
  q[0] = vpMath::rad(currentCommand.Actuators.Actuator1);
  q[1] = vpMath::rad(currentCommand.Actuators.Actuator2);
  q[2] = vpMath::rad(currentCommand.Actuators.Actuator3);
  q[3] = vpMath::rad(currentCommand.Actuators.Actuator4);
  q[4] = vpMath::rad(currentCommand.Actuators.Actuator5);
  q[5] = vpMath::rad(currentCommand.Actuators.Actuator6);
}

/*!
  Get robot position.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Position of the arm.
 */
void vpRobotKinova::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  if (frame == JOINT_STATE) {
    getJointPosition(q);
  }
  else {
    std::cout << "Not implemented ! " << std::endl;
  }
}

/*!
  Set a position to reach.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[in] q : Position to reach.
 */
void vpRobotKinova::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  if (frame == vpRobot::JOINT_STATE) {
    if (q.size() != 6) {
      throw(vpException(vpException::fatalError, "Cannot move robot to a joint position of dim %d that is not a 6-dim vector", q.size()));
    }
    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();
    // We specify that this point will be an angular(joint by joint) position.
    pointToSend.Position.Type = ANGULAR_POSITION;

    pointToSend.Position.Actuators.Actuator1 = static_cast<float>(vpMath::deg(q[0]));
    pointToSend.Position.Actuators.Actuator2 = static_cast<float>(vpMath::deg(q[1]));
    pointToSend.Position.Actuators.Actuator3 = static_cast<float>(vpMath::deg(q[2]));
    pointToSend.Position.Actuators.Actuator4 = static_cast<float>(vpMath::deg(q[3]));
    pointToSend.Position.Actuators.Actuator5 = static_cast<float>(vpMath::deg(q[4]));
    pointToSend.Position.Actuators.Actuator6 = static_cast<float>(vpMath::deg(q[5]));

    if (m_verbose) {
      vpColVector qdeg = q;
      qdeg.rad2deg();
      std::cout << "Move robot to joint position: " << qdeg.t() << std::endl;
    }
    KinovaSendBasicTrajectory(pointToSend);
  }
  else {
    throw(vpException(vpException::fatalError, "Cannot move robot to a cartesian position. Only joint positioning is implemented"));
  }
}

/*!
  Get a displacement.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Displacement in meter and rad.
 */
void vpRobotKinova::getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  (void) frame;
  (void) q;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Load functions from Jaco SDK plugin (ie. `Kinova.API.USBCommandLayerUbuntu.so` on
  unix-like platform or `CommandLayerWindows.dll` on Windows platform). There is setPluginLocation()
  that allows to modify the default location of the plugin corresponds current folder from where
  the binary is executed.

  \sa setPluginLocation()
*/
void vpRobotKinova::loadPlugin()
{
  if (m_plugin_loaded) {
    closePlugin();
  }
#ifdef __linux__ 
  // We load the API
  std::string plugin = vpIoTools::createFilePath(m_plugin_location, "Kinova.API.USBCommandLayerUbuntu.so");
  if (m_verbose) {
    std::cout << "Load plugin: \"" << plugin << "\"" << std::endl;
  }
  m_commandLayer_handle = dlopen(plugin.c_str(), RTLD_NOW | RTLD_GLOBAL);

  //We load the functions from the library
  KinovaInitAPI = (int(*)()) dlsym(m_commandLayer_handle, "InitAPI");
  KinovaCloseAPI = (int(*)()) dlsym(m_commandLayer_handle, "CloseAPI");
  KinovaMoveHome = (int(*)()) dlsym(m_commandLayer_handle, "MoveHome");
  KinovaInitFingers = (int(*)()) dlsym(m_commandLayer_handle, "InitFingers");
  KinovaGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(m_commandLayer_handle, "GetDevices");
  KinovaSetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(m_commandLayer_handle, "SetActiveDevice");
  KinovaSendBasicTrajectory = (int(*)(TrajectoryPoint)) dlsym(m_commandLayer_handle, "SendBasicTrajectory");
  KinovaGetAngularCommand = (int(*)(AngularPosition &)) dlsym(m_commandLayer_handle, "GetAngularCommand");
#elif _WIN32
  // We load the API.
  std::string plugin = vpIoTools::createFilePath(m_plugin_location, "CommandLayerWindows.dll");
  if (m_verbose) {
    std::cout << "Load plugin: \"" << plugin << "\"" << std::endl;
  }
  m_commandLayer_handle = LoadLibrary(TEXT(plugin.c_str()));

  // We load the functions from the library
  KinovaInitAPI = (int(*)()) GetProcAddress(m_commandLayer_handle, "InitAPI");
  KinovaCloseAPI = (int(*)()) GetProcAddress(m_commandLayer_handle, "CloseAPI");
  KinovaGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(m_commandLayer_handle, "GetDevices");
  KinovaSetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(m_commandLayer_handle, "SetActiveDevice");
  KinovaSendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(m_commandLayer_handle, "SendBasicTrajectory");
  KinovaGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(m_commandLayer_handle, "GetAngularCommand");
  KinovaMoveHome = (int(*)()) GetProcAddress(m_commandLayer_handle, "MoveHome");
  KinovaInitFingers = (int(*)()) GetProcAddress(m_commandLayer_handle, "InitFingers");
#endif

  // Verify that all functions has been loaded correctly
  if ((KinovaInitAPI == NULL) || (KinovaCloseAPI == NULL) || (KinovaSendBasicTrajectory == NULL) ||
    (KinovaGetDevices == NULL) || (KinovaSetActiveDevice == NULL) || (KinovaGetAngularCommand == NULL) ||
    (KinovaMoveHome == NULL) || (KinovaInitFingers == NULL)) {
    throw(vpException(vpException::fatalError, "Cannot load plugin from \"%s\" folder", m_plugin_location.c_str()));
  }
  if (m_verbose) {
    std::cout << "Plugin successfully loaded" << std::endl;
  }

  m_plugin_loaded = true;

  int result = (*KinovaInitAPI)();

  if (m_verbose) {
    std::cout << "Initialization's result: " << result << std::endl;
  }

  KinovaDevice list[MAX_KINOVA_DEVICE];

  int devicesCount = KinovaGetDevices(list, result);

  if (m_verbose) {
    std::cout << "Found " << devicesCount << " devices" << std::endl;
  }
  if (!devicesCount) {
    throw(vpException(vpException::fatalError, "No Kinova robot found"));
  }

  // Here we consider only the first device. Should be improved
  KinovaSetActiveDevice(list[0]);

}

/*!
Close plugin.
*/
void vpRobotKinova::closePlugin()
{
  if (m_plugin_loaded) {
    if (m_verbose) {
      std::cout << "Close plugin" << std::endl;
    }
#ifdef __linux__ 
    dlclose(m_commandLayer_handle);
#elif _WIN32
    FreeLibrary(m_commandLayer_handle);
#endif
    m_plugin_loaded = false;
  }
}

/*!
  Move the robot to home position.
*/
void vpRobotKinova::homing()
{
  if (!m_plugin_loaded) {
    throw(vpException(vpException::fatalError, "Cannot move robot to home position: Jaco SDK plugin not loaded"));
  }
  if (m_verbose) {
    std::cout << "Move the robot to home position" << std::endl;
  }
  KinovaMoveHome();
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotKinova.cpp.o) has
// no symbols
void dummy_vpRobotKinova(){};
#endif

