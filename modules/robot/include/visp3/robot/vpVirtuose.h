/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef __vpVirtuose_h_
#define __vpVirtuose_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpPoseVector.h>

#ifdef VISP_HAVE_VIRTUOSE

#include <VirtuoseAPI.h>

/*!
  \file vpVirtuose.h
  \brief Wrapper over Haption Virtuose SDK to control haptic devices.
*/
/*!
  \class vpVirtuose
  \ingroup group_robot_haptic

  This class was tested with Haption (http://www.haption.com) Virtuose 6D
haptic device.

  The class vpVirtuose allows to work with the original Virtuose API inside
ViSP. The Virtuose API supports the following devices:
  - Virtuose 6D35-45
  - Virtuose 3D35-40
  - Virtuose 3D10-20
  - Virtuose Desktop
  - Virtuose Inca

  Not all Virtuose API function are implemented in the class.
  Original Virtuose API functions need to be called with a VirtContext object,
provided by the function getHandler().

  The Virtuose library implements different control modes that could be set
using setCommandType(). The choice of the control mode depends on the
application. The following is the description of the main control modes as
described in the Virtuose API documentation.

  1. Force/position control (impedance mode): the application sends forces and
torques to the device and reads the position and speed of the end-effector
frame.
  2. Position/force control (admittance mode): this advanced control mode
allows direct coupling with virtual objects; in that case, the application
sends the position and speed of the center of the object to the device, and
reads the forces and torques to be applied to the object for dynamic
integration. Stiffness and damping are calculated by the embedded software,
knowing the mass and inertia of the object, in order to ensure control
  stability.
  3. Position/force with virtual guides: this is the same as above, with
addition of virtual guides (e.g. fixed translation, fixed rotation, etc.).

  The Virtuose library defines the following reference frames:
  1. The environment frame, corresponding to the origin of the virtual scene;
it is specified by the software application independently of the Virtuose API.
  2. The observation frame, corresponding generally to the position of the
camera; it is defined with respect to environment frame. This frame location
could be set using setObservationFrame().
  3. The base frame, representing the center of the haptic device; it is
defined with respect to the observation frame. This frame location could be
set using setBaseFrame().
  4. The tool frame corresponds to the base of the tool fixed at the end of
the haptic device, and is defined with respect to the environment frame.
  5. The end-effector (avatar) frame corresponds to the position of the user
hand on the device, taking into account the geometry of the tool, and is
defined with respect to tool frame.

  The position of the following frames can be defined only once using the API:
  base frame (with respect to the observation frame) thanks to setBaseFrame()
and end-effector frame (with respect to the tool frame).

  The position of the observation frame (with respect to the environment
frame) can be modified dynamically using setObservationFrame().

  The position of the tool frame (with respect to the environment frame)
cannot be modified.

  All values used in the Virtuose API are expressed in physical units using
metric conventions:
  - Durations in seconds (s)
  - Dimensions in meters (m)
  - Angles in radians (rad)
  - Linear velocities in meters per second (m.s -1 )
  - Angular velocities in radians per second (rad.s -1 )
  - Forces in Newtons (N)
  - Torques in Newton-meters (N.m)
  - Masses in kilogrammes (kg)
  - Inertia components in kg.m2

  The following sample code shows how to connect to the haptic device to get
its current joint position:
\code
#include <visp3/robot/vpVirtuose.h>

int main()
{
  vpVirtuose virtuose;
  virtuose.init();
  vpColVector q = virtuose.getArticularPosition();
  std::cout << "Joint position: " << q.t() << std::endl;
}
  \endcode
 */
class VISP_EXPORT vpVirtuose
{
public:
  vpVirtuose();
  ~vpVirtuose();

  void addForce(vpColVector &force);
  void enableForceFeedback(int enable);

  vpColVector getArticularPosition() const;
  vpColVector getArticularVelocity() const;
  vpPoseVector getAvatarPosition() const;
  vpPoseVector getBaseFrame() const;
  VirtCommandType getCommandType() const;
  bool getDeadMan() const;
  bool getEmergencyStop() const;
  vpColVector getForce() const;
  VirtContext getHandler();
  vpPoseVector getObservationFrame() const;
  vpPoseVector getPhysicalPosition() const;
  vpColVector getPhysicalVelocity() const;
  vpPoseVector getPosition() const;
  bool getPower() const;
  vpColVector getVelocity() const;

  void init();

  void setArticularForce(const vpColVector &articularForce);
  void setArticularPosition(const vpColVector &articularPosition);
  void setArticularVelocity(const vpColVector &articularVelocity);
  void setBaseFrame(const vpPoseVector &position);
  void setCommandType(const VirtCommandType &type);
  void setForce(const vpColVector &force);
  void setForceFactor(const float &forceFactor);
  void setIndexingMode(const VirtIndexingType &type);
  /*! Set haptic device ip address and port. Default value is
   * "localhost#5000".*/
  inline void setIpAddress(const std::string &ip) { m_ip = ip; }
  void setObservationFrame(const vpPoseVector &position);
  void setPeriodicFunction(VirtPeriodicFunction CallBackVirt);
  void setPosition(vpPoseVector &position);
  void setPowerOff();
  void setPowerOn();
  void setSaturation(const float &forceLimit, const float &torqueLimit);
  void setTimeStep(const float &timeStep);
  void setVelocity(vpColVector &velocity);
  void setVelocityFactor(const float &velocityFactor);
  /*!
   * Enable/disable verbose mode.
   * \param mode : true to enable, false to disable verbose.
   */
  void setVerbose(bool mode) { m_verbose = mode; }

  void startPeriodicFunction();
  void stopPeriodicFunction();

protected:
  VirtContext m_virtContext;
  std::string m_ip;
  bool m_verbose;
  int m_apiMajorVersion;
  int m_apiMinorVersion;
  int m_ctrlMajorVersion;
  int m_ctrlMinorVersion;
  VirtCommandType m_typeCommand;
  VirtIndexingType m_indexType;
  bool m_is_init;
  float m_period;
};

#endif
#endif
