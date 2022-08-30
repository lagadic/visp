/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Interface to mavlink compatible controller using mavsdk 3rd party
 *
 *****************************************************************************/

#ifndef vpRobotMavsdk_h_
#define vpRobotMavsdk_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)

#include <future>
#include <mutex>
#include <signal.h>
#include <string>

#include <visp3/core/vpHomogeneousMatrix.h>

/*!
 * \class vpRobotMavsdk
 *
 * \ingroup group_robot_real_drone
 *
 * Interface for [Mavlink](https://mavlink.io/en/) allowing to control drones or rovers using a MavLink compatible
 * controller.
 *
 * This class needs cxx17 or more recent standard enabled during ViSP cmake configuration.
 *
 * This class is enabled when [MavSDK C++](https://github.com/mavlink/MAVSDK) is installed and detected by ViSP during
 * cmake configuration step.
 *
 * \note The body frame associated to the robot controlled through MavLink is supposed to be Front-Right-Down (FRD)
 * respectively for X-Y-Z.
 *
 * \sa \ref tutorial-pixhawk-vs
 */
class VISP_EXPORT vpRobotMavsdk
{
public:
  vpRobotMavsdk();
  vpRobotMavsdk(const std::string &connection_info);
  virtual ~vpRobotMavsdk();

  //! @name Robot connection
  //@{
  void connect(const std::string &connection_info);
  //@}

  //! @name General robot information
  //@{
  float getBatteryLevel() const;
  void getPose(vpHomogeneousMatrix &pose) const;
  std::string getAddress() const;
  //@}

  //! @name Robot state checking
  //@{
  // bool isFlying() const;   // Not implemented yet
  // bool isHovering() const; // Not implemented yet
  // bool isLanded() const;   // Not implemented yet
  bool isRunning() const;
  //@}

  //! @name Sending state info
  //@{
  bool sendMocapData(const vpHomogeneousMatrix &M);
  //@}

  //! @name Commands and parameters
  //@{
  bool arm();
  void doFlatTrim();
  void holdPosition();
  bool kill();
  bool land();
  void setForwardSpeed(double vx);
  void setLateralSpeed(double vy);
  void setPosition(float dX, float dY, float dZ, float dPsi);
  void setPosition(const vpHomogeneousMatrix &M);
  void setVelocity(const vpColVector &vel, double delta_t);
  void setVelocity(const vpColVector &vel);
  void setVerticalSpeed(double vz);
  void setYawSpeed(double wz);
  void stopMoving();
  void setTakeOffAlt(double altitude);
  bool takeOff(bool interactive = true);
  //@}

private:
  [[noreturn]] static void sighandler(int signo);

  //*** Setup functions ***//
  void cleanUp();
  void createDroneController();
  void setupCallbacks();
  void startController();

  vpRobotMavsdk(const vpRobotMavsdk &);            // noncopyable
  vpRobotMavsdk &operator=(const vpRobotMavsdk &); //

  class vpRobotMavsdkImpl;
  vpRobotMavsdkImpl *m_impl;
};

#endif //#ifdef VISP_HAVE_MAVSDK
#endif //#ifndef vpRobotMavsdk_h_
