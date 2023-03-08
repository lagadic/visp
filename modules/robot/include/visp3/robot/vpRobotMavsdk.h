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
#include <tuple>

#include <visp3/core/vpHomogeneousMatrix.h>

/*!
 * \class vpRobotMavsdk
 *
 * \ingroup group_robot_real_drone
 *
 * Interface for [Mavlink](https://mavlink.io/en/) allowing to control drones or rovers using a MavLink compatible
 * controller such a Pixhawk running PX4 or Ardupilot.
 *
 * This class needs cxx17 or more recent standard enabled during ViSP cmake configuration.
 *
 * This class is enabled when [MavSDK C++](https://github.com/mavlink/MAVSDK) is installed and detected by ViSP during
 * cmake configuration step.
 *
 * \note The body frame associated to the robot controlled through MavLink is supposed to be Front-Right-Down (FRD)
 * respectively for X-Y-Z.
 *
 * 1. This class was tested to control a quadcopter equipped with a Pixhawk running PX4 firmware connected to a Jetson
 * TX2.
 *
 *    We provide a set of tests if you want to have a try on your flying vehicle:
 *    - testPixhawkDroneTakeoff.cpp
 *    - testPixhawkDronePositionControl.cpp
 *    - testPixhawkDroneVelocityControl.cpp
 *    - testPixhawkDroneKeyboard.cpp
 *
 *    We provide also this \ref tutorial-pixhawk-vs.
 *
 * 2. This class was also tested to control an AION ROBOTICS rover equipped with a Pixhawk running Ardupilot firmware
 * directly connected by serial to a laptop running Ubuntu 22.04.
 *
 *    If you want to have a try you may see:
 *    - testPixhawkRoverVelocityControl.cpp
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
  void getPose(vpHomogeneousMatrix &ned_M_frd) const;
  std::tuple<float, float> getHome() const;
  std::string getAddress() const;
  //@}

  //! @name Robot state checking
  //@{
  bool isRunning() const;
  //@}

  //! @name Sending state info
  //@{
  bool sendMocapData(const vpHomogeneousMatrix &enu_M_frd);
  //@}

  //! @name Commands and parameters
  //@{
  bool arm();
  bool disarm();
  void doFlatTrim();
  bool hasFlyingCapability();
  void holdPosition();
  bool kill();
  bool land();
  void setForwardSpeed(double body_frd_vx);
  void setLateralSpeed(double body_frd_vy);
  bool setGPSGlobalOrigin(double latitude, double longitude, double altitude);
  void setPosition(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw);
  void setPosition(const vpHomogeneousMatrix &ned_M_delta);
  void setVelocity(const vpColVector &frd_vel_cmd, double delta_t);
  void setVelocity(const vpColVector &frd_vel_cmd);
  void setVerticalSpeed(double body_frd_vz);
  void setYawSpeed(double body_frd_wz);
  void stopMoving();
  void setTakeOffAlt(double altitude);
  bool takeOff(bool interactive = true);
  //@}

private:
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

#endif // #ifdef VISP_HAVE_MAVSDK
#endif // #ifndef vpRobotMavsdk_h_
