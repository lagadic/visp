/****************************************************************************
 *
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
 * Interface to mavlink compatible controller using mavsdk 3rd party
 *
*****************************************************************************/

#ifndef vpRobotMavsdk_h_
#define vpRobotMavsdk_h_

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher.
// Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) \
  && defined(VISP_HAVE_THREADS)

#include <future>
#include <mutex>
#include <signal.h>
#include <string>
#include <tuple>

#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE
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
 * \note The body frame associated to the vehicle controlled through MavLink is supposed to be Front-Right-Down (FRD)
 * respectively for X-Y-Z.
 *
 * \image html img-pixhawk-frames.jpg
 *
 * 1. This class was tested to control a quadcopter equipped with a Pixhawk running PX4 firmware connected to a Jetson
 * TX2.
 *
 *    We provide a set of tests if you want to have a try on your flying vehicle:
 *    - testPixhawkDroneTakeoff.cpp
 *    - testPixhawkDronePositionAbsoluteControl.cpp
 *    - testPixhawkDronePositionRelativeControl.cpp
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

  //! \name Robot connection
  //@{
  void connect(const std::string &connection_info);
  //@}

  //! \name General robot information
  //@{
  float getBatteryLevel() const;
  void getPosition(float &ned_north, float &ned_east, float &ned_down, float &ned_yaw) const;
  void getPosition(vpHomogeneousMatrix &ned_M_frd) const;
  std::tuple<float, float> getHome() const;
  std::string getAddress() const;
  bool isRunning() const;
  //@}

  //! \name Robot commands
  //@{
  bool arm();
  bool disarm();
  void doFlatTrim();
  bool hasFlyingCapability();
  bool holdPosition();
  bool kill();
  bool land();
  bool releaseControl();
  bool sendMocapData(const vpHomogeneousMatrix &enu_M_flu, int display_fps = 1);
  void setAutoLand(bool auto_land);
  bool setForwardSpeed(double body_frd_vx);
  bool setLateralSpeed(double body_frd_vy);
  bool setGPSGlobalOrigin(double latitude, double longitude, double altitude);
  void setPositioningIncertitude(float position_incertitude, float yaw_incertitude);
  bool setPosition(float ned_north, float ned_east, float ned_down, float ned_yaw, bool blocking = true,
                   int timeout_sec = 10);
  bool setPosition(const vpHomogeneousMatrix &ned_M_frd, bool blocking = true, int timeout_sec = 10);
  bool setPositionRelative(float ned_delta_north, float ned_delta_east, float ned_delta_down, float ned_delta_yaw,
                           bool blocking = true, int timeout_sec = 10);
  bool setPositionRelative(const vpHomogeneousMatrix &delta_frd_M_frd, bool blocking = true, int timeout_sec = 10);
  bool setVelocity(const vpColVector &frd_vel_cmd);
  bool setVerticalSpeed(double body_frd_vz);
  bool setYawSpeed(double body_frd_wz);
  void setTakeOffAlt(double altitude);
  void setVerbose(bool verbose);
  bool stopMoving();
  bool takeControl();
  bool takeOff(bool interactive = true, int timeout_sec = 10, bool use_gps = false);
  bool takeOff(bool interactive, double takeoff_altitude, int timeout_sec = 10, bool use_gps = false);
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
END_VISP_NAMESPACE
#endif // #ifdef VISP_HAVE_MAVSDK
#endif // #ifndef vpRobotMavsdk_h_
