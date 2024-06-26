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
 * Interface for Kinova Jaco robot.
 *
*****************************************************************************/

/*!

  \file vpRobotKinova.h

  Interface for Kinova robot using Jaco SDK.

*/

#ifndef vpRobotKinova_h
#define vpRobotKinova_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_JACOSDK

#include <KinovaTypes.h>

#ifdef __linux__
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#elif _WIN32
#include <CommandLayer.h>
#include <CommunicationLayer.h>
#include <winsock2.h>
#include <windows.h>
#include <conio.h>
#include <iostream>
#endif

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>

BEGIN_VISP_NAMESPACE
/*!

  \class vpRobotKinova

  \ingroup group_robot_real_arm

  Interface for Kinova Jaco2 robot.

  This class is a wrapper over Kinova Jaco SDK that could be downloaded from Kinova Robotics
  <a href="https://www.kinovarobotics.com/en/knowledge-hub/all-kinova-products">software resources</a>
  by following the link under `Gen2 7 DoF > SDK 1.5.1`.

  It allows to control Kinova Jaco2 robot Gen 2 with 7 DoF, 6 DoF and 4 DoF.

  To select the degrees of freedom corresponding to your robot use setDoF().

*/
class VISP_EXPORT vpRobotKinova : public vpRobot
{
public:
  typedef enum { CMD_LAYER_USB, CMD_LAYER_ETHERNET, CMD_LAYER_UNSET } CommandLayer;

  vpRobotKinova();
  virtual ~vpRobotKinova() VP_OVERRIDE;

  int connect();

  void get_eJe(vpMatrix &eJe) VP_OVERRIDE;
  void get_fJe(vpMatrix &fJe) VP_OVERRIDE;

  /*!
   * Return constant transformation between end-effector and tool frame.
   * If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  vpHomogeneousMatrix get_eMc() const { return m_eMc; }

  int getActiveDevice() const { return m_active_device; }
  int getNumDevices() const { return m_devices_count; }
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position) VP_OVERRIDE;
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose);

  void homing();

  /*!
   * Set constant transformation between end-effector and tool frame.
   * If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  void set_eMc(vpHomogeneousMatrix &eMc) { m_eMc = eMc; }
  void setActiveDevice(int device);
  /*!
   * Set command layer indicating if the robot is controlled throw USB or Ethernet.
   * \param[in] command_layer : Layer used to control the robot.
   */
  void setCommandLayer(CommandLayer command_layer) { m_command_layer = command_layer; }
  void setDoF(unsigned int dof);
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;
  /*!
   * \param[in] plugin_location: Path to Jaco SDK plugins (ie. `Kinova.API.USBCommandLayerUbuntu.so` on
   * unix-like platform or `CommandLayerWindows.dll` on Windows platform). By default this location is empty,
   * meaning that we suppose that the plugins are located in the same folder as the binary that want to use
   * them.
   */
  void setPluginLocation(const std::string &plugin_location) { m_plugin_location = plugin_location; }
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) VP_OVERRIDE;
  /*!
   * Enable or disable verbose mode to print to stdout additional information.
   * \param[in] verbose : true to enable verbose, false to disable. By default verbose
   * mode is disabled.
   */
  void setVerbose(bool verbose) { m_verbose = verbose; }

protected:
  void closePlugin();
  void getJointPosition(vpColVector &q);
  void init();
  void loadPlugin();
  void setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  void setJointVelocity(const vpColVector &qdot);

protected:
  vpHomogeneousMatrix m_eMc; //!< Constant transformation between end-effector and tool (or camera) frame
  std::string m_plugin_location;
  bool m_verbose;
  bool m_plugin_loaded;
  int m_devices_count;
  KinovaDevice *m_devices_list;
  int m_active_device;
  CommandLayer m_command_layer;

#ifdef __linux__
  void *m_command_layer_handle; //!< A handle to the API.
#elif _WIN32
  HINSTANCE m_command_layer_handle; //!< A handle to the API.
#endif

private:
  int (*KinovaCloseAPI)();
  int (*KinovaGetAngularCommand)(AngularPosition &);
  int (*KinovaGetCartesianCommand)(CartesianPosition &);
  int (*KinovaGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
  int (*KinovaInitFingers)();
  int (*KinovaInitAPI)();
  int (*KinovaMoveHome)();
  int (*KinovaSendBasicTrajectory)(TrajectoryPoint command);
  int (*KinovaSetActiveDevice)(KinovaDevice device);
  int (*KinovaSetAngularControl)();
  int (*KinovaSetCartesianControl)();
};
END_VISP_NAMESPACE
#endif
#endif
