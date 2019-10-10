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

#ifndef vpRobotKinova_h
#define vpRobotKinova_h

/*!

  \file vpRobotKinova.h

  Interface for Kinova robot using Jaco SDK.

*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_JACOSDK

#include <KinovaTypes.h>

#ifdef __linux__ 
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#elif _WIN32
#include <Windows.h>
#include <conio.h>
#include <iostream>
#include <CommunicationLayer.h>
#include <CommandLayer.h>
#endif

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>

/*!

  \class vpRobotKinova

  \ingroup group_robot_real_arm

  Interface for Kinova Jaco2 robot.

*/
class VISP_EXPORT vpRobotKinova : public vpRobot
{
public:
  vpRobotKinova();
  virtual ~vpRobotKinova();

  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  /*!
    Return constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  vpHomogeneousMatrix get_eMc() const { return m_eMc; }

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);

  void homing();
  void loadPlugin();

  /*!
    Set constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  void set_eMc(vpHomogeneousMatrix &eMc) { m_eMc = eMc; }
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);
  /*!
    \param[in] plugin_location: Path to Jaco SDK plugins (ie. `Kinova.API.USBCommandLayerUbuntu.so` on
    unix-like platform or `CommandLayerWindows.dll` on Windows platform). By default this location is empty,
    meaning that we suppose that the plugins are located in the same folder as the binary that want to use
    them.
  */
  void setPluginLocation(const std::string &plugin_location) { m_plugin_location = plugin_location;  };
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
  /*!
  Enable or disable verbose mode to print to stdout additional information.
  \param[in] verbose : true to enable verbose, false to disable. By default verbose
  mode is disabled.
  */
  void setVerbose(bool verbose) { m_verbose = verbose; }

protected:
  void closePlugin();
  void getJointPosition(vpColVector &q);
  void init();
  void setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  void setJointVelocity(const vpColVector &qdot);

protected:
  vpHomogeneousMatrix m_eMc; //!< Constant transformation between end-effector and tool (or camera) frame
  std::string m_plugin_location;
  bool m_verbose;
  bool m_plugin_loaded;

#ifdef __linux__ 
  void * m_commandLayer_handle;    //!< A handle to the API.
#elif _WIN32
  HINSTANCE m_commandLayer_handle; //!< A handle to the API.
#endif

private:
  int (*KinovaInitAPI)();
  int (*KinovaCloseAPI)();
  int (*KinovaSendBasicTrajectory)(TrajectoryPoint command);
  int (*KinovaGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
  int (*KinovaSetActiveDevice)(KinovaDevice device);
  int (*KinovaMoveHome)();
  int (*KinovaInitFingers)();
  int (*KinovaGetAngularCommand)(AngularPosition &);

};

#endif
#endif
