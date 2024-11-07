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
 * Interface for Flir Ptu Cpi robot.
 *
*****************************************************************************/

/*!
  \file vpRobotFlirPtu.h
  Interface for Flir Ptu Cpi robot.
*/

#ifndef vpRobotFlirPtu_h
#define vpRobotFlirPtu_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIR_PTU_SDK

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpRobotFlirPtu
  \ingroup group_robot_real_arm
  Interface for FLIR pan-tilt units compatible with FLIR PTU-SDK.

  \note We strongly recommend to communicate with the PTU using network interface. We experienced communication issues
  using serial communication.

  \warning On Unix-like OS, if you experienced the following error when running servoFlirPtu.cpp:
  \code
  Failed to open /dev/ttyUSB0: Permission denied.
  \endcode
  1. Add users to the "dialout" group:
  \code
  $ sudo adduser <username> dialout
  \endcode
  2. Reboot

  \warning Again on Unix-like OS, if you experienced the following error during ViSP build:
  \code
  <your path>/sdk-x.y.z/libcpi.a(cerial.o): relocation R_X86_64_PC32 against symbol `serposix' can not be used when
  making a shared object; recompile with -fPIC \endcode
  1. Enter FLIR PTU SDK folder and modify `config.mk` to add `-fPIC` build flag
  \code
  $ cat <your path>/sdk-x.y.y/config.mk
  CFLAGS=-g -Wall -Werror -DLITTLE_ENDIAN -O2 -fPIC
  \endcode
  2. Rebuild PTU-SDK
  \code
  $ cd <your path>/sdk-x.y.y
  $ make clean
  $ make
  \endcode
  3. Rebuild ViSP
  \code
  $ cd $VISP_WS/visp-build
  $ make -j4
  \endcode

  \sa \ref tutorial-flir-ptu-vs
*/
class VISP_EXPORT vpRobotFlirPtu : public vpRobot
{
public:
  vpRobotFlirPtu();
  virtual ~vpRobotFlirPtu();

  void connect(const std::string &portname, int baudrate = 9600);
  void disconnect();

  void get_eJe(vpMatrix &eJe) VP_OVERRIDE;
  vpMatrix get_eJe();
  void get_fJe(vpMatrix &fJe) VP_OVERRIDE;
  vpMatrix get_fJe();
  vpMatrix get_fMe();

  /*!
    Return constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  vpHomogeneousMatrix get_eMc() const { return m_eMc; }
  vpVelocityTwistMatrix get_cVe() const;

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;

  std::string getNetworkIP();
  std::string getNetworkGateway();
  std::string getNetworkHostName();

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;
  vpColVector getPanPosLimits();
  vpColVector getTiltPosLimits();
  vpColVector getPanTiltVelMax();

  void reset();

  /*!
    Set constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  void set_eMc(vpHomogeneousMatrix &eMc) { m_eMc = eMc; }
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;
  void setPanPosLimits(const vpColVector &pan_limits);
  void setTiltPosLimits(const vpColVector &tilt_limits);

  void setPositioningVelocity(double velocity);
  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) VP_OVERRIDE;
  void stopMotion();

  static void emergencyStop(int signo);

protected:
  void init();
  void getLimits();
  void getJointPosition(vpColVector &q);
  void setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  void setJointVelocity(const vpColVector &qdot);

private:
  double tics2deg(int axis, int tics);
  double tics2rad(int axis, int tics);
  int rad2tics(int axis, double rad);

protected:
  vpHomogeneousMatrix m_eMc; //!< Constant transformation between end-effector and tool (or camera) frame

  struct cerial *m_cer;
  uint16_t m_status;
  std::vector<int> m_pos_max_tics; //!< Pan min/max position in robot tics unit
  std::vector<int> m_pos_min_tics; //!< Tilt min/max position in robot tics unit
  std::vector<int> m_vel_max_tics; //!< Pan/tilt max velocity in robot tics unit
  std::vector<double> m_res;       //!< Pan/tilt tic resolution in deg
  bool m_connected;
  int m_njoints;
  double m_positioning_velocity;
};
END_VISP_NAMESPACE
#endif
#endif
