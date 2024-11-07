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
 * Interface for Universal Robot.
 *
*****************************************************************************/

#ifndef vpRobotUniversalRobots_h
#define vpRobotUniversalRobots_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_UR_RTDE)

#include <memory>

#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpRobotUniversalRobots

  \ingroup group_robot_real_arm

  \sa \ref tutorial-universal-robot-ibvs
  \sa \ref tutorial-universal-robot-pbvs
*/
class VISP_EXPORT vpRobotUniversalRobots : public vpRobot
{
private: // Not allowed functions
  /*!
    Copy constructor not allowed.
   */
  vpRobotUniversalRobots(const vpRobotUniversalRobots &robot);

public:
  vpRobotUniversalRobots();
  vpRobotUniversalRobots(const std::string &ur_address);
  virtual ~vpRobotUniversalRobots();

  void connect(const std::string &ur_address);
  void disconnect();

  /*!
   * Return handler to RTDEReceiveInterface.
   */
  std::shared_ptr<ur_rtde::RTDEReceiveInterface> getRTDEReceiveInterfaceHandler() const { return m_rtde_receive; }

  /*!
   * Return handler to RTDEControlInterface.
   */
  std::shared_ptr<ur_rtde::RTDEControlInterface> getRTDEControlInterfaceHandler() const { return m_rtde_control; }

  /*!
   * Return handler to DashboardClient.
   */
  std::shared_ptr<ur_rtde::DashboardClient> getDashboardClientHandler() const { return m_db_client; }

  vpHomogeneousMatrix get_fMe();
  vpHomogeneousMatrix get_fMe(const vpColVector &q);
  vpHomogeneousMatrix get_fMc();
  vpHomogeneousMatrix get_eMc() const;

  void getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force);
  std::string getPolyScopeVersion();
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position) VP_OVERRIDE;
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose);
  int getRobotMode() const;
  std::string getRobotModel() const;

  void move(const std::string &filename, double velocity_percentage = 10.);

  bool readPosFile(const std::string &filename, vpColVector &q);
  bool savePosFile(const std::string &filename, const vpColVector &q);

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position) VP_OVERRIDE;
  void setPosition(const vpRobot::vpControlFrameType frame, const vpPoseVector &pose);
  void setPositioningVelocity(double velocity);

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) VP_OVERRIDE;

  void set_eMc(const vpHomogeneousMatrix &eMc);

  void stopMotion();

private:
  // Not implemented yet
  void get_eJe(vpMatrix &_eJe) VP_OVERRIDE { };
  void get_fJe(vpMatrix &_fJe) VP_OVERRIDE { };
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE { };

protected:
  void init();

  std::shared_ptr<ur_rtde::RTDEReceiveInterface> m_rtde_receive;
  std::shared_ptr<ur_rtde::RTDEControlInterface> m_rtde_control;
  std::shared_ptr<ur_rtde::DashboardClient> m_db_client;
  vpHomogeneousMatrix m_eMc;
  double m_positioningVelocity;
  double m_max_joint_speed;
  double m_max_joint_acceleration;
  double m_max_linear_speed;
  double m_max_linear_acceleration;
  vpRobot::vpControlFrameType m_vel_control_frame;
};
END_VISP_NAMESPACE
#endif
#endif
