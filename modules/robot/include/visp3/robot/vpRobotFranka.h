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
 * Interface for the Franka robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef _vpRobotFranka_h_
#define _vpRobotFranka_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>

#include <stdio.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>

#include <visp3/core/vpColVector.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/core/vpException.h>

/*!
  \class vpRobotFranka

  \ingroup group_robot_real_arm

  This class is a wrapper over the [libfranka](https://github.com/frankaemika/libfranka)
  component part of the [Franka Control Interface](https://frankaemika.github.io/docs/) (FCI).

  Before using vpRobotFranka follow the
  [installation instructions](https://frankaemika.github.io/docs/installation.html#) to install
  libfranka. We suggest to
  [build libfranka from source](https://frankaemika.github.io/docs/installation.html#building-libfranka)
  if you are not using ROS.

  Moreover, you need also to setup a real-time kernel following these
  [instructions](https://frankaemika.github.io/docs/installation.html#setting-up-the-real-time-kernel).

  Up to now, this class provides the following capabilities to:
  - move to a given joint position using setPosition() that is blocking and that returns only when the robot
    has reached the desired position.
  \code
    vpRobotFranka robot("192.168.1.1");

    vpColVector q_d(7);
    q_d[3] = -M_PI_2;
    q_d[5] = M_PI_2;
    q_d[6] = M_PI_4;
    std::cout << "Move to joint position: " << q_d.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q_d);
  \endcode
  - move applying a joint velocity using setVelocity(). This function is non-blocking.
  \code
    vpRobotFranka robot("192.168.1.1");

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    vpColVector dq_d(7, 0);
    dq_d[4] = vpMath::rad(-20.);
    dq_d[6] = vpMath::rad(20.);
    while(1) {
      robot.setVelocity(vpRobot::JOINT_STATE, dq_d);
      ...
    }
  \endcode
  - move applying a cartesian velocity to the end-effector using setVelocity(). This function is non-blocking.
  \code
    vpRobotFranka robot("192.168.1.1");

    vpColVector ve_d(6);
    ve_d[2] = 0.02; // vz = 2 cm/s goes down

    while(1) {
      robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, ve_d);
      ...
    }
  \endcode
  - move applying a cartesian velocity to the camera frame (or a given tool frame) using setVelocity().
    The camera frame (or a tool frame) location wrt the end-effector is set using set_eMc(). This function is non-blocking.
  \code
    vpRobotFranka robot("192.168.1.1");
    vpHomogeneousMatrix eMc; // Position of the camera wrt the end-effector
    // update eMc
    robot.set_eMc(eMc);

    vpColVector vc_d(6);
    vc_d[2] = 0.02; // vz = 2 cm/s is along the camera optical axis

    while(1) {
      robot.setVelocity(vpRobot::CAMERA_FRAME, vc_d);
      ...
    }
  \endcode
    If the tool attached to the end-effector is not a camera, you can do exactly the same using:
  \code
    vpRobotFranka robot("192.168.1.1");
    vpHomogeneousMatrix eMt;
    // update eMt, the position of the tool wrt the end-effector frame
    robot.set_eMc(eMt);

    vpColVector vt_d(6);
    vt_d[2] = 0.02; // vt = 2 cm/s is along tool z axis

    while(1) {
      robot.setVelocity(vpRobot::TOOL_FRAME, vt_d);
      ...
    }
  \endcode

  - get the joint position using getPosition()
  \code
    vpRobotFranka robot("192.168.1.1");

    vpColVector q;
    while(1) {
      robot.getPosition(vpRobot::JOINT_STATE, q);
      ...
    }
  \endcode
  - get the cartesian end-effector position using getPosition(). This function is non-blocking.
  \code
    vpRobotFranka robot("192.168.1.1");

    vpPoseVector wPe;
    vpHomogeneousMatrix wMe;
    while(1) {
      robot.getPosition(vpRobot::END_EFFECTOR_FRAME, wPe);
      wMe.buildFrom(wPe);
      ...
    }
  \endcode
  - get the cartesian camera (or tool) frame position using getPosition(). This function is non-blocking.
  \code
    vpRobotFranka robot("192.168.1.1");
    vpHomogeneousMatrix eMc;
    // update eMc, the position of the camera wrt the end-effector frame
    robot.set_eMc(eMc);

    vpPoseVector wPc;
    vpHomogeneousMatrix wMc;
    while(1) {
      robot.getPosition(vpRobot::CAMERA_FRAME, wPc);
      wMc.buildFrom(wPc);
      ...
    }
  \endcode
    If the tool attached to the end-effector is not a camera, you can do exactly the same using:
  \code
    vpRobotFranka robot("192.168.1.1");
    vpHomogeneousMatrix eMt;
    // update eMt, the position of the tool wrt the end-effector frame
    robot.set_eMc(eMt);

    vpPoseVector wPt;
    vpHomogeneousMatrix wMt;
    while(1) {
      robot.getPosition(vpRobot::TOOL_FRAME, wPt);
      wMt.buildFrom(wPt);
      ...
    }
  \endcode

  What is not implemented is:
  - move to a given cartesian end-effector position
  - gripper controller
  - force/torque feadback and control

  Known issues:
  - sometimes the joint to joint trajectory generator provided by Franka complains about discontinuities.

  We provide also the getHandler() function that allows to acces to the robot handler and call the native
  [libfranka API](https://frankaemika.github.io/libfranka/index.html) fonctionalities:
  \code
    vpRobotFranka robot("192.168.1.1");

    franka::Robot *handler = robot.getHandler();

    // Get end-effector cartesian position
    std::array<double, 16> pose = handler->readOnce().O_T_EE;
  \endcode

*/
class VISP_EXPORT vpRobotFranka : public vpRobot
{
private:
  /*!
    Copy constructor not allowed.
   */
  vpRobotFranka(const vpRobotFranka &robot);
  /*!
    This function is not implemented.
   */
  void getDisplacement(const vpRobot::vpControlFrameType, vpColVector &) {};
  franka::RobotState getRobotInternalState();
  void init();

  franka::Robot *m_handler; //!< Robot handler
  franka::Gripper *m_gripper; //!< Gripper handler
  franka::Model *m_model;
  double m_positionningVelocity;

  std::thread m_controlThread;
  std::atomic_bool m_controlThreadIsRunning;
  std::atomic_bool m_controlThreadStopAsked;

  std::array<double, 7> m_q_min;    // Joint min position
  std::array<double, 7> m_q_max;    // Joint max position
  std::array<double, 7> m_dq_max;   // Joint max velocity
  std::array<double, 7> m_ddq_max;  // Joint max acceleration

  franka::RobotState m_robot_state; // Robot state protected by mutex
  std::mutex m_mutex;               // Mutex to protect m_robot_state

  std::array<double, 7> m_dq_des;   // Desired joint velocity
  vpColVector m_v_cart_des;             // Desired cartesian velocity either in reference, end-effector, camera, or tool frame
  vpHomogeneousMatrix m_eMc;
  std::string m_log_folder;
  std::string m_franka_address;

public:
  vpRobotFranka();

  vpRobotFranka(const std::string &franka_address,
                franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce);

  virtual ~vpRobotFranka();

  void connect(const std::string &franka_address,
               franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce);

  vpHomogeneousMatrix get_fMe(const vpColVector &q);
  vpHomogeneousMatrix get_fMc(const vpColVector &q);
  vpHomogeneousMatrix get_eMc() const;

  void get_eJe(vpMatrix &eJe);
  void get_eJe(const vpColVector &q, vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);
  void get_fJe(const vpColVector &q, vpMatrix &fJe);

  void getCoriolis(vpColVector &coriolis);
  void getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force);

  void getGravity(vpColVector &gravity);

  /*!
   * Get gripper handler to access native libfranka functions.
   *
   * \return Robot handler if it exists, an exception otherwise.
   */
  franka::Gripper *getGripperHandler() {
    if (!m_gripper) {
      throw(vpException(vpException::fatalError, "Cannot get Franka gripper handler: gripper is not connected"));
    }

    return m_gripper;
  }


  /*!
   * Get robot handler to access native libfranka functions.
   *
   * \return Robot handler if it exists, an exception otherwise.
   */
  franka::Robot *getHandler() {
    if (!m_handler) {
      throw(vpException(vpException::fatalError, "Cannot get Franka robot handler: robot is not connected"));
    }

    return m_handler;
  }

  vpColVector getJointMin() const;
  vpColVector getJointMax() const;

  void getMass(vpMatrix &mass);

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose);

  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_position);

  int gripperClose();
  int gripperGrasp(double grasping_width, double force=60.);
  void gripperHoming();
  int gripperMove(double width);
  int gripperOpen();
  void gripperRelease();

  void move(const std::string &filename, double velocity_percentage=10.);

  bool readPosFile(const std::string &filename, vpColVector &q);
  bool savePosFile(const std::string &filename, const vpColVector &q);

  void set_eMc(const vpHomogeneousMatrix &eMc);
  void setLogFolder(const std::string &folder);
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
  void setPositioningVelocity(const double velocity);

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  void stopMotion();
};

#endif
#endif // #ifndef __vpRobotFranka_h_
