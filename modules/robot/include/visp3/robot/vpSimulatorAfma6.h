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
 * Class which provides a simulator for the robot Afma6.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpSimulatorAfma6_HH
#define vpSimulatorAfma6_HH

/*!
  \file vpSimulatorAfma6.h
  \brief Class which provides a simulator for the robot Afma6.
*/

#include <visp3/robot/vpAfma6.h>
#include <visp3/robot/vpRobotWireFrameSimulator.h>

#include <string>

#if defined(VISP_HAVE_MODULE_GUI) && ((defined(_WIN32) && !defined(WINRT_8_0)) || defined(VISP_HAVE_PTHREAD))

/*!
  \class vpSimulatorAfma6

  \ingroup group_robot_simu_gantry

  \brief Simulator of Irisa's gantry robot named Afma6.

  Implementation of the vpRobotWireFrameSimulator class in order to simulate
Irisa's Afma6 robot. This robot is a gantry robot with six degrees of freedom
manufactured in 1992 by the french Afma-Robots company.

  \warning This class uses threading capabilities. Thus on Unix-like
  platforms, the libpthread third-party library need to be
  installed. On Windows, we use the native threading capabilities.

  This class allows to control the Afma6 gantry robot in position
  and velocity:
  - in the joint space (vpRobot::ARTICULAR_FRAME),
  - in the fixed reference frame (vpRobot::REFERENCE_FRAME),
  - in the camera frame (vpRobot::CAMERA_FRAME),
  - or in a mixed frame (vpRobot::MIXT_FRAME) where translations are expressed
  in the reference frame and rotations in the camera frame.

  End-effector frame (vpRobot::END_EFFECTOR_FRAME) is not implemented.

  All the translations are expressed in meters for positions and m/s
  for the velocities. Rotations are expressed in radians for the
  positions, and rad/s for the rotation velocities.

  The direct and inverse kinematics models are implemented in the
  vpAfma6 class.

  To control the robot in position, you may set the controller
  to position control and then send the position to reach in a specific
  frame like here in the joint space:

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpSimulatorAfma6.h>

int main()
{
  vpSimulatorAfma6 robot;

  robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithoutDistortion);

  vpColVector q(6);
  // Set a joint position
  q[0] = 0.1;             // Joint 1 position, in meter
  q[1] = 0.2;             // Joint 2 position, in meter
  q[2] = 0.3;             // Joint 3 position, in meter
  q[3] = M_PI/8;          // Joint 4 position, in rad
  q[4] = M_PI/4;          // Joint 5 position, in rad
  q[5] = M_PI;            // Joint 6 position, in rad

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);

  return 0;
}
  \endcode

  To control the robot in velocity, you may set the controller to
  velocity control and then send the velocities. To end the velocity
  control and stop the robot you have to set the controller to the
  stop state. Here is an example of a velocity control in the joint
  space:

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpSimulatorAfma6.h>

int main()
{
  vpSimulatorAfma6 robot;

  robot.init(vpAfma6::TOOL_GRIPPER, vpCameraParameters::perspectiveProjWithoutDistortion);

  vpColVector qvel(6);
  // Set a joint velocity
  qvel[0] = 0.1;             // Joint 1 velocity in m/s
  qvel[1] = 0.1;             // Joint 2 velocity in m/s
  qvel[2] = 0.1;             // Joint 3 velocity in m/s
  qvel[3] = M_PI/8;          // Joint 4 velocity in rad/s
  qvel[4] = 0;               // Joint 5 velocity in rad/s
  qvel[5] = 0;               // Joint 6 velocity in rad/s

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  for ( ; ; ) {
    // Apply a velocity in the joint space
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qvel);

    // Compute new velocities qvel...
  }

  // Stop the robot
  robot.setRobotState(vpRobot::STATE_STOP);

  return 0;
}
  \endcode

  It is also possible to measure the robot current position with
  getPosition() method and the robot current velocities with the getVelocity()
  method.

  For convenience, there is also the ability to read/write joint
  positions from a position file with readPosFile() and savePosFile()
  methods.

  To know how this class can be used to achieve a visual servoing simulation,
  you can follow the \ref tutorial-ibvs.

*/

class VISP_EXPORT vpSimulatorAfma6 : public vpRobotWireFrameSimulator, public vpAfma6
{
public:
  static const double defaultPositioningVelocity;

private:
  vpColVector q_prev_getdis;
  bool first_time_getdis;

  double positioningVelocity;

  vpColVector zeroPos;
  vpColVector reposPos;

  bool toolCustom;
  std::string arm_dir;

public:
  vpSimulatorAfma6();
  explicit vpSimulatorAfma6(bool display);
  virtual ~vpSimulatorAfma6();

  void getCameraParameters(vpCameraParameters &cam, const unsigned int &image_width, const unsigned int &image_height);
  void getCameraParameters(vpCameraParameters &cam, const vpImage<unsigned char> &I);
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I);
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &displacement);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q, double &timestamp);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp);
  double getPositioningVelocity(void) { return positioningVelocity; }
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q, double &timestamp);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame, double &timestamp);

  void get_cMe(vpHomogeneousMatrix &cMe);
  void get_cVe(vpVelocityTwistMatrix &cVe);
  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  void
  init(vpAfma6::vpAfma6ToolType tool,
       vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);
  bool initialiseCameraRelativeToObject(const vpHomogeneousMatrix &cMo);
  void initialiseObjectRelativeToCamera(const vpHomogeneousMatrix &cMo);

  void move(const char *filename);

  static bool readPosFile(const std::string &filename, vpColVector &q);
  static bool savePosFile(const std::string &filename, const vpColVector &q);
  void setCameraParameters(const vpCameraParameters &cam);
  void setJointLimit(const vpColVector &limitMin, const vpColVector &limitMax);

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);
  void setPosition(const vpRobot::vpControlFrameType frame, const double pos1, const double pos2, const double pos3,
                   const double pos4, const double pos5, const double pos6);
  void setPosition(const char *filename);
  void setPositioningVelocity(const double vel) { positioningVelocity = vel; }
  bool setPosition(const vpHomogeneousMatrix &cdMo, vpImage<unsigned char> *Iint = NULL, const double &errMax = 0.001);
  vpRobot::vpRobotStateType setRobotState(const vpRobot::vpRobotStateType newState);

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity);

  void stopMotion();

protected:
  /** @name Protected Member Functions Inherited from vpSimulatorAfma6 */
  //@{
  void computeArticularVelocity();
  void compute_fMi();
  void findHighestPositioningSpeed(vpColVector &q);
  void getExternalImage(vpImage<vpRGBa> &I);
  inline void get_fMi(vpHomogeneousMatrix *fMit)
  {
#if defined(_WIN32)
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_fMi, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_fMi, INFINITE);
#endif
    for (int i = 0; i < 8; i++)
      fMit[i] = fMi[i];
    ReleaseMutex(mutex_fMi);
#elif defined(VISP_HAVE_PTHREAD)
    pthread_mutex_lock(&mutex_fMi);
    for (int i = 0; i < 8; i++)
      fMit[i] = fMi[i];
    pthread_mutex_unlock(&mutex_fMi);
#endif
  }
  void init();
  void initArms();
  void initDisplay();
  int isInJointLimit(void);
  bool singularityTest(const vpColVector &q, vpMatrix &J);
  void updateArticularPosition();
  //@}
};

#endif

#endif
