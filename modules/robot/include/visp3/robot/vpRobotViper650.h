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
 * Description:
 * Interface for the Irisa's Viper S650 robot controlled by an Adept
 *MotionBlox.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotViper650_h
#define vpRobotViper650_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_VIPER650

#include <iostream>
#include <stdio.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpViper650.h>

// low level controller api
extern "C" {
#include "irisa_Viper650.h"
#include "trycatch.h"
}

/*!
  \class vpRobotViper650

  \ingroup group_robot_real_arm

  \brief Control of Irisa's Viper S650 robot named Viper650.

  Implementation of the vpRobot class in order to control Irisa's
  Viper650 robot.  This robot is an ADEPT six degrees of freedom arm.
  A firewire camera is mounted on the end-effector to allow
  eye-in-hand visual servoing. The control of this camera is achieved
  by the vp1394TwoGrabber class.

  The model of the robot is the following:
  \image html model-viper.png Model of the Viper 650 robot.

  The non modified Denavit-Hartenberg representation of the robot is
  given in the table below, where \f$q_1^*, \ldots, q_6^*\f$
  are the variable joint positions.

  \f[
  \begin{tabular}{|c|c|c|c|c|}
  \hline
  Joint & $a_i$ & $d_i$ & $\alpha_i$ & $\theta_i$ \\
  \hline
  1 & $a_1$ & $d_1$ & $-\pi/2$ & $q_1^*$ \\
  2 & $a_2$ & 0     & 0        & $q_2^*$ \\
  3 & $a_3$ & 0     & $-\pi/2$ & $q_3^* - \pi$ \\
  4 & 0     & $d_4$ & $\pi/2$  & $q_4^*$ \\
  5 & 0     & 0     & $-\pi/2$ & $q_5^*$ \\
  6 & 0     & 0     & 0        & $q_6^*-\pi$ \\
  7 & 0     & $d_6$ & 0        & 0 \\
  \hline
  \end{tabular}
  \f]

  In this modelisation, different frames have to be considered.

  - \f$ {\cal F}_f \f$: the reference frame, also called world frame

  - \f$ {\cal F}_w \f$: the wrist frame located at the intersection of
    the last three rotations, with \f$ ^f{\bf M}_w = ^0{\bf M}_6 \f$

  - \f$ {\cal F}_e \f$: the end-effector frame located at the interface of the
    two tool changers, with \f$^f{\bf M}_e = 0{\bf M}_7 \f$

  - \f$ {\cal F}_c \f$: the camera or tool frame, with \f$^f{\bf M}_c = ^f{\bf
    M}_e \; ^e{\bf M}_c \f$ where \f$ ^e{\bf M}_c \f$ is the result of
    a calibration stage. We can also consider a custom tool
vpViper650::TOOL_CUSTOM and set this during robot initialisation or using
set_eMc().

  - \f$ {\cal F}_s \f$: the force/torque sensor frame, with \f$d7=0.0666\f$.

  This class allows to control the Viper650 arm robot in position
  and velocity:
  - in the joint space (vpRobot::ARTICULAR_FRAME),
  - in the fixed reference frame \f$ {\cal F}_f \f$
(vpRobot::REFERENCE_FRAME),
  - in the camera or tool frame \f$ {\cal F}_c \f$ (vpRobot::CAMERA_FRAME),
  - or in a mixed frame (vpRobot::MIXT_FRAME) where translations are expressed
  in the reference frame \f$ {\cal F}_f \f$ and rotations in the camera or
tool frame \f$ {\cal F}_c \f$ .

  All the translations are expressed in meters for positions and m/s
  for the velocities. Rotations are expressed in radians for the
  positions, and rad/s for the rotation velocities.

  The direct and inverse kinematics models are implemented in the
  vpViper650 class.

  \warning A Ctrl-C, a segmentation fault or other system errors are
  catched by this class to stop the robot.

  To communicate with the robot, you may first create an instance of this
  class by calling the default constructor:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;
#endif
}
  \endcode

  This initialize the robot kinematics with the \f$^e{\bf M}_c\f$
  extrinsic camera parameters obtained with a projection model without
  distortion. To set the robot kinematics with the \f$^e{\bf M}_c\f$
  transformation obtained with a camera perspective model including
  distortion you need to initialize the robot with:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;

  // Set the extrinsic camera parameters obtained with a perpective
  // projection model including a distortion parameter
  robot.init(vpViper650::TOOL_MARLIN_F033C_CAMERA,
       vpCameraParameters::perspectiveProjWithDistortion);
#endif
}
  \endcode

  You can get the intrinsic camera parameters of an image
  acquired by the camera attached to the robot, with:

  \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper650.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_VIPER650) && defined(VISP_HAVE_DC1394)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.acquire(I);

  vpRobotViper650 robot;

  // ...

  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model
  // with distortion.
#endif
}
  \endcode

  To control the robot in position, you may set the controller
  to position control and than send the position to reach in a specific
  frame like here in the joint space:

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;

  vpColVector q(6);
  // Set a joint position
  q[0] = vpMath::rad(10); // Joint 1 position, in rad
  q[1] = 0.2;             // Joint 2 position, in rad
  q[2] = 0.3;             // Joint 3 position, in rad
  q[3] = M_PI/8;          // Joint 4 position, in rad
  q[4] = M_PI/4;          // Joint 5 position, in rad
  q[5] = M_PI;            // Joint 6 position, in rad

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
#endif
}
  \endcode

  The robot moves to the specified position with the default
  positioning velocity vpRobotViper650::defaultPositioningVelocity. The
  setPositioningVelocity() method allows to change the maximal
  velocity used to reach the desired position.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;

  vpColVector q(6);
  // Set q[i] with i in [0:5]

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Set the max velocity to 40%
  robot.setPositioningVelocity(40);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
#endif
}
  \endcode

  To control the robot in velocity, you may set the controller to
  velocity control and than send the velocities. To end the velocity
  control and stop the robot you have to set the controller to the
  stop state. Here is an example of a velocity control in the joint
  space:

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;

  vpColVector qvel(6);
  // Set a joint velocity
  qvel[0] = 0.1;             // Joint 1 velocity in rad/s
  qvel[1] = vpMath::rad(15); // Joint 2 velocity in rad/s
  qvel[2] = 0;               // Joint 3 velocity in rad/s
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
#endif
}
  \endcode

  It is also possible to specify the position of a custom tool cartesian
frame. To this end this frame is to specify with respect of the end effector
frame in \f$^e {\bf M}_c\f$ transformation. This could be done by initializing
the robot thanks to init(vpViper650::vpToolType, const vpHomogeneousMatrix &)
or init(vpViper650::vpToolType, const std::string &) or using set_eMc(). The
following example illustrates this usecase:
\code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotViper650.h>

int main()
{
#ifdef VISP_HAVE_VIPER650
  vpRobotViper650 robot;

  // Set the transformation between the end-effector frame
  // and the tool frame.
  vpHomogeneousMatrix eMc(0.001, 0.0, 0.1, 0.0, 0.0, M_PI/2);

  robot.init(vpViper650::TOOL_CUSTOM, eMc);
#endif
}
  \endcode

  It is also possible to measure the robot current position with
  getPosition() method and the robot current velocities with the getVelocity()
  method.

  For convenience, there is also the ability to read/write joint
  positions from a position file with readPosFile() and savePosFile()
  methods.
*/
class VISP_EXPORT vpRobotViper650 : public vpViper650, public vpRobot
{

public: /* Constantes */
  /*! \enum vpControlModeType Control mode. */
  typedef enum {
    AUTO,   //!< Automatic control mode (default).
    MANUAL, //!< Manual control mode activated when the dead man switch is in
            //!< use.
    ESTOP   //!< Emergency stop activated.
  } vpControlModeType;

  /* Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double defaultPositioningVelocity; // = 20.0;

private: /* Not allowed functions. */
  /*!
    Copy constructor not allowed.
   */
  vpRobotViper650(const vpRobotViper650 &robot);

private: /* Attributs prives. */
  /** \brief Vrai ssi aucun objet de la classe vpRobotViper650 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotViper650, car il correspond a un seul robot AFMA6. Creer
   * simultanement deux objets peut engendrer des conflits. Le constructeur
   * lance une erreur si le champ n'est pas FAUX puis positionne le champ
   * a VRAI. Seul le destructeur repositionne le champ a FAUX, ce qui
   * alors la creation d'un nouvel objet.
   */
  static bool robotAlreadyCreated;

  double positioningVelocity;

  // Variables used to compute the measured velocities (see getVelocity() )
  vpColVector q_prev_getvel;
  vpHomogeneousMatrix fMc_prev_getvel;
  double time_prev_getvel;
  bool first_time_getvel;

  // Variables used to compute the measured displacement (see
  // getDisplacement() )
  vpColVector q_prev_getdis;
  bool first_time_getdis;
  vpControlModeType controlMode;

public: /* Methode publiques */
  explicit vpRobotViper650(bool verbose = true);
  virtual ~vpRobotViper650(void);

  // Force/Torque control
  void biasForceTorqueSensor() const;

  void closeGripper() const;

  void disableJoint6Limits() const;
  void enableJoint6Limits() const;

  /*!
    \return The control mode indicating if the robot is in automatic,
    manual (usage of the dead man switch) or emergnecy stop mode.
  */
  vpControlModeType getControlMode() const { return controlMode; }

  void getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement);
  void getForceTorque(vpColVector &H) const;
  vpColVector getForceTorque() const;

  double getMaxRotationVelocityJoint6() const;

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp);

  double getPositioningVelocity(void) const;
  bool getPowerState() const;

  double getTime() const;
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp);

  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame, double &timestamp);

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  void init(void);
  void
  init(vpViper650::vpToolType tool,
       vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);
  void init(vpViper650::vpToolType tool, const std::string &filename);
  void init(vpViper650::vpToolType tool, const vpHomogeneousMatrix &eMc_);

  void move(const std::string &filename);

  void openGripper();

  void powerOn();
  void powerOff();

  static bool readPosFile(const std::string &filename, vpColVector &q);
  static bool savePosFile(const std::string &filename, const vpColVector &q);

  void set_eMc(const vpHomogeneousMatrix &eMc_);
  void set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_);

  void setMaxRotationVelocity(double w_max);
  void setMaxRotationVelocityJoint6(double w6_max);

  // Position control
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
  void setPosition(const vpRobot::vpControlFrameType frame, const double pos1, const double pos2, const double pos3,
                   const double pos4, const double pos5, const double pos6);
  void setPosition(const std::string &filename);
  void setPositioningVelocity(const double velocity);

  // State
  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
  // Velocity control
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity);

  void stopMotion();

private:
  double maxRotationVelocity_joint6;
};

#endif
#endif /* #ifndef vpRobotViper650_h */
