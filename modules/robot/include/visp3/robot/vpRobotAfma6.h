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
 * Interface for the Irisa's Afma6 robot controlled by an Adept MotionBlox.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotAfma6_h
#define vpRobotAfma6_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_AFMA6

#include <iostream>
#include <stdio.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/robot/vpAfma6.h>
#include <visp3/robot/vpRobot.h>

// low level controller api
extern "C" {
#include "irisa_Afma6.h"
#include "trycatch.h"
}

/*!
  \class vpRobotAfma6

  \ingroup group_robot_real_gantry

  \brief Control of Irisa's gantry robot named Afma6.

  Implementation of the vpRobot class in order to control Irisa's
  Afma6 robot.  This robot is a gantry robot with six degrees of
  freedom manufactured in 1992 by the french Afma-Robots company. In
  2008, the low level controller change for a more recent Adept
  technology based on the MotionBlox controller. A firewire camera is
  mounted on the end-effector to allow eye-in-hand visual
  servoing. The control of this camera is achieved by the
  vp1394TwoGrabber class. A ring light is attached around the
  camera. The control of this ring light is possible throw the
  vpRingLight class. A CCMOP gripper is also mounted on the
  end-effector. The pneumatic control of this gripper is possible
  throw the openGripper() or closeGripper() member functions.

  This class allows to control the Afma6 gantry robot in position
  and velocity:
  - in the joint space (vpRobot::ARTICULAR_FRAME),
  - in the fixed reference frame (vpRobot::REFERENCE_FRAME),
  - in the camera frame (vpRobot::CAMERA_FRAME),
  - or in a mixed frame (vpRobot::MIXT_FRAME) where translations are expressed
  in the reference frame and rotations in the camera frame.

  All the translations are expressed in meters for positions and m/s
  for the velocities. Rotations are expressed in radians for the
  positions, and rad/s for the rotation velocities.

  The direct and inverse kinematics models are implemented in the
  vpAfma6 class.

  \warning A Ctrl-C, a segmentation fault or other system errors are
  catched by this class to stop the robot.

  To communicate with the robot, you may first create an instance of this
  class by calling the default constructor:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotAfma6.h>

#ifdef VISP_HAVE_AFMA6
int main()
{
  vpRobotAfma6 robot;
}
#else
int main() {}
#endif
  \endcode

  This initialize the robot kinematics with the eMc extrinsic camera
  parameters obtained with a projection model without distortion. To
  set the robot kinematics with the eMc matrix obtained with a camera
  perspective model including distortion you need to initialize the
  robot with:

  \code
  // Set the extrinsic camera parameters obtained with a perpective
  // projection model including a distortion parameter
  robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithDistortion);
  \endcode

  You can get the intrinsic camera parameters of the image I
  acquired with the camera, with:

  \code
  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model
  // with distortion.
  \endcode

  To control the robot in position, you may set the controller
  to position control and than send the position to reach in a specific
  frame like here in the joint space:

  \code
  vpColVector q(6);
  // Set a joint position
  q[0] = 0.1; // x axis, in meter
  q[1] = 0.2; // y axis, in meter
  q[2] = 0.3; // z axis, in meter
  q[3] = M_PI/8; // rotation around A axis, in rad
  q[4] = M_PI/4; // rotation around B axis, in rad
  q[5] = M_PI;   // rotation around C axis, in rad

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
  \endcode

  The robot moves to the specified position with the default
  positioning velocity vpRobotAfma6::defaultPositioningVelocity. The
  setPositioningVelocity() method allows to change the maximal
  velocity used to reach the desired position.

  \code
  // Set the max velocity to 40%
  robot.setPositioningVelocity(40);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
  \endcode

  To control the robot in velocity, you may set the controller to
  velocity control and than send the velocities. To end the velocity
  control and stop the robot you have to set the controller to the
  stop state. Here is an example of a velocity control in the joint
  space:

  \code
  vpColVector qvel(6);
  // Set a joint velocity
  qvel[0] = 0.1;    // x axis, in m/s
  qvel[1] = 0.2;    // y axis, in m/s
  qvel[2] = 0;      // z axis, in m/s
  qvel[3] = M_PI/8; // rotation around A axis, in rad/s
  qvel[4] = 0;      // rotation around B axis, in rad/s
  qvel[5] = 0;      // rotation around C axis, in rad/s

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  while (...) {
    // Apply a velocity in the joint space
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qvel);

    // Compute new velocities qvel...
  }

  // Stop the robot
  robot.setRobotState(vpRobot::STATE_STOP)
  \endcode

  There is also possible to measure the robot current position with
  getPosition() method and the robot current velocities with the getVelocity()
  method.

  For convenience, there is also the ability to read/write joint
  positions from a position file with readPosFile() and writePosFile()
  methods.
*/
class VISP_EXPORT vpRobotAfma6 : public vpAfma6, public vpRobot
{

private: /* Not allowed functions. */
  /*!
    Copy constructor not allowed.
   */
  vpRobotAfma6(const vpRobotAfma6 &robot);

private: /* Attributs prives. */
  /** \brief Vrai ssi aucun objet de la classe vpRobotAfma6 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotAfma6, car il correspond a un seul robot AFMA6. Creer
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
  vpHomogeneousMatrix fMc_prev_getdis;

public: /* Constantes */
  /* Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double defaultPositioningVelocity; // = 20.0;

public: /* Methode publiques */
  explicit vpRobotAfma6(bool verbose = true);
  virtual ~vpRobotAfma6(void);

  bool checkJointLimits(vpColVector &jointsStatus);

  void closeGripper();

  void getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement);

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp);

  double getPositioningVelocity(void);
  bool getPowerState();
  double getTime() const;

  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp);

  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame, double &timestamp);

  void get_cMe(vpHomogeneousMatrix &_cMe) const;
  void get_cVe(vpVelocityTwistMatrix &_cVe) const;
  void get_eJe(vpMatrix &_eJe);
  void get_fJe(vpMatrix &_fJe);

  void init(void);
  void init(vpAfma6::vpAfma6ToolType tool, const vpHomogeneousMatrix &eMc);
  void init(vpAfma6::vpAfma6ToolType tool, const std::string &filename);
  void
  init(vpAfma6::vpAfma6ToolType tool,
       vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

  void move(const std::string &filename);
  void move(const std::string &filename, const double velocity);

  void openGripper();

  void powerOn();
  void powerOff();

  static bool readPosFile(const std::string &filename, vpColVector &q);
  static bool savePosFile(const std::string &filename, const vpColVector &q);

  /* --- POSITIONNEMENT --------------------------------------------------- */
  void setPosition(const vpRobot::vpControlFrameType frame, const vpPoseVector &pose);
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
  void setPosition(const vpRobot::vpControlFrameType frame, const double pos1, const double pos2, const double pos3,
                   const double pos4, const double pos5, const double pos6);
  void setPosition(const std::string &filename);
  void setPositioningVelocity(const double velocity);
  void set_eMc(const vpHomogeneousMatrix &eMc);

  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  /* --- VITESSE ---------------------------------------------------------- */

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity);

  void stopMotion();
};

#endif
#endif /* #ifndef vpRobotAfma6_h */
