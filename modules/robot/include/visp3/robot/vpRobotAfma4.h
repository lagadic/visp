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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotAfma4_h
#define vpRobotAfma4_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_AFMA4

#include <iostream>
#include <stdio.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpAfma4.h>
#include <visp3/robot/vpRobot.h>

// low level controller api
extern "C" {
#include "irisa_Afma4.h"
#include "trycatch.h"
}

/*!
  \class vpRobotAfma4

  \ingroup group_robot_real_cylindrical

  \brief Control of Irisa's cylindrical robot named Afma4.

  Implementation of the vpRobot class in order to control Irisa's
  Afma4 robot.  This robot is a cylindrical robot with five degrees of
  freedom but only four are actuated (see vpAfma4 documentation). It
  was manufactured in 1995 by the french Afma-Robots company. In 2008,
  the low level controller change for a more recent Adept technology
  based on the MotionBlox controller. A firewire camera is mounted on
  the end-effector to allow eye-in-hand visual servoing. The control
  of this camera is achieved by the vp1394TwoGrabber class. A
  Servolens lens is attached to the camera. It allows to control the
  focal lens, the iris and the focus throw a serial link. The control
  of the lens is possible using the vpServolens class.

  This class allows to control the Afma4 cylindrical robot in position
  and velocity:
  - in the joint space (vpRobot::ARTICULAR_FRAME),
  - in the fixed reference frame (vpRobot::REFERENCE_FRAME),
  - in the camera frame (vpRobot::CAMERA_FRAME),

  Mixed frame (vpRobot::MIXT_FRAME) where translations are expressed
  in the reference frame and rotations in the camera frame is not implemented.

  All the translations are expressed in meters for positions and m/s
  for the velocities. Rotations are expressed in radians for the
  positions, and rad/s for the rotation velocities.

  The Denavit-Hartenberg representation as well as the direct and
  inverse kinematics models are given and implemented in the vpAfma4
  class.

  \warning A Ctrl-C, a segmentation fault or other system errors are
  catched by this class to stop the robot.

  To communicate with the robot, you may first create an instance of this
  class by calling the default constructor:

  \code
  vpRobotAfma4 robot;
  \endcode

  This initialize the robot kinematics with the eMc extrinsic camera
  parameters.

  To control the robot in position, you may set the controller
  to position control and than send the position to reach in a specific
  frame like here in the joint space:

  \code
  vpColVector q(4);
  // Set a joint position
  q[0] =  M_PI/2; // X axis, in radian
  q[1] =  0.2;    // Y axis, in meter
  q[2] = -M_PI/2; // A axis, in radian
  q[3] =  M_PI/8; // B axis, in radian

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
  \endcode

  The robot moves to the specified position with the default
  positioning velocity vpRobotAfma4::defaultPositioningVelocity. The
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
  qvel[0] = M_PI/8; // X axis, in rad/s
  qvel[1] = 0.2;    // Y axis, in m/s
  qvel[2] = 0;      // A axis, in rad/s
  qvel[3] = M_PI/8; // B axis, in rad/s

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
class VISP_EXPORT vpRobotAfma4 : public vpAfma4, public vpRobot
{

private: /* Not allowed functions. */
  /*!
    Copy constructor not allowed.
   */
  vpRobotAfma4(const vpRobotAfma4 &robot);

private: /* Attributs prives. */
  /** \brief Vrai ssi aucun objet de la classe vpRobotAfma4 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotAfma4, car il correspond a un seul robot AFMA4. Creer
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

public: /* Constantes */
  /* Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double defaultPositioningVelocity; // = 20.0;

public: /* Methode publiques */
  explicit vpRobotAfma4(bool verbose = true);
  virtual ~vpRobotAfma4(void);

  void getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp);

  double getPositioningVelocity(void);
  bool getPowerState();

  double getTime() const;

  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp);

  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame, double &timestamp);

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_cVf(vpVelocityTwistMatrix &cVf) const;
  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  void init(void);

  void move(const char *filename);

  void powerOn();
  void powerOff();

  static bool readPosFile(const std::string &filename, vpColVector &q);
  static bool savePosFile(const std::string &filename, const vpColVector &q);

  /* --- POSITIONNEMENT --------------------------------------------------- */
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
  void setPosition(const vpRobot::vpControlFrameType frame, const double q1, const double q2, const double q4,
                   const double q5);
  void setPosition(const char *filename);
  void setPositioningVelocity(const double velocity);

  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  /* --- VITESSE ---------------------------------------------------------- */

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity);

  void stopMotion();
};

#endif
#endif /* #ifndef vpRobotAfma4_h */
