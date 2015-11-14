/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Interface for the Irisa's Viper S850 robot controlled by an Adept MotionBlox.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotViper850_h
#define vpRobotViper850_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_VIPER850

#include <iostream>
#include <stdio.h>

#include <visp3/robot/vpRobot.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpViper850.h>

// low level controller api
extern "C" {
#  include "irisa_Viper850.h"
#  include "trycatch.h"
}


/*!
  \class vpRobotViper850

  \ingroup group_robot_real_arm

  \brief Control of Irisa's Viper S850 robot named Viper850.

  Implementation of the vpRobot class in order to control Irisa's
  Viper850 robot.  This robot is an ADEPT six degrees of freedom arm.
  A firewire camera is mounted on the end-effector to allow
  eye-in-hand visual servoing. The control of this camera is achieved
  by the vp1394TwoGrabber class.

  This class allows to control the Viper850 arm robot in position
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
  vpViper850 class.

  \warning A Ctrl-C, a segmentation fault or other system errors are
  catched by this class to stop the robot.

  To communicate with the robot, you may first create an instance of this
  class by calling the default constructor:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
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
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective 
  // projection model including a distortion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA,
	     vpCameraParameters::perspectiveProjWithDistortion);
#endif
}
  \endcode

  You can get the intrinsic camera parameters of an image
  acquired by the camera attached to the robot, with:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/core/vpCameraParameters.h>

int main()
{
#if defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.acquire(I);

  vpRobotViper850 robot;

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
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

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
  positioning velocity vpRobotViper850::defaultPositioningVelocity. The
  setPositioningVelocity() method allows to change the maximal
  velocity used to reach the desired position.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

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
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

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

  It is also possible to measure the robot current position with
  getPosition() method and the robot current velocities with the getVelocity()
  method.

  For convenience, there is also the ability to read/write joint
  positions from a position file with readPosFile() and savePosFile()
  methods.
*/
class VISP_EXPORT vpRobotViper850
  :
  public vpViper850,
  public vpRobot
{

public:  /* Constantes */

  /*! \enum vpControlModeType Control mode. */
  typedef enum {
    AUTO,   //!< Automatic control mode (default).
    MANUAL,  //!< Manual control mode activated when the dead man switch is in use.
    ESTOP  //!< Emergency stop activated.
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
  vpRobotViper850 (const vpRobotViper850 & robot);

private: /* Attributs prives. */

  /** \brief Vrai ssi aucun objet de la classe vpRobotViper850 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotViper850, car il correspond a un seul robot AFMA6. Creer
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


public:  /* Methode publiques */

  vpRobotViper850 (bool verbose=true);
  virtual ~vpRobotViper850 (void);

  // Force/Torque control
  void biasForceTorqueSensor() const;

  void closeGripper() const;

  void disableJoint6Limits() const;
  void enableJoint6Limits() const;

  void getDisplacement(vpRobot::vpControlFrameType frame,
                       vpColVector &displacement);
  /*!
    \return The control mode indicating if the robot is in automatic, 
    manual (usage of the dead man switch) or emergnecy stop mode.
  */
  vpControlModeType getControlMode() const {
    return controlMode;
  }

  void getForceTorque(vpColVector &H) const;

  double getMaxRotationVelocityJoint6() const;
  void getPosition (const vpRobot::vpControlFrameType frame,
                    vpColVector &position);
  void getPosition (const vpRobot::vpControlFrameType frame,
                    vpColVector &position, double &timestamp);
  void getPosition (const vpRobot::vpControlFrameType frame,
                    vpPoseVector &position);
  void getPosition (const vpRobot::vpControlFrameType frame,
                    vpPoseVector &position, double &timestamp);

  double getPositioningVelocity (void) const;
  bool getPowerState() const;

  void getVelocity (const vpRobot::vpControlFrameType frame,
                    vpColVector & velocity);
  void getVelocity (const vpRobot::vpControlFrameType frame,
                    vpColVector & velocity, double &timestamp);

  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity (const vpRobot::vpControlFrameType frame, double &timestamp);

  double getTime() const;

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  void init (void);
  void init (vpViper850::vpToolType tool,
             vpCameraParameters::vpCameraParametersProjType
             projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

  void move(const char *filename) ;

  void openGripper();

  void powerOn() ;
  void powerOff() ;

  static bool readPosFile(const char *filename, vpColVector &q)  ;
  static bool savePosFile(const char *filename, const vpColVector &q)  ;

  void setMaxRotationVelocity(double w_max);
  void setMaxRotationVelocityJoint6(double w6_max);

  // Position control
  void setPosition(const vpRobot::vpControlFrameType frame,
                   const vpColVector &position) ;
  void setPosition (const vpRobot::vpControlFrameType frame,
                    const double pos1, const double pos2, const double pos3,
                    const double pos4, const double pos5, const double pos6) ;
  void setPosition(const char *filename) ;
  void setPositioningVelocity (const double velocity);

  // State
  vpRobot::vpRobotStateType setRobotState (vpRobot::vpRobotStateType newState);

  // Velocity control
  void setVelocity (const vpRobot::vpControlFrameType frame,
                    const vpColVector & velocity);

  void stopMotion() ;

private:
  void getArticularDisplacement(vpColVector &displacement);
  void getCameraDisplacement(vpColVector &displacement);

  double maxRotationVelocity_joint6;
};





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
#endif /* #ifndef vpRobotViper850_h */
