/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_VIPER850

#include <iostream>
#include <stdio.h>

#include <visp/vpRobot.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpViper850.h>

// low level controller api
extern "C" {
#  include "irisa_Viper850.h"
#  include "trycatch.h"
}


/*!
  \class vpRobotViper850

  \ingroup Viper

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
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
#endif
}
  \endcode

  This initialize the robot kinematics with the \f$^e{\bf M}_c\f$
  extrinsic camera parameters obtained with a projection model without
  distorsion. To set the robot kinematics with the \f$^e{\bf M}_c\f$
  transformation obtained with a camera perspective model including
  distorsion you need to initialize the robot with:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective 
  // projection model including a distorsion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA,
	     vpCameraParameters::perspectiveProjWithDistortion);
#endif
}
  \endcode
 
  You can get the intrinsic camera parameters of an image
  acquired by the camera attached to the robot, with:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpCameraParameters.h>

int main()
{
#if defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394_2)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.acquire(I);

  vpRobotViper850 robot;

  // ...

  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model 
  // with distorsion.  
#endif
}
  \endcode

  To control the robot in position, you may set the controller
  to position control and than send the position to reach in a specific
  frame like here in the joint space:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

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
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

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
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

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

  while (1) {
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

  /*! \enum Control mode. */
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
    Copy contructor not allowed.
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

  vpRobotViper850 (void);
  virtual ~vpRobotViper850 (void);

  void init (void);
  void init (vpViper850::vpToolType tool,
             vpCameraParameters::vpCameraParametersProjType
	     projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

  // State
  vpRobot::vpRobotStateType setRobotState (vpRobot::vpRobotStateType newState);
  /*!
    \return The control mode indicating if the robot is in automatic, 
    manual (usage of the dead man switch) or emergnecy stop mode.
  */
  vpControlModeType getControlMode() {
    return controlMode;
  }

  // Position control
  void setPosition(const vpRobot::vpControlFrameType frame,
		   const vpColVector &position) ;
  void setPosition (const vpRobot::vpControlFrameType frame,
		    const double pos1, const double pos2, const double pos3,
		    const double pos4, const double pos5, const double pos6) ;
  void setPosition(const char *filename) ;
  void setPositioningVelocity (const double velocity);

  void getPosition (const vpRobot::vpControlFrameType frame,
		    vpColVector &position);
  void getPosition (const vpRobot::vpControlFrameType frame,
		    vpPoseVector &position);

  double getPositioningVelocity (void);

  // Velocity control
  void setVelocity (const vpRobot::vpControlFrameType frame,
		    const vpColVector & velocity);


  void getVelocity (const vpRobot::vpControlFrameType frame,
		    vpColVector & velocity);

  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

  // Force/Torque control
  void biasForceTorqueSensor();
  void getForceTorque(vpColVector &H);

public:
  void get_cMe(vpHomogeneousMatrix &cMe) ;
  void get_cVe(vpTwistMatrix &cVe) ;
  void get_eJe(vpMatrix &eJe)  ;
  void get_fJe(vpMatrix &fJe)  ;

  void stopMotion() ;
  void powerOn() ;
  void powerOff() ;
  bool getPowerState();

  void move(const char *filename) ;
  static bool readPosFile(const char *filename, vpColVector &q)  ;
  static bool savePosFile(const char *filename, const vpColVector &q)  ;

  void getCameraDisplacement(vpColVector &displacement);
  void getArticularDisplacement(vpColVector &displacement);
  void getDisplacement(vpRobot::vpControlFrameType frame, 
		       vpColVector &displacement);
};





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif
#endif /* #ifndef vpRobotViper850_h */
