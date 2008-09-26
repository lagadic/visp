/****************************************************************************
 *
 * $Id: vpRobotAfma6.h,v 1.23 2008-09-26 15:20:55 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Interface for the Irisa's Afma6 robot controlled by an Adept MotionBlox.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotAfma6_h
#define vpRobotAfma6_h

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA6

#include <iostream>
#include <stdio.h>

#include <visp/vpRobot.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpAfma6.h>

// low level controller api
extern "C" {
#  include "irisa.h"
#  include "trycatch.h"
}


/*!
  \class vpRobotAfma6

  \ingroup Afma6

  \brief Control of Irisa's gentry robot named Afma6.

  Implementation of the vpRobot class in order to control Irisa's
  Afma6 robot.  This robot is a gentry robot with six degrees of
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

  This class allows to control the Afma6 gentry robot in position
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
  vpRobotAfma6 robot;
  \endcode

  This initialize the robot kinematics with the eMc extrinsic camera
  parameters obtained with a projection model without distorsion. To
  set the robot kinematics with the eMc matrix obtained with a camera
  perspective model including distorsion you need to initialize the
  robot with:

  \code
  // Set the extrinsic camera parameters obtained with a perpective 
  // projection model including a distorsion parameter
  robot.init(vpAfma6::CAMERA_DRAGONFLY2_8MM, vpCameraParameters::perspectiveProjWithDistortion);
  \endcode
 
  You can get the intrinsic camera parameters of the image I
  acquired with the camera, with:

  \code
  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model 
  // with distorsion.  
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
  positions from a position file with readPosFile() and writePodFile()
  methods.
*/
class VISP_EXPORT vpRobotAfma6
  :
  public vpAfma6,
  public vpRobot
{

private: /* Methodes implicites interdites. */

  /*!
    Copy contructor not allowed.
   */
  vpRobotAfma6 (const vpRobotAfma6 & ass);

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


public: /* Methodes */

  void init (void);
  void init (vpAfma6::vpAfma6CameraRobotType camera,
             vpCameraParameters::vpCameraParametersProjType
	     projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

public:  /* Constantes */

  /* Vitesse maximale par default lors du positionnement du robot.
   * C'est la valeur a la construction de l'attribut prive \a
   * positioningVelocity. Cette valeur peut etre changee par la fonction
   * #setPositioningVelocity.
   */
  static const double defaultPositioningVelocity; // = 20.0;

public:  /* Methode publiques */

  vpRobotAfma6 (void);
  virtual ~vpRobotAfma6 (void);


  /* --- ETAT ------------------------------------------------------------- */

  vpRobot::vpRobotStateType setRobotState (vpRobot::vpRobotStateType newState);

  /* --- POSITIONNEMENT --------------------------------------------------- */
  void setPosition(const vpRobot::vpControlFrameType frame,
		   const vpColVector &position) ;
  void setPosition (const vpRobot::vpControlFrameType frame,
		    const double pos1, const double pos2, const double pos3,
		    const double pos4, const double pos5, const double pos6) ;
  void setPosition(const char *filename) ;
  void getPosition (const vpRobot::vpControlFrameType frame,
		    vpColVector &position);


  void   setPositioningVelocity (const double velocity);
  double getPositioningVelocity (void);

  /* --- VITESSE ---------------------------------------------------------- */

  void setVelocity (const vpRobot::vpControlFrameType frame,
		    const vpColVector & velocity);


  void getVelocity (const vpRobot::vpControlFrameType frame,
		    vpColVector & velocity);

  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

public:
  void get_cMe(vpHomogeneousMatrix &_cMe) ;
  void get_cVe(vpTwistMatrix &_cVe) ;
  void get_eJe(vpMatrix &_eJe)  ;
  void get_fJe(vpMatrix &_fJe)  ;

  void stopMotion() ;
  void powerOn() ;
  void powerOff() ;
  bool getPowerState();

  void move(const char *filename) ;
  static bool readPosFile(const char *filename, vpColVector &q)  ;
  static bool savePosFile(const char *filename, const vpColVector &q)  ;

  void openGripper() ;
  void closeGripper() ;

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
#endif /* #ifndef vpRobotAfma6_h */
