
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobot.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobot.h,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines a generic virtual robot
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpRobot_H
#define vpRobot_H

/*!
  \file vpRobot.h
  \brief class that defines a generic virtual robot
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>


/*!
  \class vpRobot
  \brief class that defines a generic virtual robot
*/
class vpRobot
{
public:
  enum RobotStateType
    {
      STATE_STOP,
      STATE_VELOCITY_CONTROL,
      STATE_POSITION_CONTROL,
      STATE_ACCELERATION_CONTROL
    }  ;

    /** \brief valeur utilisee par default pour l'etat du robot a
     * la construction. */
  static const vpRobot::RobotStateType
  defaultEtatRobot = vpRobot::STATE_STOP;

  enum ControlFrameType
    {
      REFERENCE_FRAME,
      ARTICULAR_FRAME,
      CAMERA_FRAME
    }  ;

    static const vpRobot::ControlFrameType
    defaultFrameRobot = vpRobot::CAMERA_FRAME ;

private:  /* Membres privees */
    vpRobot::RobotStateType   stateRobot;
    vpRobot::ControlFrameType   frameRobot;
public:
  virtual RobotStateType
  setRobotState (const vpRobot::RobotStateType newState);
  virtual
  RobotStateType     getRobotState (void) { return stateRobot ; }

  ControlFrameType   setRobotFrame (vpRobot::ControlFrameType newFrame);
  ControlFrameType   getRobotFrame (void) { return frameRobot ; }


protected:
  //! number of degrees of freedom
  int nDof ;
  //! robot Jacobian expressed in the end-effector frame
  vpMatrix eJe ;
  //! is the robot Jacobian expressed in the end-effector frame available
  int eJeAvailable ;
  //! robot Jacobian expressed in the robot reference frame available
  vpMatrix fJe ;
  //! is the robot Jacobian expressed in the robot reference frame available
  int fJeAvailable ;

public:
  virtual void init() = 0 ;


  virtual ~vpRobot() { ; }

  //---------- Jacobian -----------------------------
  //! get the robot Jacobian expressed in the end-effector frame
  virtual void get_eJe(vpMatrix &_eJe)  = 0 ;
  //! get the robot Jacobian expressed in the robot reference frame
  virtual void get_fJe(vpMatrix &_fJe)  = 0 ;


  //! set to the controller a velocity (frame as to ve specified)
  virtual void setVelocity(const vpRobot::ControlFrameType frame,
			   const vpColVector &vel) = 0 ;

  //---------- POSITION -----------------------------

  //! get a displacement (frame as to ve specified)
  virtual void getPosition(const vpRobot::ControlFrameType frame,
			   vpColVector &q)   = 0 ;

  //! get a displacement (frame as to ve specified)
  vpColVector getPosition (const vpRobot::ControlFrameType frame);

  //! set a displacement (frame as to ve specified)
  virtual void setPosition(const vpRobot::ControlFrameType frame,
			   const vpColVector &q)   = 0 ;


  //! get a displacement expressed in the camera frame
  virtual void getCameraDisplacement(vpColVector &v) const  = 0 ;
  //! get a displacement expressed  in the articular frame
  virtual void getArticularDisplacement(vpColVector  &qdot)  const = 0 ;
  //! get a displacement (frame as to ve specified)
  virtual void getDisplacement(const vpRobot::ControlFrameType frame,
			       vpColVector &q)  const = 0 ;

  /*
    Joint limits stuff
  */
private:
  int areJointLimitsAvailable ;
  double *qmin;
  double *qmax ;
public:
  //  virtual void getJointLimits() ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
