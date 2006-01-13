
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
 *  $Id: vpRobot.h,v 1.5 2006-01-13 18:18:58 fspindle Exp $
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
      CAMERA_FRAME,
      MIXT_FRAME
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
  double maxTranslationVelocity;
  static const double maxTranslationVelocityDefault;// = 0.2;
  double maxRotationVelocity;
  static const double maxRotationVelocityDefault;// = 0.7;


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

  vpRobot (void);
  virtual ~vpRobot() { ; }

  //---------- Jacobian -----------------------------
  //! get the robot Jacobian expressed in the end-effector frame
  virtual void get_eJe(vpMatrix &_eJe)  = 0 ;
  //! get the robot Jacobian expressed in the robot reference frame
  virtual void get_fJe(vpMatrix &_fJe)  = 0 ;


  //! set to the controller a velocity (frame as to ve specified)
  virtual void setVelocity(const vpRobot::ControlFrameType frame,
			   const vpColVector &vel) = 0 ;


  //! Set the maximal velocity that can be sent to the robot (in
  // translation).
  void setMaxTranslationVelocity (const double maxVt);
  //! Get the maximal velocity that can be sent to the robot (in
  // translation).
  double getMaxTranslationVelocity (void) const ;
  //! Set the maximal velocity that can be sent to the robot (in
  // rotation).
  void setMaxRotationVelocity (const double maxVr);
  //! Get the maximal velocity that can be sent to the robot (in
  // rotation).
  double getMaxRotationVelocity (void) const;

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
  virtual void getCameraDisplacement(vpColVector &v) = 0 ;
  //! get a displacement expressed  in the articular frame
  virtual void getArticularDisplacement(vpColVector  &qdot) = 0 ;
  //! get a displacement (frame as to ve specified)
  virtual void getDisplacement(const vpRobot::ControlFrameType frame,
			       vpColVector &q) = 0 ;

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
