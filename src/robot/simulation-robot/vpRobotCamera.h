
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotCamera.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotCamera.h,v 1.2 2006-01-13 18:35:50 fspindle Exp $
 *
 * Description
 * ============
 *      class that defines the simplest robot : a free flying camera
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpRobotCamera_H
#define vpRobotCamera_H

/*!
  \file vpRobotCamera
  \brief class that defines the simplest robot : a free flying  a camera
*/

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRobot.h>
#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpRobotCamera.h
  \brief class that defines the simplest robot :  a free flying a camera
*/
class vpRobotCamera : public vpRobot
{

private:
  //! robot / camera location in the world frame
  vpHomogeneousMatrix cMo ;


public:

  //! basic initialization
  void init() ;

  //! constructor
  vpRobotCamera() ;
  //! destructor
  ~vpRobotCamera() ;

  //! get the robot Jacobian expressed in the end-effector frame
  void get_eJe(vpMatrix &_eJe)    ;
  //! get the robot Jacobian expressed in the robot reference frame
  void get_fJe(vpMatrix &_fJe)    ;

  //! send to the controller a velocity expressed in the camera frame
  void setCameraVelocity(const vpColVector &v)   ;
  //! send to the controller a velocity expressed in the articular frame
  void setArticularVelocity(const vpColVector &qdot)  ;
  //! send to the controller a velocity (frame as to ve specified)
  void setVelocity(const vpRobot::ControlFrameType frame,
		   const  vpColVector &vel)  ;

  //! get a position expressed in the robot reference frame
  void getPosition(vpColVector &q)    ;
  //! get a position expressed in the robot reference frame
  void getPosition(vpHomogeneousMatrix &cMo) const   ;
  //! get a position expressed in the articular frame
  void getArticularPosition(vpColVector &q)  const  ;
  //! get a displacement (frame as to ve specified)
  void getPosition(const vpRobot::ControlFrameType repere,
		   vpColVector &q)  ;
 //! get a displacement (frame as to ve specified)
  void setPosition(const vpRobot::ControlFrameType repere,
		   const vpColVector &q)  { ; }


  void setPosition(const vpHomogeneousMatrix &_cMo) ;
  void setPosition(const vpColVector &q) { ;}

  //! get a displacement expressed in the camera frame
  void getCameraDisplacement(vpColVector &v)  ;
  //! get a displacement expressed  in the articular frame
  void getArticularDisplacement(vpColVector &qdot) ;
  //! get a displacement (frame as to ve specified)
  void getDisplacement(const vpRobot::ControlFrameType repere,
		       vpColVector &q) ;

} ;

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
