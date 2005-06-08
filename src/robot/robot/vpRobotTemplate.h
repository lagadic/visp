
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotTemplate.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotTemplate.h,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines a robot just to show which function you must
 *     implement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpRobotTemplate_H
#define vpRobotTemplate_H

/*!
  \file vpRobotTemplate.h
  \brief class that defines a robot just to show which function you must
  implement
*/

#include <visp/vpRobot.h>


/*!
  \file vpRobotTemplate
  \brief class that defines a robot just to show which function you must
  implement
*/

class vpRobotTemplate : public vpRobot
{

public:

  //! basic initialization
  void init() ;

  //! constructor
  vpRobotTemplate() ;
  //! destructor
  ~vpRobotTemplate() ;


  //! get the robot Jacobian expressed in the end-effector frame
   void get_eJe(vpMatrix &_eJe)  const  ;
  //! get the robot Jacobian expressed in the robot reference frame
   void get_fJe(vpMatrix &_fJe)  const  ;

  //! send to the controller a velocity expressed in the camera frame
   void sendCameraVelocity(const vpColVector &v)   ;
  //! send to the controller a velocity expressed in the articular frame
   void sendArticularVelocity(const vpColVector &qdot)  ;
  //! send to the controller a velocity (frame as to ve specified)
   void setVelocity(const int frame,const  vpColVector &vel)  ;

  //! get a position expressed in the robot reference frame
   void getPosition(vpPoseVector &q) const   ;
  //! get a position expressed in the articular frame
   void getArticularPosition(vpColVector &q)  const  ;
  //! get a displacement (frame as to ve specified)
   void getPosition(const int frame, vpColVector &q) const   ;

  //! get a displacement expressed in the camera frame
   void getCameraDisplacement(vpColVector &v)  const  ;
  //! get a displacement expressed  in the articular frame
   void getArticularDisplacement(vpColVector &qdot) const   ;
  //! get a displacement (frame as to ve specified)
   void getDisplacement(int frame, vpColVector &q) const   ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
