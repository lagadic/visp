/****************************************************************************
 *
 * $Id: vpRobotCamera.h,v 1.5 2007-04-27 16:40:15 fspindle Exp $
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
 * Defines the simplest robot : a free flying camera.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpRobotCamera_H
#define vpRobotCamera_H

/*!
  \file vpRobotCamera.h
  \brief class that defines the simplest robot : a free flying  a camera
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRobot.h>
#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpRobotCamera
  \brief class that defines the simplest robot :  a free flying a camera
*/
class VISP_EXPORT vpRobotCamera : public vpRobot
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
  void getPosition(const vpRobot::ControlFrameType frame,
		   vpColVector &q)  ;
 //! get a displacement (frame as to ve specified)
  void setPosition(const vpRobot::ControlFrameType /* frame */,
		   const vpColVector & /* q */)  { ; }


  void setPosition(const vpHomogeneousMatrix &_cMo) ;
  void setPosition(const vpColVector & /* q */) { ;}

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
