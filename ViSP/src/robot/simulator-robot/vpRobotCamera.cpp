/****************************************************************************
 *
 * $Id: vpRobotCamera.cpp 2456 2010-01-07 10:33:12Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Defines the simplest robot : a free flying camera.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpRobotCamera.cpp
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpRobotException.h>
#include <visp/vpDebug.h>
#include <visp/vpExponentialMap.h>


/*!
  Constructor.

  Initialise the robot by a call to init().

  Sampling time is set to 40 ms. To change it you should call
  setSamplingTime().

  Robot jacobian expressed in the end-effector frame \f$ {^e}{\bf J}_e \f$
  is set to identity (see get_eJe()).

  \code
  vpRobotCamera robot;

  robot.setSamplingTime(0.020); // Set the sampling time to 20 ms.

  \endcode

*/
vpRobotCamera::vpRobotCamera()
{
  init() ;
}

/*!
  Robot initialisation.

  Sampling time is set to 40 ms. To change it you should call
  setSamplingTime().

  Robot jacobian expressed in the end-effector frame \f$ {^e}{\bf J}_e \f$
  is set to identity (see get_eJe()).

*/
void vpRobotCamera::init()
{
  eJe.resize(6,6) ;
  eJe.setIdentity() ;
  setSamplingTime(0.040f);
}


/*!
  Destructor.

*/
vpRobotCamera::~vpRobotCamera()
{
}

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

get_eJe
get_fJe

*/

/*!
  Get the robot jacobian expressed in the end-effector frame.
  For that simple robot the Jacobian is the identity.

  \param eJe : A 6 by 6 matrix representing the robot jacobian \f$ {^e}{\bf
  J}_e\f$ expressed in the end-effector frame.
*/
void
vpRobotCamera::get_eJe(vpMatrix &eJe)
{
  eJe = this->eJe ;
}

/*!
  Get the robot Jacobian expressed in the robot reference frame.

  \warning Not implemented.
*/
void
vpRobotCamera::get_fJe(vpMatrix & /* fJe */)
{
  std::cout << "Not implemented ! " << std::endl;
}


/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

sendCameraVelocity
sendArticularVelocity


*/

/*!
  Send to the controller a velocity expressed in the camera frame.

  \param v : Camera velocity represented by a 6 dimension vector \f$ {\bf v} =
  [{\bf t}, {\bf \theta u }]^t \f$ where \f$ \bf t \f$ is a translation vector
  and \f$ {\bf \theta u} \f$ is a rotation vector (see vpThetaUVector): \f$
  {\bf v} = [t_x, t_y, t_z, {\theta u}_x, {\theta u}_y, {\theta u}_z] \f$ (see
  vpTranslationVector and vpThetaUVector).

  We use the exponential map (vpExponentialMap) to update the camera location.
  Sampling time can be set using setSamplingTime().

  \sa setSamplingTime()
*/
void
vpRobotCamera::setCameraVelocity(const vpColVector &v)
{
  cMo = vpExponentialMap::direct(v, delta_t).inverse()*cMo ;
}

/*!

  Send to the controller a velocity expressed in the articular frame.

  \param qdot : Articular velocity represented by a 6 dimension vector \f$
  \dot{{\bf q}} = [{\bf t}, {\bf \theta u}]^t \f$ where \f$ \bf t \f$ is a
  translation vector and \f$ {\bf \theta u} \f$ is a rotation vector (see
  vpThetaUVector): \f$ \dot{{\bf q}} = [t_x, t_y, t_z, {\theta u}_x, {\theta
  u}_y, {\theta u}_z] \f$ (see vpTranslationVector and vpThetaUVector). The
  robot jacobian \f$ {^e}{\bf J}_e\f$ expressed in the end-effector frame is
  here set to identity.

  We use the exponential map (vpExponentialMap) to update the camera location.
  Sampling time can be set using setSamplingTime().

  \sa setSamplingTime()
*/
void
vpRobotCamera::setArticularVelocity(const vpColVector &qdot)
{
  cMo = vpExponentialMap::direct(qdot, delta_t).inverse()*cMo ;
}

/*!
  Send to the controller a velocity.

  \param frame : Control frame type. Only articular (vpRobot::ARTICULAR_FRAME)
  and camera frame (vpRobot::CAMERA_FRAME) are implemented.

  \param v : Velocity to apply to the robot.

  - In the camera drame, this velocity is represented by a 6 dimension vector
  \f$ {\bf v} = [{\bf t}, {\bf \theta u }]^t \f$ where \f$ \bf t \f$ is a
  translation vector and \f$ {\bf \theta u} \f$ is a rotation vector (see
  vpThetaUVector): \f$ {\bf v} = [t_x, t_y, t_z, {\theta u}_x, {\theta u}_y,
  {\theta u}_z] \f$ (see vpTranslationVector and vpThetaUVector).

  - In articular, this velocity is represented by a 6 dimension vector \f$
  \dot{{\bf q}} = [{\bf t}, {\bf \theta u}]^t \f$ where \f$ \bf t \f$ is a
  translation vector and \f$ {\bf \theta u} \f$ is a rotation vector (see
  vpThetaUVector): \f$ \dot{{\bf q}} = [t_x, t_y, t_z, {\theta u}_x, {\theta
  u}_y, {\theta u}_z] \f$ (see vpTranslationVector and vpThetaUVector). The
  robot jacobian \f$ {^e}{\bf J}_e\f$ expressed in the end-effector frame is
  here set to identity.

  We use the exponential map (vpExponentialMap) to update the camera location.
  Sampling time can be set using setSamplingTime().

  \sa setSamplingTime()

*/
void
vpRobotCamera::setVelocity(const vpRobot::vpControlFrameType frame,
			   const vpColVector &v)
{
  switch (frame)
  {
  case vpRobot::ARTICULAR_FRAME:
    setArticularVelocity(v) ;
    break ;
  case vpRobot::CAMERA_FRAME:
    setCameraVelocity(v) ;
    break ;
  case vpRobot::REFERENCE_FRAME:
    vpERROR_TRACE ("Cannot set a velocity in the reference frame: "
		   "functionality not implemented");
    throw vpRobotException (vpRobotException::wrongStateError,
			    "Cannot set a velocity in the reference frame:"
			    "functionality not implemented");
    break ;
  case vpRobot::MIXT_FRAME:
    vpERROR_TRACE ("Cannot set a velocity in the mixt frame: "
		 "functionality not implemented");
    throw vpRobotException (vpRobotException::wrongStateError,
			    "Cannot set a velocity in the mixt frame:"
			    "functionality not implemented");

    break ;
  }
}


/*

THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

/*!

  Get a position expressed in the robot reference frame.  We consider that the
  "robot" reference frame is the world reference so we return \f$ {^c}{\bf M}_o
  \f$ (or at least the corresponding vpPoseVector)

  \warning Not implemented.
*/
void
vpRobotCamera::getPosition(vpColVector & /*cpo*/)
{
  std::cout << "Not implemented ! " << std::endl;
  //  cpo.buildFrom(cMo) ;
}

/*!
  Get a position expressed in the robot reference frame.

*/
void
vpRobotCamera::getPosition(vpHomogeneousMatrix &cMo) const
{
  cMo = this->cMo ;
}
/*!
  Set a position expressed in the robot reference frame.
*/
void
vpRobotCamera::setPosition(const vpHomogeneousMatrix &cMo)
{
   this->cMo = cMo ;
}

/*!
  Get a position expressed in the articular frame.

  \warning Not implemented.
*/
void
vpRobotCamera::getArticularPosition(vpColVector &/* q */) const
{
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement (frame as to ve specified).

  \warning Not implemented.
*/
void
vpRobotCamera::getPosition(const vpRobot::vpControlFrameType /* frame */,
			   vpColVector & /* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement expressed in the camera frame.

  \warning Not implemented.
*/
void
vpRobotCamera::getCameraDisplacement(vpColVector & /* v */)
{
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement expressed  in the articular frame.

  \warning Not implemented.
*/
void
vpRobotCamera::getArticularDisplacement(vpColVector & /* qdot */)
{
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement depending on the control frame type.

  \warning Not implemented.
*/
void
vpRobotCamera::getDisplacement(const vpRobot::vpControlFrameType /* frame */,
			       vpColVector &/* q */)
{
  std::cout << "Not implemented ! " << std::endl;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
