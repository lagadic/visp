
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotCamera.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotCamera.cpp,v 1.3 2005-11-09 15:22:04 marchand Exp $
 *
 * Description
 * ============
 *      class that defines the simplest robot : a free flying camera
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



/*!
  \file vpRobotCamera.cpp
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpRobotException.h>
#include <visp/vpDebug.h>

//! basic initialization
void vpRobotCamera::init()
{
  eJe.resize(6,6) ;
  eJe.setIdentity() ;
}

//! constructor
vpRobotCamera::vpRobotCamera()
{
  init() ;
}


//! constructor
vpRobotCamera::~vpRobotCamera()
{
}

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

get_eJe
get_fJe

*/

//! get the robot Jacobian expressed in the end-effector frame
//! for that simple robot the Jacobian is the identity
void
vpRobotCamera::get_eJe(vpMatrix &_eJe)
{
  _eJe = eJe ;
}

//! get the robot Jacobian expressed in the robot reference frame
void
vpRobotCamera::get_fJe(vpMatrix &_fJe)
{
  cout << "Not implemented ! " << endl;
}


/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

sendCameraVelocity
sendArticularVelocity


*/

//! send to the controller a velocity expressed in the camera frame
//! we use the exponential map to update the camera location
void
vpRobotCamera::setCameraVelocity(const vpColVector &v)
{
  cMo = expMap(v).inverse()*cMo ;
}

//! send to the controller a velocity expressed in the articular frame
void
vpRobotCamera::setArticularVelocity(const vpColVector &qdot)
{
  cMo = expMap(qdot).inverse()*cMo ;
}

//! send to the controller a velocity (frame as to ve specified)
void
vpRobotCamera::setVelocity(const vpRobot::ControlFrameType frame,
			   const vpColVector &vel)
{
  switch (frame)
  {
  case vpRobot::REFERENCE_FRAME:
    break ;
  case vpRobot::CAMERA_FRAME:
    setCameraVelocity(vel) ;
    break ;
  case vpRobot::ARTICULAR_FRAME:
    setArticularVelocity(vel) ;
    break ;
  case vpRobot::MIXT_FRAME:
    ERROR_TRACE ("Cannot set a velocity in the mixt frame: "
		 "functionality not implemented");
    throw vpRobotException (vpRobotException::wrongStateError,
			    "Cannot get a velocity in the reference frame:"
			    "functionality not implemented");

    break ;
  }
}


/*

THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

//! get a position expressed in the robot reference frame
//! we consider that the "robot" reference frame is the world reference
//! so we return cMo (or at least the corresponding vpPoseVector)
void
vpRobotCamera::getPosition(vpColVector &cpo)
{
  //  cpo.buildFrom(cMo) ;
}
//! get a position expressed in the robot reference frame
void
vpRobotCamera::getPosition(vpHomogeneousMatrix &_cMo) const
{
  _cMo = cMo ;
}
//! get a position expressed in the robot reference frame
void
vpRobotCamera::setPosition(const vpHomogeneousMatrix &_cMo)
{
   cMo = _cMo ;
}

//! get a position expressed in the articular frame
void
vpRobotCamera::getArticularPosition(vpColVector &q) const
{ cout << "Not implemented ! " << endl;
}

//! get a displacement (frame as to ve specified)
void
vpRobotCamera::getPosition(const vpRobot::ControlFrameType repere,
			   vpColVector &q)
{
  cout << "Not implemented ! " << endl;
}

//! get a displacement expressed in the camera frame
void
vpRobotCamera::getCameraDisplacement(vpColVector &v) const
{
  cout << "Not implemented ! " << endl;
}

//! get a displacement expressed  in the articular frame
void
vpRobotCamera::getArticularDisplacement(vpColVector &qdot) const
{
  cout << "Not implemented ! " << endl;
}

//! get a displacement (frame as to ve specified)
void
vpRobotCamera::getDisplacement(const vpRobot::ControlFrameType repere,
			       vpColVector &q) const
{
  cout << "Not implemented ! " << endl;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
