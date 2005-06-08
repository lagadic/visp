
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
 *  $Id: vpRobotTemplate.cpp,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines a robot just to show which function you must
 *     implement
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



/*!
  \file vpRobotTemplate.cpp
  \brief class that defines a robot just to show which function you must
  *     implement
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotTemplate.h>
#include <visp/vpDebug.h>

//! basic initialization
void vpRobotTemplate::init()
{
  TRACE(" Get the joint limits " ) ;
  cout << "Not implemented ! " << endl;
}

//! constructor
vpRobotTemplate::vpRobotTemplate()
{
  init() ;
}


//! constructor
vpRobotTemplate::~vpRobotTemplate()
{
  cout << "Not implemented ! " << endl;
}

/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

get_eJe
get_fJe

*/

//! get the robot Jacobian expressed in the end-effector frame
void
vpRobotTemplate::get_eJe(vpMatrix &_eJe) const
{
  cout << "Not implemented ! " << endl;
}

//! get the robot Jacobian expressed in the robot reference frame
void
vpRobotTemplate::get_fJe(vpMatrix &_fJe) const
{
  cout << "Not implemented ! " << endl;
}


/*

AT LEAST ONE OF THESE TWO FUNCTIONS HAS TO BE IMPLEMENTED

sendCameraVelocity
sendArticularVelocity


*/
//! send to the controller a velocity expressed in the camera frame
void vpRobotTemplate::sendCameraVelocity(const vpColVector &v)
{
  cout << "Not implemented ! " << endl;
  cout << "To implement me you need : " << endl ;
  cout << "\t to known the robot jacobian expressed in " ;
  cout << "the end-effector frame (eJe) " <<endl ;
  cout << "\t the frame transformation  between camera frame " ;
  cout << "and end-effector frame (cMe)" << endl ;
}

//! send to the controller a velocity expressed in the articular frame
void
vpRobotTemplate::sendArticularVelocity(const  vpColVector &qdot)
{
  cout << "Not implemented ! " << endl;
}
//! send to the controller a velocity (frame as to ve specified)
void
vpRobotTemplate::setVelocity(const int frame, const vpColVector &vel)
{
  cout << "Not implemented ! " << endl;
}


/*

THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

//! get a position expressed in the robot reference frame
void
vpRobotTemplate::getPosition(vpPoseVector &q) const
{
  cout << "Not implemented ! " << endl;
}
//! get a position expressed in the articular frame
void
vpRobotTemplate::getArticularPosition(vpColVector &q) const
{ cout << "Not implemented ! " << endl;
}
//! get a displacement (frame as to ve specified)
void
vpRobotTemplate::getPosition( const  int frame, vpColVector &q) const
{
  cout << "Not implemented ! " << endl;
}
//! get a displacement expressed in the camera frame
void
vpRobotTemplate::getCameraDisplacement(vpColVector &v)  const
{
  cout << "Not implemented ! " << endl;
}
//! get a displacement expressed  in the articular frame
void
vpRobotTemplate::getArticularDisplacement(vpColVector &qdot) const
{
  cout << "Not implemented ! " << endl;
}
//! get a displacement (frame as to ve specified)
void
vpRobotTemplate::getDisplacement( const  int frame, vpColVector &q) const
{
  cout << "Not implemented ! " << endl;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
