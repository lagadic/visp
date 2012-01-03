/****************************************************************************
 *
 * $Id: vpRobotCamera.h 2456 2010-01-07 10:33:12Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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


#ifndef vpRobotCamera_H
#define vpRobotCamera_H

/*!
  \file vpRobotCamera.h
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRobot.h>
#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpRobotCamera

  \ingroup RealRobotSimu

  \brief Class that defines the simplest robot: a free flying camera.


*/
class VISP_EXPORT vpRobotCamera : public vpRobot
{

private:
  //! robot / camera location in the world frame
  vpHomogeneousMatrix cMo ;
  double delta_t; // sampling time

public:
  vpRobotCamera() ;
  virtual ~vpRobotCamera() ;

  void init() ;
  void get_eJe(vpMatrix &_eJe)    ;
  void get_fJe(vpMatrix &_fJe)    ;

  /*!
    Set the sampling time.

    \param delta_t : Sampling time used to compute the robot displacement from
    the velocity applied to the robot during this time.
  */
  inline void setSamplingTime(const double &delta_t)
  {
    this->delta_t = delta_t;
  }
  void setCameraVelocity(const vpColVector &v)   ;
  void setArticularVelocity(const vpColVector &qdot)  ;
  void setVelocity(const vpRobot::vpControlFrameType frame,
		   const  vpColVector &vel)  ;

  void getPosition(vpColVector &q)    ;
  void getPosition(vpHomogeneousMatrix &cMo) const   ;
  void getArticularPosition(vpColVector &q)  const  ;
  void getPosition(const vpRobot::vpControlFrameType frame,
		   vpColVector &q)  ;
  void setPosition(const vpRobot::vpControlFrameType /* frame */,
		   const vpColVector & /* q */)  { ; }


  void setPosition(const vpHomogeneousMatrix &_cMo) ;
  void setPosition(const vpColVector & /* q */) { ;}

  void getCameraDisplacement(vpColVector &v)  ;
  void getArticularDisplacement(vpColVector &qdot) ;
  void getDisplacement(const vpRobot::vpControlFrameType repere,
		       vpColVector &q) ;

} ;

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
