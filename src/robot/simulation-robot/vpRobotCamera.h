/****************************************************************************
 *
 * $Id: vpRobotCamera.h,v 1.10 2008-09-26 15:20:57 fspindle Exp $
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
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRobot.h>
#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpRobotCamera

  \ingroup BasicRobotSimu

  \brief Class that defines the simplest robot: a free flying camera.


*/
class VISP_EXPORT vpRobotCamera : public vpRobot
{

private:
  //! robot / camera location in the world frame
  vpHomogeneousMatrix cMo ;
  float delta_t; // sampling time

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
  inline void setSamplingTime(const float &delta_t)
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
