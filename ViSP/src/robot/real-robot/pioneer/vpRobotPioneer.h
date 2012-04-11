/****************************************************************************
 *
 * $Id$
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
 * Interface for Pioneer mobile robots based on Aria 3rd party library.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef VPROBOTPIONEER_H
#define VPROBOTPIONEER_H

#include <visp/vpConfig.h>
#include <visp/vpRobot.h>

#ifdef VISP_HAVE_PIONEER

#include <Aria.h>

/*!

  \class vpRobotPioneer

  \ingroup Pioneer RobotDriver

  \brief Interface for Pioneer mobile robots based on Aria 3rd party library.

  This class provides a position and speed control interface for Pioneer mobile robots.
  It inherits from the Aria ArRobot class. For more information see ArRobot documentation.

*/
class VISP_EXPORT vpRobotPioneer: public vpRobot, public ArRobot
{
private: /* Not allowed functions. */

  /*!
    Copy contructor not allowed.
   */
  vpRobotPioneer(const vpRobotPioneer &robot);

public:
  vpRobotPioneer();
  //virtual ~vpRobotPioneer ();

  //---------- Jacobian -----------------------------
  /*!
    Get the robot Jacobian expressed in the end-effector frame.
    \warning Not implemented.
  */
  void get_eJe(vpMatrix & /*eJe*/) {} ;
  /*!
    Get the robot Jacobian expressed in the robot reference (or world) frame.
    \warning Not implemented.
  */
  void get_fJe(vpMatrix & /*fJe*/) {} ;

  /*!
    Get a displacement expressed in the joint space between two successive position control.
    \warning Not implemented.
  */
  void getArticularDisplacement(vpColVector  & /*qdot*/) {};
  /*!
    Get a displacement expressed in the camera frame between two successive position control.
    \warning Not implemented.
  */
  void getCameraDisplacement(vpColVector & /*v*/) {};
  /*!
    Get a displacement (frame as to ve specified) between two successive position control.
    \warning Not implemented.
  */
  void getDisplacement(const vpRobot::vpControlFrameType /*frame*/, vpColVector &/*q*/) {};

  void getVelocity (const vpRobot::vpControlFrameType frame, vpColVector & velocity);
  vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

  /*!
    Get the robot position (frame has to be specified).
    \warning Not implemented.
  */
  void getPosition(const vpRobot::vpControlFrameType /*frame*/, vpColVector &/*q*/) {};

  void init();

  /*!
    Set a displacement (frame has to be specified) in position control.
    \warning Not implemented.
  */
  void setPosition(const vpRobot::vpControlFrameType /*frame*/, const vpColVector &/*q*/) {};

  /*!
    Set the velocity (frame has to be specified) that will be applied to the velocity controller.
    \warning Not implemented.
  */
  void setVelocity(const vpRobot::vpControlFrameType /*frame*/, const vpColVector &/*vel*/);

private:
  bool isInitialized;
};

#endif

#endif // VPROBOTPIONEER_H

