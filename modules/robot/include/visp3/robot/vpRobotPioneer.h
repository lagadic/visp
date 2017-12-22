/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_PIONEER

#include <Aria.h>

// Warning: vpMath.h included from vpRobot.h or vpPioneer.h should be
// included after Aria.h to avoid the build issue:
// "/usr/include/Aria/ariaUtil.h:732:21: error: ‘isfinite’ was not declared
// in this scope"
// This error is due to cmath header included from vpMath.h that makes
// isfinite() ambiguous between ::isfinite() and std::isfinite()
#include <visp3/robot/vpPioneer.h>
#include <visp3/robot/vpRobot.h>

/*!

  \class vpRobotPioneer

  \ingroup group_robot_real_unicycle

  \brief Interface for Pioneer mobile robots based on Aria 3rd party library.

  This class provides a position and speed control interface for Pioneer
  mobile robots. It inherits from the Aria ArRobot class. For more information
  about the model of the robot, see vpPioneer documentation.

*/
class VISP_EXPORT vpRobotPioneer : public vpRobot, public vpPioneer, public ArRobot
{
private: /* Not allowed functions. */
  /*!
    Copy constructor not allowed.
   */
  vpRobotPioneer(const vpRobotPioneer &robot);

public:
  vpRobotPioneer();
  virtual ~vpRobotPioneer();

  /*!
    Get the robot Jacobian expressed at point E, the point located at the
    middle between the two wheels.

    \param eJe : Robot jacobian such as \f$(v_x, w_z) = {^e}{\bf J}e \; {\bf
    v}\f$ with \f$(v_x, w_z)\f$ respectively the translational and rotational
    control velocities of the mobile robot, \f$\bf v\f$ the six dimention
    velocity skew, and where

    \sa get_eJe()

  */
  void get_eJe(vpMatrix &eJe) { eJe = vpUnicycle::get_eJe(); }

private: // Set as private since not implemented
  /*!
    Get the robot Jacobian expressed in the robot reference (or world) frame.
    \warning Not implemented.
  */
  void get_fJe(vpMatrix & /*fJe*/){};

  /*!
    Get a displacement (frame as to ve specified) between two successive
    position control. \warning Not implemented.
  */
  void getDisplacement(const vpRobot::vpControlFrameType /*frame*/, vpColVector & /*q*/){};

public:
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);

private: // Set as private since not implemented
  /*!
    Get the robot position (frame has to be specified).
    \warning Not implemented.
  */
  void getPosition(const vpRobot::vpControlFrameType /*frame*/, vpColVector & /*q*/){};

public:
  void init();

private: // Set as private since not implemented
  /*!
    Set a displacement (frame has to be specified) in position control.
    \warning Not implemented.
  */
  void setPosition(const vpRobot::vpControlFrameType /*frame*/, const vpColVector & /*q*/){};

public:
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  /*!
    Enable or disable sonar device usage.
    */
  void useSonar(bool usage) { this->comInt(ArCommands::SONAR, usage); }

protected:
  bool isInitialized;
};

#endif

#endif // VPROBOTPIONEER_H
