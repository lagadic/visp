/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Interface for the ptu-46 robot.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_PTU46

#ifndef _vpRobotPtu46_h_
#define _vpRobotPtu46_h_

/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>
#include <stdio.h>

/* --- ViSP --- */
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpPtu46.h>
#include <visp3/robot/vpRobot.h>

#include <ptu.h> // Contrib for Ptu-46 robot

/* ------------------------------------------------------------------------ */
/* --- CLASS ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */
BEGIN_VISP_NAMESPACE
/*!

  \class vpRobotPtu46

  \ingroup group_robot_real_ptu

  \brief Interface for the Directed Perception ptu-46 pan, tilt head .

  See http://www.DPerception.com for more details.

  This class provide a position and a speed control interface for the ptu-46
  head.

*/
class VISP_EXPORT vpRobotPtu46 : public vpPtu46, public vpRobot
{

private:
  /*! \brief No copy constructor allowed.   */
  vpRobotPtu46(const vpRobotPtu46 &ass);

  /*! Object to control. This is a contribution. */
  Ptu ptu;

private:
  static bool robotAlreadyCreated;
  double positioningVelocity;
  int velocityMesureTempo;
  std::string device;

public:
  static const double defaultPositioningVelocity;

  VP_EXPLICIT vpRobotPtu46(const std::string &device = "/dev/ttyS0");
  VP_EXPLICIT vpRobotPtu46(vpRobotPtu46 *pub);
  virtual ~vpRobotPtu46(void);

  void get_cMe(vpHomogeneousMatrix &_cMe) const;
  void get_cVe(vpVelocityTwistMatrix &_cVe) const;
  void get_eJe(vpMatrix &_eJe) VP_OVERRIDE;
  void get_fJe(vpMatrix &_fJe) VP_OVERRIDE;

  void getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &q);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;
  double getPositioningVelocity(void);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);

  void init(void);

  bool readPositionFile(const std::string &filename, vpColVector &q);

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;
  void setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2);
  void setPosition(const char *filename);
  void setPositioningVelocity(double velocity);
  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot) VP_OVERRIDE;

  void stopMotion();
};
END_VISP_NAMESPACE
#endif /* #ifndef _vpRobotPtu46_h_ */

#endif
