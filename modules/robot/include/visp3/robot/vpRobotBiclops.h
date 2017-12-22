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
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

#ifndef __vpROBOT_BICLOPS_H
#define __vpROBOT_BICLOPS_H

/* ------------------------------------------------------------------------ */
/* --- INCLUDES ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* --- GENERAL --- */
#include <iostream>
#include <pthread.h>
#include <stdio.h>

/* --- ViSP --- */
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotBiclopsController.h>

/* ------------------------------------------------------------------------ */
/* --- CLASS -------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!
  \class vpRobotBiclops

  \ingroup group_robot_real_ptu

  \brief Interface for the biclops, pan, tilt head control.

  See http://www.traclabs.com/biclopspt.html for more details.

  This class provide a position and a speed control interface for the biclops
  head. To manage the biclops joint limits in speed control, a control loop is
  running in a seperate thread (see vpRobotBiclopsSpeedControlLoop()).

  The control of the head is done by vpRobotBiclopsController class.

  \warning Velocity control mode is not exported from the top-level Biclops
  API class provided by Traclabs. That means that there is no protection in
  this mode to prevent an axis from striking its hard limit. In position mode,
  Traclabs put soft limits in that keep any command from driving to a position
  too close to the hard limits. In velocity mode this protection does not
  exist in the current API.

  \warning With the understanding that hitting the hard limits at full
  speed/power can damage the unit, damage due to velocity mode commanding is
  under user responsibility.


*/
class VISP_EXPORT vpRobotBiclops : public vpBiclops, public vpRobot
{
private:
  static bool robotAlreadyCreated;
  pthread_t control_thread;

  std::string configfile; // Biclops config file

  vpRobotBiclopsController controller;

  double positioningVelocity;
  vpColVector q_previous;
  bool controlThreadCreated;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  /*! \brief No copy constructor allowed.   */
  //  vpRobotBiclops(const vpRobotBiclops &)
  //    : vpBiclops(), vpRobot(), control_thread(), controller(),
  //      positioningVelocity(0), q_previous(), controlThreadCreated(false)
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpRobotBiclops &operator=(const vpRobotBiclops &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  static const double defaultPositioningVelocity;

  vpRobotBiclops(void);
  explicit vpRobotBiclops(const std::string &filename);
  virtual ~vpRobotBiclops(void);

  void init(void);

  void get_cMe(vpHomogeneousMatrix &_cMe) const;
  void get_cVe(vpVelocityTwistMatrix &_cVe) const;
  void get_eJe(vpMatrix &_eJe);
  void get_fJe(vpMatrix &_fJe);

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &d);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
  double getPositioningVelocity(void);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);

  bool readPositionFile(const std::string &filename, vpColVector &q);

  void setConfigFile(const std::string &filename = "/usr/share/BiclopsDefault.cfg");
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);
  void setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2);
  void setPosition(const char *filename);
  void setPositioningVelocity(const double velocity);
  vpRobot::vpRobotStateType setRobotState(const vpRobot::vpRobotStateType newState);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot);

  void stopMotion();

  static void *vpRobotBiclopsSpeedControlLoop(void *arg);
};

#endif /* #ifndef __vpROBOT_BICLOPS_H */

#endif
