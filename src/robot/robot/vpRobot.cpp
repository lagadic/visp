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
 * Generic virtual robot.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp/vpRobot.h>
#include <visp/vpDebug.h>


const double vpRobot::maxTranslationVelocityDefault = 0.2;
const double vpRobot::maxRotationVelocityDefault = 0.7;

/* ------------------------------------------------------------------------- */
/* --- CONSTRUCTEUR -------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

vpRobot::vpRobot (void)
  :
  maxTranslationVelocity (maxTranslationVelocityDefault),
  maxRotationVelocity (maxRotationVelocityDefault)
{
  stateRobot = vpRobot::STATE_STOP ;
}


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  \file vpRobot.cpp
  \brief class that defines a generic virtual robot
*/
vpRobot::vpRobotStateType
vpRobot::setRobotState (const vpRobot::vpRobotStateType newState)
{
  stateRobot = newState ;
  return newState ;
}

vpRobot::vpControlFrameType
vpRobot::setRobotFrame (vpRobot::vpControlFrameType newFrame)
{
  frameRobot = newFrame ;
  return newFrame ;
}

/*!
  Recupere la position actuelle du robot.
  Recupere la position actuelle du robot et renvoie le resultat
  Le repere de travail dans lequel est exprime le resultat est celui
  donne par la variable \a repere.
  INPUT:
    - repere: repere de travail dans lequel est exprime le resultat.
  OUTPUT:
    - Position actuelle du robot.
*/
vpColVector
vpRobot::getPosition (vpRobot::vpControlFrameType repere)
{
  vpColVector r;
  this ->getPosition (repere, r);

  return r;
}

/* ------------------------------------------------------------------------- */
/* --- VELOCITY CONTROL ---------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*! 

  Set the maximal translation velocity that can be sent to the robot during a velocity control.

  \param maxVt : Maximum translation velocity expressed in m/s.

*/
void
vpRobot::setMaxTranslationVelocity (const double maxVt)
{
  this ->maxTranslationVelocity = maxVt;
  return;
}

/*!
  Get the maximal translation velocity that can be sent to the robot during a velocity control.

  \return Maximum translation velocity expressed in m/s.
*/
double
vpRobot::getMaxTranslationVelocity (void) const
{
  return this ->maxTranslationVelocity;
}
/*! 

  Set the maximal rotation velocity that can be sent to the robot  during a velocity control.

  \param maxVr : Maximum rotation velocity expressed in rad/s.
*/

void
vpRobot::setMaxRotationVelocity (const double maxVr)
{
  this ->maxRotationVelocity = maxVr;
  return;
}

/*! 

  Get the maximal rotation velocity that can be sent to the robot during a velocity control.

  \return Maximum rotation velocity expressed in rad/s.
*/
double
vpRobot::getMaxRotationVelocity (void) const
{
  return this ->maxRotationVelocity;
}

