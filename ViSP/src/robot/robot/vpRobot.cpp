/****************************************************************************
 *
 * $Id: vpRobot.cpp,v 1.5 2006-05-30 08:40:45 fspindle Exp $
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
  return ;
}


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  \file vpRobot.cpp
  \brief class that defines a generic virtual robot
*/
vpRobot::RobotStateType
vpRobot::setRobotState (const vpRobot::RobotStateType newState)
{
  stateRobot = newState ;
  return newState ;
}

vpRobot::ControlFrameType
vpRobot::setRobotFrame (vpRobot::ControlFrameType newFrame)
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
vpRobot::getPosition (vpRobot::ControlFrameType repere)
{
  vpColVector r;
  this ->getPosition (repere, r);

  return r;
}

/* ------------------------------------------------------------------------- */
/* --- VELOCITY CONTROL ---------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*! Set the maximal velocity that can be sent to the robot (in
 *translation).
 */
void
vpRobot::setMaxTranslationVelocity (const double maxVt)
{
  this ->maxTranslationVelocity = maxVt;
  return;
}

/*!
  Get the maximal velocity that can be sent to the robot (in
  translation).
*/
double
vpRobot::getMaxTranslationVelocity (void) const
{
  return this ->maxTranslationVelocity;
}
/*! Set the maximal velocity that can be sent to the robot (in
 * rotation).
 */

void
vpRobot::setMaxRotationVelocity (const double maxVr)
{
  this ->maxRotationVelocity = maxVr;
  return;
}

/*! Get the maximal velocity that can be sent to the robot (in
 * rotation).
 */
double
vpRobot::getMaxRotationVelocity (void) const
{
  return this ->maxRotationVelocity;
}

