#include <visp/vpRobot.h>
#include <visp/vpDebug.h>

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

/* Recupere la position actuelle du robot.
 * Recupere la position actuelle du robot et renvoie le resultat
 * Le repere de travail dans lequel est exprime le resultat est celui
 * donne par la variable \a repere.
 * INPUT:
 *   - repere: repere de travail dans lequel est exprime le resultat.
 * OUTPUT:
 *   - Position actuelle du robot.
 */
vpColVector vpRobot::
getPosition (vpRobot::ControlFrameType repere)
{
  vpColVector r;
  this ->getPosition (repere, r);

  return r;
}
