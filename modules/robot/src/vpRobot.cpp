/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Generic virtual robot.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>

const double vpRobot::maxTranslationVelocityDefault = 0.2;
const double vpRobot::maxRotationVelocityDefault = 0.7;

/* -------------------------------------------------------------------------
 */
/* --- CONSTRUCTEUR --------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

vpRobot::vpRobot(void)
  : stateRobot(vpRobot::STATE_STOP), frameRobot(vpRobot::CAMERA_FRAME),
    maxTranslationVelocity(maxTranslationVelocityDefault), maxRotationVelocity(maxRotationVelocityDefault), nDof(0),
    eJe(), eJeAvailable(false), fJe(), fJeAvailable(false), areJointLimitsAvailable(false), qmin(NULL), qmax(NULL),
    verbose_(true)
{
}

vpRobot::vpRobot(const vpRobot &robot)
  : stateRobot(vpRobot::STATE_STOP), frameRobot(vpRobot::CAMERA_FRAME),
    maxTranslationVelocity(maxTranslationVelocityDefault), maxRotationVelocity(maxRotationVelocityDefault), nDof(0),
    eJe(), eJeAvailable(false), fJe(), fJeAvailable(false), areJointLimitsAvailable(false), qmin(NULL), qmax(NULL),
    verbose_(true)
{
  *this = robot;
}

/*!
   Destructor that free allocated memory.
 */
vpRobot::~vpRobot()
{
  if (qmin != NULL) {
    delete[] qmin;
    qmin = NULL;
  }
  if (qmax != NULL) {
    delete[] qmax;
    qmax = NULL;
  }
}

/*! Copy operator. */
vpRobot &vpRobot::operator=(const vpRobot &robot)
{
  stateRobot = robot.stateRobot;
  frameRobot = robot.frameRobot;
  maxTranslationVelocity = robot.maxTranslationVelocity;
  maxRotationVelocity = robot.maxRotationVelocity;
  nDof = robot.nDof;
  eJe = robot.eJe;
  eJeAvailable = robot.eJeAvailable;
  fJe = robot.fJe;
  fJeAvailable = robot.fJeAvailable;
  areJointLimitsAvailable = robot.areJointLimitsAvailable;
  qmin = new double[nDof];
  qmax = new double[nDof];
  for (int i = 0; i < nDof; i++) {
    qmin[i] = robot.qmin[i];
    qmax[i] = robot.qmax[i];
  }
  verbose_ = robot.verbose_;

  return (*this);
}
/*!
  Saturate velocities.

  \param v_in : Vector of input velocities to saturate. Translation velocities
should be expressed in m/s while rotation velocities in rad/s.

  \param v_max : Vector of maximal allowed velocities. Maximal translation
velocities should be expressed in m/s while maximal rotation velocities in
rad/s.

  \param verbose : Print a message indicating which axis causes the
saturation.

  \return Saturated velocities.

  \exception vpRobotException::dimensionError : If the input vectors have
different dimensions.

  The code below shows how to use this static method in order to saturate a
velocity skew vector.

  \code
#include <iostream>

#include <visp3/robot/vpRobot.h>

int main()
{
  // Set a velocity skew vector
  vpColVector v(6);
  v[0] = 0.1;               // vx in m/s
  v[1] = 0.2;               // vy
  v[2] = 0.3;               // vz
  v[3] = vpMath::rad(10);   // wx in rad/s
  v[4] = vpMath::rad(-10);  // wy
  v[5] = vpMath::rad(20);   // wz

  // Set the maximal allowed velocities
  vpColVector v_max(6);
  for (int i=0; i<3; i++)
    v_max[i] = 0.3;             // in translation (m/s)
  for (int i=3; i<6; i++)
    v_max[i] = vpMath::rad(10); // in rotation (rad/s)

  // Compute the saturated velocity skew vector
  vpColVector v_sat = vpRobot::saturateVelocities(v, v_max, true);

  std::cout << "v    : " << v.t() << std::endl;
  std::cout << "v max: " << v_max.t() << std::endl;
  std::cout << "v sat: " << v_sat.t() << std::endl;

  return 0;
}
  \endcode
  */
vpColVector vpRobot::saturateVelocities(const vpColVector &v_in, const vpColVector &v_max, bool verbose)
{
  unsigned int size = v_in.size();
  if (size != v_max.size())
    throw vpRobotException(vpRobotException::dimensionError, "Velocity vectors should have the same dimension");

  double scale = 1; // global scale factor to saturate all the axis
  for (unsigned int i = 0; i < size; i++) {
    double v_i = fabs(v_in[i]);
    double v_max_i = fabs(v_max[i]);
    if (v_i > v_max_i) // Test if we should saturate the axis
    {
      double scale_i = v_max_i / v_i;
      if (scale_i < scale)
        scale = scale_i;

      if (verbose)
        std::cout << "Excess velocity " << v_in[i] << " axis nr. " << i << std::endl;
    }
  }

  vpColVector v_sat(size);
  v_sat = v_in * scale;

  return v_sat;
}

/* --------------------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  \file vpRobot.cpp
  \brief class that defines a generic virtual robot
*/
vpRobot::vpRobotStateType vpRobot::setRobotState(const vpRobot::vpRobotStateType newState)
{
  stateRobot = newState;
  return newState;
}

vpRobot::vpControlFrameType vpRobot::setRobotFrame(vpRobot::vpControlFrameType newFrame)
{
  frameRobot = newFrame;
  return newFrame;
}

/*!
  Return the current robot position in the specified frame.
*/
vpColVector vpRobot::getPosition(vpRobot::vpControlFrameType frame)
{
  vpColVector r;
  this->getPosition(frame, r);

  return r;
}

/* -------------------------------------------------------------------------
 */
/* --- VELOCITY CONTROL ----------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/*!

  Set the maximal translation velocity that can be sent to the robot during a
  velocity control.

  \param v_max : Maximum translation velocity expressed in m/s.

*/
void vpRobot::setMaxTranslationVelocity(const double v_max)
{
  this->maxTranslationVelocity = v_max;
  return;
}

/*!
  Get the maximal translation velocity that can be sent to the robot during a
  velocity control.

  \return Maximum translational velocity expressed in m/s.
*/
double vpRobot::getMaxTranslationVelocity(void) const { return this->maxTranslationVelocity; }

/*!

  Set the maximal rotation velocity that can be sent to the robot during a
  velocity control.

  \param w_max : Maximum rotational velocity expressed in rad/s.
*/
void vpRobot::setMaxRotationVelocity(const double w_max)
{
  this->maxRotationVelocity = w_max;
  return;
}

/*!

  Get the maximal rotation velocity that can be sent to the robot during a
  velocity control.

  \return Maximum rotation velocity expressed in rad/s.
*/
double vpRobot::getMaxRotationVelocity(void) const { return this->maxRotationVelocity; }
