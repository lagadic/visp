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
 * Interface for the ptu-46 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/* ----------------------------------------------------------------------- */
/* --- INCLUDE ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpPtu46.h>
#include <visp3/robot/vpRobotException.h>

/* Inclusion des fichiers standards.		*/
#include <math.h>
#include <visp3/core/vpMath.h>

/* ------------------------------------------------------------------------ */
/* --- COMPUTE ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const unsigned int vpPtu46::ndof = 2; /*<! Pan and tilt are considered. */
const float vpPtu46::L = 0.0765f;     /*! Horizontal offset along the last joint,
                                       from last joint to camera frame. */
const float vpPtu46::h = 0.068f;      /*<! Vertical offset from last joint to
                                       camera frame. */

/*!
  Compute the direct geometric model of the camera: fMc

  \param q : Articular position for pan and tilt axis.

  \param fMc : Homogeneous matrix corresponding to the direct geometric model
  of the camera. Describes the transformation between the robot reference
  frame (called fixed) and the camera frame.

*/
void vpPtu46::computeMGD(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{
  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for ptu-46 articular vector"));
  }

  double q1 = q[0]; // pan
  double q2 = q[1]; // tilt

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);

  fMc[0][0] = s1;
  fMc[0][1] = c1 * s2;
  fMc[0][2] = c1 * c2;
  fMc[0][3] = -h * c1 * s2 - L * s1;

  fMc[1][0] = -c1;
  fMc[1][1] = s1 * s2;
  fMc[1][2] = s1 * c2;
  fMc[1][3] = -h * s1 * s2 + L * c1;

  fMc[2][0] = 0;
  fMc[2][1] = -c2;
  fMc[2][2] = s2;
  fMc[2][3] = h * c2;

  fMc[3][0] = 0;
  fMc[3][1] = 0;
  fMc[3][2] = 0;
  fMc[3][3] = 1;

  vpCDEBUG(6) << "Position de la camera: " << std::endl << fMc;

  return;
}

/*!
  Return the direct geometric model of the camera: fMc

  \param q : Articular position for pan and tilt axis.

  \return fMc, the homogeneous matrix corresponding to the direct geometric
  model of the camera. Describes the transformation between the robot
  reference frame (called fixed) and the camera frame.

*/
vpHomogeneousMatrix vpPtu46::computeMGD(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;

  computeMGD(q, fMc);

  return fMc;
}

/*!
  Compute the direct geometric model of the camera in terms of pose vector.

  \param q : Articular position for pan and tilt axis.

  \param r : Pose vector corresponding to the transformation between the robot
  reference frame (called fixed) and the camera frame.

*/
void vpPtu46::computeMGD(const vpColVector &q, vpPoseVector &r) const
{
  vpHomogeneousMatrix fMc;

  computeMGD(q, fMc);
  r.buildFrom(fMc.inverse());

  return;
}

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  Default construtor. Call init().

*/
vpPtu46::vpPtu46(void) { init(); }
/* ---------------------------------------------------------------------- */
/* --- PRIVATE ---------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

/*!
  Initialization. Here nothing to do.

*/
void vpPtu46::init() { return; }

/* ----------------------------------------------------------------------- */
/* --- DISPLAY ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPtu46 & /* constant */)
{
  os << "Geometric parameters: " << std::endl
     << "L: "
     << "\t" << vpPtu46::L << std::endl
     << "h: "
     << "\t" << vpPtu46::h << std::endl;

  return os;
}

/*!

  Get the twist matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cVe : Twist transformation between camera and end effector frame to
  expess a velocity skew from end effector frame in camera frame.

*/
void vpPtu46::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  get_cMe(cMe);

  cVe.buildFrom(cMe);
}

/*!

  Get the homogeneous matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cMe :  Homogeneous matrix between camera and end effector frame.

*/
void vpPtu46::get_cMe(vpHomogeneousMatrix &cMe) const
{
  vpHomogeneousMatrix eMc;

  eMc[0][0] = 0;
  eMc[0][1] = -1;
  eMc[0][2] = 0;
  eMc[0][3] = h;

  eMc[1][0] = 1;
  eMc[1][1] = 0;
  eMc[1][2] = 0;
  eMc[1][3] = -L;

  eMc[2][0] = 0;
  eMc[2][1] = 0;
  eMc[2][2] = 1;
  eMc[2][3] = 0;

  eMc[3][0] = 0;
  eMc[3][1] = 0;
  eMc[3][2] = 0;
  eMc[3][3] = 1;

  cMe = eMc.inverse();
}

/*!
  Get the robot jacobian expressed in the end-effector frame.

  \warning Re is not the embedded camera frame. It corresponds to the frame
  associated to the tilt axis (see also get_cMe).

  \param q : Articular position for pan and tilt axis.

  \param eJe : Jacobian between end effector frame and end effector frame (on
  tilt axis).

*/
void vpPtu46::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{

  eJe.resize(6, 2);

  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for ptu-46 articular vector"));
  }

  double s2 = sin(q[1]);
  double c2 = cos(q[1]);

  eJe = 0;

  eJe[3][0] = c2;
  eJe[4][1] = 1;
  eJe[5][0] = s2;
}

/*!
  Get the robot jacobian expressed in the robot reference frame

  \param q : Articular position for pan and tilt axis.

  \param fJe : Jacobian between reference frame (or fix frame) and end
  effector frame (on tilt axis).

*/
void vpPtu46::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{

  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for ptu-46 articular vector"));
  }

  fJe.resize(6, 2);

  double s1 = sin(q[0]);
  double c1 = cos(q[0]);

  fJe = 0;

  fJe[3][1] = s1;
  fJe[4][1] = -c1;
  fJe[5][0] = 1;
}
