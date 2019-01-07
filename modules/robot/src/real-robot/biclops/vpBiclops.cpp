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
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/* ----------------------------------------------------------------------- */
/* --- INCLUDE ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

#include <math.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobotException.h>

/* ------------------------------------------------------------------------ */
/* --- COMPUTE ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const unsigned int vpBiclops::ndof = 2;                      /*<! Only pan and tilt are considered. */
const float vpBiclops::h = 0.048f;                           /*<! Vertical offset from last joint to camera frame. */
const float vpBiclops::panJointLimit = (float)(M_PI);        /*!< Pan range (in rad): from -panJointLimit to +
                                                                panJointLimit */
const float vpBiclops::tiltJointLimit = (float)(M_PI / 4.5); /*!< Tilt range (in rad): from -tiltJointLimit to +
                                                                tiltJointLimit */

const float vpBiclops::speedLimit = (float)(M_PI / 3.0); /*!< Maximum speed (in rad/s) to perform a displacement */

/*!
  Compute the direct geometric model of the camera: fMc

  \warning Provided for compatibilty with previous versions. Use rather
  get_fMc(const vpColVector &, vpHomogeneousMatrix &).

  \param q : Articular position for pan and tilt axis.

  \param fMc : Homogeneous matrix corresponding to the direct geometric model
  of the camera. Describes the transformation between the robot reference
  frame (called fixed) and the camera frame.

  \sa get_fMc(const vpColVector &, vpHomogeneousMatrix &)
*/
void vpBiclops::computeMGD(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{
  vpHomogeneousMatrix fMe = get_fMe(q);
  fMc = fMe * cMe_.inverse();

  vpCDEBUG(6) << "camera position: " << std::endl << fMc;

  return;
}

/*!
  Compute the direct geometric model of the camera: fMc

  \param q : Articular position for pan and tilt axis.

  \param fMc : Homogeneous matrix corresponding to the direct geometric model
  of the camera. Describes the transformation between the robot reference
  frame (called fixed) and the camera frame.

*/
void vpBiclops::get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{
  vpHomogeneousMatrix fMe = get_fMe(q);
  fMc = fMe * cMe_.inverse();

  vpCDEBUG(6) << "camera position: " << std::endl << fMc;

  return;
}

/*!
  Return the direct geometric model of the camera: fMc

  \warning Provided for compatibilty with previous versions. Use rather
  get_fMc(const vpColVector &).

  \param q : Articular position for pan and tilt axis.

  \return fMc, the homogeneous matrix corresponding to the direct geometric
  model of the camera. Describes the transformation between the robot
  reference frame (called fixed) and the camera frame.

  \sa get_fMc(const vpColVector &)
*/
vpHomogeneousMatrix vpBiclops::computeMGD(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;

  computeMGD(q, fMc);

  return fMc;
}

/*!
  Return the direct geometric model of the camera: fMc

  \param q : Articular position for pan and tilt axis.

  \return fMc, the homogeneous matrix corresponding to the direct geometric
  model of the camera. Discribes the transformation between the robot
  reference frame (called fixed) and the camera frame.

*/
vpHomogeneousMatrix vpBiclops::get_fMc(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);

  return fMc;
}

/*!
  Return the direct geometric model of the end effector: fMe

  \param q : Articular position for pan and tilt axis.

  \return fMe, the homogeneous matrix corresponding to the direct geometric
  model of the end effector. Describes the transformation between the robot
  reference frame (called fixed) and the end effector frame.

*/
vpHomogeneousMatrix vpBiclops::get_fMe(const vpColVector &q) const
{
  vpHomogeneousMatrix fMe;

  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for biclops articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for biclops articular vector"));
  }

  double q1 = q[0]; // pan
  double q2 = q[1]; // tilt

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);

  if (dh_model_ == DH1) {
    fMe[0][0] = -c1 * s2;
    fMe[0][1] = -s1;
    fMe[0][2] = c1 * c2;
    fMe[0][3] = 0;

    fMe[1][0] = -s1 * s2;
    fMe[1][1] = c1;
    fMe[1][2] = s1 * c2;
    fMe[1][3] = 0;

    fMe[2][0] = -c2;
    fMe[2][1] = 0;
    fMe[2][2] = -s2;
    fMe[2][3] = 0;

    fMe[3][0] = 0;
    fMe[3][1] = 0;
    fMe[3][2] = 0;
    fMe[3][3] = 1;
  } else {
    fMe[0][0] = c1 * s2;
    fMe[0][1] = -s1;
    fMe[0][2] = c1 * c2;
    fMe[0][3] = 0;

    fMe[1][0] = s1 * s2;
    fMe[1][1] = c1;
    fMe[1][2] = s1 * c2;
    fMe[1][3] = 0;

    fMe[2][0] = -c2;
    fMe[2][1] = 0;
    fMe[2][2] = s2;
    fMe[2][3] = 0;

    fMe[3][0] = 0;
    fMe[3][1] = 0;
    fMe[3][2] = 0;
    fMe[3][3] = 1;
  }

  return fMe;
}

/*!
  Compute the direct geometric model of the camera in terms of pose vector.

  \warning Provided for compatibilty with previous versions. Use rather
  get_fMc(const vpColVector &, vpPoseVector &).

  \param q : Articular position for pan and tilt axis.

  \param fvc : Pose vector corresponding to the transformation between the
  robot reference frame (called fixed) and the camera frame.

  \sa get_fMc(const vpColVector &, vpPoseVector &)
*/

void vpBiclops::computeMGD(const vpColVector &q, vpPoseVector &fvc) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);
  fvc.buildFrom(fMc.inverse());

  return;
}

/*!
  Compute the direct geometric model of the camera in terms of pose vector.

  \param q : Articular position for pan and tilt axis.

  \param fvc : Pose vector corresponding to the transformation between the
  robot reference frame (called fixed) and the camera frame.

*/

void vpBiclops::get_fMc(const vpColVector &q, vpPoseVector &fvc) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);
  fvc.buildFrom(fMc.inverse());

  return;
}

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  Default construtor. Call init().

*/
vpBiclops::vpBiclops(void) : dh_model_(DH1), cMe_() { init(); }
/* ---------------------------------------------------------------------- */
/* --- PRIVATE ---------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

/*!
  Initialization.
  Set the default \f${^c}{\bf M}_e\f$ transformation.

*/
void vpBiclops::init()
{
  dh_model_ = DH1;
  set_cMe();
  return;
}

/* ----------------------------------------------------------------------- */
/* --- DISPLAY ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpBiclops & /*constant*/)
{
  os << "Geometric parameters: " << std::endl
     << "h: "
     << "\t" << vpBiclops::h << std::endl;

  return os;
}

/*!

  Get the twist matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cVe : Twist transformation between camera and end effector frame to
  expess a velocity skew from end effector frame in camera frame.

*/
void vpBiclops::get_cVe(vpVelocityTwistMatrix &cVe) const { cVe.buildFrom(cMe_); }

/*!

  Set the default homogeneous matrix corresponding to the transformation
  between the camera frame and the end effector frame. The end effector frame
  is located on the tilt axis.

*/
void vpBiclops::set_cMe()
{
  vpHomogeneousMatrix eMc;

  eMc[0][0] = 0;
  eMc[0][1] = -1;
  eMc[0][2] = 0;
  eMc[0][3] = h;

  eMc[1][0] = 1;
  eMc[1][1] = 0;
  eMc[1][2] = 0;
  eMc[1][3] = 0;

  eMc[2][0] = 0;
  eMc[2][1] = 0;
  eMc[2][2] = 1;
  eMc[2][3] = 0;

  eMc[3][0] = 0;
  eMc[3][1] = 0;
  eMc[3][2] = 0;
  eMc[3][3] = 1;

  cMe_ = eMc.inverse();
}

/*!
  Get the robot jacobian expressed in the end-effector frame.

  \warning Re is not the embedded camera frame. It corresponds to the frame
  associated to the tilt axis (see also get_cMe).

  \param q : Articular position for pan and tilt axis.

  \param eJe : Jacobian between end effector frame and end effector frame (on
  tilt axis).

*/
void vpBiclops::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{
  eJe.resize(6, 2);

  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for biclops articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for biclops articular vector"));
  }

  double s2 = sin(q[1]);
  double c2 = cos(q[1]);

  eJe = 0;

  if (dh_model_ == DH1) {
    eJe[3][0] = -c2;
    eJe[4][1] = 1;
    eJe[5][0] = -s2;
  } else {
    eJe[3][0] = -c2;
    eJe[4][1] = -1;
    eJe[5][0] = s2;
  }
}
/*!
  Get the robot jacobian expressed in the robot reference frame

  \param q : Articular position for pan and tilt axis.

  \param fJe : Jacobian between reference frame (or fix frame) and end
  effector frame (on tilt axis).

*/

void vpBiclops::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{
  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for biclops articular vector");
    throw(vpException(vpException::dimensionError, "Bad dimension for biclops articular vector"));
  }

  fJe.resize(6, 2);

  double s1 = sin(q[0]);
  double c1 = cos(q[0]);

  fJe = 0;

  if (dh_model_ == DH1) {
    fJe[3][1] = -s1;
    fJe[4][1] = c1;
    fJe[5][0] = 1;
  } else {
    fJe[3][1] = s1;
    fJe[4][1] = -c1;
    fJe[5][0] = 1;
  }
}
