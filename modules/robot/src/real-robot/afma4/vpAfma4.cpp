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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpAfma4.cpp

  Control of Irisa's cylindrical robot named Afma4.

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpAfma4.h>
#include <visp3/robot/vpRobotException.h>

/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */
const unsigned int vpAfma4::njoint = 4;

/*!

  Default constructor.

*/
vpAfma4::vpAfma4() : _a1(0), _d3(0), _d4(0), _etc(), _erc(), _eMc()
{
  // Set the default parameters in case of the config files are not available.

  //
  // Geometric model constant parameters
  //
  this->_a1 = 0.205; // distance along x2
  this->_d3 = 0.403; // distance along z2
  this->_d4 = 0.14;  // distance along z3

  // Maximal value of the joints
  this->_joint_max[0] = 1.8;  // rad
  this->_joint_max[1] = 0.9;  // meter
  this->_joint_max[2] = 0.9;  // rad
  this->_joint_max[3] = 0.76; // rad
  // Minimal value of the joints
  this->_joint_min[0] = -1.5;  // rad
  this->_joint_min[1] = -0.9;  // meter
  this->_joint_min[2] = -3.5;  // rad
  this->_joint_min[3] = -0.76; // rad

  // Camera extrinsic parameters: effector to camera frame
  this->_etc[0] = 0.; // Translation
  this->_etc[1] = 0.;
  this->_etc[2] = 0.;
  this->_erc[0] = 0.; // Rotation
  this->_erc[1] = -M_PI / 2.;
  this->_erc[2] = 0;

  vpRotationMatrix eRc(_erc);
  this->_eMc.buildFrom(_etc, eRc);

  init();
}

/*!

  Does nothing for the moment.
 */
void vpAfma4::init(void) { return; }

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
 all the four joints.

  This method is the same than get_fMc(const vpColVector & q).

 \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (\f${^f}M_c\f$) with:
  \f[
  {^f}M_c =  {^f}M_e *  {^e}M_c
  \f]

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix vpAfma4::getForwardKinematics(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
  all the four joints.

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (\f${^f}M_c\f$) with:
  \f[
  {^f}M_c =  {^f}M_e *  {^e}M_c
  \f]

  \sa getForwardKinematics(const vpColVector & q)
*/
vpHomogeneousMatrix vpAfma4::get_fMc(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
  all the four joints.

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param fMc : The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (\f${^f}M_c\f$) with:
  \f[
  {^f}M_c =  {^f}M_e *  {^e}M_c
  \f]

*/
void vpAfma4::get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{

  // Compute the direct geometric model: fMe = transformation between
  // fix and end effector frame.
  vpHomogeneousMatrix fMe;

  get_fMe(q, fMe);

  fMc = fMe * this->_eMc;

  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the end effector with respect to the base frame given the
  articular positions of all the four variable joints.

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param fMe The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  end effector frame (\f${^f}M_e\f$) with

  \f[
  {^f}M_e = \left[\begin{array}{cccc}
  c_1s_4c_5+s_1c_4c_5  & -c_1s_4s_5-s_1c_4s_5 & c_1c_4-s_1s_4 &a_1c_1-d_3s_1
  \\
  s_1s_4c_5-c_1c_4c_5  & -s_1s_4s_5+c_1c_4s_5 & s_1c_4+c_1s_4 &a_1s_1+d_3c_1
  \\
  -s_5 & -c_5  & d_4+q_2 \\
  0  &   0  &   0  &   1    \\
  \end{array}
  \right]
  \f]

*/
void vpAfma4::get_fMe(const vpColVector &q, vpHomogeneousMatrix &fMe) const
{
  double q1 = q[0]; // rot touret
  double q2 = q[1]; // vertical translation
  double q4 = q[2]; // pan
  double q5 = q[3]; // tilt

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);

  /* Calcul du modele d'apres les angles. */
  fMe[0][0] = c1 * s4 * c5 + s1 * c4 * c5;
  fMe[0][1] = -c1 * s4 * s5 - s1 * c4 * s5;
  fMe[0][2] = c1 * c4 - s1 * s4;
  fMe[0][3] = c1 * this->_a1 - s1 * (this->_d3);

  fMe[1][0] = s1 * s4 * c5 - c1 * c4 * c5;
  fMe[1][1] = -s1 * s4 * s5 + c1 * c4 * s5;
  fMe[1][2] = s1 * c4 + c1 * s4;
  fMe[1][3] = s1 * this->_a1 + c1 * (this->_d3);

  fMe[2][0] = -s5;
  fMe[2][1] = -c5;
  fMe[2][2] = 0.f;
  fMe[2][3] = this->_d4 + q2;

  fMe[3][0] = 0.f;
  fMe[3][1] = 0.f;
  fMe[3][2] = 0.f;
  fMe[3][3] = 1;

  //  vpCTRACE << "Effector position fMe: " << std::endl << fMe;

  return;
}

/*!

  Get the geometric transformation between the camera frame and the
  end-effector frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by hand or by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void vpAfma4::get_cMe(vpHomogeneousMatrix &cMe) const { cMe = this->_eMc.inverse(); }

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void vpAfma4::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  get_cMe(cMe);

  cVe.buildFrom(cMe);

  return;
}

/*!

  Get the twist transformation from camera frame to the reference
  frame.  This transformation allows to compute a velocity expressed
  in the reference frame into the camera frame.

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param cVf : Twist transformation.

*/
void vpAfma4::get_cVf(const vpColVector &q, vpVelocityTwistMatrix &cVf) const
{
  vpHomogeneousMatrix fMc, cMf;
  get_fMc(q, fMc);
  cMf = fMc.inverse();

  cVf.buildFrom(cMf);

  return;
}

/*!

  Get the robot jacobian expressed in the end-effector frame:

  \f[
  {^e}J_e = \left[\begin{array}{cccc}
  -c_5(a_1c_4+d_3s_4) & -s_5 & 0 & 0   \\
  s_5(a_1c_4+d_3s_4) & -c_5 & 0 & 0   \\
  a_1s_4-d_3c_4 & 0 & 0 & 0 \\
  -s_5 & 0 & -s_5 & 0 \\
  -c_5 & 0 & -c_5 & 0 \\
  0 & 0 & 0 & 1 \\
  \end{array}
  \right]
  \f]

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param eJe : Robot jacobian expressed in the end-effector frame, with:
  \f[
  {^e}J_e = \left[\begin{array}{cc}
  {^f}R_e^T & 0_{3 \times 3}    \\
  0_{3 \times 3} & {^f}R_e^T \\
  \end{array}
  \right]  {^f}J_e
  \f]

  \sa get_fJe()
*/
void vpAfma4::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{
  double q4 = q[2]; // pan
  double q5 = q[3]; // tilt

  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);

  eJe.resize(6, 4);

  eJe = 0;

  eJe[0][0] = -(this->_a1 * c4 + this->_d3 * s4) * c5;
  eJe[0][1] = -s5;
  eJe[1][0] = (this->_a1 * c4 + this->_d3 * s4) * s5;
  eJe[1][1] = -c5;
  eJe[2][0] = (this->_a1 * s4 - this->_d3 * c4);
  eJe[3][0] = eJe[3][2] = -s5;
  eJe[4][0] = eJe[4][2] = -c5;
  eJe[5][3] = 1.;
}

/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame:

  \f[
  {^f}J_e = \left[\begin{array}{cccc}
  -a_1s_1-d_3c_1 & 0 & 0 & 0   \\
  a_1c_1-d_3s_1 & 0 & 0 & 0   \\
  0 & 1 & 0 & 0 \\
  0 & 0 & 0 & c_{14} \\
  0 & 0 & 0 & s_{14} \\
  1 & 0 & 1 & 0 \\
  \end{array}
  \right]
  \f]

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param fJe : Robot jacobian expressed in the robot reference frame.

  \sa get_eJe() and get_fJe_inverse()
*/

void vpAfma4::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{
  fJe.resize(6, 4);

  double q1 = q[0]; // rot touret
  double q4 = q[2]; // pan

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c14 = cos(q1 + q4);
  double s14 = sin(q1 + q4);

  fJe = 0;

  fJe[0][0] = -s1 * this->_a1 - c1 * this->_d3;

  fJe[1][0] = c1 * this->_a1 - s1 * this->_d3;

  fJe[2][1] = 1.0;

  fJe[3][3] = c14;

  fJe[4][3] = s14;

  fJe[5][0] = fJe[5][2] = 1.0;
}

/*!

  Get the inverse jacobian.

  \f[
  {^f}J_e^+ = \left[\begin{array}{cccccc}
  -(a_1s_1+d_3c_1)/(a_1^2+d_3^2) & (a_1c_1-d_3s_1)/(a_1^2+d_3^2) & 0&0&0&0 \\
  0 & 0 & 1 & 0 & 0 & 0  \\
  (a_1s_1+d_3c_1)/(a_1^2+d_3^2) & -(a_1c_1-d_3s_1)/(a_1^2+d_3^2) & 0&0&0&1 \\
  0 & 0 & 0 & c_{14} & s_{14} & 0  \\
  \end{array}
  \right]
  \f]

  \param q : Articular position of the four joints: q[0] corresponds to
  the first rotation (joint 1 with value \f$q_1\f$) of the turret
  around the vertical axis, while q[1] corresponds to the vertical
  translation (joint 2 with value \f$q_2\f$), while q[2] and q[3]
  correspond to the pan and tilt of the camera (respectively joint 4
  and 5 with values \f$q_4\f$ and \f$q_5\f$). Rotations q[0], q[2] and
  q[3] are expressed in radians. The translation q[1] is expressed in
  meters.

  \param fJe_inverse : Inverse robot jacobian expressed in the robot
  reference frame.

  \sa get_eJe() and get_fJe()

*/
void vpAfma4::get_fJe_inverse(const vpColVector &q, vpMatrix &fJe_inverse) const
{
  fJe_inverse.resize(4, 6);
  fJe_inverse = 0;

  double q1 = q[0]; // rot touret
  double q4 = q[2]; // pan

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c14 = cos(q1 + q4);
  double s14 = sin(q1 + q4);

  double det = this->_a1 * this->_a1 + this->_d3 * this->_d3;

  fJe_inverse[0][0] = (-s1 * this->_a1 - c1 * this->_d3) / det;
  fJe_inverse[0][1] = (c1 * this->_a1 - s1 * this->_d3) / det;

  fJe_inverse[1][2] = fJe_inverse[2][5] = 1.;

  fJe_inverse[2][0] = -fJe_inverse[0][0];
  fJe_inverse[2][1] = -fJe_inverse[0][1];

  fJe_inverse[3][3] = c14;
  fJe_inverse[3][4] = s14;
}

/*!
  Get min joint values.

  \return Minimal joint values for the 4 dof
  X, Y, A, B. Translation Y is expressed in meters. Rotations
  X,A and B in radians.

*/
vpColVector vpAfma4::getJointMin() const
{
  vpColVector qmin(4);
  for (unsigned int i = 0; i < 4; i++)
    qmin[i] = this->_joint_min[i];
  return qmin;
}

/*!
  Get max joint values.

  \return Maximal joint values for the 4 dof
  X, Y, A, B. Translation Y is expressed in meters. Rotations
  X, A and B in radians.

*/
vpColVector vpAfma4::getJointMax() const
{
  vpColVector qmax(4);
  for (unsigned int i = 0; i < 4; i++)
    qmax[i] = this->_joint_max[i];
  return qmax;
}

/*!

  Print on the output stream \e os the robot parameters (joint
  min/max, distance between axis 5 and 6, coupling factor between axis
  5 and 6, hand-to-eye homogeneous matrix.

  \param os : Output stream.
  \param afma4 : Robot parameters.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAfma4 &afma4)
{
  vpRotationMatrix eRc;
  afma4._eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  os << "Joint Max:" << std::endl
     << "\t" << afma4._joint_max[0] << "\t" << afma4._joint_max[1] << "\t" << afma4._joint_max[2] << "\t"
     << afma4._joint_max[3] << "\t" << std::endl

     << "Joint Min: " << std::endl
     << "\t" << afma4._joint_min[0] << "\t" << afma4._joint_min[1] << "\t" << afma4._joint_min[2] << "\t"
     << afma4._joint_min[3] << "\t" << std::endl

     << "a1: " << std::endl
     << "\t" << afma4._a1 << "\t" << std::endl

     << "d3: " << std::endl
     << "\t" << afma4._d3 << "\t" << std::endl

     << "d4: " << std::endl
     << "\t" << afma4._d4 << "\t" << std::endl

     << "eMc: " << std::endl
     << "\tTranslation (m): " << afma4._eMc[0][3] << " " << afma4._eMc[1][3] << " " << afma4._eMc[2][3] << "\t"
     << std::endl
     << "\tRotation Rxyz (rad) : " << rxyz[0] << " " << rxyz[1] << " " << rxyz[2] << "\t" << std::endl
     << "\tRotation Rxyz (deg) : " << vpMath::deg(rxyz[0]) << " " << vpMath::deg(rxyz[1]) << " " << vpMath::deg(rxyz[2])
     << "\t" << std::endl;

  return os;
}
