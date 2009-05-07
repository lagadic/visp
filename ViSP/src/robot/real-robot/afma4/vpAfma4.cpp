/****************************************************************************
 *
 * $Id$
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


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpRobotException.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpAfma4.h>


/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */
const int vpAfma4::njoint = 4;


/*!

  Default constructor.

*/
vpAfma4::vpAfma4()
{
  // Set the default parameters in case of the config files on the NAS
  // at Inria are not available.

  //
  // Geometric model constant parameters
  //
  // Distance between joint 2 and 3
  this->_long_23 = 0.205;
  this->_Long_23 = 0.403;

  // Maximal value of the joints
  this->_joint_max[0] = 1.8;  // rad
  this->_joint_max[1] = 0.9;  // meter
  this->_joint_max[2] = 0.9;  // rad
  this->_joint_max[3] = 0.76; // rad
  // Minimal value of the joints
  this->_joint_min[0] = -1.5; // rad
  this->_joint_min[1] = -0.9; // meter
  this->_joint_min[2] = -3.5; // rad
  this->_joint_min[3] = -0.76;// rad

  // Camera extrinsic parameters: effector to camera frame
  this->_etc[0] = 0.; // Translation
  this->_etc[1] = 0.;
  this->_etc[2] = 0.;
  this->_erc[0] = 0.; // Rotation
  this->_erc[1] = M_PI/2.;
  this->_erc[2] = M_PI;

  vpRotationMatrix eRc(_erc);
  this->_eMc.buildFrom(_etc, eRc);

  init();
}


/*!

  Does nothing for the moment.
 */
void
vpAfma4::init (void)
{
  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the four joints.

  This method is the same than get_fMc(const vpColVector & q).

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix
vpAfma4::getForwardKinematics(const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the four joints.

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa getForwardKinematics(const vpColVector & q)
*/
vpHomogeneousMatrix
vpAfma4::get_fMc (const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the four joints.

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \param fMc The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (fMc).

*/
void
vpAfma4::get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc)
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
  articular positions of all the four joints.

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \param fMe The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  end effector frame (fMe).

*/
void
vpAfma4::get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe)
{
  double            q0 = q[0]; // rot tourelle
  double            q1 = q[1]; // vertical translation
  double            q2 = q[2]; // pan
  double            q3 = q[3]; // tilt

  double            c1 = cos(q0);
  double            s1 = sin(q0);
  double            c2 = cos(q2);
  double            s2 = sin(q2);
  double            c3 = cos(q3);
  double            s3 = sin(q3);

  /* Calcul du modele d'apres les angles. */
  fMe[0][0] = -c1*s2*c3 - s1*c2*c3;
  fMe[0][1] = c1*s2*s3 + s1*c2*s3;
  fMe[0][2] = c1*c2 - s1*s2;
  fMe[0][3] = c1*this->_long_23 - s1*(this->_Long_23);

  fMe[1][0] = -s1*s2*c3 + c1*c2*c3;
  fMe[1][1] = s1*s2*s3 - c1*c2*s3;
  fMe[1][2] = s1*c2+c1*s2;
  fMe[1][3] = s1*this->_long_23 + c1*(this->_Long_23);

  fMe[2][0] = s3;
  fMe[2][1] = c3;
  fMe[2][2] = 0.f;
  fMe[2][3] = q1;

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
  to the extrinsic camera parameters estimated by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void
vpAfma4::get_cMe(vpHomogeneousMatrix &cMe)
{
  cMe = this->_eMc.inverse();
}

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void
vpAfma4::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;

  return;
}



/*!

  Get the robot jacobian expressed in the end-effector frame.

  \warning Not implemented.

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \param eJe : Robot jacobian expressed in the end-effector frame.
*/
void
vpAfma4::get_eJe(const vpColVector &/*q*/, vpMatrix &eJe)
{

  eJe = 0;
  vpERROR_TRACE("Jacobian expressed in the end-effector frame not implemented");
}

/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame.

  \param q : Articular position of the four joints: q[0] correspond to
  the first rotation of the turret around the vertical axis, q[1]
  correspond to the vertical translation, while q[2] and q[3]
  correspond to the pan and tilt of the camera respectively. Rotations
  q[0], q[2] and q[3] are expressed in radians. The translation q[1]
  is expressed in meters.

  \param fJe : Robot jacobian expressed in the robot reference frame.
*/

void
vpAfma4::get_fJe(const vpColVector &q, vpMatrix &fJe)
{

  fJe.resize(6,4) ;

  double c1 = cos(q[0]);
  double s1 = sin(q[0]);
  double c13 = cos(q[0] + q[2]);
  double s13 = sin(q[0] + q[2]);

  fJe[0][0] = -s1*this->_long_23 - c1*this->_Long_23;
  fJe[0][1] = fJe[0][2] = fJe[0][3] = 0.0;;

  fJe[1][0] = c1*this->_long_23 - s1*this->_Long_23;
  fJe[1][1] = fJe[1][2] = fJe[1][3] = 0.0;;

  fJe[2][1] = 1.0;
  fJe[2][0] = fJe[2][2] = fJe[2][3] = 0.0;;

  fJe[3][0] = fJe[3][1] = fJe[3][2] = 0.0;;
  fJe[3][3] = c13;

  fJe[4][0] = fJe[4][1] = fJe[4][2] = 0.0;;
  fJe[4][3] = s13;

  fJe[5][0] = fJe[5][3] = 1.0;
  fJe[5][2] = fJe[5][4] = 0.0;
}

/*!
  Get min joint values.

  \return Minimal joint values for the 4 dof 
  X, Y, A, B. Translation Y is expressed in meters. Rotations
  X,A and B in radians.

*/
vpColVector
vpAfma4::getJointMin()
{
  vpColVector qmin(4);
  for (int i=0; i < 4; i ++)
    qmin[i] = this->_joint_min[i];
  return qmin;
}

/*!
  Get max joint values.

  \return Maximal joint values for the 4 dof
  X, Y, A, B. Translation Y is expressed in meters. Rotations
  X, A and B in radians.

*/
vpColVector
vpAfma4::getJointMax()
{
  vpColVector qmax(4);
  for (int i=0; i < 4; i ++)
    qmax[i] = this->_joint_max[i];
  return qmax;
}


/*!

  Return the distance between join 2 and 3.

  \return Distance between join 2 and 3.
*/
double
vpAfma4::getLong23()
{
  return _Long_23;
}

/*!

  Return the distance between join 2 and 3.

  \return Distance between join 2 and 3.
*/
double
vpAfma4::getlong23()
{
  return _long_23;
}

/*!

  Print on the output stream \e os the robot parameters (joint
  min/max, distance between axis 5 and 6, coupling factor between axis
  5 and 6, hand-to-eye homogeneous matrix.

  \param os : Output stream.
  \param afma4 : Robot parameters.
*/
std::ostream & operator << (std::ostream & os,
			    const vpAfma4 & afma4)
{
  vpRotationMatrix eRc;
  afma4._eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  os
    << "Joint Max:" << std::endl
    << "\t" << afma4._joint_max[0]
    << "\t" << afma4._joint_max[1]
    << "\t" << afma4._joint_max[2]
    << "\t" << afma4._joint_max[3]
    << "\t" << std::endl

    << "Joint Min: " << std::endl
    << "\t" << afma4._joint_min[0]
    << "\t" << afma4._joint_min[1]
    << "\t" << afma4._joint_min[2]
    << "\t" << afma4._joint_min[3]
    << "\t" << std::endl

    << "long 2-3: " << std::endl
    << "\t" << afma4._long_23
    << "\t" << std::endl

    << "Long 2-3: " << std::endl
    << "\t" << afma4._Long_23
    << "\t" << std::endl

    << "eMc: "<< std::endl
    << "\tTranslation (m): "
    << afma4._eMc[0][3] << " "
    << afma4._eMc[1][3] << " "
    << afma4._eMc[2][3]
    << "\t" << std::endl
    << "\tRotation Rxyz (rad) : "
    << rxyz[0] << " "
    << rxyz[1] << " "
    << rxyz[2]
    << "\t" << std::endl
    << "\tRotation Rxyz (deg) : "
    << vpMath::deg(rxyz[0])  << " "
    << vpMath::deg(rxyz[1])  << " "
    << vpMath::deg(rxyz[2])
    << "\t" << std::endl;

  return os;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
