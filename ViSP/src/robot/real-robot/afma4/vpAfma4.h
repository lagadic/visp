/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpAfma4_h
#define __vpAfma4_h

/*!

  \file vpAfma4.h

  Modelisation of Irisa's cylindrical robot named Afma4.

*/

/*!

  \class vpAfma4

  \ingroup Afma4

  \brief Modelisation of Irisa's cylindrical robot named Afma4.

  This robot has five degrees of freedom, but only four motorized
  joints (joint 3 is not motorized). Joint 2 and 3 are prismatic. The
  other ones are revolute joints.

  The non modified Denavit-Hartenberg representation of the robot is
  given in the table below, where \f$q_1^*, q_2^*,q_4^*, q_5^*\f$
  are the variable joint positions.

  \f[
  \begin{tabular}{|c|c|c|c|c|}
  \hline
  Joint & $a_i$ & $d_i$ & $\alpha_i$ & $\theta_i$ \\
  \hline
  1 & 0     & 0       & 0        & $q_1^*$ \\
  2 & $a_1$ & $q_2^*$ & $-\pi/2$ & 0 \\
  3 & 0     & $d_3$   & $\pi/2$  & 0 \\
  4 & 0     & $d_4$   & $-\pi/2$ & $q_4^*-\pi/2$ \\
  5 & 0     & 0       & 0        & $q_5^*$ \\
  \hline
  \end{tabular}
  \f]

  The forward kinematics of the robot is given by the homogeneous
  matrix \f${^f}M_e\f$ which is implemented in get_fMe(). 

  \f[
  {^f}M_e = \left[\begin{array}{cccc}
  c_1s_4c_5+s_1c_4c_5  & -c_1s_4s_5-s_1c_4s_5 & c_1c_4-s_1s_4 &a_1c_1-d_3s_1 \\
  s_1s_4c_5-c_1c_4c_5  & -s_1s_4s_5+c_1c_4s_5 & s_1c_4+c_1s_4 &a_1s_1+d_3c_1 \\
  -s_5 & -c_5  & d_4+q_2 \\
  0  &   0  &   0  &   1    \\
  \end{array}
  \right]
  \f]

  The robot forward jacobian used to compute the cartesian velocities
  from joint ones is given and implemented in get_fJe() and
  get_eJe().

  The robot inverse jacobian used to compute the joint velocities from
  cartesian ones are given and implemented in get_fJe_inverse().

*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpVelocityTwistMatrix.h>


class VISP_EXPORT vpAfma4
{
 public:
  vpAfma4();

  void init (void);

  vpHomogeneousMatrix getForwardKinematics(const vpColVector & q);
/*   int getInverseKinematics(const vpHomogeneousMatrix & fMc, */
/* 			   vpColVector & q, const bool &nearest=true); */
  vpHomogeneousMatrix get_fMc (const vpColVector & q);
  void get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe);
  void get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc);

  void get_cMe(vpHomogeneousMatrix &cMe) ;
  void get_cVe(vpVelocityTwistMatrix &cVe) ;
  void get_cVf(const vpColVector & q, vpVelocityTwistMatrix &cVf);
  void get_eJe(const vpColVector &q, vpMatrix &eJe)  ;
  void get_fJe(const vpColVector &q, vpMatrix &fJe)  ;
  void get_fJe_inverse(const vpColVector &q, vpMatrix &fJe_inverse)  ;

  friend VISP_EXPORT std::ostream & operator << (std::ostream & os,
						 const vpAfma4 & afma4);

  vpColVector getJointMin();
  vpColVector getJointMax();

 public:

  static const unsigned int njoint; ///< Number of joint.

 protected:
  // Denavit Hartenberg parameters
  double _a1; // distance along x2
  double _d3; // distance along z2
  double _d4; // distance along z3
  double _joint_max[4]; // Maximal value of the joints
  double _joint_min[4]; // Minimal value of the joints

  // Minimal representation of _eMc
  vpTranslationVector _etc; // meters
  vpRxyzVector        _erc; // radian

  vpHomogeneousMatrix _eMc; // Camera extrinsic parameters: effector to camera
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

