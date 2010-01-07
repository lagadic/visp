/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
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
 * Interface for a  generic ADEPT Viper (either 650 or 850) robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpViper_h
#define vpViper_h

/*!

  \file vpViper.h

  Modelisation of the ADEPT Viper 650 or 850 robot.

*/

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpRobotException.h>

/*!

  \class vpViper

  \ingroup Viper

  \brief Modelisation of the ADEPT Viper robot 

  This robot has six degrees of freedom.

  The non modified Denavit-Hartenberg representation of the robot is
  given in the table below, where \f$q_1^*, \ldots, q_6^*\f$
  are the variable joint positions.

  \f[
  \begin{tabular}{|c|c|c|c|c|}
  \hline
  Joint & $a_i$ & $d_i$ & $\alpha_i$ & $\theta_i$ \\
  \hline
  1 & $a_1$ & $d_1$ & $-\pi/2$ & $q_1^*$ \\
  2 & $a_2$ & 0     & 0        & $q_2^*$ \\
  3 & $a_3$ & 0     & $-\pi/2$ & $q_3^* - \pi$ \\
  4 & 0     & $d_4$ & $\pi/2$  & $q_4^*$ \\
  5 & 0     & 0     & $-\pi/2$ & $q_5^*$ \\
  6 & 0     & 0     & 0        & $q_6^*-\pi$ \\
  7 & 0     & $d_6$ & 0        & 0 \\
  \hline
  \end{tabular}
  \f]
  
  In this modelisation, different frames have to be considered.
  - \f$ {\cal F}_f \f$: the reference frame, also called world frame,

  - \f$ {\cal F}_w \f$: the wrist frame located at the intersection of
    the last three rotations, with \f$ ^f{\bf M}_w = ^0{\bf M}_6 \f$,

  - \f$ {\cal F}_e \f$: the end-effector frame, with \f$^f{\bf M}_e =
    0{\bf M}_7 \f$,

  - \f$ {\cal F}_c \f$: the camera frame, with \f$^f{\bf M}_c = ^f{\bf
    M}_e \; ^e{\bf M}_c \f$ where \f$ ^e{\bf M}_c \f$ is the result of
    a calibration stage.
  
  The forward kinematics of the robot is implemented in get_fMw(),
  get_fMe() and get_fMc().

  The robot forward jacobian used to compute the cartesian velocities
  from joint ones is given and implemented in get_fJw(), get_fJe() and
  get_eJe().

*/
class VISP_EXPORT vpViper
{
 public:
  vpViper();
  virtual ~vpViper() {};

  vpHomogeneousMatrix getForwardKinematics(const vpColVector & q);
  int getInverseKinematics(const vpHomogeneousMatrix & fMc,
			   vpColVector & q, const bool &nearest=true);
  vpHomogeneousMatrix get_fMc (const vpColVector & q);
  void get_fMw(const vpColVector & q, vpHomogeneousMatrix & fMw);
  void get_wMe(vpHomogeneousMatrix & wMe);
  void get_eMc(vpHomogeneousMatrix & eMc);
  void get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe);
  void get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc);

  void get_cMe(vpHomogeneousMatrix &cMe) ;
  void get_cVe(vpTwistMatrix &cVe) ;
  void get_fJw(const vpColVector &q, vpMatrix &fJw)  ;
  void get_fJe(const vpColVector &q, vpMatrix &fJe)  ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe)  ;

  friend VISP_EXPORT std::ostream & operator << (std::ostream & os,
						 const vpViper & viper);

  vpColVector getJointMin();
  vpColVector getJointMax();
  double getCoupl56();

 public:
  static const int njoint; ///< Number of joint.

 protected:
  vpHomogeneousMatrix eMc; //!< End effector to camera transformation
  // Minimal representation of eMc
  vpTranslationVector etc; // meters
  vpRxyzVector        erc; // radian

  // Denavit-Hartenberg parameters
  double a1, d1; //!< for joint 1
  double a2;     //!< for joint 2
  double a3;     //!< for joint 3
  double d4;     //!< for joint 4
  double d6;     //!< for joint 6
  double c56;    //!< Mechanical coupling between joint 5 and joint 6
  
  // Software joint limits in radians
  vpColVector joint_max; // Maximal value of the joints
  vpColVector joint_min; // Minimal value of the joints


};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

