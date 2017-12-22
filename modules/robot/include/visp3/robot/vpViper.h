/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpRobotException.h>

/*!

  \class vpViper

  \ingroup group_robot_real_arm group_robot_simu_arm

  \brief Modelisation of the ADEPT Viper robot

  This robot has six degrees of freedom. The model of the robot is the
  following: \image html model-viper.png Model of the Viper 850 robot.

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

  - \f$ {\cal F}_f \f$: the reference frame, also called world frame

  - \f$ {\cal F}_w \f$: the wrist frame located at the intersection of
    the last three rotations, with \f$ ^f{\bf M}_w = ^0{\bf M}_6 \f$

  - \f$ {\cal F}_e \f$: the end-effector frame located at the interface of the
    two tool changers, with \f$^f{\bf M}_e = 0{\bf M}_7 \f$

  - \f$ {\cal F}_c \f$: the camera or tool frame, with \f$^f{\bf M}_c = ^f{\bf
    M}_e \; ^e{\bf M}_c \f$ where \f$ ^e{\bf M}_c \f$ is the result of
    a calibration stage. We can also consider a custom tool TOOL_CUSTOM and
  set this during robot initialisation or using set_eMc().

  - \f$ {\cal F}_s \f$: the force/torque sensor frame, with \f$d7=0.0666\f$.

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
  virtual ~vpViper(){};

  /** @name Inherited functionalities from vpViper */
  //@{
  vpHomogeneousMatrix getForwardKinematics(const vpColVector &q) const;
  unsigned int getInverseKinematicsWrist(const vpHomogeneousMatrix &fMw, vpColVector &q,
                                         const bool &verbose = false) const;
  unsigned int getInverseKinematics(const vpHomogeneousMatrix &fMc, vpColVector &q, const bool &verbose = false) const;
  vpHomogeneousMatrix get_fMc(const vpColVector &q) const;
  void get_fMw(const vpColVector &q, vpHomogeneousMatrix &fMw) const;
  void get_wMe(vpHomogeneousMatrix &wMe) const;
  void get_eMc(vpHomogeneousMatrix &eMc) const;
  void get_eMs(vpHomogeneousMatrix &eMs) const;
  void get_fMe(const vpColVector &q, vpHomogeneousMatrix &fMe) const;
  void get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const;

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_fJw(const vpColVector &q, vpMatrix &fJw) const;
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const;
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const;

  virtual void set_eMc(const vpHomogeneousMatrix &eMc_);
  virtual void set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_);

  vpColVector getJointMin() const;
  vpColVector getJointMax() const;
  double getCoupl56() const;
  //@}

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpViper &viper);

private:
  bool convertJointPositionInLimits(unsigned int joint, const double &q, double &q_mod,
                                    const bool &verbose = false) const;

public:
  static const unsigned int njoint; ///< Number of joint.

protected:
  vpHomogeneousMatrix eMc; //!< End effector to camera transformation
  // Minimal representation of eMc
  vpTranslationVector etc; // meters
  vpRxyzVector erc;        // radian

  // Denavit-Hartenberg parameters
  double a1, d1; //!< for joint 1
  double a2;     //!< for joint 2
  double a3;     //!< for joint 3
  double d4;     //!< for joint 4
  double d6;     //!< for joint 6
  double d7;     //!< for force/torque location
  double c56;    //!< Mechanical coupling between joint 5 and joint 6

  // Software joint limits in radians
  vpColVector joint_max; // Maximal value of the joints
  vpColVector joint_min; // Minimal value of the joints
};

#endif
