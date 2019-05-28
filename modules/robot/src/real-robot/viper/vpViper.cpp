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
 * Interface for a  generic ADEPT Viper (either 650 or 850) robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpViper.cpp

  Modelisation of the ADEPT Viper 650 or 850 robot.

*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpViper.h>

const unsigned int vpViper::njoint = 6;

/*!

  Default constructor.

*/
vpViper::vpViper()
  : eMc(), etc(), erc(), a1(0), d1(0), a2(), a3(), d4(0), d6(0), d7(0), c56(0), joint_max(), joint_min()
{
  // Default values are initialized

  // Denavit Hartenberg parameters
  a1 = 0.075;
  a2 = 0.365;
  a3 = 0.090;
  d1 = 0.335;
  d4 = 0.405;
  d6 = 0.080;
  d7 = 0.0666;
  c56 = -341.33 / 9102.22;

  // Software joint limits in radians
  joint_min.resize(njoint);
  joint_min[0] = vpMath::rad(-170);
  joint_min[1] = vpMath::rad(-190);
  joint_min[2] = vpMath::rad(-29);
  joint_min[3] = vpMath::rad(-190);
  joint_min[4] = vpMath::rad(-120);
  joint_min[5] = vpMath::rad(-360);
  joint_max.resize(njoint);
  joint_max[0] = vpMath::rad(170);
  joint_max[1] = vpMath::rad(45);
  joint_max[2] = vpMath::rad(256);
  joint_max[3] = vpMath::rad(190);
  joint_max[4] = vpMath::rad(120);
  joint_max[5] = vpMath::rad(360);

  // End effector to camera transformation
  eMc.eye();
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the six joint positions.

  This method is the same than get_fMc(const vpColVector & q).

  \param q : A six dimension vector corresponding to the robot joint
  positions expressed in radians.

  \return The homogeneous matrix \f$^f{\bf M}_c \f$ corresponding to
  the direct geometric model which expresses the transformation
  between the base frame and the camera frame.

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix vpViper::getForwardKinematics(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Convert a joint position by applying modulo \f$2 \pi\f$ to ensure
  that the position is in the joint limits.

  \param joint : Joint to consider.

  \param q : A joint position.

  \param q_mod : The joint position modified by considering modulo
  \f$2 \pi\f$ to be in the joint limits.

  \return true if the joint position is in the joint limits. false otherwise.
 */
bool vpViper::convertJointPositionInLimits(unsigned int joint, const double &q, double &q_mod,
                                           const bool &verbose) const
{
  double eps = 0.01;
  if (q >= joint_min[joint] - eps && q <= joint_max[joint] + eps) {
    q_mod = q;
    return true;
  }

  q_mod = q + 2 * M_PI;
  if (q_mod >= joint_min[joint] - eps && q_mod <= joint_max[joint] + eps) {
    return true;
  }

  q_mod = q - 2 * M_PI;
  if (q_mod >= joint_min[joint] - eps && q_mod <= joint_max[joint] + eps) {
    return true;
  }

  if (verbose) {
    std::cout << "Joint " << joint << " not in limits: " << this->joint_min[joint] << " < " << q << " < "
              << this->joint_max[joint] << std::endl;
  }

  return false;
}

/*!

  Compute the inverse kinematics (inverse geometric model).

  By inverse kinematics we mean here the six joint values given the
  position and the orientation of the camera frame relative to the
  base frame.

  \param fMw : Homogeneous matrix \f$^f{\bf M}_w \f$ describing the
  transformation from base frame to the wrist frame.

  \param q : In input, a six dimension vector corresponding to the
  current joint positions expressed in radians. In output, the
  solution of the inverse kinematics, ie. the joint positions
  corresponding to \f$^f{\bf M}_w \f$.

  \param verbose : Add extra printings.

  \return Add printings if no solution was found.

  \return The number of solutions (1 to 8) of the inverse geometric
  model. O, if no solution can be found.

  The code below shows how to compute the inverse geometric model:

  \code
  vpColVector q1(6), q2(6);
  vpHomogeneousMatrix fMw;

  vpViper robot;

  // Get the current joint position of the robot
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q1);

  // Compute the pose of the wrist in the reference frame using the
  // direct geometric model
  robot.get_fMw(q1, fMw);

  // Compute the inverse geometric model
  int nbsol; // number of solutions (0, 1 to 8) of the inverse geometric model
  // get the nearest solution to the current joint position
  nbsol = robot.getInverseKinematicsWrist(fMw, q1);

  if (nbsol == 0)
    std::cout << "No solution of the inverse geometric model " << std::endl;
  else if (nbsol >= 1)
    std::cout << "Nearest solution: " << q1 << std::endl;
  \endcode

  \sa getForwardKinematics(), getInverseKinematics()

*/
unsigned int vpViper::getInverseKinematicsWrist(const vpHomogeneousMatrix &fMw, vpColVector &q,
                                                const bool &verbose) const
{
  vpColVector q_sol[8];

  for (unsigned int i = 0; i < 8; i++)
    q_sol[i].resize(6);

  double c1[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s1[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double c3[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s3[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double c23[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s23[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double c4[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s4[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double c5[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s5[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double c6[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double s6[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  bool ok[8];

  if (q.getRows() != njoint)
    q.resize(6);

  for (unsigned int i = 0; i < 8; i++)
    ok[i] = true;

  double px = fMw[0][3]; // a*c1
  double py = fMw[1][3]; // a*s1
  double pz = fMw[2][3];

  // Compute q1
  double a_2 = px * px + py * py;
  // if (a_2 == 0) {// singularity
  if (std::fabs(a_2) <= std::numeric_limits<double>::epsilon()) { // singularity
    c1[0] = cos(q[0]);
    s1[0] = sin(q[0]);
    c1[4] = cos(q[0] + M_PI);
    s1[4] = sin(q[0] + M_PI);
  } else {
    double a = sqrt(a_2);
    c1[0] = px / a;
    s1[0] = py / a;
    c1[4] = -px / a;
    s1[4] = -py / a;
  }

  double q1_mod;
  for (unsigned int i = 0; i < 8; i += 4) {
    q_sol[i][0] = atan2(s1[i], c1[i]);
    if (convertJointPositionInLimits(0, q_sol[i][0], q1_mod, verbose) == true) {
      q_sol[i][0] = q1_mod;
      for (unsigned int j = 1; j < 4; j++) {
        c1[i + j] = c1[i];
        s1[i + j] = s1[i];
        q_sol[i + j][0] = q_sol[i][0];
      }
    } else {
      for (unsigned int j = 1; j < 4; j++)
        ok[i + j] = false;
    }
  }

  // Compute q3
  double K, q3_mod;
  for (unsigned int i = 0; i < 8; i += 4) {
    if (ok[i] == true) {
      K = (px * px + py * py + pz * pz + a1 * a1 - a2 * a2 - a3 * a3 + d1 * d1 - d4 * d4 -
           2 * (a1 * c1[i] * px + a1 * s1[i] * py + d1 * pz)) /
          (2 * a2);
      double d4_a3_K = d4 * d4 + a3 * a3 - K * K;

      q_sol[i][2] = atan2(a3, d4) + atan2(K, sqrt(d4_a3_K));
      q_sol[i + 2][2] = atan2(a3, d4) + atan2(K, -sqrt(d4_a3_K));

      for (unsigned int j = 0; j < 4; j += 2) {
        if (d4_a3_K < 0) {
          for (unsigned int k = 0; k < 2; k++)
            ok[i + j + k] = false;
        } else {
          if (convertJointPositionInLimits(2, q_sol[i + j][2], q3_mod, verbose) == true) {
            for (unsigned int k = 0; k < 2; k++) {
              q_sol[i + j + k][2] = q3_mod;
              c3[i + j + k] = cos(q3_mod);
              s3[i + j + k] = sin(q3_mod);
            }
          } else {
            for (unsigned int k = 0; k < 2; k++)
              ok[i + j + k] = false;
          }
        }
      }
    }
  }
  //   std::cout << "ok apres q3: ";
  //   for (unsigned int i=0; i< 8; i++)
  //     std::cout << ok[i] << " ";
  //   std::cout << std::endl;

  // Compute q2
  double q23[8], q2_mod;
  for (unsigned int i = 0; i < 8; i += 2) {
    if (ok[i] == true) {
      // Compute q23 = q2+q3
      c23[i] = (-(a3 - a2 * c3[i]) * (c1[i] * px + s1[i] * py - a1) - (d1 - pz) * (d4 + a2 * s3[i])) /
               ((c1[i] * px + s1[i] * py - a1) * (c1[i] * px + s1[i] * py - a1) + (d1 - pz) * (d1 - pz));
      s23[i] = ((d4 + a2 * s3[i]) * (c1[i] * px + s1[i] * py - a1) - (d1 - pz) * (a3 - a2 * c3[i])) /
               ((c1[i] * px + s1[i] * py - a1) * (c1[i] * px + s1[i] * py - a1) + (d1 - pz) * (d1 - pz));
      q23[i] = atan2(s23[i], c23[i]);
      // std::cout << i << " c23 = " << c23[i] << " s23 = " << s23[i] <<
      // std::endl;
      // q2 = q23 - q3
      q_sol[i][1] = q23[i] - q_sol[i][2];

      if (convertJointPositionInLimits(1, q_sol[i][1], q2_mod, verbose) == true) {
        for (unsigned int j = 0; j < 2; j++) {
          q_sol[i + j][1] = q2_mod;
          c23[i + j] = c23[i];
          s23[i + j] = s23[i];
        }
      } else {
        for (unsigned int j = 0; j < 2; j++)
          ok[i + j] = false;
      }
    }
  }
  //   std::cout << "ok apres q2: ";
  //   for (unsigned int i=0; i< 8; i++)
  //     std::cout << ok[i] << " ";
  //   std::cout << std::endl;

  // Compute q4 as long as s5 != 0
  double r13 = fMw[0][2];
  double r23 = fMw[1][2];
  double r33 = fMw[2][2];
  double s4s5, c4s5, q4_mod, q5_mod;
  for (unsigned int i = 0; i < 8; i += 2) {
    if (ok[i] == true) {
      s4s5 = -s1[i] * r13 + c1[i] * r23;
      c4s5 = c1[i] * c23[i] * r13 + s1[i] * c23[i] * r23 - s23[i] * r33;
      if (fabs(s4s5) < vpMath::rad(0.5) && fabs(c4s5) < vpMath::rad(0.5)) {
        // s5 = 0
        c5[i] = c1[i] * s23[i] * r13 + s1[i] * s23[i] * r23 + c23[i] * r33;
        // std::cout << "Singularity: s5 near 0: ";
        if (c5[i] > 0.)
          q_sol[i][4] = 0.0;
        else
          q_sol[i][4] = M_PI;

        if (convertJointPositionInLimits(4, q_sol[i][4], q5_mod, verbose) == true) {
          for (unsigned int j = 0; j < 2; j++) {
            q_sol[i + j][3] = q[3]; // keep current q4
            q_sol[i + j][4] = q5_mod;
            c4[i] = cos(q_sol[i + j][3]);
            s4[i] = sin(q_sol[i + j][3]);
          }
        } else {
          for (unsigned int j = 0; j < 2; j++)
            ok[i + j] = false;
        }
      } else {
// s5 != 0
#if 0 // Modified 2016/03/10 since if and else are the same
      // if (c4s5 == 0) {
        if (std::fabs(c4s5) <= std::numeric_limits<double>::epsilon()) {
          // c4 = 0
          //  vpTRACE("c4 = 0");
          // q_sol[i][3] = q[3]; // keep current position
          q_sol[i][3] = atan2(s4s5, c4s5);
        }
        else {
          q_sol[i][3] = atan2(s4s5, c4s5);
        }
#else
        q_sol[i][3] = atan2(s4s5, c4s5);
#endif
        if (convertJointPositionInLimits(3, q_sol[i][3], q4_mod, verbose) == true) {
          q_sol[i][3] = q4_mod;
          c4[i] = cos(q4_mod);
          s4[i] = sin(q4_mod);
        } else {
          ok[i] = false;
        }
        if (q_sol[i][3] > 0.)
          q_sol[i + 1][3] = q_sol[i][3] + M_PI;
        else
          q_sol[i + 1][3] = q_sol[i][3] - M_PI;
        if (convertJointPositionInLimits(3, q_sol[i + 1][3], q4_mod, verbose) == true) {
          q_sol[i + 1][3] = q4_mod;
          c4[i + 1] = cos(q4_mod);
          s4[i + 1] = sin(q4_mod);
        } else {
          ok[i + 1] = false;
        }

        // Compute q5
        for (unsigned int j = 0; j < 2; j++) {
          if (ok[i + j] == true) {
            c5[i + j] = c1[i + j] * s23[i + j] * r13 + s1[i + j] * s23[i + j] * r23 + c23[i + j] * r33;
            s5[i + j] = (c1[i + j] * c23[i + j] * c4[i + j] - s1[i + j] * s4[i + j]) * r13 +
                        (s1[i + j] * c23[i + j] * c4[i + j] + c1[i + j] * s4[i + j]) * r23 -
                        s23[i + j] * c4[i + j] * r33;

            q_sol[i + j][4] = atan2(s5[i + j], c5[i + j]);
            if (convertJointPositionInLimits(4, q_sol[i + j][4], q5_mod, verbose) == true) {
              q_sol[i + j][4] = q5_mod;
            } else {

              ok[i + j] = false;
            }
          }
        }
      }
    }
  }

  // Compute q6
  // 4 solutions for q6 and 4 more solutions by flipping the wrist (see below)
  double r12 = fMw[0][1];
  double r22 = fMw[1][1];
  double r32 = fMw[2][1];
  double q6_mod;
  for (unsigned int i = 0; i < 8; i++) {
    c6[i] = -(c1[i] * c23[i] * s4[i] + s1[i] * c4[i]) * r12 + (c1[i] * c4[i] - s1[i] * c23[i] * s4[i]) * r22 +
            s23[i] * s4[i] * r32;
    s6[i] = -(c1[i] * c23[i] * c4[i] * c5[i] - c1[i] * s23[i] * s5[i] - s1[i] * s4[i] * c5[i]) * r12 -
            (s1[i] * c23[i] * c4[i] * c5[i] - s1[i] * s23[i] * s5[i] + c1[i] * s4[i] * c5[i]) * r22 +
            (c23[i] * s5[i] + s23[i] * c4[i] * c5[i]) * r32;

    q_sol[i][5] = atan2(s6[i], c6[i]);
    if (convertJointPositionInLimits(5, q_sol[i][5], q6_mod, verbose) == true) {
      q_sol[i][5] = q6_mod;
    } else {
      ok[i] = false;
    }
  }

  // Select the best config in terms of distance from the current position
  unsigned int nbsol = 0;
  unsigned int sol = 0;
  vpColVector dist(8);
  for (unsigned int i = 0; i < 8; i++) {
    if (ok[i] == true) {
      nbsol++;
      sol = i;
      //      dist[i] = vpColVector::distance(q, q_sol[i]);
      vpColVector weight(6);
      weight = 1;
      weight[0] = 8;
      weight[1] = weight[2] = 4;
      dist[i] = 0;
      for (unsigned int j = 0; j < 6; j++) {
        double rought_dist = q[j] - q_sol[i][j];
        double modulo_dist = rought_dist;
        if (rought_dist > 0) {
          if (fabs(rought_dist - 2 * M_PI) < fabs(rought_dist))
            modulo_dist = rought_dist - 2 * M_PI;
        } else {
          if (fabs(rought_dist + 2 * M_PI) < fabs(rought_dist))
            modulo_dist = rought_dist + 2 * M_PI;
        }
        // std::cout << "dist " << i << ": " << rought_dist << " modulo: " <<
        // modulo_dist << std::endl;
        dist[i] += weight[j] * vpMath::sqr(modulo_dist);
      }
    }
    //  std::cout << "sol " << i << " [" << ok[i] << "] dist: " << dist[i] <<
    //  " q: " << q_sol[i].t() << std::endl;
  }
  // std::cout << "dist: " << dist.t() << std::endl;
  if (nbsol) {
    for (unsigned int i = 0; i < 8; i++) {
      if (ok[i] == true)
        if (dist[i] < dist[sol])
          sol = i;
    }
    // Update the inverse kinematics solution
    q = q_sol[sol];

    //     std::cout << "Nearest solution (" << sol << ") with distance ("
    // 	      << dist[sol] << "): " << q_sol[sol].t() << std::endl;
  }
  return nbsol;
}

/*!

  Compute the inverse kinematics (inverse geometric model).

  By inverse kinematics we mean here the six joint values given the
  position and the orientation of the camera frame relative to the
  base frame.

  \param fMc : Homogeneous matrix \f$^f{\bf M}_c \f$ describing the
  transformation from base frame to the camera frame.

  \param q : In input, a six dimension vector corresponding to the
  current joint positions expressed in radians. In output, the
  solution of the inverse kinematics, ie. the joint positions
  corresponding to \f$^f{\bf M}_c \f$.

  \param verbose : Add extra printings.

  \return Add printings if no solution was found.

  \return The number of solutions (1 to 8) of the inverse geometric
  model. O, if no solution can be found.

  The code below shows how to compute the inverse geometric model:

  \code
  vpColVector q1(6), q2(6);
  vpHomogeneousMatrix fMc;

  vpViper robot;

  // Get the current joint position of the robot
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q1);

  // Compute the pose of the camera in the reference frame using the
  // direct geometric model
  fMc = robot.getForwardKinematics(q1);
  // this is similar to  fMc = robot.get_fMc(q1);
  // or robot.get_fMc(q1, fMc);

  // Compute the inverse geometric model
  int nbsol; // number of solutions (0, 1 to 8) of the inverse geometric model
  // get the nearest solution to the current joint position
  nbsol = robot.getInverseKinematics(fMc, q1);

  if (nbsol == 0)
    std::cout << "No solution of the inverse geometric model " << std::endl;
  else if (nbsol >= 1)
    std::cout << "Nearest solution: " << q1 << std::endl;
  \endcode

  \sa getForwardKinematics(), getInverseKinematicsWrist

*/
unsigned int vpViper::getInverseKinematics(const vpHomogeneousMatrix &fMc, vpColVector &q, const bool &verbose) const
{
  vpHomogeneousMatrix fMw;
  vpHomogeneousMatrix wMe;
  vpHomogeneousMatrix eMc_;
  this->get_wMe(wMe);
  this->get_eMc(eMc_);
  fMw = fMc * eMc_.inverse() * wMe.inverse();

  return (getInverseKinematicsWrist(fMw, q, verbose));
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the joint positions of all
  the six joints.

  \f[
  ^f{\bf M}_c = ^f{\bf M}_e \; ^e{\bf M}_c
  \f]

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Vector of six joint positions expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa getForwardKinematics(const vpColVector & q), get_fMe(), get_eMc()

*/
vpHomogeneousMatrix vpViper::get_fMc(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the six joint positions.

  \f[
  ^f{\bf M}_c = ^f{\bf M}_e \; {^e}{\bf M}_c
  \f]

  \param q : Vector of six joint positions expressed in
  radians.

  \param fMc The homogeneous matrix \f$^f{\bf M}_c\f$corresponding to
  the direct geometric model which expresses the transformation
  between the fix frame and the camera frame.

  \sa get_fMe(), get_eMc()
*/
void vpViper::get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{

  // Compute the direct geometric model: fMe = transformation between
  // fix and end effector frame.
  vpHomogeneousMatrix fMe;

  get_fMe(q, fMe);

  fMc = fMe * this->eMc;

  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix \f${^f}{\bf M}_e\f$.

  By forward kinematics we mean here the position and the orientation
  of the end effector with respect to the base frame given the
  motor positions of all the six joints.

  \f[
  {^f}M_e = \left(\begin{array}{cccc}
  r_{11} & r_{12} & r_{13} & t_x  \\
  r_{21} & r_{22} & r_{23} & t_y  \\
  r_{31} & r_{32} & r_{33} & t_z  \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  r_{11} = c1(c23(c4c5c6-s4s6)-s23s5c6)-s1(s4c5c6+c4s6) \\
  r_{21} = -s1(c23(-c4c5c6+s4s6)+s23s5c6)+c1(s4c5c6+c4s6) \\
  r_{31} = s23(s4s6-c4c5c6)-c23s5c6 \\
  \\
  r_{12} = -c1(c23(c4c5s6+s4c6)-s23s5s6)+s1(s4c5s6-c4c6)\\
  r_{22} = -s1(c23(c4c5s6+s4c6)-s23s5s6)-c1(s4c5s6-c4c6)\\
  r_{32} = s23(c4c5s6+s4c6)+c23s5s6\\
  \\
  r_{13} = c1(c23c4s5+s23c5)-s1s4s5\\
  r_{23} = s1(c23c4s5+s23c5)+c1s4s5\\
  r_{33} = -s23c4s5+c23c5\\
  \\
  t_x = c1(c23(c4s5d6-a3)+s23(c5d6+d4)+a1+a2c2)-s1s4s5d6\\
  t_y = s1(c23(c4s5d6-a3)+s23(c5d6+d4)+a1+a2c2)+c1s4s5d6\\
  t_z = s23(a3-c4s5d6)+c23(c5d6+d4)-a2s2+d1\\
  \end{array}
  \f]

  \param q : A 6-dimension vector that contains the 6 joint positions
  expressed in radians.

  \param fMe The homogeneous matrix \f${^f}{\bf M}_e\f$ corresponding to the
direct geometric model which expresses the transformation between the fix
frame and the end effector frame.

  Note that this transformation can also be computed by considering the wrist
  frame \f${^f}{\bf M}_e = {^f}{\bf M}_w *{^w}{\bf M}_e\f$.

  \code
#include <visp3/robot/vpViper.h>

int main()
{
  vpViper robot;
  vpColVector q(6); // The measured six joint positions

  vpHomogeneousMatrix fMe; // Transformation from fix frame to end-effector
  robot.get_fMe(q, fMe); // Get the forward kinematics

  // The forward kinematics can also be computed by considering the wrist frame
  vpHomogeneousMatrix fMw; // Transformation from fix frame to wrist frame
  robot.get_fMw(q, fMw);
  vpHomogeneousMatrix wMe; // Transformation from wrist frame to end-effector
  robot.get_wMe(wMe); // Constant transformation

  // Compute the forward kinematics
  fMe = fMw * wMe;
}
  \endcode

*/
void vpViper::get_fMe(const vpColVector &q, vpHomogeneousMatrix &fMe) const
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
  double q6 = q[5];
  //  We turn off the coupling since the measured positions are joint position
  //  taking into account the coupling factor. The coupling factor is relevant
  //  if positions are motor position.
  // double q6 = q[5] + c56 * q[4];

  //   std::cout << "q6 motor: " << q[5] << " rad "
  // 	    << vpMath::deg(q[5]) << " deg" << std::endl;
  //   std::cout << "q6 joint: " << q6 << " rad "
  // 	    << vpMath::deg(q6) << " deg" << std::endl;

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  // double c3 = cos(q3);
  // double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c6 = cos(q6);
  double s6 = sin(q6);
  double c23 = cos(q2 + q3);
  double s23 = sin(q2 + q3);

  fMe[0][0] = c1 * (c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6) - s1 * (s4 * c5 * c6 + c4 * s6);
  fMe[1][0] = -s1 * (c23 * (-c4 * c5 * c6 + s4 * s6) + s23 * s5 * c6) + c1 * (s4 * c5 * c6 + c4 * s6);
  fMe[2][0] = s23 * (s4 * s6 - c4 * c5 * c6) - c23 * s5 * c6;

  fMe[0][1] = -c1 * (c23 * (c4 * c5 * s6 + s4 * c6) - s23 * s5 * s6) + s1 * (s4 * c5 * s6 - c4 * c6);
  fMe[1][1] = -s1 * (c23 * (c4 * c5 * s6 + s4 * c6) - s23 * s5 * s6) - c1 * (s4 * c5 * s6 - c4 * c6);
  fMe[2][1] = s23 * (c4 * c5 * s6 + s4 * c6) + c23 * s5 * s6;

  fMe[0][2] = c1 * (c23 * c4 * s5 + s23 * c5) - s1 * s4 * s5;
  fMe[1][2] = s1 * (c23 * c4 * s5 + s23 * c5) + c1 * s4 * s5;
  fMe[2][2] = -s23 * c4 * s5 + c23 * c5;

  fMe[0][3] = c1 * (c23 * (c4 * s5 * d6 - a3) + s23 * (c5 * d6 + d4) + a1 + a2 * c2) - s1 * s4 * s5 * d6;
  fMe[1][3] = s1 * (c23 * (c4 * s5 * d6 - a3) + s23 * (c5 * d6 + d4) + a1 + a2 * c2) + c1 * s4 * s5 * d6;
  fMe[2][3] = s23 * (a3 - c4 * s5 * d6) + c23 * (c5 * d6 + d4) - a2 * s2 + d1;

  // std::cout << "Effector position fMe: " << std::endl << fMe;

  return;
}
/*!

  Compute the transformation between the fix frame and the wrist frame. The
  wrist frame is located on the intersection of the 3 last rotations.

  \param q : A 6-dimension vector that contains the 6 joint positions
  expressed in radians.

  \param fMw The homogeneous matrix corresponding to the transformation
  between the fix frame and the wrist frame (fMw).

  \f[
  {^f}M_w = \left(\begin{array}{cccc}
  r_{11} & r_{12} & r_{13} & t_x  \\
  r_{21} & r_{22} & r_{23} & t_y  \\
  r_{31} & r_{32} & r_{33} & t_z  \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  r_{11} = c1(c23(c4c5c6-s4s6)-s23s5c6)-s1(s4c5c6+c4s6) \\
  r_{21} = -s1(c23(-c4c5c6+s4s6)+s23s5c6)+c1(s4c5c6+c4s6) \\
  r_{31} = s23(s4s6-c4c5c6)-c23s5c6 \\
  \\
  r_{12} = -c1(c23(c4c5s6+s4c6)-s23s5s6)+s1(s4c5s6-c4c6)\\
  r_{22} = -s1(c23(c4c5s6+s4c6)-s23s5s6)-c1(s4c5s6-c4c6)\\
  r_{32} = s23(c4c5s6+s4c6)+c23s5s6\\
  \\
  r_{13} = c1(c23c4s5+s23c5)-s1s4s5\\
  r_{23} = s1(c23c4s5+s23c5)+c1s4s5\\
  r_{33} = -s23c4s5+c23c5\\
  \\
  t_x = c1(-c23a3+s23d4+a1+a2c2)\\
  t_y = s1(-c23a3+s23d4+a1+a2c2)\\
  t_z = s23a3+c23d4-a2s2+d1\\
  \end{array}
  \f]

*/
void vpViper::get_fMw(const vpColVector &q, vpHomogeneousMatrix &fMw) const
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
  double q6 = q[5];
  //  We turn off the coupling since the measured positions are joint position
  //  taking into account the coupling factor. The coupling factor is relevant
  //  if positions are motor position.
  // double q6 = q[5] + c56 * q[4];

  //   std::cout << "q6 motor: " << q[5] << " rad "
  // 	    << vpMath::deg(q[5]) << " deg" << std::endl;
  //   std::cout << "q6 joint: " << q6 << " rad "
  // 	    << vpMath::deg(q6) << " deg" << std::endl;

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  //  double c3 = cos(q3);
  // double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c6 = cos(q6);
  double s6 = sin(q6);
  double c23 = cos(q2 + q3);
  double s23 = sin(q2 + q3);

  fMw[0][0] = c1 * (c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6) - s1 * (s4 * c5 * c6 + c4 * s6);
  fMw[1][0] = -s1 * (c23 * (-c4 * c5 * c6 + s4 * s6) + s23 * s5 * c6) + c1 * (s4 * c5 * c6 + c4 * s6);
  fMw[2][0] = s23 * (s4 * s6 - c4 * c5 * c6) - c23 * s5 * c6;

  fMw[0][1] = -c1 * (c23 * (c4 * c5 * s6 + s4 * c6) - s23 * s5 * s6) + s1 * (s4 * c5 * s6 - c4 * c6);
  fMw[1][1] = -s1 * (c23 * (c4 * c5 * s6 + s4 * c6) - s23 * s5 * s6) - c1 * (s4 * c5 * s6 - c4 * c6);
  fMw[2][1] = s23 * (c4 * c5 * s6 + s4 * c6) + c23 * s5 * s6;

  fMw[0][2] = c1 * (c23 * c4 * s5 + s23 * c5) - s1 * s4 * s5;
  fMw[1][2] = s1 * (c23 * c4 * s5 + s23 * c5) + c1 * s4 * s5;
  fMw[2][2] = -s23 * c4 * s5 + c23 * c5;

  fMw[0][3] = c1 * (-c23 * a3 + s23 * d4 + a1 + a2 * c2);
  fMw[1][3] = s1 * (-c23 * a3 + s23 * d4 + a1 + a2 * c2);
  fMw[2][3] = s23 * a3 + c23 * d4 - a2 * s2 + d1;

  // std::cout << "Wrist position fMw: " << std::endl << fMw;

  return;
}

/*!

  Return the transformation between the wrist frame and the end-effector. The
  wrist frame is located on the intersection of the 3 last rotations.


  \param wMe The homogeneous matrix corresponding to the transformation
  between the wrist frame and the end-effector frame (wMe).

*/
void vpViper::get_wMe(vpHomogeneousMatrix &wMe) const
{
  // Set the rotation as identity
  wMe.eye();

  // Set the translation
  wMe[2][3] = d6;
}

/*!

  Get the geometric transformation between the end-effector frame and
  the camera frame. This transformation is constant and correspond to
  the extrinsic camera parameters estimated by calibration.

  \param eMc_ : Transformation between the the
  end-effector frame and the camera frame.

  \sa get_cMe()
*/
void vpViper::get_eMc(vpHomogeneousMatrix &eMc_) const { eMc_ = this->eMc; }

/*!

  Get the geometric transformation between the end-effector frame and
  the force/torque sensor frame. This transformation is constant.

  \param eMs : Transformation between the the
  end-effector frame and the force/torque sensor frame.

*/
void vpViper::get_eMs(vpHomogeneousMatrix &eMs) const
{
  eMs.eye();
  eMs[2][3] = -d7; // tz = -d7
}

/*!

  Get the geometric transformation between the camera frame and the
  end-effector frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

  \sa get_eMc()
*/
void vpViper::get_cMe(vpHomogeneousMatrix &cMe) const { cMe = this->eMc.inverse(); }

/*!

  Get the twist transformation \f$^c{\bf V}_e\f$ from camera frame to
  end-effector frame.  This transformation allows to compute a velocity
  expressed in the end-effector frame into the camera frame. \f[ ^c{\bf V}_e =
  \left(\begin{array}{cc}
  ^c{\bf R}_e & [^c{\bf t}_e]_\times ^c{\bf R}_e\\
  {\bf 0}_{3\times 3} & ^c{\bf R}_e
  \end{array}
  \right)
  \f]
  \param cVe : Twist transformation \f$^c{\bf V}_e\f$.

*/
void vpViper::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  get_cMe(cMe);

  cVe.buildFrom(cMe);

  return;
}

/*!

  Get the robot jacobian \f${^e}{\bf J}_e\f$ which gives the velocity
  of the origin of the end-effector frame expressed in end-effector frame.

  \f[
  {^e}{\bf J}_e = \left[\begin{array}{cc}
  {^w}{\bf R}_f &  {[{^e}{\bf t}_w}]_\times \; {^w}{\bf R}_f \\
  0_{3\times3} & {^w}{\bf R}_f
  \end{array}
  \right] \;
  {^f}{\bf J}_w
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param eJe : Robot jacobian \f${^e}{\bf J}_e\f$ that express the
  velocity of the end-effector in the robot end-effector frame.

  \sa get_fJw()
*/
void vpViper::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{
  vpMatrix V(6, 6);
  V = 0;
  // Compute the first and last block of V
  vpHomogeneousMatrix fMw;
  get_fMw(q, fMw);
  vpRotationMatrix fRw;
  fMw.extract(fRw);
  vpRotationMatrix wRf;
  wRf = fRw.inverse();
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      V[i][j] = V[i + 3][j + 3] = wRf[i][j];
    }
  }
  // Compute the second block of V
  vpHomogeneousMatrix wMe;
  get_wMe(wMe);
  vpHomogeneousMatrix eMw;
  eMw = wMe.inverse();
  vpTranslationVector etw;
  eMw.extract(etw);
  vpMatrix block2 = etw.skew() * wRf;
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      V[i][j + 3] = block2[i][j];
    }
  }
  // Compute eJe
  vpMatrix fJw;
  get_fJw(q, fJw);
  eJe = V * fJw;

  return;
}

/*!

  Get the robot jacobian \f${^f}{\bf J}_w\f$ which express the
  velocity of the origin of the wrist frame in the robot reference
  frame also called fix frame.

  \f[
  {^f}J_w = \left(\begin{array}{cccccc}
  J_{11} & J_{12} & J_{13} &   0     &   0    &   0    \\
  J_{21} & J_{22} & J_{23} &   0     &   0    &   0    \\
  0      & J_{32} & J_{33} &   0     &   0    &   0    \\
  0      &   -s1  &   -s1  &  c1s23 & J_{45} & J_{46} \\
  0      &    c1  &    c1  &  s1s23 & J_{55} & J_{56} \\
  1      &    0   &    0   &  c23    & s23s4 & J_{56} \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  J_{11} = -s1(-c23a3+s23d4+a1+a2c2) \\
  J_{21} = c1(-c23a3+s23d4+a1+a2c2) \\
  J_{12} = c1(s23a3+c23d4-a2s2) \\
  J_{22} = s1(s23a3+c23d4-a2s2) \\
  J_{32} = c23a3-s23d4-a2c2 \\
  J_{13} = c1(a3(s2c3+c2s3)+(-s2s3+c2c3)d4)\\
  J_{23} = s1(a3(s2c3+c2s3)+(-s2s3+c2c3)d4)\\
  J_{33} = -a3(s2s3-c2c3)-d4(s2c3+c2s3)\\
  J_{45} = -c23c1s4-s1c4\\
  J_{55} = c1c4-c23s1s4\\
  J_{46} = (c1c23c4-s1s4)s5+c1s23c5\\
  J_{56} = (s1c23c4+c1s4)s5+s1s23c5\\
  J_{66} = -s23c4s5+c23c5\\
  \end{array}
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param fJw : Robot jacobian \f${^f}{\bf J}_w\f$ that express the
  velocity of the point \e w (origin of the wrist frame) in the robot
  reference frame.

  \sa get_fJe(), get_eJe()
*/

void vpViper::get_fJw(const vpColVector &q, vpMatrix &fJw) const
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  double c3 = cos(q3);
  double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c23 = cos(q2 + q3);
  double s23 = sin(q2 + q3);

  vpColVector J1(6);
  vpColVector J2(6);
  vpColVector J3(6);
  vpColVector J4(6);
  vpColVector J5(6);
  vpColVector J6(6);

  // Jacobian when d6 is set to zero
  J1[0] = -s1 * (-c23 * a3 + s23 * d4 + a1 + a2 * c2);
  J1[1] = c1 * (-c23 * a3 + s23 * d4 + a1 + a2 * c2);
  J1[2] = 0;
  J1[3] = 0;
  J1[4] = 0;
  J1[5] = 1;

  J2[0] = c1 * (s23 * a3 + c23 * d4 - a2 * s2);
  J2[1] = s1 * (s23 * a3 + c23 * d4 - a2 * s2);
  J2[2] = c23 * a3 - s23 * d4 - a2 * c2;
  J2[3] = -s1;
  J2[4] = c1;
  J2[5] = 0;

  J3[0] = c1 * (a3 * (s2 * c3 + c2 * s3) + (-s2 * s3 + c2 * c3) * d4);
  J3[1] = s1 * (a3 * (s2 * c3 + c2 * s3) + (-s2 * s3 + c2 * c3) * d4);
  J3[2] = -a3 * (s2 * s3 - c2 * c3) - d4 * (s2 * c3 + c2 * s3);
  J3[3] = -s1;
  J3[4] = c1;
  J3[5] = 0;

  J4[0] = 0;
  J4[1] = 0;
  J4[2] = 0;
  J4[3] = c1 * s23;
  J4[4] = s1 * s23;
  J4[5] = c23;

  J5[0] = 0;
  J5[1] = 0;
  J5[2] = 0;
  J5[3] = -c23 * c1 * s4 - s1 * c4;
  J5[4] = c1 * c4 - c23 * s1 * s4;
  J5[5] = s23 * s4;

  J6[0] = 0;
  J6[1] = 0;
  J6[2] = 0;
  J6[3] = (c1 * c23 * c4 - s1 * s4) * s5 + c1 * s23 * c5;
  J6[4] = (s1 * c23 * c4 + c1 * s4) * s5 + s1 * s23 * c5;
  J6[5] = -s23 * c4 * s5 + c23 * c5;

  fJw.resize(6, 6);
  for (unsigned int i = 0; i < 6; i++) {
    fJw[i][0] = J1[i];
    fJw[i][1] = J2[i];
    fJw[i][2] = J3[i];
    fJw[i][3] = J4[i];
    fJw[i][4] = J5[i];
    fJw[i][5] = J6[i];
  }
  return;
}
/*!

  Get the robot jacobian \f${^f}{\bf J}_e\f$ which gives the velocity
  of the origin of the end-effector frame expressed in the robot
  reference frame also called fix frame.

  \f[
  {^f}{\bf J}_e = \left[\begin{array}{cc}
  I_{3\times3} & [{^f}{\bf R}_w \; {^e}{\bf t}_w]_\times \\
  0_{3\times3} & I_{3\times3}
  \end{array}
  \right]
  {^f}{\bf J}_w
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param fJe : Robot jacobian \f${^f}{\bf J}_e\f$ that express the
  velocity of the end-effector in the robot reference frame.

  \sa get_fJw
*/
void vpViper::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{
  vpMatrix V(6, 6);
  V = 0;
  // Set the first and last block to identity
  for (unsigned int i = 0; i < 6; i++)
    V[i][i] = 1;

  // Compute the second block of V
  vpHomogeneousMatrix fMw;
  get_fMw(q, fMw);
  vpRotationMatrix fRw;
  fMw.extract(fRw);
  vpHomogeneousMatrix wMe;
  get_wMe(wMe);
  vpHomogeneousMatrix eMw;
  eMw = wMe.inverse();
  vpTranslationVector etw;
  eMw.extract(etw);
  vpMatrix block2 = (fRw * etw).skew();
  // Set the second block
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      V[i][j + 3] = block2[i][j];

  // Compute fJe
  vpMatrix fJw;
  get_fJw(q, fJw);
  fJe = V * fJw;

  return;
}

/*!
  Get minimal joint values.

  \return A 6-dimension vector that contains the minimal joint values
  for the 6 dof. All the values are expressed in radians.

*/
vpColVector vpViper::getJointMin() const { return joint_min; }

/*!
  Get maximal joint values.

  \return A 6-dimension vector that contains the maximal joint values
  for the 6 dof. All the values are expressed in radians.

*/
vpColVector vpViper::getJointMax() const { return joint_max; }

/*!

  Return the coupling factor between join 5 and joint 6.

  This factor should be only useful when motor positions are
  considered.  Since the positions returned by the robot are joint
  positions which takes into account the coupling factor, it has not to
  be considered in the modelization of the robot.

*/
double vpViper::getCoupl56() const { return c56; }

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame (commonly a camera).

  \param eMc_ : Transformation between the end-effector frame
  and the tool frame.
*/
void vpViper::set_eMc(const vpHomogeneousMatrix &eMc_)
{
  this->eMc = eMc_;
  this->eMc.extract(etc);
  vpRotationMatrix R(this->eMc);
  this->erc.buildFrom(R);
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame (commonly a camera frame).

  \param etc_ : Translation between the end-effector frame
  and the tool frame.
  \param erc_ : Rotation between the end-effector frame and the tool
  frame using the Euler angles in radians with the XYZ convention.
*/
void vpViper::set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_)
{
  this->etc = etc_;
  this->erc = erc_;
  vpRotationMatrix eRc(erc);
  this->eMc.buildFrom(etc, eRc);
}

/*!

  Print on the output stream \e os the robot parameters (joint
  min/max, coupling factor between axis 5 and 6, hand-to-eye constant
  homogeneous matrix \f$^e{\bf M}_c \f$.

  \param os : Output stream.
  \param viper : Robot parameters.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpViper &viper)
{
  vpRotationMatrix eRc;
  viper.eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  // Convert joint limits in degrees
  vpColVector jmax = viper.joint_max;
  vpColVector jmin = viper.joint_min;
  jmax.rad2deg();
  jmin.rad2deg();

  os << "Joint Max (deg):" << std::endl
     << "\t" << jmax.t() << std::endl

     << "Joint Min (deg): " << std::endl
     << "\t" << jmin.t() << std::endl

     << "Coupling 5-6:" << std::endl
     << "\t" << viper.c56 << std::endl

     << "eMc: " << std::endl
     << "\tTranslation (m): " << viper.eMc[0][3] << " " << viper.eMc[1][3] << " " << viper.eMc[2][3] << "\t"
     << std::endl
     << "\tRotation Rxyz (rad) : " << rxyz[0] << " " << rxyz[1] << " " << rxyz[2] << "\t" << std::endl
     << "\tRotation Rxyz (deg) : " << vpMath::deg(rxyz[0]) << " " << vpMath::deg(rxyz[1]) << " " << vpMath::deg(rxyz[2])
     << "\t" << std::endl;

  return os;
}
