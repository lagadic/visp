/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Test the joint limit avoidance secondary task of vpServo.
 */

/*!
  \example catchServoJointLimitAvoidance.cpp

  Test vpServo::secondaryTaskJointLimitAvoidance(): no avoidance in the safe zone, continuity of the avoidance
  velocity at the inner and outer thresholds, and full correction beyond the outer threshold.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#if defined(VISP_BUILD_CATCH2)
#include <catch_amalgamated.hpp>
#else // Since v3.1.1
#include <catch2/catch_all.hpp>
#endif

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

#include <cmath>
#include <limits>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
const unsigned int nb_joints = 7;
const double q_min = -1.;
const double q_max = 1.;
const double rho = 0.1;
const double rho1 = 0.5;
const double lambda_tune = 0.15;
// Thresholds of the upper limit: q_l0 = q_max - rho (q_max - q_min), q_l1 = q_l0 + rho rho1 (q_max - q_min)
const double q_l0_max = q_max - rho * (q_max - q_min);
const double q_l1_max = q_l0_max + rho * rho1 * (q_max - q_min);
// Thresholds of the lower limit
const double q_l0_min = q_min + rho * (q_max - q_min);
const double q_l1_min = q_l0_min - rho * rho1 * (q_max - q_min);

/*!
  Task of dimension 3 (translation) for a 7 joints robot, so that the projection operator is not null.
*/
class JointLimitAvoidanceFixture
{
public:
  JointLimitAvoidanceFixture()
    : m_qmin(nb_joints, q_min), m_qmax(nb_joints, q_max), m_q(nb_joints, 0.)
  {
    vpMatrix eJe(6, nb_joints);
    for (unsigned int i = 0; i < 6; ++i) {
      for (unsigned int j = 0; j < nb_joints; ++j) {
        eJe[i][j] = std::cos(1. + i + 2. * j); // Arbitrary full rank Jacobian
      }
    }
    m_task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    m_task.setInteractionMatrixType(vpServo::CURRENT);
    m_task.setLambda(0.5);
    m_task.set_cVe(vpVelocityTwistMatrix());
    m_task.set_eJe(eJe);

    m_s.buildFrom(vpHomogeneousMatrix(0.1, -0.2, 0.3, 0.1, 0.2, -0.1));
    m_s.setFeatureTranslationType(vpFeatureTranslation::cdMc);
    m_task.addFeature(m_s);
    m_dq = m_task.computeControlLaw();

    // Projection operator, that does not depend on the joint positions
    m_task.secondaryTaskJointLimitAvoidance(m_q, m_dq, m_qmin, m_qmax, rho, rho1, lambda_tune);
    m_P_00 = std::fabs(m_task.getLargeP()[0][0]);
  }

  /*!
    Avoidance velocity when only joint 0 is out of the safe zone, at position q0, with a primary task velocity of joint
    0 giving the adaptive gain b = |dq[0]| / |(P g_0)[0]|.
  */
  vpColVector avoidance(double q0, double b)
  {
    vpColVector q = m_q;
    q[0] = q0;
    vpColVector dq = m_dq;
    dq[0] = b * m_P_00;
    return m_task.secondaryTaskJointLimitAvoidance(q, dq, m_qmin, m_qmax, rho, rho1, lambda_tune);
  }

  double primaryVelocity(double b) const { return b * m_P_00; }

  double P_00() const { return m_P_00; }

  /*!
    Remove the limits of the last joints, as recommended for unmeasured DoFs like the ones of a mobile base.
  */
  void setUnlimitedJoints(unsigned int nb_unlimited_joints)
  {
    for (unsigned int j = nb_joints - nb_unlimited_joints; j < nb_joints; ++j) {
      m_qmin[j] = std::numeric_limits<double>::lowest();
      m_qmax[j] = std::numeric_limits<double>::max();
    }
  }

private:
  vpServo m_task;
  vpFeatureTranslation m_s;
  vpColVector m_qmin, m_qmax, m_q, m_dq;
  double m_P_00 = 0.;
};
} // namespace

TEST_CASE_METHOD(JointLimitAvoidanceFixture, "Joint limit avoidance", "[vpServo][joint_limit_avoidance]")
{
  REQUIRE(P_00() > 1e-6);

  const double eps = 1e-9;

  for (double b : { 0.5, 1., 3. }) {
    DYNAMIC_SECTION("b = " << b)
    {
      SECTION("No avoidance in the safe zone")
      {
        REQUIRE(avoidance(0., b).frobeniusNorm() < std::numeric_limits<double>::epsilon());
        REQUIRE(avoidance(q_l0_max - eps, b).frobeniusNorm() < std::numeric_limits<double>::epsilon());
        REQUIRE(avoidance(q_l0_min + eps, b).frobeniusNorm() < std::numeric_limits<double>::epsilon());
      }

      SECTION("Continuity at q_l0")
      {
        CHECK(avoidance(q_l0_max + eps, b).frobeniusNorm() < 1e-6);
        CHECK(avoidance(q_l0_min - eps, b).frobeniusNorm() < 1e-6);
      }

      SECTION("Continuity at q_l1")
      {
        // Without continuity, the avoidance velocity jumps by a factor b at q_l1 when b > 1
        for (const auto &q_l1 : { q_l1_max, q_l1_min }) {
          const vpColVector q2_inside = avoidance(q_l1 + (q_l1 > 0 ? -eps : eps), b);
          const vpColVector q2_outside = avoidance(q_l1 + (q_l1 > 0 ? eps : -eps), b);
          INFO("q_l1 = " << q_l1 << " inside: " << q2_inside.t() << " outside: " << q2_outside.t());
          CHECK((q2_outside - q2_inside).frobeniusNorm() < 1e-6 * q2_outside.frobeniusNorm());
        }
      }

      SECTION("Avoidance velocity of the joint increases in the transition zone")
      {
        double previous = 0.;
        for (unsigned int k = 0; k <= 20; ++k) {
          const double q0 = q_l0_max + (q_l1_max - q_l0_max) * k / 20.;
          const double current = std::fabs(avoidance(q0, b)[0]);
          CHECK(current >= previous);
          previous = current;
        }
      }

      SECTION("Full correction beyond q_l1, opposed to the closest limit")
      {
        // |q2[i]| = (1 + lambda_tune) |dq[i]|, so that the avoidance dominates the primary task
        const double expected = (1. + lambda_tune) * primaryVelocity(b);
        const vpColVector q2_upper = avoidance(0.5 * (q_l1_max + q_max), b);
        const vpColVector q2_lower = avoidance(0.5 * (q_l1_min + q_min), b);
        CHECK(q2_upper[0] == Catch::Approx(-expected));
        CHECK(q2_lower[0] == Catch::Approx(expected));
      }
    }
  }
}

TEST_CASE_METHOD(JointLimitAvoidanceFixture, "Joint limit avoidance with unlimited joints",
                 "[vpServo][joint_limit_avoidance]")
{
  // Joints without limits (lowest() / max(), like the DoFs of a mobile base) must always be in the safe zone, although
  // qmax - qmin overflows
  setUnlimitedJoints(3);

  SECTION("No avoidance when the limited joints are in the safe zone")
  {
    for (double b : { 0.5, 1., 3. }) {
      INFO("b = " << b);
      CHECK(avoidance(0., b).frobeniusNorm() < std::numeric_limits<double>::epsilon());
    }
  }

  SECTION("Avoidance of a limited joint still continuous at q_l1")
  {
    const double eps = 1e-9;
    const vpColVector q2_inside = avoidance(q_l1_max - eps, 3.);
    const vpColVector q2_outside = avoidance(q_l1_max + eps, 3.);
    CHECK(q2_outside.frobeniusNorm() > 0.);
    CHECK((q2_outside - q2_inside).frobeniusNorm() < 1e-6 * q2_outside.frobeniusNorm());
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

#include <cstdlib>

int main() { return EXIT_SUCCESS; }

#endif
