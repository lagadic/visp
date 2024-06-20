/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test theta.u and quaternion multiplication.
 */

/*!
  \example testRotation.cpp

  Test theta.u and quaternion multiplication.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpUniRand.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
vpThetaUVector generateThetaU(vpUniRand &rng)
{
  return vpThetaUVector(
      vpMath::rad(rng.uniform(-180.0, 180.0)) *
      vpColVector({ rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0) }).normalize());
}

vpQuaternionVector generateQuat(vpUniRand &rng)
{
  const double angle = vpMath::rad(rng.uniform(-180.0, 180.0));
  const double ctheta = std::cos(angle);
  const double stheta = std::sin(angle);
  const double ax = rng.uniform(-1.0, 1.0);
  const double ay = rng.uniform(-1.0, 1.0);
  const double az = rng.uniform(-1.0, 1.0);
  return vpQuaternionVector(stheta * ax, stheta * ay, stheta * az, ctheta);
}
} // namespace


bool test(const std::string &s, const vpArray2D<double> &v, const std::vector<double> &bench)
{
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v << "]" << std::endl;
  if (bench.size() != v.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < v.size(); i++) {
    if (std::fabs(v.data[i] - bench[i]) > std::fabs(v.data[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

bool test(const std::string &s, const vpArray2D<double> &v, const vpColVector &bench)
{
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v << "]" << std::endl;
  if (bench.size() != v.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < v.size(); i++) {
    if (std::fabs(v.data[i] - bench[i]) > std::fabs(v.data[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

bool test(const std::string &s, const vpRotationVector &v, const double &bench)
{
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v << "]" << std::endl;
  for (unsigned int i = 0; i < v.size(); i++) {
    if (std::fabs(v[i] - bench) > std::fabs(v[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

bool test_matrix_equal(const vpHomogeneousMatrix &M1, const vpHomogeneousMatrix &M2, double epsilon = 1e-10)
{
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      if (!vpMath::equal(M1[i][j], M2[i][j], epsilon)) {
        return false;
      }
    }
  }
  return true;
}

TEST_CASE("Common rotation operations", "[rotation]")
{
  SECTION("Theta u initialization")
  {
    vpThetaUVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    std::vector<double> bench1(3, vpMath::rad(10));
    vpColVector bench3(3, vpMath::rad(10));
    CHECK(test("r1", r1, bench1));

    bench1.clear();
    bench1 = r1.toStdVector();
    CHECK(test("r1", r1, bench1));

    r1.build(bench3);
    CHECK(test("r1", r1, bench3));

    vpThetaUVector r2 = r1;
    CHECK(test("r2", r2, bench1));
    CHECK(r2.data != r1.data);

    CHECK(test("r2", r2, vpMath::rad(10)));

    vpThetaUVector r3;
    r3 = vpMath::rad(10);
    CHECK(test("r3", r3, bench1));

    for (unsigned int i = 0; i < r3.size(); i++) {
      CHECK(std::fabs(r3[i] - bench1[i]) < std::fabs(r3[i]) * std::numeric_limits<double>::epsilon());
    }

    const vpColVector r4 = 0.5 * r1;
    std::vector<double> bench2(3, vpMath::rad(5));
    CHECK(test("r4", r4, bench2));

    const vpThetaUVector r5(r3);
    CHECK(test("r5", r5, bench1));
  }
  SECTION("Rxyz initialization")
  {
    vpRxyzVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    std::vector<double> bench1(3, vpMath::rad(10));
    vpColVector bench3(3, vpMath::rad(10));
    CHECK(test("r1", r1, bench1));

    bench1.clear();
    bench1 = r1.toStdVector();
    CHECK(test("r1", r1, bench1));

    r1.build(bench3);
    CHECK(test("r1", r1, bench3));

    vpRxyzVector r2 = r1;
    CHECK(test("r2", r2, bench1));

    CHECK(test("r2", r2, vpMath::rad(10)));

    vpRxyzVector r3;
    r3 = vpMath::rad(10);
    CHECK(test("r3", r3, bench1));

    for (unsigned int i = 0; i < r3.size(); i++) {
      CHECK(std::fabs(r3[i] - bench1[i]) <= std::fabs(r3[i]) * std::numeric_limits<double>::epsilon());
    }

    vpColVector r4 = 0.5 * r1;
    std::vector<double> bench2(3, vpMath::rad(5));
    CHECK(test("r4", r4, bench2));

    vpRxyzVector r5(r3);
    CHECK(test("r5", r5, bench1));
  }
  SECTION("rzyx initialization")
  {
    vpRzyxVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    std::vector<double> bench1(3, vpMath::rad(10));
    vpColVector bench3(3, vpMath::rad(10));
    CHECK(test("r1", r1, bench1));

    bench1.clear();
    bench1 = r1.toStdVector();
    CHECK(test("r1", r1, bench1));

    r1.build(bench3);
    CHECK(test("r1", r1, bench3));

    vpRzyxVector r2 = r1;
    CHECK(test("r2", r2, bench1));

    CHECK(test("r2", r2, vpMath::rad(10)));

    vpRzyxVector r3;
    r3 = vpMath::rad(10);
    CHECK(test("r3", r3, bench1));

    for (unsigned int i = 0; i < r3.size(); i++) {
      CHECK(std::fabs(r3[i] - bench1[i]) <= std::fabs(r3[i]) * std::numeric_limits<double>::epsilon());
    }

    vpColVector r4 = 0.5 * r1;
    std::vector<double> bench2(3, vpMath::rad(5));
    CHECK(test("r4", r4, bench2));

    vpRzyxVector r5(r3);
    CHECK(test("r5", r5, bench1));
  }
  SECTION("rzyz initialiation")
  {
    vpRzyzVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    std::vector<double> bench1(3, vpMath::rad(10));
    vpColVector bench3(3, vpMath::rad(10));
    CHECK(test("r1", r1, bench1));

    bench1.clear();
    bench1 = r1.toStdVector();
    CHECK(test("r1", r1, bench1));

    r1.build(bench3);
    CHECK(test("r1", r1, bench3));

    vpRzyzVector r2 = r1;
    CHECK(test("r2", r2, bench1));

    CHECK(test("r2", r2, vpMath::rad(10)));

    vpRzyzVector r3;
    r3 = vpMath::rad(10);
    CHECK(test("r3", r3, bench1));

    for (unsigned int i = 0; i < r3.size(); i++) {
      CHECK(std::fabs(r3[i] - bench1[i]) <= std::fabs(r3[i]) * std::numeric_limits<double>::epsilon());
    }

    vpColVector r4 = 0.5 * r1;
    std::vector<double> bench2(3, vpMath::rad(5));
    CHECK(test("r4", r4, bench2));

    vpRzyzVector r5(r3);
    CHECK(test("r5", r5, bench1));
  }
  SECTION("Test quaternion initialization", "[quaternion]")
  {
    vpQuaternionVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    std::vector<double> bench1(4, vpMath::rad(10));
    vpColVector bench3(4, vpMath::rad(10));
    CHECK(test("r1", r1, bench1));

    bench1.clear();
    bench1 = r1.toStdVector();
    CHECK(test("r1", r1, bench1));

    r1.build(bench3);
    CHECK(test("r1", r1, bench3));

    vpQuaternionVector r2 = r1;
    CHECK(test("r2", r2, bench1));

    CHECK(test("r2", r2, vpMath::rad(10)));

    vpQuaternionVector r3;
    r3.set(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
    CHECK(test("r3", r3, bench1));

    for (unsigned int i = 0; i < r3.size(); i++) {
      CHECK(std::fabs(r3[i] - bench1[i]) <= std::fabs(r3[i]) * std::numeric_limits<double>::epsilon());
    }

    vpColVector r4 = 0.5 * r1;
    std::vector<double> bench2(4, vpMath::rad(5));
    CHECK(test("r4", r4, bench2));

    vpQuaternionVector r5(r3);
    CHECK(test("r5", r5, bench1));
  }
  SECTION("Conversions")
  {
    vpRotationMatrix R;
    for (int i = -10; i < 10; i++) {
      for (int j = -10; j < 10; j++) {
        vpThetaUVector tu(vpMath::rad(90 + i), vpMath::rad(170 + j), vpMath::rad(45));
        tu.build(vpRotationMatrix(tu)); // put some coherence into rotation convention

        std::cout << "Initialization " << std::endl;

        double theta;
        vpColVector u;
        tu.extract(theta, u);

        std::cout << "theta=" << vpMath::deg(theta) << std::endl;
        std::cout << "u=" << u << std::endl;

        std::cout << "From vpThetaUVector to vpRotationMatrix " << std::endl;
        R.build(tu);

        std::cout << "Matrix R";
        CHECK(R.isARotationMatrix());

        std::cout << R << std::endl;

        std::cout << "From vpRotationMatrix to vpQuaternionVector " << std::endl;
        vpQuaternionVector q(R);
        CHECK(q.magnitude() == Approx(1.0).margin(1e-4));
        std::cout << q << std::endl;

        R.build(q);
        CHECK(R.isARotationMatrix());
        std::cout << "From vpQuaternionVector to vpRotationMatrix  " << std::endl;

        std::cout << "From vpRotationMatrix to vpRxyzVector " << std::endl;
        vpRxyzVector RxyzbuildR(R);
        std::cout << RxyzbuildR << std::endl;

        std::cout << "From vpRxyzVector to vpThetaUVector " << std::endl;
        std::cout << "  use From vpRxyzVector to vpRotationMatrix " << std::endl;
        std::cout << "  use From vpRotationMatrix to vpThetaUVector " << std::endl;

        vpThetaUVector tubuildEu;
        tubuildEu.build(R);

        std::cout << std::endl;
        std::cout << "result : should equivalent to the first one " << std::endl;

        double theta2;
        vpColVector u2;

        tubuildEu.extract(theta2, u2);
        std::cout << "theta=" << vpMath::deg(theta2) << std::endl;
        std::cout << "u=" << u2 << std::endl;

        CHECK(vpMath::abs(theta2 - theta) < std::numeric_limits<double>::epsilon() * 1e10);
        CHECK(vpMath::abs(u[0] - u2[0]) < std::numeric_limits<double>::epsilon() * 1e10);
        CHECK(vpMath::abs(u[1] - u2[1]) < std::numeric_limits<double>::epsilon() * 1e10);
        CHECK(vpMath::abs(u[2] - u2[2]) < std::numeric_limits<double>::epsilon() * 1e10);
      }
    }
    SECTION("Conversion from and to rzyz vector")
    {
      vpRzyzVector rzyz(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45));
      std::cout << "Initialization vpRzyzVector " << std::endl;
      std::cout << rzyz << std::endl;
      std::cout << "From vpRzyzVector to vpRotationMatrix  " << std::endl;
      R.build(rzyz);
      CHECK(R.isARotationMatrix());
      std::cout << "From vpRotationMatrix to vpRzyzVector " << std::endl;
      vpRzyzVector rzyz_final;
      rzyz_final.build(R);
      CHECK(test("rzyz", rzyz_final, vpColVector(rzyz)));
      std::cout << rzyz_final << std::endl;
    }
    SECTION("Conversion from and to rzyx vector")
    {
      vpRzyxVector rzyx(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45));
      std::cout << "Initialization vpRzyxVector " << std::endl;
      std::cout << rzyx << std::endl;
      std::cout << "From vpRzyxVector to vpRotationMatrix  " << std::endl;
      R.build(rzyx);
      CHECK(R.isARotationMatrix());
      std::cout << R << std::endl;
      std::cout << "From vpRotationMatrix to vpRzyxVector " << std::endl;
      vpRzyxVector rzyx_final;
      rzyx_final.build(R);
      bool ret = test("rzyx", rzyx_final, vpColVector(rzyx));
      if (ret == false) {
        // Euler angle representation is not unique
        std::cout << "Rzyx vector differ. Test rotation matrix..." << std::endl;
        vpRotationMatrix RR(rzyx_final);
        if (R == RR) {
          std::cout << "Rzyx vector differ but rotation matrix is valid" << std::endl;
          ret = true;
        }
      }
      CHECK(ret);
      std::cout << rzyx_final << std::endl;
    }
  }
  SECTION("Rotation matrix extraction from homogeneous matrix and multiplication")
  {
    // Test rotation_matrix * homogeneous_matrix
    vpHomogeneousMatrix  _1_M_2_truth;
    _1_M_2_truth[0][0] = 0.9835;
    _1_M_2_truth[0][1] = -0.0581;
    _1_M_2_truth[0][2] = 0.1716;
    _1_M_2_truth[0][3] = 0;
    _1_M_2_truth[1][0] = -0.0489;
    _1_M_2_truth[1][1] = -0.9972;
    _1_M_2_truth[1][2] = -0.0571;
    _1_M_2_truth[1][3] = 0;
    _1_M_2_truth[2][0] = 0.1744;
    _1_M_2_truth[2][1] = 0.0478;
    _1_M_2_truth[2][2] = -0.9835;
    _1_M_2_truth[2][3] = 0;
    vpHomogeneousMatrix _2_M_3_;
    _2_M_3_[0][0] = 0.9835;
    _2_M_3_[0][1] = -0.0581;
    _2_M_3_[0][2] = 0.1716;
    _2_M_3_[0][3] = 0.0072;
    _2_M_3_[1][0] = -0.0489;
    _2_M_3_[1][1] = -0.9972;
    _2_M_3_[1][2] = -0.0571;
    _2_M_3_[1][3] = 0.0352;
    _2_M_3_[2][0] = 0.1744;
    _2_M_3_[2][1] = 0.0478;
    _2_M_3_[2][2] = -0.9835;
    _2_M_3_[2][3] = 0.9470;

    vpRotationMatrix _1_R_2_ = _1_M_2_truth.getRotationMatrix();
    vpHomogeneousMatrix _1_M_3_(_1_R_2_* _2_M_3_);
    vpHomogeneousMatrix _1_M_3_truth(_1_M_2_truth * _2_M_3_);
    CHECK(test_matrix_equal(_1_M_3_, _1_M_3_truth));
  }
}

TEST_CASE("Theta u multiplication", "[theta.u]")
{
  const int nTrials = 100;
  const uint64_t seed = 0x123456789;
  vpUniRand rng(seed);
  for (int iter = 0; iter < nTrials; iter++) {
    const vpThetaUVector tu0 = generateThetaU(rng);
    const vpThetaUVector tu1 = generateThetaU(rng);

    const vpRotationMatrix c1Rc2(tu0);
    const vpRotationMatrix c2Rc3(tu1);
    const vpRotationMatrix c1Rc3_ref = c1Rc2 * c2Rc3;
    const vpThetaUVector c1_tu_c3 = tu0 * tu1;
    // two rotation vectors can represent the same rotation,
    // that is why we compare the rotation matrices
    const vpRotationMatrix c1Rc3(c1_tu_c3);

    const double tolerance = 1e-9;
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        CHECK(c1Rc3_ref[i][j] == Approx(c1Rc3[i][j]).epsilon(0).margin(tolerance));
      }
    }
  }
}

TEST_CASE("Quaternion multiplication", "[quaternion]")
{
  const int nTrials = 100;
  const uint64_t seed = 0x123456789;
  vpUniRand rng(seed);
  for (int iter = 0; iter < nTrials; iter++) {
    const vpQuaternionVector q0 = generateQuat(rng);
    const vpQuaternionVector q1 = generateQuat(rng);

    const vpRotationMatrix c1Rc2(q0);
    const vpRotationMatrix c2Rc3(q1);
    const vpRotationMatrix c1Rc3_ref = c1Rc2 * c2Rc3;

    const vpQuaternionVector c1_q_c3 = q0 * q1;
    // two quaternions of opposite sign can represent the same rotation,
    // that is why we compare the rotation matrices
    const vpRotationMatrix c1Rc3(c1_q_c3);

    const double tolerance = 1e-9;
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        CHECK(c1Rc3_ref[i][j] == Approx(c1Rc3[i][j]).epsilon(0).margin(tolerance));
      }
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
