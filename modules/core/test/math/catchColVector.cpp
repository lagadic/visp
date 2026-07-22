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
 */

/*!
  \example catchColVector.cpp

  Test some vpColVector functionalities.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#include <limits>
#include <vector>
#include <cmath>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>

#if defined(VISP_BUILD_CATCH2)
#include <catch_amalgamated.hpp>
#else
#include <catch2/catch_all.hpp>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void checkData(const vpColVector &v, const std::vector<double> &bench)
{
  REQUIRE(v.size() == bench.size());
  for (unsigned int i = 0; i < v.size(); i++) {
    REQUIRE(v[i] == Catch::Approx(bench[i]).epsilon(std::numeric_limits<double>::epsilon()));
  }
}
} // namespace

TEST_CASE("vpColVector basic operations", "[vpColVector]")
{
  SECTION("Comparison operators")
  {
    vpColVector v1(7, 0.1), v2;
    REQUIRE_FALSE(v1 == v2);
    v2 = v1;
    REQUIRE(v1 == v2);
    v2[3] = 0.2;
    REQUIRE(v1 != v2);
  }

  SECTION("Resizing and normalization")
  {
    vpColVector v(4);
    v = 3;
    checkData(v, { 3, 3, 3, 3 });
    v.normalize();
    checkData(v, { 3./6, 3./6, 3./6, 3./6 });

    v.resize(5, 1, true);
    checkData(v, { 0, 0, 0, 0, 0 });
  }

  SECTION("Clear and resize")
  {
    vpColVector r(5, 1);
    r.clear();
    r.resize(5);
    r = 5;
    checkData(r, { 5, 5, 5, 5, 5 });
  }
}

TEST_CASE("vpColVector extraction and stacking", "[vpColVector]")
{
  vpColVector v(4);
  for (unsigned int i = 0; i < v.size(); i++) v[i] = static_cast<double>(i);

  vpColVector w;
  w.init(v, 0, 2);
  checkData(w, { 0, 1 });

  vpColVector r1;
  for (size_t i = 0; i < 4; i++) r1.stack(static_cast<double>(i));
  vpColVector r2 = r1.extract(1, 3);
  checkData(r2, { 1, 2, 3 });
}

TEST_CASE("vpColVector constructors and assignment", "[vpColVector]")
{
  vpMatrix M(4, 1);
  std::vector<double> bench(4);
  for (unsigned int i = 0; i < M.getRows(); i++) { M[i][0] = i; bench[i] = i; }

  checkData(vpColVector(M), bench);
  vpColVector v;
  v = M;
  checkData(v, bench);
  vpColVector w(M);
  checkData(w, bench);
  vpColVector z1(bench);
  checkData(z1, bench);
}

TEST_CASE("vpColVector math operations", "[vpColVector]")
{
  SECTION("Multiplication and stacking")
  {
    vpColVector v(3);
    v[0] = 1; v[1] = 2; v[2] = 3;
    std::vector<double> bench1 = { 3, 6, 9 };

    checkData(v * 3, bench1);

    vpColVector r1(3, 1);
    vpColVector r2 = -r1;
    checkData(r2, { -1, -1, -1 });

    r2.stack(-2);
    vpColVector r3 = vpColVector::stack(r1, r2);
    checkData(r3, { 1, 1, 1, -1, -1, -1, -2 });
  }

  SECTION("Addition and subtraction")
  {
    vpColVector r1(3, 2);
    vpColVector r2(3, 4);

    vpColVector r_add = r1 + r2;
    checkData(r_add, { 6, 6, 6 });
    r1 += r2;
    checkData(r1, { 6, 6, 6 });

    vpColVector r3(3, 2);
    vpColVector r4(3, 4);
    vpColVector r_sub = r3 - r4;
    checkData(r_sub, { -2, -2, -2 });
    r3 -= r4;
    checkData(r3, { -2, -2, -2 });
  }

  SECTION("Operator += and -=")
  {
    vpColVector r1(3, -2);
    vpColVector r2;
    r2 += r1;
    checkData(r2, { -2, -2, -2 });
    vpColVector r3;
    r3 -= r1;
    checkData(r3, { 2, 2, 2 });
    vpColVector r4;
    vpTranslationVector t(1, 2, 3);
    r4 += t;
    checkData(r4, { 1, 2, 3 });
    vpColVector r5;
    r5 -= t;
    checkData(r5, { -1, -2, -3 });
  }
}

TEST_CASE("vpColVector statistics", "[vpColVector]")
{
  vpColVector r(10);
  r = { 8.1472, 9.0579, 1.2699, 9.1338, 6.3236, 0.9754, 2.7850, 5.4688, 9.5751, 9.6489 };

  REQUIRE(vpColVector::mean(r) == Catch::Approx(6.2386).margin(0.001));
  REQUIRE(vpColVector::stdev(r) == Catch::Approx(3.2810).margin(0.001));
  REQUIRE(vpColVector::stdev(r, true) == Catch::Approx(3.4585).margin(0.001));
  REQUIRE(vpColVector::median(r) == Catch::Approx(7.2354).margin(0.001));

  SECTION("Median (odd)")
  {
    r.stack(1.5761);
    REQUIRE(vpColVector::median(r) == Catch::Approx(6.3236).margin(0.001));
  }
}

TEST_CASE("vpColVector insertion", "[vpColVector]")
{
  unsigned int nb = 10, size = 100;
  std::vector<vpColVector> vec(nb, vpColVector(size));
  for (auto &v : vec) for (unsigned int j = 0; j<size; ++j) v[j] = 1.0;

  vpColVector v_big(nb * size);
  for (unsigned int i = 0; i < nb; i++) v_big.insert(i * size, vec[i]);

  for (unsigned int i = 0; i < v_big.size(); i++) {
    REQUIRE(v_big[i] == Catch::Approx(1.0));
  }

  SECTION("Insert empty vectors")
  {
    vpColVector v1(2), v2, v3;
    REQUIRE_NOTHROW(v1.insert(0, v2));
    REQUIRE_NOTHROW(v3.insert(0, v2));
    REQUIRE(v1.size() == 2);
  }
}

TEST_CASE("vpColVector conversion and scalar comparison", "[vpColVector]")
{
  SECTION("Std vector conversion")
  {
    std::vector<double> std_v = { 0, 1, 2, 3, 4 };
    vpColVector v(std_v);
    checkData(v, std_v);

    std::vector<double> std_v2 = v.toStdVector();
    checkData(v, std_v2);
  }

  SECTION("Scalar comparison")
  {
    vpColVector v(3, 1.);
    REQUIRE_FALSE(v != 1.0);
    REQUIRE_FALSE(v == 0.0);
    v[1] = 0.0;
    REQUIRE_FALSE(v == 0.0);
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  std::cout << (numFailed ? "Test failed" : "Test succeed") << std::endl;
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
