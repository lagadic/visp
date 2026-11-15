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
  \example catchRowVector.cpp

  Test some vpRowVector functionalities.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#include <limits>
#include <vector>
#include <cmath>

#include <visp3/core/vpMath.h>
#include <visp3/core/vpRowVector.h>

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
void checkData(const vpRowVector &v, const std::vector<double> &bench)
{
  REQUIRE(v.size() == bench.size());
  for (unsigned int i = 0; i < v.size(); i++) {
    REQUIRE(v[i] == Catch::Approx(bench[i]).epsilon(std::numeric_limits<double>::epsilon()));
  }
}
} // namespace

TEST_CASE("vpRowVector basic operations", "[vpRowVector]")
{
  SECTION("Resizing and normalization")
  {
    vpRowVector v(4);
    v = 3;
    checkData(v, { 3, 3, 3, 3 });
    v.normalize();
    checkData(v, { 3./6, 3./6, 3./6, 3./6 });

    v.resize(1, 5, true);
    checkData(v, { 0, 0, 0, 0, 0 });
  }

  SECTION("Clear and resize")
  {
    vpRowVector r(5, 1);
    r.clear();
    r.resize(5);
    r = 5;
    checkData(r, { 5, 5, 5, 5, 5 });
  }
}

TEST_CASE("vpRowVector extraction and stacking", "[vpRowVector]")
{
  vpRowVector v(4);
  for (unsigned int i = 0; i < v.size(); i++) v[i] = static_cast<double>(i);

  vpRowVector w;
  w.init(v, 0, 2);
  checkData(w, { 0, 1 });

  vpRowVector r1;
  for (size_t i = 0; i < 4; i++) r1.stack(static_cast<double>(i));
  vpRowVector r2 = r1.extract(1, 3);
  checkData(r2, { 1, 2, 3 });
}

TEST_CASE("vpRowVector constructors and assignment", "[vpRowVector]")
{
  SECTION("Constructor with vpMatrix")
  {
    vpMatrix M(1, 4);
    std::vector<double> bench(4);
    for (unsigned int i = 0; i < M.getCols(); i++) { M[0][i] = i; bench[i] = i; }

    checkData(vpRowVector(M), bench);
    vpRowVector v;
    v = M;
    checkData(v, bench);
    vpRowVector w(M);
    checkData(w, bench);
    vpRowVector z1(bench);
    checkData(z1, bench);
  }

  SECTION("Constructor with std::vector<float>")
  {
    std::vector<float> bench_f = { 3.0f, 6.0f, 9.0f };
    std::vector<double> bench_d = { 3.0, 6.0, 9.0 };
    vpRowVector y1(bench_f);
    checkData(y1, bench_d);
    vpRowVector y2 = vpRowVector(bench_f);
    checkData(y2, bench_d);
  }
}

TEST_CASE("vpRowVector math operations", "[vpRowVector]")
{
  SECTION("Multiplication and stacking")
  {
    vpRowVector v(3);
    v[0] = 1; v[1] = 2; v[2] = 3;
    std::vector<double> bench1 = { 3, 6, 9 };

    checkData(v * 3, bench1);

    vpRowVector r1(3, 1);
    vpRowVector r2 = -r1;
    checkData(r2, { -1, -1, -1 });

    r2.stack(-2);
    vpRowVector r3 = vpRowVector::stack(r1, r2);
    checkData(r3, { 1, 1, 1, -1, -1, -1, -2 });
  }

  SECTION("Addition and subtraction")
  {
    vpRowVector r1(3, 2);
    vpRowVector r2(3, 4);

    vpRowVector r_add = r1 + r2;
    checkData(r_add, { 6, 6, 6 });
    r1 += r2;
    checkData(r1, { 6, 6, 6 });

    vpRowVector r3(3, 2);
    vpRowVector r4(3, 4);
    vpRowVector r_sub = r3 - r4;
    checkData(r_sub, { -2, -2, -2 });
    r3 -= r4;
    checkData(r3, { -2, -2, -2 });
  }

  SECTION("Operator += and -=")
  {
    vpRowVector r1(3, -2);
    vpRowVector r2;
    r2 += r1;
    checkData(r2, { -2, -2, -2 });
    vpRowVector r3;
    r3 -= r1;
    checkData(r3, { 2, 2, 2 });
  }
}

TEST_CASE("vpRowVector statistics", "[vpRowVector]")
{
  vpRowVector r(10);
  r = { 8.1472, 9.0579, 1.2699, 9.1338, 6.3236, 0.9754, 2.7850, 5.4688, 9.5751, 9.6489 };

  REQUIRE(vpRowVector::mean(r) == Catch::Approx(6.2386).margin(0.001));
  REQUIRE(vpRowVector::stdev(r) == Catch::Approx(3.2810).margin(0.001));
  REQUIRE(vpRowVector::stdev(r, true) == Catch::Approx(3.4585).margin(0.001));
  REQUIRE(vpRowVector::median(r) == Catch::Approx(7.2354).margin(0.001));

  SECTION("Median (odd)")
  {
    r.stack(1.5761);
    REQUIRE(vpRowVector::median(r) == Catch::Approx(6.3236).margin(0.001));
  }
}

TEST_CASE("vpRowVector conversions", "[vpRowVector]")
{
  std::vector<double> std_v = { 0, 1, 2, 3, 4 };
  vpRowVector v(std_v);
  checkData(v, std_v);

  std::vector<double> std_v2 = v.toStdVector();
  checkData(v, std_v2);
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
