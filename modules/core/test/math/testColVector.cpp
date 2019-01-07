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
 * Test some vpColVector functionalities.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \example testColVector.cpp

  Test some vpColVector functionalities.
*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMath.h>

namespace
{
bool test(const std::string &s, const vpColVector &v, const std::vector<double> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v.t() << "]^T" << std::endl;
  if (bench.size() != v.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < v.size(); i++) {
    if (std::fabs(v[i] - bench[i]) > std::fabs(v[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

double computeRegularSum(const vpColVector &v)
{
  double sum = 0.0;

  for (unsigned int i = 0; i < v.getRows(); i++) {
    sum += v[i];
  }

  return sum;
}

double computeRegularSumSquare(const vpColVector &v)
{
  double sum_square = 0.0;

  for (unsigned int i = 0; i < v.getRows(); i++) {
    sum_square += v[i] * v[i];
  }

  return sum_square;
}

double computeRegularStdev(const vpColVector &v)
{
  double mean_value = computeRegularSum(v) / v.getRows();
  double sum_squared_diff = 0.0;

  for (unsigned int i = 0; i < v.size(); i++) {
    sum_squared_diff += (v[i] - mean_value) * (v[i] - mean_value);
  }

  double divisor = (double)v.size();

  return std::sqrt(sum_squared_diff / divisor);
}

double getRandomValues(const double min, const double max)
{
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}
}

int main()
{
  {
    vpColVector v1(7, 0.1), v2;
    if (v1 == v2) {
      std::cerr << "Issue with vpColVector comparison operator." << std::endl;
      return EXIT_FAILURE;
    }
    v2 = v1;
    if (v1 != v2) {
      std::cerr << "Issue with vpColVector comparison operator." << std::endl;
      return EXIT_FAILURE;
    }
    v2[3] = 0.2;
    if (v1 == v2) {
      std::cerr << "Issue with vpColVector comparison operator." << std::endl;
      return EXIT_FAILURE;
    }
  }
  {
    vpColVector v;

    v.resize(4);
    v = 3;
    std::vector<double> bench1(4, 3);
    if (test("v", v, bench1) == false)
      return EXIT_FAILURE;
    std::vector<double> bench2(4, 3. / 6);
    v.normalize();
    if (test("v", v, bench2) == false)
      return EXIT_FAILURE;

    v.resize(5, 1, true);
    std::vector<double> bench3(5, 0);
    if (test("v", v, bench3) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector v(4);
    std::vector<double> bench1(4);
    for (unsigned int i = 0; i < v.size(); i++) {
      v[i] = (double)i;
      bench1[i] = (double)i;
    }
    if (test("v", v, bench1) == false)
      return EXIT_FAILURE;

    vpColVector w;
    w.init(v, 0, 2);
    std::vector<double> bench2;
    bench2.push_back(0);
    bench2.push_back(1);
    if (test("w", w, bench2) == false)
      return EXIT_FAILURE;

    std::vector<double> bench3;
    bench3.push_back(1);
    bench3.push_back(2);
    bench3.push_back(3);

    vpColVector r1;
    for (size_t i = 0; i < 4; i++)
      r1.stack((double)i);

    vpColVector r2 = r1.extract(1, 3);
    if (test("r2", r2, bench3) == false)
      return EXIT_FAILURE;
  }

  {
    vpMatrix M(4, 1);
    std::vector<double> bench(4);
    for (unsigned int i = 0; i < M.getRows(); i++) {
      M[i][0] = i;
      bench[i] = i;
    }
    if (test("M", M, bench) == false)
      return EXIT_FAILURE;
    vpColVector v;
    v = M;
    if (test("v", v, bench) == false)
      return EXIT_FAILURE;
    vpColVector w(M);
    if (test("w", w, bench) == false)
      return EXIT_FAILURE;
    vpColVector z1(bench);
    if (test("z1", z1, bench) == false)
      return EXIT_FAILURE;
    vpColVector z2 = bench;
    if (test("z2", z2, bench) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector v(3);
    v[0] = 1;
    v[1] = 2;
    v[2] = 3;
    std::vector<double> bench1;
    bench1.push_back(3);
    bench1.push_back(6);
    bench1.push_back(9);

    vpColVector w = v * 3;
    // v is unchanged
    // w is now equal to : [3 6 9]
    if (test("w", w, bench1) == false)
      return EXIT_FAILURE;

    vpColVector x(w);
    if (test("x", x, bench1) == false)
      return EXIT_FAILURE;

    std::vector<float> bench2;
    bench2.push_back(3);
    bench2.push_back(6);
    bench2.push_back(9);
    vpColVector y1(bench2);
    if (test("y1", y1, bench1) == false)
      return EXIT_FAILURE;
    vpColVector y2 = bench2;
    if (test("y2", y2, bench1) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector r1(3, 1);
    vpColVector r2 = -r1;
    std::vector<double> bench(3, -1);
    // v contains [-1 -1 -1]
    if (test("r2", r2, bench) == false)
      return EXIT_FAILURE;
    r2.stack(-2);
    bench.push_back(-2);
    if (test("r2", r2, bench) == false)
      return EXIT_FAILURE;
    vpColVector r3 = vpColVector::stack(r1, r2);
    std::vector<double> bench3(7, 1);
    bench3[3] = bench3[4] = bench3[5] = -1;
    bench3[6] = -2;
    if (test("r3", r3, bench3) == false)
      return EXIT_FAILURE;

    r1.stack(r2);
    if (test("r1", r1, bench3) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector r1(3, 2);
    vpColVector r2(3, 4);
    std::cout << "test r1: " << r1 << std::endl;
    std::cout << "test r2: " << r2 << std::endl;
    vpColVector r = r1 + r2;
    std::cout << "test r1+r2: " << r1 + r2 << std::endl;
    std::cout << "test r: " << r << std::endl;
    std::vector<double> bench(3, 6);
    if (test("r", r, bench) == false)
      return EXIT_FAILURE;
    r1 += r2;
    if (test("r1", r1, bench) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector r1(3, 2);
    vpColVector r2(3, 4);
    vpColVector r = r1 - r2;
    std::vector<double> bench(3, -2);
    if (test("r", r, bench) == false)
      return EXIT_FAILURE;
    r1 -= r2;
    if (test("r1", r1, bench) == false)
      return EXIT_FAILURE;
  }

  {
    vpColVector r(5, 1);
    r.clear();
    r.resize(5);
    r = 5;
    std::vector<double> bench(5, 5);
    if (test("r", r, bench) == false)
      return EXIT_FAILURE;
  }

  {
    // Test mean, median and standard deviation against Matlab with rng(0) and
    // rand(10,1)*10
    vpColVector r(10);
    r[0] = 8.1472;
    r[1] = 9.0579;
    r[2] = 1.2699;
    r[3] = 9.1338;
    r[4] = 6.3236;
    r[5] = 0.9754;
    r[6] = 2.7850;
    r[7] = 5.4688;
    r[8] = 9.5751;
    r[9] = 9.6489;

    std::cout << "** Test mean" << std::endl;
    double res = vpColVector::mean(r);
    if (!vpMath::equal(res, 6.2386, 0.001)) {
      std::cout << "Test fails: bad mean " << res << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "** Test stdev" << std::endl;
    res = vpColVector::stdev(r);
    if (!vpMath::equal(res, 3.2810, 0.001)) {
      std::cout << "Test fails: bad stdev " << res << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "** Test stdev(bessel)" << std::endl;
    res = vpColVector::stdev(r, true);
    if (!vpMath::equal(res, 3.4585, 0.001)) {
      std::cout << "Test fails: bad stdev(bessel) " << res << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "** Test median" << std::endl;
    res = vpColVector::median(r);
    if (!vpMath::equal(res, 7.2354, 0.001)) {
      std::cout << "Test fails: bad median " << res << std::endl;
      return EXIT_FAILURE;
    }

    // Test median with odd number of elements
    std::cout << "** Test median (odd)" << std::endl;
    r.stack(1.5761);
    res = vpColVector::median(r);
    if (!vpMath::equal(res, 6.3236, 0.001)) {
      std::cout << "Test fails: bad median (odd) " << res << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "r: [" << r << "]^T" << std::endl;
    r.print(std::cout, 8, "r");
  }

  // Test sum, sumSquare, stdev
  {
    srand(0);
    vpGaussRand noise(10.0, 0.0);

    int nbIterations = 1000;
    unsigned int size = 117;

    vpColVector v(size);
    for (unsigned int cpt = 0; cpt < v.getRows(); cpt++) {
      v[cpt] = rand() % 1000 + noise();
    }

    std::cout << "\nv.getRows()=" << v.getRows() << std::endl;

    double regular_sum = 0.0;
    double t_regular = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      regular_sum += computeRegularSum(v);
    }
    t_regular = vpTime::measureTimeMs() - t_regular;

    double sse_sum = 0.0;
    double t_sse = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      sse_sum += v.sum();
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "\nregular_sum=" << regular_sum << " ; sse_sum=" << sse_sum << std::endl;
    std::cout << "t_regular=" << t_regular << " ms ; t_sse=" << t_sse << " ms" << std::endl;
    std::cout << "Speed-up: " << (t_regular / t_sse) << "X" << std::endl;

    if (!vpMath::equal(regular_sum, sse_sum, std::numeric_limits<double>::epsilon())) {
      std::cerr << "Problem when computing v.sum()!" << std::endl;
      return EXIT_FAILURE;
    }

    double regular_sumSquare = 0.0;
    t_regular = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      regular_sumSquare += computeRegularSumSquare(v);
    }
    t_regular = vpTime::measureTimeMs() - t_regular;

    double sse_sumSquare = 0.0;
    t_sse = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      sse_sumSquare += v.sumSquare();
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "\nregular_sumSquare=" << regular_sumSquare << " ; sse_sumSquare=" << sse_sumSquare << std::endl;
    std::cout << "t_regular=" << t_regular << " ms ; t_sse=" << t_sse << " ms" << std::endl;
    std::cout << "Speed-up: " << (t_regular / t_sse) << "X" << std::endl;

    if (!vpMath::equal(regular_sumSquare, sse_sumSquare, std::numeric_limits<double>::epsilon())) {
      std::cerr << "Problem when computing v.sumSquare()!" << std::endl;
      return EXIT_FAILURE;
    }

    double regular_stdev = 0.0;
    t_regular = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      regular_stdev += computeRegularStdev(v);
    }
    t_regular = vpTime::measureTimeMs() - t_regular;

    double sse_stdev = 0.0;
    t_sse = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      sse_stdev += vpColVector::stdev(v, false);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "\nregular_stdev=" << regular_stdev << " ; sse_stdev=" << sse_stdev << std::endl;
    std::cout << "t_regular=" << t_regular << " ms ; t_sse=" << t_sse << " ms" << std::endl;
    std::cout << "Speed-up: " << (t_regular / t_sse) << "X" << std::endl;

    if (!vpMath::equal(regular_stdev, sse_stdev, std::numeric_limits<double>::epsilon())) {
      std::cerr << "Problem when computing vpColVector::stdev()!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  {
    // Test insert with big vector
    const unsigned int nb = 1000;
    const unsigned int size = 10000;
    std::vector<vpColVector> vec(nb);

    for (size_t i = 0; i < nb; i++) {
      vpColVector v(size);
      for (unsigned int j = 0; j < size; j++) {
        v[j] = getRandomValues(-100.0, 100.0);
      }
      vec[i] = v;
    }

    vpColVector v_big(nb * size);
    double t = vpTime::measureTimeMs();
    for (unsigned int i = 0; i < nb; i++) {
      v_big.insert(i * size, vec[(size_t)i]);
    }
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nBig insert: " << t << " ms" << std::endl;

    for (unsigned int i = 0; i < nb; i++) {
      for (unsigned int j = 0; j < size; j++) {
        if (!vpMath::equal(v_big[i * size + j], vec[(size_t)i][j], std::numeric_limits<double>::epsilon())) {
          std::cerr << "Problem in vpColVector insert()!" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    // Try to insert empty vpColVector
    vpColVector v1(2), v2, v3;
    v1.insert(0, v2);
    v3.insert(0, v2);

    std::cout << "Insert empty vectors:" << std::endl;
    std::cout << "v1: " << v1.t() << std::endl;
    std::cout << "v2: " << v2.t() << std::endl;
    std::cout << "v3: " << v3.t() << std::endl;
  }

  {
    std::cout << "** Test conversion to/from std::vector" << std::endl;
    std::vector<double> std_vector(5);
    for (size_t i = 0; i < std_vector.size(); i++) {
      std_vector[i] = (double) i;
    }
    vpColVector v(std_vector);
    if (test("v", v, std_vector) == false)
      return EXIT_FAILURE;

    std_vector.clear();
    std_vector = v.toStdVector();
    if (test("v", v, std_vector) == false)
      return EXIT_FAILURE;
  }
  std::cout << "\nAll tests succeed" << std::endl;
  return EXIT_SUCCESS;
}
