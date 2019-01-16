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
  \example testRowVector.cpp

  Test some vpRowVector functionalities.
*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpMath.h>
#include <visp3/core/vpRowVector.h>

bool test(const std::string &s, const vpRowVector &v, const std::vector<double> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v << "]" << std::endl;
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

int main()
{
  int err = 1;

  {
    vpRowVector v;

    v.resize(4);
    v = 3;
    std::vector<double> bench1(4, 3);
    if (test("v", v, bench1) == false)
      return err;
    std::vector<double> bench2(4, 3. / 6);
    v.normalize();
    if (test("v", v, bench2) == false)
      return err;

    v.resize(1, 5, true);
    std::vector<double> bench3(5, 0);
    if (test("v", v, bench3) == false)
      return err;
  }

  {
    vpRowVector v(4);
    std::vector<double> bench1(4);
    for (unsigned int i = 0; i < v.size(); i++) {
      v[i] = (double)i;
      bench1[i] = (double)i;
    }
    if (test("v", v, bench1) == false)
      return err;

    vpRowVector w;
    w.init(v, 0, 2);
    std::vector<double> bench2;
    bench2.push_back(0);
    bench2.push_back(1);
    if (test("w", w, bench2) == false)
      return err;

    std::vector<double> bench3;
    bench3.push_back(1);
    bench3.push_back(2);
    bench3.push_back(3);

    vpRowVector r1;
    for (size_t i = 0; i < 4; i++)
      r1.stack((double)i);

    vpRowVector r2 = r1.extract(1, 3);
    if (test("r2", r2, bench3) == false)
      return err;
  }
  {
    vpMatrix M(1, 4);
    std::vector<double> bench(4);
    for (unsigned int i = 0; i < M.getCols(); i++) {
      M[0][i] = i;
      bench[i] = i;
    }
    if (test("M", M, bench) == false)
      return err;
    vpRowVector v;
    v = M;
    if (test("v", v, bench) == false)
      return err;
    vpRowVector w(M);
    if (test("w", w, bench) == false)
      return err;
    vpRowVector z1(bench);
    if (test("z1", z1, bench) == false)
      return err;
    vpRowVector z2 = bench;
    if (test("z2", z2, bench) == false)
      return err;
  }
  {
    vpRowVector v(3);
    v[0] = 1;
    v[1] = 2;
    v[2] = 3;
    std::vector<double> bench1;
    bench1.push_back(3);
    bench1.push_back(6);
    bench1.push_back(9);

    vpRowVector w = v * 3;
    // v is unchanged
    // w is now equal to : [3 6 9]
    if (test("w", w, bench1) == false)
      return err;

    vpRowVector x(w);
    if (test("x", x, bench1) == false)
      return err;

    std::vector<float> bench2;
    bench2.push_back(3);
    bench2.push_back(6);
    bench2.push_back(9);
    vpRowVector y1(bench2);
    if (test("y1", y1, bench1) == false)
      return err;
    vpRowVector y2 = bench2;
    if (test("y2", y2, bench1) == false)
      return err;
  }
  {
    vpRowVector r1(3, 1);
    vpRowVector r2 = -r1;
    std::vector<double> bench(3, -1);
    // v contains [-1 -1 -1]
    if (test("r2", r2, bench) == false)
      return err;
    r2.stack(-2);
    bench.push_back(-2);
    if (test("r2", r2, bench) == false)
      return err;
    vpRowVector r3 = vpRowVector::stack(r1, r2);
    std::vector<double> bench3(7, 1);
    bench3[3] = bench3[4] = bench3[5] = -1;
    bench3[6] = -2;
    if (test("r3", r3, bench3) == false)
      return err;

    r1.stack(r2);
    if (test("r1", r1, bench3) == false)
      return err;
  }
  {
    vpRowVector r1(3, 2);
    vpRowVector r2(3, 4);
    vpRowVector r = r1 + r2;
    std::vector<double> bench(3, 6);
    if (test("r", r, bench) == false)
      return err;
    r1 += r2;
    if (test("r1", r1, bench) == false)
      return err;
  }
  {
    vpRowVector r1(3, 2);
    vpRowVector r2(3, 4);
    vpRowVector r = r1 - r2;
    std::vector<double> bench(3, -2);
    if (test("r", r, bench) == false)
      return err;
    r1 -= r2;
    if (test("r1", r1, bench) == false)
      return err;
  }
  {
    vpRowVector r(5, 1);
    r.clear();
    r.resize(5);
    r = 5;
    std::vector<double> bench(5, 5);
    if (test("r", r, bench) == false)
      return err;
  }
  {
    // Test mean, median and standard deviation against Matlab with rng(0) and
    // rand(10,1)*10
    vpRowVector r(10);
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
    double res = vpRowVector::mean(r);
    if (!vpMath::equal(res, 6.2386, 0.001)) {
      std::cout << "Test fails: bad mean " << res << std::endl;
      return err;
    }

    std::cout << "** Test stdev" << std::endl;
    res = vpRowVector::stdev(r);
    if (!vpMath::equal(res, 3.2810, 0.001)) {
      std::cout << "Test fails: bad stdev " << res << std::endl;
      return err;
    }

    std::cout << "** Test stdev(bessel)" << std::endl;
    res = vpRowVector::stdev(r, true);
    if (!vpMath::equal(res, 3.4585, 0.001)) {
      std::cout << "Test fails: bad stdev(bessel) " << res << std::endl;
      return err;
    }

    std::cout << "** Test median" << std::endl;
    res = vpRowVector::median(r);
    if (!vpMath::equal(res, 7.2354, 0.001)) {
      std::cout << "Test fails: bad median " << res << std::endl;
      return err;
    }

    // Test median with odd number of elements
    std::cout << "** Test median (odd)" << std::endl;
    r.stack(1.5761);
    res = vpRowVector::median(r);
    if (!vpMath::equal(res, 6.3236, 0.001)) {
      std::cout << "Test fails: bad median (odd) " << res << std::endl;
      return err;
    }
    std::cout << "r: [" << r << "]" << std::endl;
    r.print(std::cout, 8, "r");
  }

  {
    std::cout << "** Test conversion to/from std::vector" << std::endl;
    std::vector<double> std_vector(5);
    for (size_t i = 0; i < std_vector.size(); i++) {
      std_vector[i] = (double) i;
    }
    vpRowVector v(std_vector);
    if (test("v", v, std_vector) == false)
      return EXIT_FAILURE;

    std_vector.clear();
    std_vector = v.toStdVector();
    if (test("v", v, std_vector) == false)
      return EXIT_FAILURE;
  }
  std::cout << "All tests succeed" << std::endl;
  return 0;
}
