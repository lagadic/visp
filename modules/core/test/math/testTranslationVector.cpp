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
  \example testTranslationVector.cpp

  Test some vpTranslationVector functionalities.
*/

#include <cmath>
#include <limits>
#include <vector>

#include <visp3/core/vpTranslationVector.h>

bool test(const std::string &s, const vpArray2D<double> &A, const std::vector<double> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << A.getRows() << "," << A.getCols() << ") =" << A << std::endl;
  if (bench.size() != A.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < A.size(); i++) {
    if (std::fabs(A.data[i] - bench[i]) > std::fabs(A.data[i]) * std::numeric_limits<double>::epsilon()) {
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
    vpTranslationVector t;
    std::vector<double> bench(3, 0);
    if (test("t", t, bench) == false)
      return err;
  }
  {
    vpTranslationVector t1(1, 2, 3);
    std::vector<double> bench(3);
    bench[0] = 1;
    bench[1] = 2;
    bench[2] = 3;
    if (test("t1", t1, bench) == false)
      return err;

    vpTranslationVector t2(4, 5, 6);
    bench[0] = 4;
    bench[1] = 5;
    bench[2] = 6;
    if (test("t2", t2, bench) == false)
      return err;

    vpTranslationVector t3 = t1 + t2;
    bench[0] = 5;
    bench[1] = 7;
    bench[2] = 9;
    if (test("t3", t3, bench) == false)
      return err;

    vpMatrix skew = t3.skew();
    std::vector<double> bench1(9, 0);
    bench1[1] = -9;
    bench1[2] = 7;
    bench1[3] = 9;
    bench1[5] = -5;
    bench1[6] = -7;
    bench1[7] = 5;
    if (test("skew", skew, bench1) == false)
      return err;

    vpTranslationVector t4;
    t4.set(-1, -2, -3);
    bench[0] = -1;
    bench[1] = -2;
    bench[2] = -3;
    if (test("t4", t4, bench) == false)
      return err;

    vpTranslationVector t5 = t4 * 2;
    bench[0] = -2;
    bench[1] = -4;
    bench[2] = -6;
    if (test("t5", t5, bench) == false)
      return err;

    vpTranslationVector t6 = vpTranslationVector::cross(t4, t5);
    bench[0] = 0;
    bench[1] = 0;
    bench[2] = 0;
    if (test("t6", t6, bench) == false)
      return err;
  }
  std::cout << "All tests succeed" << std::endl;
  return 0;
}
