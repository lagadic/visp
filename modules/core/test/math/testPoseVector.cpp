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
  \example testPoseVector.cpp

  Test some vpPoseVector functionalities.
*/

#include <cmath>
#include <limits>
#include <vector>

#include <visp3/core/vpPoseVector.h>

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
  {
    vpPoseVector p;
    std::vector<double> bench(6, 0);
    int err = 1;
    if (test("p", p, bench) == false)
      return err;
    p[0] = bench[0] = 0.1;
    p[1] = bench[1] = 0.2;
    p[2] = bench[2] = 0.3;
    p[3] = bench[3] = vpMath::rad(10);
    p[4] = bench[4] = vpMath::rad(20);
    p[5] = bench[5] = vpMath::rad(30);

    if (test("p", p, bench) == false)
      return err;

    vpPoseVector p1(p[0], p[1], p[2], p[3], p[4], p[5]);
    if (test("p1", p1, bench) == false)
      return err;
    vpPoseVector p2(p1);
    if (test("p2", p2, bench) == false)
      return err;
    vpPoseVector p3 = p1;
    if (test("p3", p3, bench) == false)
      return err;
    vpPoseVector p4;
    p4.set(p[0], p[1], p[2], p[3], p[4], p[5]);
    if (test("p4", p4, bench) == false)
      return err;

    vpTranslationVector t(p[0], p[1], p[2]);
    vpThetaUVector tu(p[3], p[4], p[5]);
    vpPoseVector p5(t, tu);
    if (test("p5", p5, bench) == false)
      return err;
    vpPoseVector p6;
    p6.buildFrom(t, tu);
    if (test("p6", p6, bench) == false)
      return err;

    vpHomogeneousMatrix M(t, tu);
    vpPoseVector p7(M);
    if (test("p7", p7, bench) == false)
      return err;
    vpPoseVector p8;
    p8.buildFrom(M);
    if (test("p8", p8, bench) == false)
      return err;

    vpRotationMatrix R(tu);
    vpPoseVector p9(t, R);
    if (test("p9", p9, bench) == false)
      return err;
    vpPoseVector p10;
    p10.buildFrom(t, R);
    if (test("p10", p10, bench) == false)
      return err;
  }
  std::cout << "All tests succeed" << std::endl;
  return 0;
}
