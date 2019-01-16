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
 * Tests transformation from various representations of rotation.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file testRotation.cpp
  \brief Tests transformation within various representations of rotation.
*/

#include <visp3/core/vpMath.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/io/vpParseArgv.h>

#include <cassert>
#include <limits>
#include <stdio.h>
#include <stdlib.h>

static unsigned int cpt = 0;

bool test(const std::string &s, const vpArray2D<double> &v, const std::vector<double> &bench)
{
  std::cout << "** Test " << ++cpt << std::endl;
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
  std::cout << "** Test " << ++cpt << std::endl;
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
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << v.getRows() << "," << v.getCols() << ") = [" << v << "]" << std::endl;
  for (unsigned int i = 0; i < v.size(); i++) {
    if (std::fabs(v[i] - bench) > std::fabs(v[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

int main()
{
  try {
    {
      vpThetaUVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      std::vector<double> bench1(3, vpMath::rad(10));
      vpColVector bench3(3, vpMath::rad(10));
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      bench1.clear();
      bench1 = r1.toStdVector();
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      r1.buildFrom(bench3);
      if (test("r1", r1, bench3) == false)
        return EXIT_FAILURE;

      vpThetaUVector r2 = r1;
      if (test("r2", r2, bench1) == false)
        return EXIT_FAILURE;

      if (test("r2", r2, vpMath::rad(10)) == false)
        return EXIT_FAILURE;

      vpThetaUVector r3;
      r3 = vpMath::rad(10);
      if (test("r3", r3, bench1) == false)
        return EXIT_FAILURE;

      std::cout << "** Test " << ++cpt << std::endl;
      for (unsigned int i = 0; i < r3.size(); i++) {
        if (std::fabs(r3[i] - bench1[i]) > std::fabs(r3[i]) * std::numeric_limits<double>::epsilon()) {
          std::cout << "Test fails: bad content" << std::endl;
          return EXIT_FAILURE;
        }
      }

      vpColVector r4 = 0.5 * r1;
      std::vector<double> bench2(3, vpMath::rad(5));
      if (test("r4", r4, bench2) == false)
        return EXIT_FAILURE;

      vpThetaUVector r5(r3);
      if (test("r5", r5, bench1) == false)
        return EXIT_FAILURE;
    }
    {
      vpRxyzVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      std::vector<double> bench1(3, vpMath::rad(10));
      vpColVector bench3(3, vpMath::rad(10));
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      bench1.clear();
      bench1 = r1.toStdVector();
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      r1.buildFrom(bench3);
      if (test("r1", r1, bench3) == false)
        return EXIT_FAILURE;

      vpRxyzVector r2 = r1;
      if (test("r2", r2, bench1) == false)
        return EXIT_FAILURE;

      if (test("r2", r2, vpMath::rad(10)) == false)
        return EXIT_FAILURE;

      vpRxyzVector r3;
      r3 = vpMath::rad(10);
      if (test("r3", r3, bench1) == false)
        return EXIT_FAILURE;

      std::cout << "** Test " << ++cpt << std::endl;
      for (unsigned int i = 0; i < r3.size(); i++) {
        if (std::fabs(r3[i] - bench1[i]) > std::fabs(r3[i]) * std::numeric_limits<double>::epsilon()) {
          std::cout << "Test fails: bad content" << std::endl;
          return EXIT_FAILURE;
        }
      }

      vpColVector r4 = 0.5 * r1;
      std::vector<double> bench2(3, vpMath::rad(5));
      if (test("r4", r4, bench2) == false)
        return EXIT_FAILURE;

      vpRxyzVector r5(r3);
      if (test("r5", r5, bench1) == false)
        return EXIT_FAILURE;
    }
    {
      vpRzyxVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      std::vector<double> bench1(3, vpMath::rad(10));
      vpColVector bench3(3, vpMath::rad(10));
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      bench1.clear();
      bench1 = r1.toStdVector();
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      r1.buildFrom(bench3);
      if (test("r1", r1, bench3) == false)
        return EXIT_FAILURE;

      vpRzyxVector r2 = r1;
      if (test("r2", r2, bench1) == false)
        return EXIT_FAILURE;

      if (test("r2", r2, vpMath::rad(10)) == false)
        return EXIT_FAILURE;

      vpRzyxVector r3;
      r3 = vpMath::rad(10);
      if (test("r3", r3, bench1) == false)
        return EXIT_FAILURE;

      std::cout << "** Test " << ++cpt << std::endl;
      for (unsigned int i = 0; i < r3.size(); i++) {
        if (std::fabs(r3[i] - bench1[i]) > std::fabs(r3[i]) * std::numeric_limits<double>::epsilon()) {
          std::cout << "Test fails: bad content" << std::endl;
          return EXIT_FAILURE;
        }
      }

      vpColVector r4 = 0.5 * r1;
      std::vector<double> bench2(3, vpMath::rad(5));
      if (test("r4", r4, bench2) == false)
        return EXIT_FAILURE;

      vpRzyxVector r5(r3);
      if (test("r5", r5, bench1) == false)
        return EXIT_FAILURE;
    }
    {
      vpRzyzVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      std::vector<double> bench1(3, vpMath::rad(10));
      vpColVector bench3(3, vpMath::rad(10));
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      bench1.clear();
      bench1 = r1.toStdVector();
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      r1.buildFrom(bench3);
      if (test("r1", r1, bench3) == false)
        return EXIT_FAILURE;

      vpRzyzVector r2 = r1;
      if (test("r2", r2, bench1) == false)
        return EXIT_FAILURE;

      if (test("r2", r2, vpMath::rad(10)) == false)
        return EXIT_FAILURE;

      vpRzyzVector r3;
      r3 = vpMath::rad(10);
      if (test("r3", r3, bench1) == false)
        return EXIT_FAILURE;

      std::cout << "** Test " << ++cpt << std::endl;
      for (unsigned int i = 0; i < r3.size(); i++) {
        if (std::fabs(r3[i] - bench1[i]) > std::fabs(r3[i]) * std::numeric_limits<double>::epsilon()) {
          std::cout << "Test fails: bad content" << std::endl;
          return EXIT_FAILURE;
        }
      }

      vpColVector r4 = 0.5 * r1;
      std::vector<double> bench2(3, vpMath::rad(5));
      if (test("r4", r4, bench2) == false)
        return EXIT_FAILURE;

      vpRzyzVector r5(r3);
      if (test("r5", r5, bench1) == false)
        return EXIT_FAILURE;
    }
    {
      vpQuaternionVector r1(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      std::vector<double> bench1(4, vpMath::rad(10));
      vpColVector bench3(4, vpMath::rad(10));
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      bench1.clear();
      bench1 = r1.toStdVector();
      if (test("r1", r1, bench1) == false)
        return EXIT_FAILURE;

      r1.buildFrom(bench3);
      if (test("r1", r1, bench3) == false)
        return EXIT_FAILURE;

      vpQuaternionVector r2 = r1;
      if (test("r2", r2, bench1) == false)
        return EXIT_FAILURE;

      if (test("r2", r2, vpMath::rad(10)) == false)
        return EXIT_FAILURE;

      vpQuaternionVector r3;
      r3.set(vpMath::rad(10), vpMath::rad(10), vpMath::rad(10), vpMath::rad(10));
      if (test("r3", r3, bench1) == false)
        return EXIT_FAILURE;

      std::cout << "** Test " << ++cpt << std::endl;
      for (unsigned int i = 0; i < r3.size(); i++) {
        if (std::fabs(r3[i] - bench1[i]) > std::fabs(r3[i]) * std::numeric_limits<double>::epsilon()) {
          std::cout << "Test fails: bad content" << std::endl;
          return EXIT_FAILURE;
        }
      }

      vpColVector r4 = 0.5 * r1;
      std::vector<double> bench2(4, vpMath::rad(5));
      if (test("r4", r4, bench2) == false)
        return EXIT_FAILURE;

      vpQuaternionVector r5(r3);
      if (test("r5", r5, bench1) == false)
        return EXIT_FAILURE;
    }
    {
      vpRotationMatrix R;
      for (int i = -10; i < 10; i++) {
        for (int j = -10; j < 10; j++) {
          vpThetaUVector tu(vpMath::rad(90 + i), vpMath::rad(170 + j), vpMath::rad(45));
          tu.buildFrom(vpRotationMatrix(tu)); // put some coherence into rotation convention

          std::cout << "Initialization " << std::endl;

          double theta;
          vpColVector u;
          tu.extract(theta, u);

          std::cout << "theta=" << vpMath::deg(theta) << std::endl;
          std::cout << "u=" << u << std::endl;

          std::cout << "From vpThetaUVector to vpRotationMatrix " << std::endl;
          R.buildFrom(tu);

          std::cout << "Matrix R";
          if (R.isARotationMatrix() == 1)
            std::cout << " is a rotation matrix " << std::endl;
          else
            std::cout << " is not a rotation matrix " << std::endl;

          std::cout << R << std::endl;

          std::cout << "From vpRotationMatrix to vpQuaternionVector " << std::endl;
          vpQuaternionVector q(R);
          std::cout << q << std::endl;

          R.buildFrom(q);
          std::cout << "From vpQuaternionVector to vpRotationMatrix  " << std::endl;

          std::cout << "From vpRotationMatrix to vpRxyzVector " << std::endl;
          vpRxyzVector RxyzBuildFromR(R);
          std::cout << RxyzBuildFromR << std::endl;

          std::cout << "From vpRxyzVector to vpThetaUVector " << std::endl;
          std::cout << "  use From vpRxyzVector to vpRotationMatrix " << std::endl;
          std::cout << "  use From vpRotationMatrix to vpThetaUVector " << std::endl;

          vpThetaUVector tuBuildFromEu;
          tuBuildFromEu.buildFrom(R);

          std::cout << std::endl;
          std::cout << "result : should equivalent to the first one " << std::endl;

          double theta2;
          vpColVector u2;

          tuBuildFromEu.extract(theta2, u2);
          std::cout << "theta=" << vpMath::deg(theta2) << std::endl;
          std::cout << "u=" << u2 << std::endl;

          assert(vpMath::abs(theta2 - theta) < std::numeric_limits<double>::epsilon() * 1e10);
          assert(vpMath::abs(u[0] - u2[0]) < std::numeric_limits<double>::epsilon() * 1e10);
          assert(vpMath::abs(u[1] - u2[1]) < std::numeric_limits<double>::epsilon() * 1e10);
          assert(vpMath::abs(u[2] - u2[2]) < std::numeric_limits<double>::epsilon() * 1e10);
        }
        vpRzyzVector rzyz(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45));
        std::cout << "Initialization vpRzyzVector " << std::endl;
        std::cout << rzyz << std::endl;
        std::cout << "From vpRzyzVector to vpRotationMatrix  " << std::endl;
        R.buildFrom(rzyz);
        std::cout << "From vpRotationMatrix to vpRzyzVector " << std::endl;
        vpRzyzVector rzyz_final;
        rzyz_final.buildFrom(R);
        std::cout << rzyz_final << std::endl;

        vpRzyxVector rzyx(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45));
        std::cout << "Initialization vpRzyxVector " << std::endl;
        std::cout << rzyx << std::endl;
        std::cout << "From vpRzyxVector to vpRotationMatrix  " << std::endl;
        R.buildFrom(rzyx);
        std::cout << R << std::endl;
        std::cout << "From vpRotationMatrix to vpRzyxVector " << std::endl;
        vpRzyxVector rzyx_final;
        rzyx_final.buildFrom(R);
        std::cout << rzyx_final << std::endl;
      }
    }
    std::cout << "All tests succeed" << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
