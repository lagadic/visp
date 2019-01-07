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
  \example testArray2D.cpp

  Test some vpArray2D functionalities.
*/

#include <cmath>
#include <limits>
#include <vector>

#include <visp3/core/vpTranslationVector.h>

template <typename Type> bool test(const std::string &s, const vpArray2D<Type> &A, const std::vector<Type> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << A.getRows() << "," << A.getCols() << ") = \n" << A << std::endl;
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
    // test default constructor
    vpArray2D<double> A;
    std::vector<double> bench;
    if (test("A", A, bench) == false)
      return EXIT_FAILURE;
  }
  {
    // test copy constructor
    vpArray2D<double> A(3, 4);

    std::vector<double> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        A[i][j] = (double)(i + j);
        bench[i * 4 + j] = (double)(i + j);
      }
    }
    if (test("A", A, bench) == false)
      return EXIT_FAILURE;

    vpArray2D<double> B(A);
    if (test("B", B, bench) == false)
      return EXIT_FAILURE;
    std::cout << "Min/Max: " << B.getMinValue() << " " << B.getMaxValue() << std::endl;
  }
  {
    // test constructor with initial value
    vpArray2D<double> A(3, 4, 2.);
    std::vector<double> bench1(12, 2);
    if (test("A", A, bench1) == false)
      return EXIT_FAILURE;

    A.resize(5, 6);
    std::vector<double> bench2(30, 0);
    if (test("A", A, bench2) == false)
      return EXIT_FAILURE;

    A = -2.;
    std::vector<double> bench3(30, -2);
    if (test("A", A, bench3) == false)
      return EXIT_FAILURE;
  }

  // Test with float
  {
    // test default constructor
    vpArray2D<float> A;
    std::vector<float> bench;
    if (test("A", A, bench) == false)
      return EXIT_FAILURE;
  }
  {
    // test copy constructor
    vpArray2D<float> A(3, 4);

    std::vector<float> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        A[i][j] = (float)(i + j);
        bench[i * 4 + j] = (float)(i + j);
      }
    }
    if (test("A", A, bench) == false)
      return EXIT_FAILURE;

    vpArray2D<float> B(A);
    if (test("B", B, bench) == false)
      return EXIT_FAILURE;
    std::cout << "Min/Max: " << B.getMinValue() << " " << B.getMaxValue() << std::endl;
  }
  {
    // test constructor with initial value
    vpArray2D<float> A(3, 4, 2.);
    std::vector<float> bench1(12, 2);
    if (test("A", A, bench1) == false)
      return EXIT_FAILURE;

    A.resize(5, 6);
    std::vector<float> bench2(30, 0);
    if (test("A", A, bench2) == false)
      return EXIT_FAILURE;

    A = -2.;
    std::vector<float> bench3(30, -2);
    if (test("A", A, bench3) == false)
      return EXIT_FAILURE;
  }
  {
    // Test Hadamard product
    std::cout << "\nTest Hadamard product" << std::endl;
    vpArray2D<int> A1(3, 5), A2(3, 5);
    vpRowVector R1(15), R2(15);
    vpColVector C1(15), C2(15);

    for (unsigned int i = 0; i < A1.size(); i++) {
      A1.data[i] = i;
      A2.data[i] = i + 2;
      R1.data[i] = i;
      R2.data[i] = i + 2;
      C1.data[i] = i;
      C2.data[i] = i + 2;
    }

    std::cout << "A1:\n" << A1 << std::endl;
    std::cout << "\nA2:\n" << A2 << std::endl;
    A2 = A1.hadamard(A2);
    std::cout << "\nRes:\n" << A2 << std::endl;

    std::cout << "\nR1:\n" << R1 << std::endl;
    std::cout << "\nR2:\n" << R2 << std::endl;
    R2 = R1.hadamard(R2);
    std::cout << "\nRes:\n" << R2 << std::endl;

    std::cout << "\nC1:\n" << C1 << std::endl;
    std::cout << "\nC2:\n" << C2 << std::endl;
    C2 = C1.hadamard(C2);
    std::cout << "\nRes:\n" << C2 << std::endl;
  }
  std::cout << "All tests succeed" << std::endl;
  return 0;
}
