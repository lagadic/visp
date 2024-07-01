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
 * Test some vpColVector functionalities.
 */

/*!
  \example testArray2D.cpp

  Test some vpArray2D functionalities.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER

#include <catch.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include <visp3/core/vpTranslationVector.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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

template <typename Type>
bool test_hadamar(const vpArray2D<Type> &A, const vpArray2D<Type> &B, const vpArray2D<Type> &H)
{
  if (A.getCols() != B.getCols() || A.getCols() != H.getCols()) {
    std::cout << "Test fails: bad columns size" << std::endl;
    return false;
  }
  if (A.getRows() != B.getRows() || A.getRows() != H.getRows()) {
    std::cout << "Test fails: bad rows size" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < A.size(); i++) {
    if (std::fabs((A.data[i] * B.data[i]) - H.data[i]) > std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

TEST_CASE("Test constructors with double", "[constructors]")
{
  SECTION("Default constructor")
  {
    vpArray2D<double> A;
    std::vector<double> bench;
    CHECK(test("A", A, bench));
  }
  SECTION("Copy constructor")
  {
    vpArray2D<double> A(3, 4);

    std::vector<double> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        A[i][j] = (double)(i + j);
        bench[i * 4 + j] = (double)(i + j);
      }
    }
    CHECK(test("A", A, bench));

    vpArray2D<double> B(A);
    CHECK(test("B", B, bench));
  }
  SECTION("Constructor with initial value")
  {
    vpArray2D<double> A(3, 4, 2.);
    std::vector<double> bench1(12, 2);
    CHECK(test("A", A, bench1));

    A.resize(5, 6);
    std::vector<double> bench2(30, 0);
    CHECK(test("A", A, bench2));

    A = -2.;
    std::vector<double> bench3(30, -2);
    CHECK(test("A", A, bench3));
  }

  SECTION("Constructor from std::vector")
  {
    std::vector<double> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        bench[i * 4 + j] = (double)(i + j);
      }
    }
    SECTION("Keep default size (r=0, c=0)")
    {
      std::cout << "A with default size (r=0, c=0):\n" << std::endl;
      REQUIRE_THROWS(vpArray2D<double>(bench));
    }
    SECTION("Keep row size to 0")
    {
      unsigned int size = static_cast<unsigned int>(bench.size());
      vpArray2D<double> A(bench, 0, size);
      std::cout << "A with row size to 0:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == 1);
      CHECK(A.getCols() == bench.size());
    }
    SECTION("Keep col size to 0")
    {
      unsigned int size = static_cast<unsigned int>(bench.size());
      vpArray2D<double> A(bench, size, 0);
      std::cout << "A with col size to 0:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == bench.size());
      CHECK(A.getCols() == 1);
    }
    SECTION("Set r=3 and c=4")
    {
      vpArray2D<double> A(bench, 3, 4);
      std::cout << "A with r=3 and c=4:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == 3);
      CHECK(A.getCols() == 4);
    }
  }
}

TEST_CASE("Test constructors with float", "[constructors]")
{
  SECTION("Default constructor")
  {
    vpArray2D<float> A;
    std::vector<float> bench;
    CHECK(test("A", A, bench));
  }
  SECTION("Copy constructor")
  {
    vpArray2D<float> A(3, 4);

    std::vector<float> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        A[i][j] = (float)(i + j);
        bench[i * 4 + j] = (float)(i + j);
      }
    }
    CHECK(test("A", A, bench));

    vpArray2D<float> B(A);
    CHECK(test("B", B, bench));
  }
  SECTION("Constructor with initial value")
  {
    vpArray2D<float> A(3, 4, 2.);
    std::vector<float> bench1(12, 2);
    CHECK(test("A", A, bench1));

    A.resize(5, 6);
    std::vector<float> bench2(30, 0);
    CHECK(test("A", A, bench2));

    A = -2.;
    std::vector<float> bench3(30, -2);
    CHECK(test("A", A, bench3));
  }
  SECTION("Constructor from std::vector")
  {
    std::vector<float> bench(12);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        bench[i * 4 + j] = (float)(i + j);
      }
    }
    SECTION("Keep default size (r=0, c=0)")
    {
      std::cout << "A with default size (r=0, c=0):\n" << std::endl;
      REQUIRE_THROWS(vpArray2D<float>(bench));
    }
    SECTION("Keep row size to 0")
    {
      unsigned int size = static_cast<unsigned int>(bench.size());
      vpArray2D<float> A(bench, 0, size);
      std::cout << "A with row size to 0:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == 1);
      CHECK(A.getCols() == bench.size());
    }
    SECTION("Keep col size to 0")
    {
      unsigned int size = static_cast<unsigned int>(bench.size());
      vpArray2D<float> A(bench, size, 0);
      std::cout << "A with col size to 0:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == bench.size());
      CHECK(A.getCols() == 1);
    }
    SECTION("Set r=3 and c=4")
    {
      vpArray2D<float> A(bench, 3, 4);
      std::cout << "A with r=3 and c=4:\n" << A << std::endl;
      CHECK(test("A", A, bench));
      CHECK(A.getRows() == 3);
      CHECK(A.getCols() == 4);
    }
  }
}

TEST_CASE("Test Hadamar product", "[hadamar]")
{
  vpArray2D<int> A1(3, 5), A2(3, 5), A3;
  vpRowVector R1(15), R2(15), R3;
  vpColVector C1(15), C2(15), C3;

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
  A3 = A1.hadamard(A2);
  CHECK(test_hadamar(A1, A2, A3));
  std::cout << "\nRes hadamar(A1, A2):\n" << A3 << std::endl;

  std::cout << "\nR1:\n" << R1 << std::endl;
  std::cout << "\nR2:\n" << R2 << std::endl;
  R3 = R1.hadamard(R2);
  CHECK(test_hadamar(R1, R2, R3));
  std::cout << "\nRes hadamar(R1, R2):\n" << R3 << std::endl;

  std::cout << "\nC1:\n" << C1 << std::endl;
  std::cout << "\nC2:\n" << C2 << std::endl;
  C3 = C1.hadamard(C2);
  CHECK(test_hadamar(C1, C2, C3));
  std::cout << "\nRes hadamar(C1, C2):\n" << C3 << std::endl;
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
  std::cout << (numFailed ? "Test failed" : "Test succeed") << std::endl;
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
