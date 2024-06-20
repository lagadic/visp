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
 * Test matrix condition number.
 */

/*!
  \example testMatrixCholesky.cpp
  \brief Test matrix Cholesky's decomposition.
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpMatrix.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool equal(const vpMatrix &A, const vpMatrix &B)
{
  unsigned int rowsA = A.getRows(), rowsB = B.getRows();
  unsigned int colsA = A.getCols(), colsB = B.getCols();
  bool areEqual = (rowsA == rowsB) && (colsA == colsB);

  if (!areEqual) {
    return false;
  }

  for (unsigned int r = 0; r < rowsA; ++r) {
    for (unsigned int c = 0; c < colsA; ++c) {
      areEqual = areEqual && vpMath::equal(A[r][c], B[r][c]);
      if (!areEqual) {
        return false;
      }
    }
  }
  return areEqual;
}

bool testCholeskyDecomposition(const vpMatrix &M, const vpMatrix &gtResult, const vpMatrix &result,
                               const std::string &title, const bool &verbose)
{
  if (verbose) {
    std::cout << "-------------------------" << std::endl;
    std::cout << "Test: " << title << std::endl;
    std::cout << std::endl;
  }
  unsigned int gtRows = gtResult.getRows(), resRows = result.getRows();
  unsigned int gtCols = gtResult.getCols(), resCols = result.getCols();
  bool areEqual = (gtRows == resRows) && (gtCols == resCols);
  if (!areEqual) {
    if (verbose) {
      std::cout << "Failed: dimensions mismatch (" << gtRows << " x " << gtCols << ")";
      std::cout << " vs (" << resRows << " x " << resCols << ")" << std::endl;
    }
    return false;
  }

  areEqual = equal(gtResult, result);

  if ((!areEqual) && verbose) {
    std::cout << "Failed: L matrices differ." << std::endl;
    std::cout << "Result =\n" << result << std::endl;
    std::cout << "GT =\n" << gtResult << std::endl;
    return false;
  }

  vpMatrix LLt = result * result.transpose();
  areEqual = equal(M, LLt);

  if ((!areEqual) && verbose) {
    std::cout << "Failed: LL^T differ from M." << std::endl;
    std::cout << "LL^T =\n" << LLt << std::endl;
    std::cout << "GT M =\n" << M << std::endl;
  }
  if (areEqual && verbose) {
    std::cout << "Test " << title << " succeeded" << std::endl;
  }

  return areEqual;
}


TEST_CASE("3 x 3 input", "[cholesky]")
{
  vpMatrix M(3, 3);
  M[0][0] = 4; M[0][1] = 12; M[0][2] = -16;
  M[1][0] = 12; M[1][1] = 37; M[1][2] = -43;
  M[2][0] = -16; M[2][1] = -43; M[2][2] = 98;

  vpMatrix gtL(3, 3, 0.);
  gtL[0][0] = 2; // M[0][1] = 0; M[0][2] = 0;
  gtL[1][0] = 6; gtL[1][1] = 1; // M[1][2] = 0;
  gtL[2][0] = -8; gtL[2][1] = 5; gtL[2][2] = 3;

#if defined(VISP_HAVE_LAPACK)
  SECTION("LAPACK")
  {
    vpMatrix L = M.choleskyByLapack();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test 1 Cholesky's decomposition using Lapack", true));
  }
#endif

#if defined(VISP_HAVE_EIGEN3)
  SECTION("EIGEN3")
  {
    vpMatrix L = M.choleskyByEigen3();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test Cholesky's decomposition using Eigen3", true));
  }
#endif

#if defined(VISP_HAVE_OPENCV)
  SECTION("OPENCV")
  {
    vpMatrix L = M.choleskyByOpenCV();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test Cholesky's decomposition using OpenCV", true));
  }
#endif
}

TEST_CASE("4 x 4 input", "[cholesky]")
{
  vpMatrix M(4, 4);
  M[0][0] = 4.16; M[0][1] = -3.12; M[0][2] = 0.56; M[0][3] = -0.10;
  M[1][0] = -3.12; M[1][1] = 5.03; M[1][2] = -0.83; M[1][3] = 1.18;
  M[2][0] = 0.56; M[2][1] = -0.83; M[2][2] = 0.76; M[2][3] = 0.34;
  M[3][0] = -0.10; M[3][1] = 1.18; M[3][2] = 0.34; M[3][3] = 1.18;

  vpMatrix gtL(4, 4, 0.);
  gtL[0][0] = 2.0396;
  gtL[1][0] = -1.5297; gtL[1][1] = 1.6401;
  gtL[2][0] = 0.2746; gtL[2][1] = -0.2500; gtL[2][2] = 0.7887;
  gtL[3][0] = -0.0490; gtL[3][1] = 0.6737; gtL[3][2] = 0.6617; gtL[3][3] = 0.5347;

#if defined(VISP_HAVE_LAPACK)
  SECTION("LAPACK")
  {
    vpMatrix L = M.choleskyByLapack();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test 1 Cholesky's decomposition using Lapack", true));
  }
#endif

#if defined(VISP_HAVE_EIGEN3)
  SECTION("EIGEN3")
  {
    vpMatrix L = M.choleskyByEigen3();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test Cholesky's decomposition using Eigen3", true));
  }
#endif

#if defined(VISP_HAVE_OPENCV)
  SECTION("OPENCV")
  {
    vpMatrix L = M.choleskyByOpenCV();
    CHECK(testCholeskyDecomposition(M, gtL, L, "Test Cholesky's decomposition using OpenCV", true));
  }
#endif
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

int main()
{
  std::cout << "Test ignored: Catch2 is not available" << std::endl;
  return EXIT_SUCCESS;
}
#endif
