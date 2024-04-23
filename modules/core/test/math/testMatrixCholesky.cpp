/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 *
*****************************************************************************/

/*!
  \example testMatrixCholesky.cpp
  \brief Test matrix Cholesky's decomposition.
*/

#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpMatrix.h>

bool testCholeskyDecomposition(const vpMatrix &gtResult, const vpMatrix &result,
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

  for (unsigned int r = 0; r < gtRows; ++r) {
    for (unsigned int c = 0; c < gtCols; ++c) {
      areEqual = areEqual && vpMath::equal(gtResult[r][c], result[r][c]);
    }
  }

  if ((!areEqual) && verbose) {
    std::cout << "Failed: matrices differ." << std::endl;
    std::cout << "Result =\n" << result << std::endl;
    std::cout << "GT =\n" << gtResult << std::endl;
  }
  else if (areEqual && verbose) {
    std::cout << "Test " << title << " succeeded" << std::endl;
  }

  return areEqual;
}

void printHelp(const std::string &progName)
{
  std::cout << "SYNOPSIS: " << std::endl;
  std::cout << "  " << progName << " [-v, --verbose] [-h, --help]" << std::endl;
  std::cout << "DETAILS:" << std::endl;
  std::cout << "  -v, --verbose" << std::endl;
  std::cout << "    Activate verbose mode to have some logs in the console." << std::endl;
  std::cout << std::endl;
  std::cout << "  -h, --help" << std::endl;
  std::cout << "    Display the help about the program." << std::endl;
  std::cout << std::endl;
}

int main(const int argc, const char *argv[])
{
  bool verbose = false;
  for (int i = 1; i < argc; ++i) {
    std::string argName(argv[i]);
    if ((argName == "-v") || (argName == "--verbose")) {
      verbose = true;
    }
    else if ((argName == "-h") || (argName == "--help")) {
      printHelp(std::string(argv[0]));
      return EXIT_FAILURE;
    }
  }

#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3)
  vpMatrix M(3, 3);
  M[0][0] = 4; M[0][1] = 12; M[0][2] = -16;
  M[1][0] = 12; M[1][1] = 37; M[1][2] = -43;
  M[2][0] = -16; M[2][1] = -43; M[2][2] = 98;

  vpMatrix gtL(3, 3, 0.);
  gtL[0][0] = 2; // M[0][1] = 0; M[0][2] = 0;
  gtL[1][0] = 6; gtL[1][1] = 1; // M[1][2] = 0;
  gtL[2][0] = -8; gtL[2][1] = 5; gtL[2][2] = 3;

  vpMatrix M2(4, 4);
  M2[0][0] = 4.16; M2[0][1] = -3.12; M2[0][2] = 0.56; M2[0][3] = -0.10;
  M2[1][0] = -3.12; M2[1][1] = 5.03; M2[1][2] = -0.83; M2[1][3] = 1.18;
  M2[2][0] = 0.56; M2[2][1] = -0.83; M2[2][2] = 0.76; M2[2][3] = 0.34;
  M2[3][0] = -0.10; M2[3][1] = 1.18; M2[3][2] = 0.34; M2[3][3] = 1.18;

  vpMatrix gtL2(4, 4, 0.);
  gtL2[0][0] = 2.0396;
  gtL2[1][0] = -1.5297; gtL2[1][1] = 1.6401;
  gtL2[2][0] = 0.2746; gtL2[2][1] = -0.2500; gtL2[2][2] = 0.7887;
  gtL2[3][0] = -0.0490; gtL2[3][1] = 0.6737; gtL2[3][2] = 0.6617; gtL2[3][3] = 0.5347;

  bool allSuccess = true;

#if defined(VISP_HAVE_LAPACK)
  vpMatrix L = M.choleskyByLapack();
  bool success = testCholeskyDecomposition(gtL, L,
                               "Test 1 Cholesky's decomposition using Lapack", verbose);
  allSuccess = allSuccess && success;

  L = M2.choleskyByLapack();
  success = testCholeskyDecomposition(gtL2, L,
                               "Test 2 Cholesky's decomposition using Lapack", verbose);
  allSuccess = allSuccess && success;
#endif

#if defined(VISP_HAVE_EIGEN3)
  L = M.choleskyByEigen3();
  success = testCholeskyDecomposition(gtL, L,
                               "Test Cholesky's decomposition using Eigen3", verbose);
  allSuccess = allSuccess && success;

  L = M2.choleskyByEigen3();
  success = testCholeskyDecomposition(gtL2, L,
                               "Test 2 Cholesky's decomposition using Eigen3", verbose);
  allSuccess = allSuccess && success;
#endif

  if (allSuccess) {
    std::cout << "Test succeeded" << std::endl;
    return EXIT_SUCCESS;
  }
  else {
    std::cout << "Test failed" << std::endl;
    return EXIT_FAILURE;
  }
#else
  std::cout << "Test ignored: install Lapack or Eigen3" << std::endl;
  return EXIT_SUCCESS;
#endif
}
