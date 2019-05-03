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
 * Test matrix condition number.
 *
 *****************************************************************************/

/*!
  \example testMatrixConditionNumber.cpp
  \brief Test matrix condition number computation.
*/

#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpMatrix.h>

int test_condition_number(const std::string &test_name, const vpMatrix &M)
{
  double precision = 1e-6;

  bool is_square = (M.getCols() == M.getRows() ? true : false);
  double inducedL2_norm_inv = 0, cond_inv = 0;

  double inducedL2_norm = M.inducedL2Norm();
  double inducedL2_norm_pinv = M.pseudoInverse().inducedL2Norm();
  double cond = M.cond();
  double cond_pinv = inducedL2_norm * inducedL2_norm_pinv;

  vpMatrix kerMt;
  double rank = M.kernel(kerMt);

  if (is_square) {
    inducedL2_norm_inv = M.inverseByLU().inducedL2Norm();
    cond_inv = inducedL2_norm * inducedL2_norm_inv;
  }

  M.print(std::cout, 4, test_name);
  std::cout << "  Matrix rank: " << rank << std::endl;
  std::cout << "  Matrix induced L2 norm ||M||_L2: " << inducedL2_norm << std::endl;
  if (is_square) {
    std::cout << "  Inverse induced L2 norm ||M^-1||_L2: " << inducedL2_norm_inv << std::endl;
  }
  std::cout << "  Pseudo inverse induced L2 norm norm ||M^+||_L2: " << inducedL2_norm_pinv << std::endl;
  if (is_square) {
    std::cout << "  Condition number such as cond(M)=||M||_L2 * ||M^-1||_L2: " << cond_inv << std::endl;
  }
  std::cout << "  Condition number such as cond(M)=||M||_L2 * ||M^+||_L2: " << cond_pinv << std::endl;
  std::cout << "  Condition number cond(M): " << cond << std::endl;
  if (! vpMath::equal(cond, cond_pinv, precision)) {
    std::cout << "  Condition number differ from the one computed with the pseudo inverse" << std::endl;
    return EXIT_FAILURE;
  }
  if (is_square) {
    if (! vpMath::equal(cond, cond_inv, precision)) {
      std::cout << "  Condition number differ from the one computed with the inverse" << std::endl;
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}

int main()
{
  vpMatrix M(3, 3);
  M.eye();

  if (test_condition_number("* Test square matrix M", M)) {
    std::cout << "  - Condition number computation fails" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "  + Condition number computation succeed" << std::endl;
  }

  M.resize(2, 3);
  M[0][0] = 1; M[0][1] = 2; M[0][2] = 3;
  M[1][0] = 4; M[1][1] = 5; M[1][2] = 6;

  if (test_condition_number("* Test rect matrix M", M)) {
    std::cout << "  - Condition number computation fails" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "  + Condition number computation succeed" << std::endl;
  }

  M = M.transpose();

  if (test_condition_number("* Test rect matrix M", M)) {
    std::cout << "  - Condition number computation fails" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "  + Condition number computation succeed" << std::endl;
  }

  M.resize(3, 4);
  M[0][0] = 1; M[0][1] = 2; M[0][2] = 3; M[0][3] = 0;
  M[1][0] = 4; M[1][1] = 5; M[1][2] = 6; M[1][3] = 0;
  M[2][0] = 0; M[2][1] = 0; M[2][2] = 0; M[2][3] = 0;

  if (test_condition_number("* Test rect matrix M", M)) {
    std::cout << "  - Condition number computation fails" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "  + Condition number computation succeed" << std::endl;
  }

  M = M.transpose();

  if (test_condition_number("* Test rect matrix M", M)) {
    std::cout << "  - Condition number computation fails" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "  + Condition number computation succeed" << std::endl;
  }

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
