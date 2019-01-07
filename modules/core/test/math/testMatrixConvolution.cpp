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
 * Test matrix convolution.
 *
 *****************************************************************************/

/*!
  \example testMatrixConvolution.cpp
  \brief Test matrix convolution.
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTime.h>

namespace {
  bool compareMatrix(const vpMatrix &m, const double * const array) {
    for (unsigned int i = 0; i < m.getRows(); i++) {
      for (unsigned int j = 0; j < m.getCols(); j++) {
        if (!vpMath::equal(m[i][j], array[i*m.getCols()+j], std::numeric_limits<double>::epsilon()))
          return false;
      }
    }

    return true;
  }
}

int main()
{
  try {
    {
      vpMatrix A(4,4);
      A[0][0] = 16;  A[0][1] = 2;  A[0][2] = 3;  A[0][3] = 13;
      A[1][0] = 5;  A[1][1] = 11;  A[1][2] = 10;  A[1][3] = 8;
      A[2][0] = 9;  A[2][1] = 7;  A[2][2] = 6;  A[2][3] = 12;
      A[3][0] = 4;  A[3][1] = 14;  A[3][2] = 15;  A[3][3] = 1;

      vpMatrix B(2,2);
      B[0][0] = 1;  B[0][1] = 3;
      B[1][0] = 4;  B[1][1] = 2;

      {
        vpMatrix res = vpMatrix::conv2(A, B, "full");
        double ground_truth[5*5] = {16, 50, 9, 22, 39,
                                    69, 66, 59, 96, 50,
                                    29, 88, 89, 82, 52,
                                    40, 72, 95, 106, 27,
                                    16, 64, 88, 34, 2};

        std::cout << "A:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, full):\n" << res << std::endl;

        if (res.getRows() != 5 || res.getCols() != 5 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "same");
        double ground_truth[4*4] = { 66, 59, 96, 50,
                                     88, 89, 82, 52,
                                     72, 95, 106, 27,
                                     64, 88, 34, 2};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, same):\n" << res << std::endl;

        if (res.getRows() != 4 || res.getCols() != 4 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "valid");
        double ground_truth[3*3] = { 66, 59, 96,
                                     88, 89, 82,
                                     72, 95, 106};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, valid):\n" << res << std::endl;

        if (res.getRows() != 3 || res.getCols() != 3 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
    }

    {
      vpMatrix A(2,6);
      for (unsigned int i = 0; i < A.getRows(); i++)
        for (unsigned int j = 0; j < A.getCols(); j++)
          A[i][j] = i*A.getCols()+j;

      vpMatrix B(4,2);
      for (unsigned int i = 0; i < B.getRows(); i++)
        for (unsigned int j = 0; j < B.getCols(); j++)
          B[i][j] = i*B.getCols()+j;

      {
        vpMatrix res = vpMatrix::conv2(A, B, "full");
        double ground_truth[5*7] = { 0, 0, 1, 2, 3, 4, 5,
                                     0, 8, 14, 20, 26, 32, 26,
                                     12, 36, 50, 64, 78, 92, 58,
                                     24, 64, 86, 108, 130, 152, 90,
                                     36, 84, 97, 110, 123, 136, 77};

        std::cout << "A:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, full):\n" << res << std::endl;

        if (res.getRows() != 5 || res.getCols() != 7 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "same");
        double ground_truth[2*6] = { 36, 50, 64, 78, 92, 58,
                                     64, 86, 108, 130, 152, 90};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, same):\n" << res << std::endl;

        if (res.getRows() != 2 || res.getCols() != 6 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "valid");

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, valid):\n" << res << std::endl;

        if (res.getRows() != 0 || res.getCols() != 0) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }

      {
        vpMatrix res = vpMatrix::conv2(B, A, "full");
        double ground_truth[5*7] = { 0, 0, 1, 2, 3, 4, 5,
                                     0, 8, 14, 20, 26, 32, 26,
                                     12, 36, 50, 64, 78, 92, 58,
                                     24, 64, 86, 108, 130, 152, 90,
                                     36, 84, 97, 110, 123, 136, 77};

        std::cout << "A:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(B, A, full):\n" << res << std::endl;

        if (res.getRows() != 5 || res.getCols() != 7 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(B, A, "same");
        double ground_truth[4*2] = { 20, 26,
                                     64, 78,
                                     108, 130,
                                     110, 123};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(B, A, same):\n" << res << std::endl;

        if (res.getRows() != 4 || res.getCols() != 2 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(B, A, "valid");

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(B, A, valid):\n" << res << std::endl;

        if (res.getRows() != 0 || res.getCols() != 0) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
    }

    {
      vpMatrix A(4,4);
      A[0][0] = 16;  A[0][1] = 2;  A[0][2] = 3;  A[0][3] = 13;
      A[1][0] = 5;  A[1][1] = 11;  A[1][2] = 10;  A[1][3] = 8;
      A[2][0] = 9;  A[2][1] = 7;  A[2][2] = 6;  A[2][3] = 12;
      A[3][0] = 4;  A[3][1] = 14;  A[3][2] = 15;  A[3][3] = 1;

      vpMatrix B(3,3);
      B[0][0] = 8;  B[0][1] = 1;  B[0][2] = 6;
      B[1][0] = 3;  B[1][1] = 5;  B[1][2] = 7;
      B[2][0] = 4;  B[2][1] = 9;  B[2][2] = 2;

      {
        vpMatrix res = vpMatrix::conv2(A, B, "full");
        double ground_truth[6*6] = { 128, 32, 122, 119, 31, 78,
                                     88, 179, 252, 208, 154, 139,
                                     151, 275, 291, 378, 281, 154,
                                     79, 271, 423, 366, 285, 106,
                                     48, 171, 248, 292, 230, 31,
                                     16, 92, 194, 167, 39, 2};

        std::cout << "A:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, full):\n" << res << std::endl;

        if (res.getRows() != 6 || res.getCols() != 6 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "same");
        double ground_truth[4*4] = { 179, 252, 208, 154,
                                     275, 291, 378, 281,
                                     271, 423, 366, 285,
                                     171, 248, 292, 230};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, same):\n" << res << std::endl;

        if (res.getRows() != 4 || res.getCols() != 4 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
      {
        vpMatrix res = vpMatrix::conv2(A, B, "valid");
        double ground_truth[2*2] = { 291, 378,
                                     423, 366};

        std::cout << "\nA:\n" << A << "\nB:\n" << B << "\nvpMatrix::conv2(A, B, valid):\n" << res << std::endl;

        if (res.getRows() != 2 || res.getCols() != 2 || !compareMatrix(res, ground_truth)) {
          throw vpException(vpException::badValue, "Issue with vpMatrix::conv2()");
        }
      }
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
