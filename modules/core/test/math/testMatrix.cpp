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
 * Test some vpMatrix functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrix.cpp

  Test some vpMatrix functionalities.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpGEMM.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#include <stdio.h>
#include <stdlib.h>

namespace
{
bool test(const std::string &s, const vpMatrix &M, const std::vector<double> &bench)
{
  static unsigned int cpt = 0;
  std::cout << "** Test " << ++cpt << std::endl;
  std::cout << s << "(" << M.getRows() << "," << M.getCols() << ") = \n" << M << std::endl;
  if (bench.size() != M.size()) {
    std::cout << "Test fails: bad size wrt bench" << std::endl;
    return false;
  }
  for (unsigned int i = 0; i < M.size(); i++) {
    if (std::fabs(M.data[i] - bench[i]) > std::fabs(M.data[i]) * std::numeric_limits<double>::epsilon()) {
      std::cout << "Test fails: bad content" << std::endl;
      return false;
    }
  }

  return true;
}

double getRandomValues(const double min, const double max)
{
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}

bool equalMatrix(const vpMatrix &A, const vpMatrix &B, const double tol = std::numeric_limits<double>::epsilon())
{
  if (A.getRows() != B.getRows() || A.getCols() != B.getCols()) {
    return false;
  }

  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = 0; j < A.getCols(); j++) {
      if (!vpMath::equal(A[i][j], B[i][j], tol)) {
        return false;
      }
    }
  }

  return true;
}

vpMatrix generateRandomMatrix(const unsigned int rows, const unsigned int cols, const double min, const double max)
{
  vpMatrix M(rows, cols);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = getRandomValues(min, max);
    }
  }

  return M;
}

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
vpColVector generateRandomVector(const unsigned int rows, const double min, const double max)
{
  vpColVector v(rows);

  for (unsigned int i = 0; i < v.getRows(); i++) {
    v[i] = getRandomValues(min, max);
  }

  return v;
}

// Copy of vpMatrix::mult2Matrices
vpMatrix dgemm_regular(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C;

  if ((A.getRows() != C.getRows()) || (B.getCols() != C.getCols()))
    C.resize(A.getRows(), B.getCols(), false);

  if (A.getCols() != B.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  // 5/12/06 some "very" simple optimization to avoid indexation
  unsigned int BcolNum = B.getCols();
  unsigned int BrowNum = B.getRows();
  unsigned int i, j, k;
  for (i = 0; i < A.getRows(); i++) {
    double *rowptri = A[i];
    double *ci = C[i];
    for (j = 0; j < BcolNum; j++) {
      double s = 0;
      for (k = 0; k < BrowNum; k++)
        s += rowptri[k] * B[k][j];
      ci[j] = s;
    }
  }

  return C;
}

// Copy of vpMatrix::AtA
vpMatrix AtA_regular(const vpMatrix &A)
{
  vpMatrix B;
  B.resize(A.getCols(), A.getCols(), false);

  unsigned int i, j, k;
  double s;
  double *ptr;
  for (i = 0; i < A.getCols(); i++) {
    double *Bi = B[i];
    for (j = 0; j < i; j++) {
      ptr = A.data;
      s = 0;
      for (k = 0; k < A.getRows(); k++) {
        s += (*(ptr + i)) * (*(ptr + j));
        ptr += A.getCols();
      }
      *Bi++ = s;
      B[j][i] = s;
    }
    ptr = A.data;
    s = 0;
    for (k = 0; k < A.getRows(); k++) {
      s += (*(ptr + i)) * (*(ptr + i));
      ptr += A.getCols();
    }
    *Bi = s;
  }

  return B;
}

// Copy of vpMatrix::multMatrixVector
vpMatrix dgemv_regular(const vpMatrix &A, const vpColVector &v)
{
  vpColVector w;

  if (A.getCols() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply a (%dx%d) matrix by a (%d) column vector",
                      A.getRows(), A.getCols(), v.getRows()));
  }

  w.resize(A.getRows(), true);

  for (unsigned int j = 0; j < A.getCols(); j++) {
    double vj = v[j]; // optimization em 5/12/2006
    for (unsigned int i = 0; i < A.getRows(); i++) {
      w[i] += A[i][j] * vj;
    }
  }

  return w;
}

// Copy of vpMatrix::operator*(const vpVelocityTwistMatrix &V)
vpMatrix mat_mul_twist_matrix(const vpMatrix &A, const vpVelocityTwistMatrix &V)
{
  vpMatrix M;

  if (A.getCols() != V.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (6x6) velocity twist matrix",
                      A.getRows(), A.getCols()));
  }

  M.resize(A.getRows(), 6, false);

  unsigned int VcolNum = V.getCols();
  unsigned int VrowNum = V.getRows();

  for (unsigned int i = 0; i < A.getRows(); i++) {
    double *rowptri = A[i];
    double *ci = M[i];
    for (unsigned int j = 0; j < VcolNum; j++) {
      double s = 0;
      for (unsigned int k = 0; k < VrowNum; k++)
        s += rowptri[k] * V[k][j];
      ci[j] = s;
    }
  }

  return M;
}
#endif
}

int main(int argc, char *argv[])
{
  try {
    bool ctest = true;
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--benchmark") {
        ctest = false;
      }
    }

    {
      const double val = 10.0;
      vpMatrix M, M2(5, 5, val);
      M.resize(5, 5, false, false);
      M = val;
      for (unsigned int i = 0; i < M.getRows(); i++) {
        for (unsigned int j = 0; j < M.getCols(); j++) {
          if (!vpMath::equal(M[i][j], val, std::numeric_limits<double>::epsilon())) {
            std::cerr << "Issue with matrix assignment with value." << std::endl;
            return EXIT_FAILURE;
          }

          if (!vpMath::equal(M2[i][j], val, std::numeric_limits<double>::epsilon())) {
            std::cerr << "Issue with matrix constructor initialized with value." << std::endl;
            return EXIT_FAILURE;
          }
        }
      }
    }
    {
      // Test vpRotationMatrix construction
      std::vector<double> bench(9, 0);
      bench[2] = bench[4] = bench[6] = 1.;
      vpMatrix M(3, 3);
      M[2][0] = M[1][1] = M[0][2] = 1.;
      vpRotationMatrix R1(M);
      if (test("R1", R1, bench) == false)
        return EXIT_FAILURE;
      vpRotationMatrix R2;
      R2 = M;
      if (test("R2", R2, bench) == false)
        return EXIT_FAILURE;
    }
    {
      vpColVector c(6, 1);
      vpRowVector r(6, 1);
      std::vector<double> bench(6, 1);
      vpMatrix M1(c);
      if (test("M1", M1, bench) == false)
        return EXIT_FAILURE;
      vpMatrix M2(r);
      if (test("M2", M2, bench) == false)
        return EXIT_FAILURE;
    }
    {
      vpMatrix M(4, 5);
      int val = 0;
      for (unsigned int i = 0; i < M.getRows(); i++) {
        for (unsigned int j = 0; j < M.getCols(); j++) {
          M[i][j] = val++;
        }
      }
      std::cout << "M ";
      M.print(std::cout, 4);

      vpMatrix N;
      N.init(M, 0, 1, 2, 3);
      std::cout << "N ";
      N.print(std::cout, 4);
      std::string header("My 4-by-5 matrix\nwith a second line");

      // Save matrix in text format
      if (vpMatrix::saveMatrix("matrix.mat", M, false, header.c_str()))
        std::cout << "Matrix saved in matrix.mat file" << std::endl;
      else
        return EXIT_FAILURE;

      // Load matrix in text format
      vpMatrix M1;
      char header_[100];
      if (vpMatrix::loadMatrix("matrix.mat", M1, false, header_))
        std::cout << "Matrix loaded from matrix.mat file with header \"" << header_ << "\": \n" << M1 << std::endl;
      else
        return EXIT_FAILURE;
      if (header != std::string(header_)) {
        std::cout << "Bad header in matrix.mat" << std::endl;
        return EXIT_FAILURE;
      }

      // Save matrix in binary format
      if (vpMatrix::saveMatrix("matrix.bin", M, true, header.c_str()))
        std::cout << "Matrix saved in matrix.bin file" << std::endl;
      else
        return EXIT_FAILURE;

      // Load matrix in binary format
      if (vpMatrix::loadMatrix("matrix.bin", M1, true, header_))
        std::cout << "Matrix loaded from matrix.bin file with header \"" << header_ << "\": \n" << M1 << std::endl;
      else
        return EXIT_FAILURE;
      if (header != std::string(header_)) {
        std::cout << "Bad header in matrix.bin" << std::endl;
        return EXIT_FAILURE;
      }

      // Save matrix in YAML format
      if (vpMatrix::saveMatrixYAML("matrix.yml", M, header.c_str()))
        std::cout << "Matrix saved in matrix.yml file" << std::endl;
      else
        return EXIT_FAILURE;

      // Read matrix in YAML format
      vpMatrix M2;
      if (vpMatrix::loadMatrixYAML("matrix.yml", M2, header_))
        std::cout << "Matrix loaded from matrix.yml file with header \"" << header_ << "\": \n" << M2 << std::endl;
      else
        return EXIT_FAILURE;
      if (header != std::string(header_)) {
        std::cout << "Bad header in matrix.mat" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      vpRotationMatrix R(vpMath::rad(10), vpMath::rad(20), vpMath::rad(30));
      std::cout << "R: \n" << R << std::endl;
      vpMatrix M1(R);
      std::cout << "M1: \n" << M1 << std::endl;
      vpMatrix M2(M1);
      std::cout << "M2: \n" << M2 << std::endl;
      vpMatrix M3 = R;
      std::cout << "M3: \n" << M3 << std::endl;
      vpMatrix M4 = M1;
      std::cout << "M4: \n" << M4 << std::endl;
    }
    {

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST PRETTY PRINT---" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix M;
      M.eye(4);

      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 4);" << std::endl;
      M.print(std::cout, 4);

      std::cout << "------------------------" << std::endl;
      M.resize(3, 3);
      M.eye(3);
      M[1][0] = 1.235;
      M[1][1] = 12.345;
      M[1][2] = .12345;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M;
      std::cout << "call M.print (std::cout, 6);" << std::endl;
      M.print(std::cout, 6);
      std::cout << std::endl;

      std::cout << "------------------------" << std::endl;
      M[0][0] = -1.235;
      M[1][0] = -12.235;

      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 10);" << std::endl;
      M.print(std::cout, 10);
      std::cout << std::endl;

      std::cout << "call M.print (std::cout, 2);" << std::endl;
      M.print(std::cout, 2);
      std::cout << std::endl;

      std::cout << "------------------------" << std::endl;
      M.resize(3, 3);
      M.eye(3);
      M[0][2] = -0.0000000876;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 4);" << std::endl;
      M.print(std::cout, 4);
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 10, \"M\");" << std::endl;
      M.print(std::cout, 10, "M");
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 20, \"M\");" << std::endl;
      M.print(std::cout, 20, "M");
      std::cout << std::endl;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST RESIZE --------" << std::endl;
      std::cout << "------------------------" << std::endl;
      std::cout << "5x5" << std::endl;
      M.resize(5, 5, false);
      std::cout << M << std::endl;
      std::cout << "3x2" << std::endl;
      M.resize(3, 2, false);
      std::cout << M << std::endl;
      std::cout << "2x2" << std::endl;
      M.resize(2, 2, false);
      std::cout << M << std::endl;
      std::cout << "------------------------" << std::endl;

      vpVelocityTwistMatrix vMe;
      vpMatrix A(1, 6), B;

      A = 1.0;
      // vMe=1.0;
      B = A * vMe;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpRowVector * vpColVector" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpRowVector r(3);
      r[0] = 2;
      r[1] = 3;
      r[2] = 4;

      vpColVector c(3);
      c[0] = 1;
      c[1] = 2;
      c[2] = -1;

      double rc = r * c;

      r.print(std::cout, 2, "r");
      c.print(std::cout, 2, "c");
      std::cout << "r * c = " << rc << std::endl;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpRowVector * vpMatrix" << std::endl;
      std::cout << "------------------------" << std::endl;
      M.resize(3, 3);
      M.eye(3);

      M[1][0] = 1.5;
      M[2][0] = 2.3;

      vpRowVector rM = r * M;

      r.print(std::cout, 2, "r");
      M.print(std::cout, 10, "M");
      std::cout << "r * M = " << rM << std::endl;

      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpGEMM " << std::endl;
      std::cout << "------------------------" << std::endl;
      M.resize(3, 3);
      M.eye(3);
      vpMatrix N(3, 3);
      N[0][0] = 2;
      N[1][0] = 1.2;
      N[1][2] = 0.6;
      N[2][2] = 0.25;

      vpMatrix C(3, 3);
      C.eye(3);

      vpMatrix D;

      // realise the operation D = 2 * M^T * N + 3 C
      vpGEMM(M, N, 2, C, 3, D, VP_GEMM_A_T);
      std::cout << D << std::endl;
    }

    {
      std::cout << "------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix insert() with same colNum " << std::endl;
      std::cout << "------------------------" << std::endl;
      const unsigned int nb = ctest ? 10 : 100; // 10000;
      const unsigned int size = ctest ? 10 : 100;

      vpMatrix m_big(nb * size, 6);
      std::vector<vpMatrix> submatrices(nb);
      for (size_t cpt = 0; cpt < submatrices.size(); cpt++) {
        vpMatrix m(size, 6);

        for (unsigned int i = 0; i < m.getRows(); i++) {
          for (unsigned int j = 0; j < m.getCols(); j++) {
            m[i][j] = getRandomValues(-100.0, 100.0);
          }
        }

        submatrices[cpt] = m;
      }

      double t = vpTime::measureTimeMs();
      for (unsigned int i = 0; i < nb; i++) {
        m_big.insert(submatrices[(size_t)i], i * size, 0);
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "Matrix insert(): " << t << " ms" << std::endl;

      for (unsigned int cpt = 0; cpt < nb; cpt++) {
        for (unsigned int i = 0; i < size; i++) {
          for (unsigned int j = 0; j < 6; j++) {
            if (!vpMath::equal(m_big[cpt * size + i][j], submatrices[(size_t)cpt][i][j],
                               std::numeric_limits<double>::epsilon())) {
              std::cerr << "Problem with vpMatrix insert()!" << std::endl;
              return EXIT_FAILURE;
            }
          }
        }
      }

      // Try to insert empty matrices
      vpMatrix m1(2, 3), m2, m3;
      m1.insert(m2, 0, 0);
      m3.insert(m2, 0, 0);

      std::cout << "Insert empty matrices:" << std::endl;
      std::cout << "m1:\n" << m1 << std::endl;
      std::cout << "m2:\n" << m2 << std::endl;
      std::cout << "m3:\n" << m3 << std::endl;

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix stack()" << std::endl;
      std::cout << "------------------------" << std::endl;

      {
        vpMatrix L, L2(2, 6);
        L2 = 2;
        L.stack(L2);
        std::cout << "L:\n" << L << std::endl;
        L2.resize(3, 6);
        L2 = 3;
        L.stack(L2);
        std::cout << "L:\n" << L << std::endl;
      }

      {
        vpMatrix m_big_stack;
        t = vpTime::measureTimeMs();
        for (unsigned int i = 0; i < nb; i++) {
          m_big_stack.stack(submatrices[(size_t)i]);
        }
        t = vpTime::measureTimeMs() - t;
        std::cout << "Matrix stack(): " << t << " ms" << std::endl;

        if (!equalMatrix(m_big, m_big_stack)) {
          std::cerr << "Problem with vpMatrix stack()!" << std::endl;
          return EXIT_FAILURE;
        }
      }

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix stack(vpRowVector)" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix m_big_stack = generateRandomMatrix(10000, ctest ? 10 : 100, -1000.0, 1000.0);
      std::cout << "m_big_stack: " << m_big_stack.getRows() << "x" << m_big_stack.getCols() << std::endl;

      vpMatrix m_big_stack_row;
      t = vpTime::measureTimeMs();
      for (unsigned int i = 0; i < m_big_stack.getRows(); i++) {
        m_big_stack_row.stack(m_big_stack.getRow(i));
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "Matrix stack(vpRowVector): " << t << " ms" << std::endl;

      if (!equalMatrix(m_big_stack, m_big_stack_row)) {
        std::cerr << "Problem with vpMatrix stack(vpRowVector)!" << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix stack(vpColVector)" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix m_big_stack_col;
      t = vpTime::measureTimeMs();
      for (unsigned int j = 0; j < m_big_stack.getCols(); j++) {
        m_big_stack_col.stack(m_big_stack.getCol(j));
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "Matrix stack(vpColVector): " << t << " ms" << std::endl;

      if (!equalMatrix(m_big_stack, m_big_stack_col)) {
        std::cerr << "Problem with vpMatrix stack(vpColVector)!" << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::stack()" << std::endl;
      std::cout << "------------------------" << std::endl;

      {
        vpMatrix L, L2(2, 6), L_tmp;
        L2 = 2;
        vpMatrix::stack(L_tmp, L2, L);
        std::cout << "L:\n" << L << std::endl;
        L2.resize(3, 6);
        L2 = 3;
        L_tmp = L;
        vpMatrix::stack(L_tmp, L2, L);
        std::cout << "L:\n" << L << std::endl;
      }

      {
        vpMatrix m_big_stack_static, m_big_stack_static_tmp;
        t = vpTime::measureTimeMs();
        for (unsigned int i = 0; i < nb; i++) {
          vpMatrix::stack(m_big_stack_static_tmp, submatrices[(size_t)i], m_big_stack_static);
          m_big_stack_static_tmp = m_big_stack_static;
        }
        t = vpTime::measureTimeMs() - t;
        std::cout << "Matrix::stack(): " << t << " ms" << std::endl;

        if (!equalMatrix(m_big, m_big_stack_static)) {
          std::cerr << "Problem with vpMatrix::stack()!" << std::endl;
          return EXIT_FAILURE;
        }
      }

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::stack(vpMatrix, vpRowVector, vpMatrix)" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix m_big_stack_static = generateRandomMatrix(ctest ? 100 : 1000, ctest ? 10 : 100, -1000.0, 1000.0);
      std::cout << "m_big_stack_static: " << m_big_stack_static.getRows() << "x" << m_big_stack_static.getCols() << std::endl;

      vpMatrix m_big_stack_static_row, m_big_stack_static_row_tmp;
      t = vpTime::measureTimeMs();
      for (unsigned int i = 0; i < m_big_stack_static.getRows(); i++) {
        vpMatrix::stack(m_big_stack_static_row_tmp, m_big_stack_static.getRow(i), m_big_stack_static_row);
        m_big_stack_static_row_tmp = m_big_stack_static_row;
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "Matrix::stack(vpMatrix, vpRowVector, vpMatrix): " << t << " ms" << std::endl;

      if (!equalMatrix(m_big_stack_static, m_big_stack_static_row)) {
        std::cerr << "Problem with vpMatrix::stack(vpMatrix, vpRowVector, "
                     "vpMatrix)!"
                  << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::stack(vpMatrix, vpColVector, vpMatrix)" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix m_big_stack_static_col, m_big_stack_static_col_tmp;
      t = vpTime::measureTimeMs();
      for (unsigned int j = 0; j < m_big_stack_static.getCols(); j++) {
        vpMatrix::stack(m_big_stack_static_col_tmp, m_big_stack_static.getCol(j), m_big_stack_static_col);
        m_big_stack_static_col_tmp = m_big_stack_static_col;
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "Matrix::stack(vpMatrix, vpColVector, vpMatrix): " << t << " ms" << std::endl;

      if (!equalMatrix(m_big_stack_static, m_big_stack_static_col)) {
        std::cerr << "Problem with vpMatrix::stack(vpMatrix, vpColVector, "
                     "vpMatrix)!"
                  << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      vpMatrix m1(11, 9), m2(3, 4);
      for (unsigned int i = 0; i < m2.getRows(); i++) {
        for (unsigned int j = 0; j < m2.getCols(); j++) {
          m2[i][j] = getRandomValues(-100.0, 100.0);
        }
      }

      unsigned int offset_i = 4, offset_j = 3;
      m1.insert(m2, offset_i, offset_j);

      for (unsigned int i = 0; i < m2.getRows(); i++) {
        for (unsigned int j = 0; j < m2.getCols(); j++) {
          if (!vpMath::equal(m1[i + offset_i][j + offset_j], m2[i][j], std::numeric_limits<double>::epsilon())) {
            std::cerr << "Problem with vpMatrix insert()!" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }

      offset_i = 4, offset_j = 5;
      m1.insert(m2, offset_i, offset_j);

      for (unsigned int i = 0; i < m2.getRows(); i++) {
        for (unsigned int j = 0; j < m2.getCols(); j++) {
          if (!vpMath::equal(m1[i + offset_i][j + offset_j], m2[i][j], std::numeric_limits<double>::epsilon())) {
            std::cerr << "Problem with vpMatrix insert()!" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }

      offset_i = 8, offset_j = 5;
      m1.insert(m2, offset_i, offset_j);

      for (unsigned int i = 0; i < m2.getRows(); i++) {
        for (unsigned int j = 0; j < m2.getCols(); j++) {
          if (!vpMath::equal(m1[i + offset_i][j + offset_j], m2[i][j], std::numeric_limits<double>::epsilon())) {
            std::cerr << "Problem with vpMatrix insert()!" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::juxtaposeMatrices()" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix A(5, 6), B(5, 4);
      for (unsigned int i = 0; i < A.getRows(); i++) {
        for (unsigned int j = 0; j < A.getCols(); j++) {
          A[i][j] = i * A.getCols() + j;

          if (j < B.getCols()) {
            B[i][j] = (i * B.getCols() + j) * 10;
          }
        }
      }

      vpMatrix juxtaposeM;
      vpMatrix::juxtaposeMatrices(A, B, juxtaposeM);
      std::cout << "juxtaposeM:\n" << juxtaposeM << std::endl;
    }

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- BENCHMARK dgemm/dgemv" << std::endl;
      std::cout << "------------------------" << std::endl;

      size_t nb_matrices = ctest ? 100 : 10000;
      unsigned int rows = 200, cols = 6;
      double min = -1.0, max = 1.0;
      std::vector<vpMatrix> vec_A, vec_B, vec_C, vec_C_regular;
      vec_C.reserve(nb_matrices);
      vec_C_regular.reserve(nb_matrices);

      for (size_t i = 0; i < nb_matrices; i++) {
        vec_A.push_back(generateRandomMatrix(cols, rows, min, max));
        vec_B.push_back(generateRandomMatrix(rows, cols, min, max));
      }

      double t = vpTime::measureTimeMs();
      for (size_t i = 0; i < nb_matrices; i++) {
        vec_C.push_back(vec_A[i] * vec_B[i]);
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << nb_matrices << " matrix multiplication: (6x200) x (200x6)" << std::endl;
      std::cout << "Lapack: " << t << " ms" << std::endl;
      std::cout << "vec_C:\n" << vec_C.back() << std::endl;

      t = vpTime::measureTimeMs();
      for (size_t i = 0; i < nb_matrices; i++) {
        vec_C_regular.push_back(dgemm_regular(vec_A[i], vec_B[i]));
      }
      t = vpTime::measureTimeMs() - t;
      std::cout << "\nRegular: " << t << " ms" << std::endl;
      std::cout << "vec_C_regular:\n" << vec_C_regular.back() << std::endl;

      vpMatrix A = generateRandomMatrix(480, 640, min, max), B = generateRandomMatrix(640, 480, min, max);
      vpMatrix AB, AB_regular;

      t = vpTime::measureTimeMs();
      AB = A * B;
      t = vpTime::measureTimeMs() - t;
      std::cout << "\nMatrix multiplication: (480x640) x (640x480)" << std::endl;
      std::cout << "Lapack: " << t << " ms" << std::endl;
      std::cout << "Min=" << AB.getMinValue() << " ; Max=" << AB.getMaxValue() << std::endl;

      t = vpTime::measureTimeMs();
      AB_regular = dgemm_regular(A, B);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Regular: " << t << " ms" << std::endl;
      std::cout << "Min=" << AB_regular.getMinValue() << " ; Max=" << AB_regular.getMaxValue() << std::endl;
      bool res = equalMatrix(AB, AB_regular, 1e-9);
      std::cout << "Check result: " << res << std::endl;
      if (!res) {
        std::cerr << "Problem with matrix multiplication!" << std::endl;
        return EXIT_FAILURE;
      }

      int nb_iterations = 1000;
      vpMatrix L = generateRandomMatrix(1000, 6, min, max);
      vpMatrix LTL, LTL_regular;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LTL = L.AtA();
      t = vpTime::measureTimeMs() - t;
      std::cout << "\n" << nb_iterations << " iterations of AtA for size: (1000x6)" << std::endl;
      std::cout << "Lapack: " << t << " ms" << std::endl;
      std::cout << "LTL:\n" << LTL << std::endl;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LTL_regular = AtA_regular(L);
      t = vpTime::measureTimeMs() - t;
      std::cout << "\nRegular: " << t << " ms" << std::endl;
      std::cout << "LTL_regular:\n" << LTL_regular << std::endl;
      res = equalMatrix(LTL, LTL_regular, 1e-9);
      std::cout << "Check result: " << res << std::endl;
      if (!res) {
        std::cerr << "Problem with vpMatrix::AtA()!" << std::endl;
        return EXIT_FAILURE;
      }

      vpMatrix LT = generateRandomMatrix(6, 1000, min, max);
      vpColVector R = generateRandomVector(1000, min, max);
      vpMatrix LTR, LTR_regular;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LTR = LT * R;
      t = vpTime::measureTimeMs() - t;
      std::cout << "\n"
                << nb_iterations
                << " iterations of matrix vector multiplication: (6x1000) x "
                   "(1000x1)"
                << std::endl;
      std::cout << "Lapack: " << t << " ms" << std::endl;
      std::cout << "LTR:\n" << LTR.t() << std::endl;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LTR_regular = dgemv_regular(LT, R);
      t = vpTime::measureTimeMs() - t;
      std::cout << "\nRegular: " << t << " ms" << std::endl;
      std::cout << "LTR_regular:\n" << LTR_regular.t() << std::endl;
      res = equalMatrix(LTR, LTR_regular, 1e-9);
      std::cout << "Check result: " << res << std::endl;
      if (!res) {
        std::cerr << "Problem with dgemv!" << std::endl;
        return EXIT_FAILURE;
      }

      vpVelocityTwistMatrix V(getRandomValues(min, max), getRandomValues(min, max), getRandomValues(min, max),
                              getRandomValues(min, max), getRandomValues(min, max), getRandomValues(min, max));
      vpMatrix LV, LV_regular;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LV = L * V;
      t = vpTime::measureTimeMs() - t;
      std::cout << "\n"
                << nb_iterations
                << " iterations of matrix velocity twist matrix "
                   "multiplication: (1000x6) x (6x6)"
                << std::endl;
      std::cout << "Lapack: " << t << " ms" << std::endl;

      t = vpTime::measureTimeMs();
      for (int i = 0; i < nb_iterations; i++)
        LV_regular = mat_mul_twist_matrix(L, V);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Regular: " << t << " ms" << std::endl;
      res = equalMatrix(LV, LV_regular, 1e-9);
      std::cout << "Check result: " << res << std::endl;
      if (!res) {
        std::cerr << "Problem with matrix and velocity twist matrix multiplication!" << std::endl;
        return EXIT_FAILURE;
      }
    }
#endif

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
    {
      std::vector<vpMatrix> vec_mat;
      vec_mat.emplace_back(5, 5);

      vpMatrix A(4, 4), B(4, 4);
      A = 1;
      B = 2;
      vpMatrix res = A + B;
      std::cout << "\n1) A+B:\n" << res << std::endl;

      vpMatrix res2;
      res2 = A + B;
      std::cout << "\n2) A+B:\n" << res2 << std::endl;
    }
#endif

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::hadamard()" << std::endl;
      std::cout << "------------------------" << std::endl;

      vpMatrix M1(3, 5), M2(3, 5);
      for (unsigned int i = 0; i < M1.size(); i++) {
        M1.data[i] = i;
        M2.data[i] = i + 2;
      }

      std::cout << "M1:\n" << M1 << std::endl;
      std::cout << "\nM2:\n" << M2 << std::endl;
      M2 = M1.hadamard(M2);
      std::cout << "\nRes:\n" << M2 << std::endl;
    }

    std::cout << "\nAll tests succeed" << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
