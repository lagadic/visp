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
 * Test some vpMatrix functionalities.
 */

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

#include <iterator> // for std::back_inserter

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
bool test_memory(unsigned int nrows, unsigned int ncols, const vpMatrix &M, const std::string &matrix_name, bool pointer_is_null)
{
  if (pointer_is_null) {
    if (M.data) {
      std::cerr << "Wrong data pointer (" << M.data << ") in matrix " << matrix_name << ": should be null" << std::endl;
    }
  }
  else {
    if (!M.data) {
      std::cerr << "Wrong data pointer (" << M.data << ") in matrix " << matrix_name << ": should be non null" << std::endl;
      return false;
    }
  }

  if (M.getRows() != nrows || M.getCols() != ncols) {
    std::cerr << "Wrong matrix " << matrix_name << "(" << nrows << ", " << ncols << " size: "
      << M.getRows() << " x " << M.getCols() << std::endl;
    return false;
  }
  std::cout << "Test matrix " << matrix_name << " succeed" << std::endl;
  return true;
}

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

double getRandomValues(double min, double max) { return (max - min) * ((double)rand() / (double)RAND_MAX) + min; }

bool equalMatrix(const vpMatrix &A, const vpMatrix &B, double tol = std::numeric_limits<double>::epsilon())
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

vpMatrix generateRandomMatrix(unsigned int rows, unsigned int cols, double min, double max)
{
  vpMatrix M(rows, cols);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = getRandomValues(min, max);
    }
  }

  return M;
}

std::vector<double> computeHadamard(const std::vector<double> &v1, const std::vector<double> &v2)
{
  std::vector<double> result;
  std::transform(v1.begin(), v1.end(), v2.begin(), std::back_inserter(result), std::multiplies<double>());
  return result;
}
} // namespace

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
      unsigned int nrows = 2, ncols = 3;
      vpMatrix A(nrows, ncols);
      if (test_memory(nrows, ncols, A, "A", false) == false) {
        return EXIT_FAILURE;
      }
      vpMatrix B, C;
      if (test_memory(0, 0, B, "B", true) == false) {
        return EXIT_FAILURE;
      }
      if (test_memory(0, 0, C, "C", true)== false) {
        return EXIT_FAILURE;
      }
      B = A;
      if (test_memory(nrows, ncols, B, "B", false)== false) {
        return EXIT_FAILURE;
      }
      B = C;
      if (test_memory(0, 0, C, "C", true)== false) {
        return EXIT_FAILURE;
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
      if (test("R1", static_cast<vpMatrix>(R1), bench) == false)
        return EXIT_FAILURE;
      vpRotationMatrix R2;
      R2 = M;
      if (test("R2", static_cast<vpMatrix>(R2), bench) == false)
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
      vpMatrix M3 = static_cast<vpMatrix>(R);
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
      std::cout << M << std::endl;
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
      M[0][2] = -0.0000876;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;

      std::cout << "call M.print (std::cout, 4);" << std::endl;
      M.print(std::cout, 4);
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 6, \"M\");" << std::endl;
      M.print(std::cout, 6, "M");
      std::cout << std::endl;
      std::cout << "call M.print (std::cout, 10, \"M\");" << std::endl;
      M.print(std::cout, 10, "M");
      std::cout << std::endl;

      M.resize(2, 3);
      M[0][0] = -1;
      M[0][1] = -2;
      M[0][2] = -3;
      M[1][0] = 4;
      M[1][1] = 5.5;
      M[1][2] = 6.0f;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;
      std::cout << "call M.print (std::cout, 5, \"M\");" << std::endl;
      M.print(std::cout, 5, "M");
      std::cout << std::endl;

      M.resize(2, 3);
      M[0][0] = -1;
      M[0][1] = -2;
      M[0][2] = -3;
      M[1][0] = 4;
      M[1][1] = 5.;
      M[1][2] = 6;
      std::cout << "call std::cout << M;" << std::endl;
      std::cout << M << std::endl;
      std::cout << "call M.print (std::cout, 5, \"M\");" << std::endl;
      M.print(std::cout, 5, "M");
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
      unsigned int nb = ctest ? 10 : 100; // 10000;
      const unsigned int size = ctest ? 10 : 100;

      vpMatrix m_big(nb *size, 6);
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
      std::cout << "m_big_stack_static: " << m_big_stack_static.getRows() << "x" << m_big_stack_static.getCols()
        << std::endl;

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

      offset_i = 4;
      offset_j = 5;
      m1.insert(m2, offset_i, offset_j);

      for (unsigned int i = 0; i < m2.getRows(); i++) {
        for (unsigned int j = 0; j < m2.getCols(); j++) {
          if (!vpMath::equal(m1[i + offset_i][j + offset_j], m2[i][j], std::numeric_limits<double>::epsilon())) {
            std::cerr << "Problem with vpMatrix insert()!" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }

      offset_i = 8;
      offset_j = 5;
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
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

      // Reference
      std::vector<double> references = computeHadamard(std::vector<double>(M1.data, M1.data + M1.size()),
                                                       std::vector<double>(M2.data, M2.data + M2.size()));

      std::cout << "M1:\n" << M1 << std::endl;
      std::cout << "\nM2:\n" << M2 << std::endl;
      M2 = M1.hadamard(M2);
      std::cout << "\nRes:\n" << M2 << std::endl;

      if (!test("M2", M2, references)) {
        std::cerr << "Error with Hadamard product" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::stackColums()" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix M(3, 5);
      for (unsigned int j = 0; j < M.getCols(); j++) {
        for (unsigned int i = 0; i < M.getRows(); i++) {
          M[i][j] = i + j * M.getRows();
        }
      }
      std::cout << "M:\n" << M << std::endl;
      vpColVector v = M.stackColumns();
      std::cout << "Column stack: " << v.t() << std::endl;
      if (M.size() != v.size()) {
        std::cerr << "Problem in vpMatrix::stackColumns(): size differ" << std::endl;
        return EXIT_FAILURE;
      }
      for (unsigned int i = 0; i < v.size(); i++) {
        if (std::fabs(v[i] - static_cast<double>(i)) > std::numeric_limits<double>::epsilon()) {
          std::cerr << "Problem in vpMatrix::stackColumns(): content differ" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::stackRows()" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix M(3, 5);
      for (unsigned int i = 0; i < M.getRows(); i++) {
        for (unsigned int j = 0; j < M.getCols(); j++) {
          M[i][j] = i * M.getCols() + j;
        }
      }
      std::cout << "M:\n" << M << std::endl;
      vpRowVector v = M.stackRows();
      std::cout << "Rows stack: " << v << std::endl;
      if (M.size() != v.size()) {
        std::cerr << "Problem in vpMatrix::stackRows(): size differ" << std::endl;
        return EXIT_FAILURE;
      }
      for (unsigned int i = 0; i < v.size(); i++) {
        if (std::fabs(v[i] - static_cast<double>(i)) > std::numeric_limits<double>::epsilon()) {
          std::cerr << "Problem in vpMatrix::stackRows(): content differ" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::getCol()" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix A(4, 4);
      for (unsigned int i = 0; i < A.getRows(); i++)
        for (unsigned int j = 0; j < A.getCols(); j++)
          A[i][j] = i * A.getCols() + j;

      {
        vpColVector cv = A.getCol(1, 1, 3);
        vpColVector ref;
        ref << 5, 9, 13;
        if (cv != ref) {
          std::cerr << "Problem in vpMatrix::getCol(): values are different" << std::endl;
          return EXIT_FAILURE;
        }
      }
      {
        vpColVector cv = A.getCol(1);
        vpColVector ref;
        ref << 1, 5, 9, 13;
        if (cv != ref) {
          std::cerr << "Problem in vpMatrix::getCol(): values are different" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::getRow()" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix A(4, 4);
      for (unsigned int i = 0; i < A.getRows(); i++)
        for (unsigned int j = 0; j < A.getCols(); j++)
          A[i][j] = i * A.getCols() + j;

      {
        vpRowVector rv = A.getRow(1, 1, 3);
        vpRowVector ref;
        ref << 5, 6, 7;
        if (rv != ref) {
          std::cerr << "Problem in vpMatrix::getRow(): values are different" << std::endl;
          return EXIT_FAILURE;
        }
      }
      {
        vpRowVector rv = A.getRow(1);
        vpRowVector ref;
        ref << 4, 5, 6, 7;
        if (rv != ref) {
          std::cerr << "Problem in vpMatrix::getRow(): values are different" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    {
      std::cout << "\n------------------------" << std::endl;
      std::cout << "--- TEST vpMatrix::getDiag()" << std::endl;
      std::cout << "------------------------" << std::endl;
      vpMatrix A(3, 4);
      for (unsigned int i = 0; i < A.getRows(); i++)
        for (unsigned int j = 0; j < A.getCols(); j++)
          A[i][j] = i * A.getCols() + j;

      vpColVector diag = A.getDiag();
      vpColVector ref;
      ref << 0.0, 5.0, 10.0;
      if (diag != ref) {
        std::cerr << "Problem in vpMatrix::getDiag(): values are different" << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::cout << "\nAll tests succeeded" << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
