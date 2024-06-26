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
 * Benchmark matrix multiplication.
 */

/*!
  \example perfMatrixMultiplication.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpMatrix.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
#include <opencv2/core.hpp>
#endif

#ifdef VISP_HAVE_EIGEN3
#include <Eigen/Dense>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{

bool runBenchmark = false;
bool runBenchmarkAll = false;

double getRandomValues(double min, double max) { return (max - min) * ((double)rand() / (double)RAND_MAX) + min; }

vpMatrix generateRandomMatrix(unsigned int rows, unsigned int cols, double min = -1, double max = 1)
{
  vpMatrix M(rows, cols);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = getRandomValues(min, max);
    }
  }

  return M;
}

vpColVector generateRandomVector(unsigned int rows, double min = -1, double max = 1)
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

  if ((A.getRows() != C.getRows()) || (B.getCols() != C.getCols())) {
    C.resize(A.getRows(), B.getCols(), false);
  }

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
      for (k = 0; k < BrowNum; k++) {
        s += rowptri[k] * B[k][j];
      }
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

  for (unsigned int i = 0; i < A.getCols(); i++) {
    double *Bi = B[i];
    for (unsigned int j = 0; j < i; j++) {
      double *ptr = A.data;
      double s = 0;
      for (unsigned int k = 0; k < A.getRows(); k++) {
        s += (*(ptr + i)) * (*(ptr + j));
        ptr += A.getCols();
      }
      *Bi++ = s;
      B[j][i] = s;
    }
    double *ptr = A.data;
    double s = 0;
    for (unsigned int k = 0; k < A.getRows(); k++) {
      s += (*(ptr + i)) * (*(ptr + i));
      ptr += A.getCols();
    }
    *Bi = s;
  }

  return B;
}

// Copy of vpMatrix::AAt()
vpMatrix AAt_regular(const vpMatrix &A)
{
  vpMatrix B;
  B.resize(A.getRows(), A.getRows(), false);

  // compute A*A^T
  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = i; j < A.getRows(); j++) {
      double *pi = A[i]; // row i
      double *pj = A[j]; // row j

      // sum (row i .* row j)
      double ssum = 0;
      for (unsigned int k = 0; k < A.getCols(); k++)
        ssum += *(pi++) * *(pj++);

      B[i][j] = ssum; // upper triangle
      if (i != j)
        B[j][i] = ssum; // lower triangle
    }
  }
  return B;
}

// Copy of vpMatrix::multMatrixVector
vpColVector dgemv_regular(const vpMatrix &A, const vpColVector &v)
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

bool equalMatrix(const vpMatrix &A, const vpMatrix &B, double tol = 1e-9)
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

} // namespace

TEST_CASE("Benchmark matrix-matrix multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {3, 3},   {6, 6},     {8, 8},    {10, 10},   {20, 20},  {6, 200},
                                               {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpMatrix B = generateRandomMatrix(sz.second, sz.first);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      vpMatrix C, C_true;
      BENCHMARK(oss.str().c_str())
      {
        C_true = dgemm_regular(A, B);
        return C_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        C = A * B;
        return C;
      };
      REQUIRE(equalMatrix(C, C_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matB(sz.second, sz.first, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
            matB.at<double>(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(" << matB.rows << "x" << matB.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matC = matA * matB;
          return matC;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenB(sz.second, sz.first);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
            eigenB(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols()
          << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenC = eigenA * eigenB;
          return eigenC;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 63;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpMatrix B = generateRandomMatrix(cols, rows);

    vpMatrix C_true = dgemm_regular(A, B);
    vpMatrix C = A * B;
    REQUIRE(equalMatrix(C, C_true));
  }
}

TEST_CASE("Benchmark matrix-rotation matrix multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {3, 3} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpRotationMatrix B(vpMath::deg(getRandomValues(0, 360)), vpMath::deg(getRandomValues(0, 360)),
                         vpMath::deg(getRandomValues(0, 360)));

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      vpMatrix AB, AB_true;
      BENCHMARK(oss.str().c_str())
      {
        AB_true = dgemm_regular(A, static_cast<vpMatrix>(B));
        return AB_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AB = A * B;
        return AB;
      };
      REQUIRE(equalMatrix(AB, AB_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matB(3, 3, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < B.getRows(); i++) {
          for (unsigned int j = 0; j < B.getCols(); j++) {
            matB.at<double>(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(" << matB.rows << "x" << matB.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matC = matA * matB;
          return matC;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenB(3, 3);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < B.getRows(); i++) {
          for (unsigned int j = 0; j < B.getCols(); j++) {
            eigenB(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols()
          << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenC = eigenA * eigenB;
          return eigenC;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 3, cols = 3;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpRotationMatrix B(vpMath::deg(getRandomValues(0, 360)), vpMath::deg(getRandomValues(0, 360)),
                       vpMath::deg(getRandomValues(0, 360)));

    vpMatrix AB_true = dgemm_regular(A, static_cast<vpMatrix>(B));
    vpMatrix AB = A * B;
    REQUIRE(equalMatrix(AB, AB_true));
  }
}

TEST_CASE("Benchmark matrix-homogeneous matrix multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {4, 4} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpHomogeneousMatrix B(getRandomValues(0, 1), getRandomValues(0, 1), getRandomValues(0, 1),
                            vpMath::deg(getRandomValues(0, 360)), vpMath::deg(getRandomValues(0, 360)),
                            vpMath::deg(getRandomValues(0, 360)));

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      vpMatrix AB, AB_true;
      BENCHMARK(oss.str().c_str())
      {
        AB_true = dgemm_regular(A, static_cast<vpMatrix>(B));
        return AB_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AB = A * B;
        return AB;
      };
      REQUIRE(equalMatrix(AB, AB_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matB(4, 4, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < B.getRows(); i++) {
          for (unsigned int j = 0; j < B.getCols(); j++) {
            matB.at<double>(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(" << matB.rows << "x" << matB.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matC = matA * matB;
          return matC;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenB(4, 4);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < B.getRows(); i++) {
          for (unsigned int j = 0; j < B.getCols(); j++) {
            eigenB(j, i) = B[j][i];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols()
          << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenC = eigenA * eigenB;
          return eigenC;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 4, cols = 4;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpHomogeneousMatrix B(getRandomValues(0, 1), getRandomValues(0, 1), getRandomValues(0, 1),
                          vpMath::deg(getRandomValues(0, 360)), vpMath::deg(getRandomValues(0, 360)),
                          vpMath::deg(getRandomValues(0, 360)));

    vpMatrix AB_true = dgemm_regular(A, static_cast<vpMatrix>(B));
    vpMatrix AB;
    vpMatrix::mult2Matrices(A, static_cast<vpMatrix>(B), AB);
    REQUIRE(equalMatrix(AB, AB_true));
  }
}

TEST_CASE("Benchmark matrix-vector multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {3, 3},   {6, 6},     {8, 8},    {10, 10},   {20, 20},  {6, 200},
                                               {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpColVector B = generateRandomVector(sz.second);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      vpColVector C, C_true;
      BENCHMARK(oss.str().c_str())
      {
        C_true = dgemv_regular(A, static_cast<vpColVector>(B));
        return C_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        C = A * B;
        return C;
      };
      REQUIRE(equalMatrix(static_cast<vpMatrix>(C), static_cast<vpMatrix>(C_true)));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matB(sz.second, 1, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
            if (i == 0) {
              matB.at<double>(j, 0) = B[j];
            }
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(" << matB.rows << "x" << matB.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matC = matA * matB;
          return matC;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenB(sz.second, 1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
            if (i == 0) {
              eigenB(j, 0) = B[j];
            }
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols()
          << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenC = eigenA * eigenB;
          return eigenC;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 63;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpColVector B = generateRandomVector(cols);

    vpColVector C_true = dgemv_regular(A, B);
    vpColVector C = A * B;
    REQUIRE(equalMatrix(static_cast<vpMatrix>(C), static_cast<vpMatrix>(C_true)));
  }
}

TEST_CASE("Benchmark AtA", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {3, 3},   {6, 6},     {8, 8},    {10, 10},   {20, 20},  {6, 200},
                                               {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - Naive code";
      vpMatrix AtA, AtA_true;
      BENCHMARK(oss.str().c_str())
      {
        AtA_true = AtA_regular(A);
        return AtA_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AtA = A.AtA();
        return AtA;
      };
      REQUIRE(equalMatrix(AtA, AtA_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matAtA = matA.t() * matA;
          return matAtA;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenAtA = eigenA.transpose() * eigenA;
          return eigenAtA;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 63;
    vpMatrix A = generateRandomMatrix(rows, cols);

    vpMatrix AtA_true = AtA_regular(A);
    vpMatrix AtA = A.AtA();
    REQUIRE(equalMatrix(AtA, AtA_true));
  }
}

TEST_CASE("Benchmark AAt", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = {
        {3, 3},   {6, 6},   {8, 8},  {10, 10},
        {20, 20}, {6, 200}, {200, 6} }; //, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - Naive code";
      vpMatrix AAt_true, AAt;
      BENCHMARK(oss.str().c_str())
      {
        AAt_true = AAt_regular(A);
        return AAt_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AAt = A.AAt();
        return AAt;
      };
      REQUIRE(equalMatrix(AAt, AAt_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ") - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matAAt = matA * matA.t();
          return matAAt;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ") - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenAAt = eigenA * eigenA.transpose();
          return eigenAAt;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 63;
    vpMatrix A = generateRandomMatrix(rows, cols);

    vpMatrix AAt_true = AAt_regular(A);
    vpMatrix AAt = A.AAt();
    REQUIRE(equalMatrix(AAt, AAt_true));
  }
}

TEST_CASE("Benchmark matrix-velocity twist multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {6, 6}, {20, 6}, {207, 6}, {600, 6}, {1201, 6} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpVelocityTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - Naive code";
      vpMatrix AV, AV_true;
      BENCHMARK(oss.str().c_str())
      {
        AV_true = dgemm_regular(A, static_cast<vpMatrix>(V));
        return AV_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AV = A * V;
        return AV;
      };
      REQUIRE(equalMatrix(AV, AV_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matV(6, 6, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < V.getRows(); i++) {
          for (unsigned int j = 0; j < V.getCols(); j++) {
            matV.at<double>(i, j) = V[i][j];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(6x6) - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matAV = matA * matV;
          return matAV;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenV(6, 6);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < V.getRows(); i++) {
          for (unsigned int j = 0; j < V.getCols(); j++) {
            eigenV(i, j) = V[i][j];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(6x6) - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenAV = eigenA * eigenV;
          return eigenAV;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 6;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpVelocityTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

    vpMatrix AV_true = dgemm_regular(A, static_cast<vpMatrix>(V));
    vpMatrix AV = A * V;
    REQUIRE(equalMatrix(AV, AV_true));
  }
}

TEST_CASE("Benchmark matrix-force twist multiplication", "[benchmark]")
{
  if (runBenchmark || runBenchmarkAll) {
    std::vector<std::pair<int, int> > sizes = { {6, 6}, {20, 6}, {207, 6}, {600, 6}, {1201, 6} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpForceTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - Naive code";
      vpMatrix AV, AV_true;
      BENCHMARK(oss.str().c_str())
      {
        AV_true = dgemm_regular(A, static_cast<vpMatrix>(V));
        return AV_true;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - ViSP";
      BENCHMARK(oss.str().c_str())
      {
        AV = A * V;
        return AV;
      };
      REQUIRE(equalMatrix(AV, AV_true));

      if (runBenchmarkAll) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
        cv::Mat matA(sz.first, sz.second, CV_64FC1);
        cv::Mat matV(6, 6, CV_64FC1);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            matA.at<double>(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < V.getRows(); i++) {
          for (unsigned int j = 0; j < V.getCols(); j++) {
            matV.at<double>(i, j) = V[i][j];
          }
        }

        oss.str("");
        oss << "(" << matA.rows << "x" << matA.cols << ")x(6x6) - OpenCV";
        BENCHMARK(oss.str().c_str())
        {
          cv::Mat matAV = matA * matV;
          return matAV;
        };
#endif

#ifdef VISP_HAVE_EIGEN3
        Eigen::MatrixXd eigenA(sz.first, sz.second);
        Eigen::MatrixXd eigenV(6, 6);

        for (unsigned int i = 0; i < A.getRows(); i++) {
          for (unsigned int j = 0; j < A.getCols(); j++) {
            eigenA(i, j) = A[i][j];
          }
        }
        for (unsigned int i = 0; i < V.getRows(); i++) {
          for (unsigned int j = 0; j < V.getCols(); j++) {
            eigenV(i, j) = V[i][j];
          }
        }

        oss.str("");
        oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(6x6) - Eigen";
        BENCHMARK(oss.str().c_str())
        {
          Eigen::MatrixXd eigenAV = eigenA * eigenV;
          return eigenAV;
        };
#endif
      }
    }
  }

  {
    const unsigned int rows = 47, cols = 6;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpForceTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

    vpMatrix AV_true = dgemm_regular(A, static_cast<vpMatrix>(V));
    vpMatrix AV = A * V;
    REQUIRE(equalMatrix(AV, AV_true));
  }
}

int main(int argc, char *argv[])
{
  // Set random seed explicitly to avoid confusion
  // See: https://en.cppreference.com/w/cpp/numeric/random/srand
  // If rand() is used before any calls to srand(), rand() behaves as if it was seeded with srand(1).
  srand(1);

  Catch::Session session; // There must be exactly one instance
  unsigned int lapackMinSize = vpMatrix::getLapackMatrixMinSize();

  std::cout << "Default matrix/vector min size to enable Blas/Lapack optimization: " << lapackMinSize << std::endl;
  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()         // Get Catch's composite command line parser
    | Opt(runBenchmark)   // bind variable to a new option, with a hint string
    ["--benchmark"] // the option names it will respond to
    ("run benchmark comparing naive code with ViSP implementation") // description string for the help output
    | Opt(runBenchmarkAll)    // bind variable to a new option, with a hint string
    ["--benchmark-all"] // the option names it will respond to
    ("run benchmark comparing naive code with ViSP, OpenCV, Eigen implementation") // description string for
                                                                                   // the help output
    | Opt(lapackMinSize, "min size") // bind variable to a new option, with a hint string
    ["--lapack-min-size"]      // the option names it will respond to
    ("matrix/vector min size to enable blas/lapack usage"); // description string for the help output

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  vpMatrix::setLapackMatrixMinSize(lapackMinSize);
  std::cout << "Used matrix/vector min size to enable Blas/Lapack optimization: " << vpMatrix::getLapackMatrixMinSize()
    << std::endl;

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
