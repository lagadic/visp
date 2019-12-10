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
 * Benchmark matrix multiplication.
 *
 *****************************************************************************/

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

namespace
{

bool runBenchmark = false;

double getRandomValues(double min, double max)
{
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}

vpMatrix generateRandomMatrix(unsigned int rows, unsigned int cols, double min=-1, double max=1)
{
  vpMatrix M(rows, cols);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = getRandomValues(min, max);
    }
  }

  return M;
}

vpColVector generateRandomVector(unsigned int rows, double min=-1, double max=1)
{
  vpColVector v(rows);

  for (unsigned int i = 0; i < v.getRows(); i++) {
    v[i] = getRandomValues(min, max);
  }

  return v;
}

// Copy of vpMatrix::mult2Matrices
vpMatrix dgemm_regular(const vpMatrix& A, const vpMatrix& B)
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
vpMatrix AtA_regular(const vpMatrix& A)
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
vpColVector dgemv_regular(const vpMatrix& A, const vpColVector& v)
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
vpMatrix mat_mul_twist_matrix(const vpMatrix& A, const vpVelocityTwistMatrix& V)
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

bool equalMatrix(const vpMatrix& A, const vpMatrix& B, double tol=1e-9)
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

}

TEST_CASE("Benchmark matrix-matrix multiplication", "[benchmark]") {
  if (runBenchmark) {
    std::vector<std::pair<int, int>> sizes = { {6, 200}, {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpMatrix B = generateRandomMatrix(sz.second, sz.first);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix C = dgemm_regular(A, B);
        return C;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix C = A * B;
        return C;
      };
      {
        vpMatrix C_true = dgemm_regular(A, B);
        vpMatrix C = A * B;
        REQUIRE(equalMatrix(C, C_true));
      }

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
      BENCHMARK(oss.str().c_str()) {
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
      oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols() << ") - Eigen";
      BENCHMARK(oss.str().c_str()) {
        Eigen::MatrixXd eigenC = eigenA * eigenB;
        return eigenC;
      };
#endif
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

TEST_CASE("Benchmark matrix-vector multiplication", "[benchmark]") {
  if (runBenchmark) {
    std::vector<std::pair<int, int>> sizes = { {6, 200}, {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpColVector B = generateRandomVector(sz.second);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - Naive code";
      BENCHMARK(oss.str().c_str()) {
        vpColVector C = dgemv_regular(A, B);
        return C;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(" << B.getRows() << "x" << B.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str()) {
        vpColVector C = A * B;
        return C;
      };
      {
        vpColVector C_true = dgemv_regular(A, B);
        vpColVector C = A * B;
        REQUIRE(equalMatrix(C, C_true));
      }

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
      BENCHMARK(oss.str().c_str()) {
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
      oss << "(" << eigenA.rows() << "x" << eigenA.cols() << ")x(" << eigenB.rows() << "x" << eigenB.cols() << ") - Eigen";
      BENCHMARK(oss.str().c_str()) {
        Eigen::MatrixXd eigenC = eigenA * eigenB;
        return eigenC;
      };
#endif
    }
  }

  {
    const unsigned int rows = 47, cols = 63;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpColVector B = generateRandomVector(cols);

    vpColVector C_true = dgemv_regular(A, B);
    vpColVector C = A * B;
    REQUIRE(equalMatrix(C, C_true));
  }
}

TEST_CASE("Benchmark AtA", "[benchmark]") {
  if (runBenchmark) {
    std::vector<std::pair<int, int>> sizes = { {6, 200}, {200, 6}, {207, 119}, {83, 201}, {600, 400}, {400, 600} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - Naive code";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix AtA = AtA_regular(A);
        return AtA;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ") - ViSP";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix AtA = A.AtA();
        return AtA;
      };
      {
        vpMatrix AtA_true = AtA_regular(A);
        vpMatrix AtA = A.AtA();
        REQUIRE(equalMatrix(AtA, AtA_true));
      }

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
      cv::Mat matA(sz.first, sz.second, CV_64FC1);

      for (unsigned int i = 0; i < A.getRows(); i++) {
        for (unsigned int j = 0; j < A.getCols(); j++) {
          matA.at<double>(i, j) = A[i][j];
        }
      }

      oss.str("");
      oss << "(" << matA.rows << "x" << matA.cols << ") - OpenCV";
      BENCHMARK(oss.str().c_str()) {
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
      BENCHMARK(oss.str().c_str()) {
        Eigen::MatrixXd eigenAtA = eigenA.transpose() * eigenA;
        return eigenAtA;
      };
#endif
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

TEST_CASE("Benchmark matrix-velocity twist multiplication", "[benchmark]") {
  if (runBenchmark) {
    std::vector<std::pair<int, int>> sizes = { {20, 6}, {207, 6}, {600, 6}, {1201, 6} };

    for (auto sz : sizes) {
      vpMatrix A = generateRandomMatrix(sz.first, sz.second);
      vpVelocityTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

      std::ostringstream oss;
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - Naive code";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix AV = mat_mul_twist_matrix(A, V);
        return AV;
      };

      oss.str("");
      oss << "(" << A.getRows() << "x" << A.getCols() << ")x(6x6) - ViSP";
      BENCHMARK(oss.str().c_str()) {
        vpMatrix AV = A * V;
        return AV;
      };
      {
        vpMatrix AV_true = mat_mul_twist_matrix(A, V);
        vpMatrix AV = A * V;
        REQUIRE(equalMatrix(AV, AV_true));
      }

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
      BENCHMARK(oss.str().c_str()) {
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
      BENCHMARK(oss.str().c_str()) {
        Eigen::MatrixXd eigenAV = eigenA * eigenV;
        return eigenAV;
      };
#endif
    }
  }

  {
    const unsigned int rows = 47, cols = 6;
    vpMatrix A = generateRandomMatrix(rows, cols);
    vpVelocityTwistMatrix V(vpTranslationVector(0.1, -0.4, 1.5), vpThetaUVector(0.4, -0.1, 0.7));

    vpMatrix AV_true = mat_mul_twist_matrix(A, V);
    vpMatrix AV = A * V;
    REQUIRE(equalMatrix(AV, AV_true));
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli() // Get Catch's composite command line parser
    | Opt(runBenchmark)    // bind variable to a new option, with a hint string
    ["--benchmark"]        // the option names it will respond to
    ("run benchmark?");    // description string for the help output

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

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
  return 0;
}
#endif
