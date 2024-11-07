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
 * Benchmark matrix transpose.
 */

/*!
  \example perfMatrixTranspose.cpp
 */

#include <visp3/core/vpConfig.h>                      // for VISP_HAVE_EIGEN3

#if defined(VISP_HAVE_CATCH2)

#include <iterator>                                   // for pair
#include <new>                                        // for operator new
#include <sstream>                                    // for basic_ostream
#include <string>                                     // for char_traits
#include <utility>                                    // for pair, move
#include <vector>                                     // for vector

#include <catch_amalgamated.hpp>                      // for Benchmark, Sour...

#include <visp3/core/vpMatrix.h>                      // for vpMatrix
#include <visp3/core/vpArray2D.h>                     // for vpArray2D

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
#include <opencv2/core/hal/interface.h>               // for CV_64FC1
#include <opencv2/core/mat.hpp>                       // for Mat, MatExpr
#include <opencv2/core/mat.inl.hpp>                   // for Mat::at, MatExp...
#endif

#ifdef VISP_HAVE_EIGEN3
#include <Eigen/Dense>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{

bool g_runBenchmark = false;
int g_tileSize = 16;

vpMatrix generateMatrix(unsigned int sz1, unsigned int sz2)
{
  vpMatrix M(sz1, sz2);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = i * M.getCols() + j;
    }
  }

  return M;
}

vpMatrix generateMatrixTranspose(unsigned int sz1, unsigned int sz2)
{
  vpMatrix M(sz2, sz1);

  for (unsigned int j = 0; j < M.getCols(); j++) {
    for (unsigned int i = 0; i < M.getRows(); i++) {
      M[i][j] = j * M.getRows() + i;
    }
  }

  return M;
}

vpMatrix transposeIterateSrc(const vpMatrix &A)
{
  vpMatrix At;

  At.resize(A.getCols(), A.getRows(), false, false);

  for (unsigned int i = 0; i < A.getRows(); i++) {
    double *coli = A[i];
    for (unsigned int j = 0; j < A.getCols(); j++) {
      At[j][i] = coli[j];
    }
  }

  return At;
}

vpMatrix transposeIterateDst(const vpMatrix &A)
{
  vpMatrix At;

  At.resize(A.getCols(), A.getRows(), false, false);

  for (unsigned int j = 0; j < A.getCols(); j++) {
    double *coli = At[j];
    for (unsigned int i = 0; i < A.getRows(); i++) {
      coli[i] = A[i][j];
    }
  }

  return At;
}

vpMatrix transposeTilingSO(const vpMatrix &A, unsigned int tileSize = 16)
{
  vpMatrix At;

  At.resize(A.getCols(), A.getRows(), false, false);

  for (unsigned int i = 0; i < A.getRows(); i += tileSize) {
    for (unsigned int j = 0; j < A.getCols(); j++) {
      for (unsigned int b = 0; b < tileSize && i + b < A.getRows(); b++) {
        At[j][i + b] = A[i + b][j];
      }
    }
  }

  return At;
}

vpMatrix transposeTiling(const vpMatrix &A, int tileSize = 16)
{
  vpMatrix At;

  At.resize(A.getCols(), A.getRows(), false, false);

  const int nrows = static_cast<int>(A.getRows());
  const int ncols = static_cast<int>(A.getCols());

  for (int i = 0; i < nrows;) {
    for (; i <= nrows - tileSize; i += tileSize) {
      int j = 0;
      for (; j <= ncols - tileSize; j += tileSize) {
        for (int k = i; k < i + tileSize; k++) {
          for (int l = j; l < j + tileSize; l++) {
            At[l][k] = A[k][l];
          }
        }
      }

      for (int k = i; k < i + tileSize; k++) {
        for (int l = j; l < ncols; l++) {
          At[l][k] = A[k][l];
        }
      }
    }

    for (; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        At[j][i] = A[i][j];
      }
    }
  }

  return At;
}

} // namespace

TEST_CASE("Benchmark vpMatrix transpose", "[benchmark]")
{
  if (g_runBenchmark) {
    const std::vector<std::pair<int, int> > sizes = {
        {701, 1503}, {1791, 837}, {1201, 1201}, {1024, 1024}, {2000, 2000}, {10, 6},    {25, 6},    {100, 6},  {200, 6},
        {500, 6},    {1000, 6},   {1500, 6},    {2000, 6},    {6, 10},      {6, 25},    {6, 100},   {6, 200},  {6, 500},
        {6, 1000},   {6, 1500},   {6, 2000},    {640, 1000},  {800, 640},   {640, 500}, {500, 640}, {640, 837} };

    for (auto sz : sizes) {
      vpMatrix M = generateMatrix(sz.first, sz.second);
      vpMatrix Mt_true = generateMatrixTranspose(sz.first, sz.second);

      std::ostringstream oss;
      oss << sz.first << "x" << sz.second;
      oss << " - M.t()";
      BENCHMARK(oss.str().c_str())
      {
        vpMatrix Mt = M.t();
        REQUIRE(Mt == Mt_true);
        return Mt;
      };

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - transposeIterateSrc(M)";
      BENCHMARK(oss.str().c_str())
      {
        vpMatrix Mt = transposeIterateSrc(M);
        REQUIRE(Mt == Mt_true);
        return Mt;
      };

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - transposeIterateDst(M)";
      BENCHMARK(oss.str().c_str())
      {
        vpMatrix Mt = transposeIterateDst(M);
        REQUIRE(Mt == Mt_true);
        return Mt;
      };

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - transposeTilingSO(M, tileSize=" << g_tileSize << ")";
      BENCHMARK(oss.str().c_str())
      {
        vpMatrix Mt = transposeTilingSO(M, g_tileSize);
        REQUIRE(Mt == Mt_true);
        return Mt;
      };

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - transposeTiling(M, tileSize=" << g_tileSize << ")";
      BENCHMARK(oss.str().c_str())
      {
        vpMatrix Mt = transposeTiling(M, g_tileSize);
        REQUIRE(Mt == Mt_true);
        return Mt;
      };

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
      cv::Mat matM(sz.first, sz.second, CV_64FC1);

      for (unsigned int i = 0; i < M.getRows(); i++) {
        for (unsigned int j = 0; j < M.getCols(); j++) {
          matM.at<double>(i, j) = M[i][j];
        }
      }

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - OpenCV";
      BENCHMARK(oss.str().c_str())
      {
        cv::Mat matMt = matM.t();
        return matMt;
      };
#endif

#ifdef VISP_HAVE_EIGEN3
      Eigen::MatrixXd eigenM(sz.first, sz.second);

      for (unsigned int i = 0; i < M.getRows(); i++) {
        for (unsigned int j = 0; j < M.getCols(); j++) {
          eigenM(i, j) = M[i][j];
        }
      }

      oss.str("");
      oss << sz.first << "x" << sz.second;
      oss << " - Eigen";
      BENCHMARK(oss.str().c_str())
      {
        Eigen::MatrixXd eigenMt = eigenM.transpose();
        return eigenMt;
      };
#endif
    }
  }
  else {
    vpMatrix M = generateMatrix(11, 17);
    vpMatrix Mt_true = generateMatrixTranspose(11, 17);

    vpMatrix Mt = M.t();
    REQUIRE(Mt == Mt_true);
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  auto cli = session.cli()
    | Catch::Clara::Opt(g_runBenchmark)["--benchmark"]("run benchmark?")
    | Catch::Clara::Opt(g_tileSize, "tileSize")["--tileSize"]("Tile size?");

  session.cli(cli);
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  return numFailed;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
