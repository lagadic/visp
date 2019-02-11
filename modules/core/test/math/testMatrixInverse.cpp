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
 * Test various inversions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrixInverse.cpp
  \brief Test various matrix inversions.
*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "cdn:i:pf:R:C:vh"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

 */
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test matrix inversions\n\
using LU, QR and Cholesky methods as well as Pseudo-inverse.\n\
Outputs a comparison of these methods.\n\
\n\
SYNOPSIS\n\
  %s [-n <number of matrices>] [-f <plot filename>]\n\
     [-R <number of rows>] [-C <number of columns>]\n\
     [-i <number of iterations>] [-p] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -n <number of matrices>                               \n\
     Number of matrices inverted during each test loop.\n\
\n\
  -i <number of iterations>                               \n\
     Number of iterations of the test.\n\
\n\
  -f <plot filename>                               \n\
     Set output path for plot output.\n\
     The plot logs the times of \n\
     the different inversion methods: \n\
     QR,LU,Cholesky and Pseudo-inverse.\n\
\n\
  -R <number of rows>\n\
     Number of rows of the automatically generated matrices  \n\
     we test on.\n\
\n\
  -C <number of columns>\n\
     Number of colums of the automatically generated matrices  \n\
     we test on.\n\
\n\
  -p                                             \n\
     Plot into filename in the gnuplot format. \n\
     If this option is used, tests results will be logged \n\
     into a filename specified with -f.\n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, unsigned int &nb_matrices, unsigned int &nb_iterations,
                bool &use_plot_file, std::string &plotfile, unsigned int &nbrows, unsigned int &nbcols, bool &verbose)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;
    case 'n':
      nb_matrices = (unsigned int)atoi(optarg_);
      break;
    case 'i':
      nb_iterations = (unsigned int)atoi(optarg_);
      break;
    case 'f':
      plotfile = optarg_;
      use_plot_file = true;
      break;
    case 'p':
      use_plot_file = true;
      break;
    case 'R':
      nbrows = (unsigned int)atoi(optarg_);
      break;
    case 'C':
      nbcols = (unsigned int)atoi(optarg_);
      break;
    case 'v':
      verbose = true;
      break;
    // add default options -c -d
    case 'c':
      break;
    case 'd':
      break;
    default:
      usage(argv[0], optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

vpMatrix make_random_matrix(unsigned int nbrows, unsigned int nbcols)
{
  vpMatrix A;
  A.resize(nbrows, nbcols);

  for (unsigned int i = 0; i < A.getRows(); i++)
    for (unsigned int j = 0; j < A.getCols(); j++)
      A[i][j] = (double)rand() / (double)RAND_MAX;
  return A;
}

vpMatrix make_random_symmetric_positive_matrix(unsigned int n)
{
  vpMatrix A;
  A.resize(n, n);
  vpMatrix I;
  I.eye(n);

  for (unsigned int i = 0; i < A.getRows(); i++)
    for (unsigned int j = 0; j < A.getCols(); j++)
      A[i][j] = (double)rand() / (double)RAND_MAX;

  A = 0.5 * (A + A.t());
  A = A + n * I;
  return A;
}

void create_bench_random_matrix(unsigned int nb_matrices, unsigned int nb_rows, unsigned int nb_cols, bool verbose,
                                std::vector<vpMatrix> &bench)
{
  if (verbose)
    std::cout << "Create a bench of " << nb_matrices << " " << nb_rows << " by " << nb_cols << " matrices" << std::endl;
  bench.clear();
  for (unsigned int i = 0; i < nb_matrices; i++) {
    vpMatrix M;
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL)
    double det = 0.;
    // don't put singular matrices in the benchmark
    for (M = make_random_matrix(nb_rows, nb_cols); std::fabs(det = M.AtA().det()) < .01;
         M = make_random_matrix(nb_rows, nb_cols)) {
      if (verbose) {
        std::cout << "  Generated random matrix AtA=" << std::endl << M.AtA() << std::endl;
        std::cout << "  Generated random matrix not invertible: det=" << det << ". Retrying..." << std::endl;
      }
    }
#else
    M = make_random_matrix(nb_rows, nb_cols);
#endif
    bench.push_back(M);
  }
}

void create_bench_symmetric_positive_matrix(unsigned int nb_matrices, unsigned int n, bool verbose,
                                            std::vector<vpMatrix> &bench)
{
  if (verbose)
    std::cout << "Create a bench of " << nb_matrices << " " << n << " by " << n << " symmetric positive matrices"
              << std::endl;
  bench.clear();
  for (unsigned int i = 0; i < nb_matrices; i++) {
    vpMatrix M;
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL)
    double det = 0.;
    // don't put singular matrices in the benchmark
    for (M = make_random_symmetric_positive_matrix(n); std::fabs(det = M.det()) < .01;
         M = make_random_symmetric_positive_matrix(n)) {
      if (verbose) {
        std::cout << "  Generated random symmetric positive matrix A=" << std::endl << M << std::endl;
        std::cout << "  Generated random symmetric positive matrix not "
                     "invertibleL: det="
                  << det << ". Retrying..." << std::endl;
      }
    }
#else
    M = make_random_symmetric_positive_matrix(n);
#endif
    bench.push_back(M);
  }
}

int test_inverse(const std::vector<vpMatrix> &bench, const std::vector<vpMatrix> &result)
{
  for (unsigned int i = 0; i < bench.size(); i++) {
    vpMatrix I = bench[i] * result[i];
    if (std::fabs(I.frobeniusNorm() - sqrt((double)bench[0].AtA().getRows())) > 1e-10) {
      std::cout << "Bad inverse[" << i << "]: " << I.frobeniusNorm() << " " << sqrt((double)bench[0].AtA().getRows())
                << std::endl;
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

#if defined(VISP_HAVE_EIGEN3)
int test_inverse_lu_eigen3(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by LU using Eigen3 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using LU decomposition (Eigen3)." << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByLUEigen3() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}
#endif

#if defined(VISP_HAVE_LAPACK)
int test_inverse_lu_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by LU using Lapack 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using LU decomposition (Lapack)." << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByLULapack() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}

int test_inverse_cholesky_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by Cholesky using Lapack 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using cholesky decomposition (Lapack)." << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByCholeskyLapack() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}

int test_inverse_qr_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by QR using Lapack 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using QR decomposition (Lapack)" << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByQRLapack() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}
#endif

#if defined(VISP_HAVE_GSL)
int test_inverse_lu_gsl(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by LU using GSL 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using LU decomposition (GSL)" << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByLUGsl() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
int test_inverse_lu_opencv(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by LU using OpenCV 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using LU decomposition (OpenCV)" << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByLUOpenCV() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}

int test_inverse_cholesky_opencv(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test inverse by Cholesky using OpenCV 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols()
              << " matrix using Cholesky decomposition (OpenCV)" << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().inverseByCholeskyOpenCV() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}
#endif

#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL)
// SVD is only available for these 3rd parties
int test_pseudo_inverse(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test pseudo inverse using either Eigen3, Lapack, OpenCV or "
                 "GSL 3rd party"
              << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Pseudo inverting " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << " matrix"
              << std::endl;
  std::vector<vpMatrix> result(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().pseudoInverse() * bench[i].transpose();
  }
  time = vpTime::measureTimeMs() - t;

  // Test inverse
  return test_inverse(bench, result);
}
#endif

void save_time(const std::string &method, bool verbose, bool use_plot_file, std::ofstream &of, double time)
{
  if (use_plot_file)
    of << time << "\t";
  if (verbose || !use_plot_file) {
    std::cout << method << time << std::endl;
  }
}

int main(int argc, const char *argv[])
{
  try {
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL) || defined(VISP_HAVE_MKL)
    unsigned int nb_matrices = 1000;
    unsigned int nb_iterations = 10;
    unsigned int nb_rows = 6;
    unsigned int nb_cols = 6;
    bool verbose = false;
    std::string plotfile("plot-inv.csv");
    bool use_plot_file = false;
    std::ofstream of;

    // Read the command line options
    if (getOptions(argc, argv, nb_matrices, nb_iterations, use_plot_file, plotfile, nb_rows, nb_cols, verbose) ==
        false) {
      exit(-1);
    }

    if (use_plot_file) {
      of.open(plotfile.c_str());
      of << "iter"
         << "\t";

#if defined(VISP_HAVE_LAPACK)
      of << "\"LU Lapack\""
         << "\t";
#endif
#if defined(VISP_HAVE_EIGEN3)
      of << "\"LU Eigen3\""
         << "\t";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      of << "\"LU OpenCV\""
         << "\t";
#endif
#if defined(VISP_HAVE_GSL)
      of << "\"LU GSL\""
         << "\t";
#endif

#if defined(VISP_HAVE_LAPACK)
      of << "\"Cholesky Lapack\""
         << "\t";
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      of << "\"Cholesky OpenCV\""
         << "\t";
#endif

#if defined(VISP_HAVE_LAPACK)
      of << "\"QR Lapack\""
         << "\t";
#endif

#if defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) || defined(VISP_HAVE_GSL)
      of << "\"Pseudo inverse (Lapack, OpenCV, GSL)\""
         << "\t";
#endif
      of << std::endl;
    }

    int ret = EXIT_SUCCESS;
    for (unsigned int iter = 0; iter < nb_iterations; iter++) {
      std::vector<vpMatrix> bench_random_matrices;
      create_bench_random_matrix(nb_matrices, nb_rows, nb_cols, verbose, bench_random_matrices);
      std::vector<vpMatrix> bench_symmetric_positive_matrices;
      create_bench_symmetric_positive_matrix(nb_matrices, nb_rows, verbose, bench_symmetric_positive_matrices);

      if (use_plot_file)
        of << iter << "\t";

      double time;
// LU decomposition
#if defined(VISP_HAVE_LAPACK)
      ret += test_inverse_lu_lapack(verbose, bench_random_matrices, time);
      save_time("Inverse by LU (Lapack): ", verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_EIGEN3)
      ret += test_inverse_lu_eigen3(verbose, bench_random_matrices, time);
      save_time("Inverse by LU (Eigen3): ", verbose, use_plot_file, of, time);
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      ret += test_inverse_lu_opencv(verbose, bench_random_matrices, time);
      save_time("Inverse by LU (OpenCV): ", verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_GSL)
      ret += test_inverse_lu_gsl(verbose, bench_random_matrices, time);
      save_time("Inverse by LU (GSL): ", verbose, use_plot_file, of, time);
#endif

// Cholesky for symmetric positive matrices
#if defined(VISP_HAVE_LAPACK)
      ret += test_inverse_cholesky_lapack(verbose, bench_symmetric_positive_matrices, time);
      save_time("Inverse by Cholesly (Lapack): ", verbose, use_plot_file, of, time);
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      ret += test_inverse_cholesky_opencv(verbose, bench_symmetric_positive_matrices, time);
      save_time("Inverse by Cholesky (OpenCV): ", verbose, use_plot_file, of, time);
#endif

// QR decomposition
#if defined(VISP_HAVE_LAPACK)
      ret += test_inverse_qr_lapack(verbose, bench_random_matrices, time);
      save_time("Inverse by QR (Lapack): ", verbose, use_plot_file, of, time);
#endif

// Pseudo-inverse with SVD
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL)
      ret += test_pseudo_inverse(verbose, bench_random_matrices, time);
      save_time("Pseudo inverse (Lapack, Eigen3, OpenCV or GSL): ", verbose, use_plot_file, of, time);
#endif

      if (use_plot_file)
        of << std::endl;
    }
    if (use_plot_file) {
      of.close();
      std::cout << "Result saved in " << plotfile << std::endl;
    }

    if (ret == EXIT_SUCCESS) {
      std::cout << "Test succeed" << std::endl;
    } else {
      std::cout << "Test failed" << std::endl;
    }

    return ret;
#else
    (void)argc;
    (void)argv;
    std::cout << "Test does nothing since you dont't have Eigen3, Lapack, "
                 "OpenCV or GSL 3rd party"
              << std::endl;
    return EXIT_SUCCESS;
#endif
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
