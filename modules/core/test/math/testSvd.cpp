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
 * Test various svd decompositions.
 */

/*!
  \example testSvd.cpp
  \brief Test various svd decompositions.
*/

#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "cdn:i:pf:R:C:vh"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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
     [-i <number of iterations>] [-p] [-h]\n",
          name);

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
      usage(argv[0], nullptr);
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
    usage(argv[0], nullptr);
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

  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = 0; j < A.getCols(); j++) {
      A[i][j] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }
  }

  return A;
}

vpMatrix make_random_symmetric_matrix(unsigned int nbrows)
{
  vpMatrix A;
  A.resize(nbrows, nbrows);

  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = i; j < A.getCols(); j++) {
      A[i][j] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
      if (i != j) {
        A[j][i] = A[i][j];
      }
    }
  }

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
    //#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) ||
    //(VISP_HAVE_OPENCV_VERSION >= 0x020101)
    //    double det = 0.;
    //    // don't put singular matrices in the benchmark
    //    for(M = make_random_matrix(nb_rows, nb_cols);
    //    std::fabs(det=M.AtA().det())<.01; M = make_random_matrix(nb_rows,
    //    nb_cols)) {
    //      if(verbose) {
    //        std::cout << "  Generated random matrix AtA=" << std::endl <<
    //        M.AtA() << std::endl; std::cout << "  Generated random matrix
    //        not invertible: det=" << det << ". Retrying..." << std::endl;
    //      }
    //    }
    //#else
    M = make_random_matrix(nb_rows, nb_cols);
    //#endif
    bench.push_back(M);
  }
}

void create_bench_random_symmetric_matrix(unsigned int nb_matrices, unsigned int nb_rows, bool verbose,
                                          std::vector<vpMatrix> &bench)
{
  if (verbose)
    std::cout << "Create a bench of " << nb_matrices << " " << nb_rows << " by " << nb_rows << " symmetric matrices" << std::endl;
  bench.clear();
  for (unsigned int i = 0; i < nb_matrices; i++) {
    vpMatrix M;
    //#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) ||
    //(VISP_HAVE_OPENCV_VERSION >= 0x020101) || defined(VISP_HAVE_GSL)
    //    double det = 0.;
    //    // don't put singular matrices in the benchmark
    //    for(M = make_random_matrix(nb_rows, nb_cols);
    //    std::fabs(det=M.AtA().det())<.01; M = make_random_matrix(nb_rows,
    //    nb_cols)) {
    //      if(verbose) {
    //        std::cout << "  Generated random matrix AtA=" << std::endl <<
    //        M.AtA() << std::endl; std::cout << "  Generated random matrix
    //        not invertible: det=" << det << ". Retrying..." << std::endl;
    //      }
    //    }
    //#else
    M = make_random_symmetric_matrix(nb_rows);
    //#endif
    bench.push_back(M);
  }
}

int test_svd(std::vector<vpMatrix> M, std::vector<vpMatrix> U, std::vector<vpColVector> s, std::vector<vpMatrix> V, double &error)
{
  for (unsigned int i = 0; i < M.size(); i++) {
    vpMatrix S;
    S.diag(s[i]);
    vpMatrix U_S_Vt = U[i] * S * V[i].t();
    vpMatrix D = M[i] - U_S_Vt;
    error = D.frobeniusNorm();
    if (error > 1e-6) {
      std::cout << "SVD decomposition failed. Error: " << error << std::endl;
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

int test_eigen_values(std::vector<vpMatrix> M, std::vector<vpColVector> e, std::vector<vpMatrix> V,
                      std::vector<vpColVector> e2)
{
  for (unsigned int i = 0; i < M.size(); i++) {
    vpColVector error_e = e[i] - e2[i];
    if (error_e.frobeniusNorm() > 1e-6) {
      std::cout << "Eigen values differ" << std::endl;
      return EXIT_FAILURE;
    }
    vpMatrix D;
    D.diag(e[i]);
    vpMatrix MV_VD = M[i] * V[i] - V[i] * D;
    if (MV_VD.frobeniusNorm() > 1e-6) {
      std::cout << "Eigen values/vector decomposition failed" << std::endl;
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

#if defined(VISP_HAVE_EIGEN3)
int test_svd_eigen3(bool verbose, const std::vector<vpMatrix> &bench, double &time, double &error)
{
  if (verbose)
    std::cout << "Test SVD using Eigen3 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  SVD on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  std::vector<vpMatrix> U = bench;
  std::vector<vpMatrix> V(bench.size());
  std::vector<vpColVector> s(bench.size());

  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    U[i].svdEigen3(s[i], V[i]);
  }

  time = vpTime::measureTimeMs() - t;

  return test_svd(bench, U, s, V, error);
}
#endif

#if defined(VISP_HAVE_LAPACK)
int test_svd_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time, double &error)
{
  if (verbose)
    std::cout << "Test SVD using Lapack 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  SVD on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  std::vector<vpMatrix> U = bench;
  std::vector<vpMatrix> V(bench.size());
  std::vector<vpColVector> s(bench.size());

  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    U[i].svdLapack(s[i], V[i]);
  }
  time = vpTime::measureTimeMs() - t;

  return test_svd(bench, U, s, V, error);
}

int test_eigen_values_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time)
{
  if (verbose)
    std::cout << "Test eigenValues() using Lapack 3rd party" << std::endl;

  std::vector<vpColVector> e(bench.size());
  std::vector<vpColVector> e2(bench.size());
  std::vector<vpMatrix> V(bench.size());

  for (unsigned int i = 0; i < bench.size(); i++) {
    e2[i] = bench[i].eigenValues();
  }

  // Compute the eigenvalues and eigenvectors
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].eigenValues(e[i], V[i]);
  }
  time = vpTime::measureTimeMs() - t;

  return test_eigen_values(bench, e, V, e2);
}
#endif

#if defined(VISP_HAVE_OPENCV)
int test_svd_opencv(bool verbose, const std::vector<vpMatrix> &bench, double &time, double &error)
{
  if (verbose)
    std::cout << "Test SVD using OpenCV 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  SVD on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  std::vector<vpMatrix> U = bench;
  std::vector<vpMatrix> V(bench.size());
  std::vector<vpColVector> s(bench.size());

  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    U[i].svdOpenCV(s[i], V[i]);
  }
  time = vpTime::measureTimeMs() - t;

  return test_svd(bench, U, s, V, error);
}
#endif

void save_time(const std::string &method, bool verbose, bool use_plot_file, std::ofstream &of, double time, double error)
{
  if (use_plot_file)
    of << time << "\t";
  if (verbose || !use_plot_file) {
    std::cout << method << "took " << time << "s, error = " << error << std::endl;
  }
}

bool testAllSvds(const std::string &test_name, unsigned nb_matrices, unsigned nb_iterations,
                 unsigned nb_rows, unsigned nb_cols,
                 bool doEigenValues, bool verbose, bool use_plot_file, std::ofstream &of)
{
  int ret = EXIT_SUCCESS;
  int ret_test = 0;
  for (unsigned int iter = 0; iter < nb_iterations; iter++) {
    std::cout << "\n-> Iteration: " << iter << std::endl;
    std::vector<vpMatrix> bench_random_matrices;
    create_bench_random_matrix(nb_matrices, nb_rows, nb_cols, verbose, bench_random_matrices);
    std::vector<vpMatrix> bench_random_symmetric_matrices;
    create_bench_random_symmetric_matrix(nb_matrices, nb_rows, verbose, bench_random_symmetric_matrices);

    if (use_plot_file)
      of << test_name << iter << "\t";
    double time;
    double error;

#if defined(VISP_HAVE_LAPACK)
    std::cout << "\n-- Test SVD using lapack" << std::endl;
    ret_test = test_svd_lapack(verbose, bench_random_matrices, time, error);
    ret += ret_test;
    std::cout << test_name << ": SVD (Lapack) " << (ret_test ? "failed" : "succeed") << std::endl;
    save_time("SVD (Lapack): ", verbose, use_plot_file, of, time, error);
#endif

#if defined(VISP_HAVE_EIGEN3)
    std::cout << "\n-- Test SVD using eigen" << std::endl;
    ret_test = test_svd_eigen3(verbose, bench_random_matrices, time, error);
    ret += ret_test;
    std::cout << test_name <<  ": SVD (Eigen) " << (ret_test ? "failed" : "succeed") << std::endl;
    save_time("SVD (Eigen3): ", verbose, use_plot_file, of, time, error);
#endif

#if defined(VISP_HAVE_OPENCV)
    std::cout << "\n-- Test SVD using OpenCV" << std::endl;
    ret_test = test_svd_opencv(verbose, bench_random_matrices, time, error);
    ret += ret_test;
    std::cout << test_name << ": SVD (OpenCV) " << (ret_test ? "failed" : "succeed") << std::endl;
    save_time("SVD (OpenCV): ", verbose, use_plot_file, of, time, error);
#endif

#if defined(VISP_HAVE_LAPACK)
    if (doEigenValues) {
      std::cout << "\n-- Test Eigen Values using lapack" << std::endl;
      ret_test = test_eigen_values_lapack(verbose, bench_random_symmetric_matrices, time);
      ret += ret_test;
      std::cout << "Eigen values (Lapack) " << (ret_test ? "failed" : "succeed") << std::endl;
      error = 0.0;
      save_time("Eigen values (Lapack): ", verbose, use_plot_file, of, time, error);
    }
#endif
    std::cout << "Result after iteration " << iter << ": " << (ret ? "failed" : "succeed") << std::endl;
    if (use_plot_file)
      of << std::endl;
  }
  return (ret == EXIT_SUCCESS);
}

int main(int argc, const char *argv[])
{
  try {
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV)
    unsigned int nb_matrices = 100;
    unsigned int nb_iterations = 10;
    unsigned int nb_rows = 6;
    unsigned int nb_cols = 6;
    bool verbose = false;
    std::string plotfile("plot-svd.csv");
    bool use_plot_file = false;
    std::ofstream of;

    // Read the command line options
    if (getOptions(argc, argv, nb_matrices, nb_iterations, use_plot_file, plotfile, nb_rows, nb_cols, verbose) ==
        false) {
      return EXIT_FAILURE;
    }

    if (use_plot_file) {
      of.open(plotfile.c_str());
      of << "iter"
        << "\t";

#if defined(VISP_HAVE_LAPACK)
      of << "\"SVD Lapack\""
        << "\t";
#endif
#if defined(VISP_HAVE_EIGEN3)
      of << "\"SVD Eigen3\""
        << "\t";
#endif
#if defined(VISP_HAVE_OPENCV)
      of << "\"SVD OpenCV\""
        << "\t";
#endif
      of << std::endl;
    }
    bool success = true;
    std::string test_case;
    test_case = "Test case: Square matrices";
    std::cout << "\n== " << test_case << ": " << nb_rows << " x " << nb_cols << " ==" << std::endl;
    bool defaultSuccess = testAllSvds(test_case, nb_matrices, nb_iterations, nb_rows, nb_cols,
                                      true, verbose, use_plot_file, of);
    std::cout << "=> " << test_case << ": " << (defaultSuccess ? "succeed" : "failed") << std::endl;

    test_case = "Test case: More rows than columns";
    std::cout << "\n== " << test_case << ": " << nb_cols * 2 << " x " << nb_cols << " ==" << std::endl;
    bool rowsSuccess = testAllSvds(test_case, nb_matrices, nb_iterations, nb_cols * 2, nb_cols,
                                   false, verbose, use_plot_file, of);
    std::cout << "=> " << test_case << ": " << (rowsSuccess ? "succeed" : "failed") << std::endl;

    test_case = "Test case: More columns than rows";
    std::cout << "\n== " << test_case << ": " << nb_rows << " x " << nb_rows * 2 << " ==" << std::endl;
    bool colsSuccess = testAllSvds(test_case, nb_matrices, nb_iterations, nb_rows, nb_rows * 2,
                false, verbose, use_plot_file, of);
    std::cout << "=> " << test_case << ": " << (colsSuccess ? "succeed" : "failed") << std::endl;

    std::cout << "\nResume:" << std::endl;
    std::cout << "- Square matrices (" << nb_rows << "x" << nb_cols << "): " << (defaultSuccess ? "succeed" : "failed") << std::endl;

    std::cout << "- More rows case (" << nb_cols * 2 << "x" << nb_cols << "): " << (rowsSuccess ? "succeed" : "failed") << std::endl;

    std::cout << "- More columns case (" << nb_rows << "x" << nb_rows * 2 << "): " << (colsSuccess ? "succeed" : "failed") << std::endl;

    success = defaultSuccess && rowsSuccess && colsSuccess;

    if (use_plot_file) {
      of.close();
      std::cout << "Result saved in " << plotfile << std::endl;
    }

    if (success) {
      std::cout << "Test succeed" << std::endl;
    }
    else {
      std::cout << "Test failed" << std::endl;
    }

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
#else
    (void)argc;
    (void)argv;
    std::cout << "Test does nothing since you dont't have Lapack, Eigen3 or OpenCV 3rd party" << std::endl;
    return EXIT_SUCCESS;
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
