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
 * Test various svd decompositions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrixPseudoInverse.cpp
  \brief Test various pseudo inverse implementations.
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

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

 */
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test matrix pseudo-inverse.\n\
Outputs a comparison of the results obtained by supported 3rd parties.\n\
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

  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = 0; j < A.getCols(); j++) {
      A[i][j] = (double)rand() / (double)RAND_MAX;
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
    vpMatrix M = make_random_matrix(nb_rows, nb_cols);
    bench.push_back(M);
  }
}

int test_pseudo_inverse(const std::vector<vpMatrix> &A, const std::vector<vpMatrix> &Api)
{
  double allowed_error = 1e-3;

  for (unsigned int i = 0; i < A.size(); i++) {
    double error = (A[i] * Api[i] * A[i] - A[i]).frobeniusNorm();
    if (error > allowed_error) {
      std::cout << "Bad pseudo-inverse [" << i << "]: euclidean norm: " << error << std::endl;
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

int test_pseudo_inverse(const std::vector<vpMatrix> &A, const std::vector<vpMatrix> &Api,
                        const std::vector<vpColVector> &sv, const std::vector<vpMatrix> &imA,
                        const std::vector<vpMatrix> &imAt, const std::vector<vpMatrix> &kerAt)
{
  double allowed_error = 1e-3;
  // test Api
  for (unsigned int i = 0; i < A.size(); i++) {
    double error = (A[i] * Api[i] * A[i] - A[i]).frobeniusNorm();
    if (error > allowed_error) {
      std::cout << "Bad pseudo-inverse [" << i << "]: euclidean norm: " << error << std::endl;
      return EXIT_FAILURE;
    }
  }

  // test kerA
  for (unsigned int i = 0; i < kerAt.size(); i++) {
    if (kerAt[i].size()) {
      vpMatrix nullspace = A[i] * kerAt[i].t();
      double error = nullspace.frobeniusNorm();

      if (error > allowed_error) {
        std::cout << "Bad kernel [" << i << "]: euclidean norm: " << error << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  // test sv, imA, imAt, kerA
  for (unsigned int i = 0; i < kerAt.size(); i++) {
    unsigned int rank = imA[i].getCols();
    vpMatrix U, S(rank, A[i].getCols()), Vt(A[i].getCols(), A[i].getCols());
    U = imA[i];

    for (unsigned int j = 0; j < rank; j++)
      S[j][j] = sv[i][j];

    Vt.insert(imAt[i].t(), 0, 0);
    Vt.insert(kerAt[i], imAt[i].getCols(), 0);

    double error = (U * S * Vt - A[i]).frobeniusNorm();

    if (error > allowed_error) {
      std::cout << "Bad imA, imAt, sv, kerAt [" << i << "]: euclidean norm: " << error << std::endl;
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}

int test_pseudo_inverse_default(bool verbose, const std::vector<vpMatrix> &bench, std::vector<double> &time)
{
  if (verbose)
    std::cout << "Test pseudo-inverse using default 3rd party" << std::endl;
  if (verbose)
    std::cout << "  Pseudo-inverse on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  size_t size = bench.size();
  std::vector<vpMatrix> PI(size), imA(size), imAt(size), kerAt(size);
  std::vector<vpColVector> sv(size);
  int ret = EXIT_SUCCESS;

  // test 0
  unsigned int test = 0;
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    PI[i] = bench[i].pseudoInverse();
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 1
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverse(PI[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 2
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverse(PI[i], sv[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 3
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverse(PI[i], sv[i], 1e-6, imA[i], imAt[i], kerAt[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;

  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI, sv, imA, imAt, kerAt);
  }

  return ret;
}

#if defined(VISP_HAVE_EIGEN3)
int test_pseudo_inverse_eigen3(bool verbose, const std::vector<vpMatrix> &bench, std::vector<double> &time)
{
  if (verbose)
    std::cout << "Test pseudo-inverse using Eigen3 3rd party" << std::endl;
  if (verbose)
    std::cout << "  Pseudo-inverse on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  size_t size = bench.size();
  std::vector<vpMatrix> PI(size), imA(size), imAt(size), kerAt(size);
  std::vector<vpColVector> sv(size);
  int ret = EXIT_SUCCESS;

  // test 0
  unsigned int test = 0;
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    PI[i] = bench[i].pseudoInverseEigen3();
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 1
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseEigen3(PI[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 2
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseEigen3(PI[i], sv[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 3
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseEigen3(PI[i], sv[i], 1e-6, imA[i], imAt[i], kerAt[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;

  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI, sv, imA, imAt, kerAt);
  }

  return ret;
}
#endif

#if defined(VISP_HAVE_LAPACK)
int test_pseudo_inverse_lapack(bool verbose, const std::vector<vpMatrix> &bench, std::vector<double> &time)
{
  if (verbose)
    std::cout << "Test pseudo-inverse using Lapack 3rd party" << std::endl;
  if (verbose)
    std::cout << "  Pseudo-inverse on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  size_t size = bench.size();
  std::vector<vpMatrix> PI(size), imA(size), imAt(size), kerAt(size);
  std::vector<vpColVector> sv(size);
  int ret = EXIT_SUCCESS;

  // test 0
  unsigned int test = 0;
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    PI[i] = bench[i].pseudoInverseLapack();
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 1
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseLapack(PI[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 2
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseLapack(PI[i], sv[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 3
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseLapack(PI[i], sv[i], 1e-6, imA[i], imAt[i], kerAt[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;

  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI, sv, imA, imAt, kerAt);
  }

  return ret;
}
#endif

#if defined(VISP_HAVE_GSL)
int test_pseudo_inverse_gsl(bool verbose, const std::vector<vpMatrix> &bench, std::vector<double> &time)
{
  if (verbose)
    std::cout << "Test pseudo-inverse using Gsl 3rd party" << std::endl;
  if (verbose)
    std::cout << "  Pseudo-inverse on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  size_t size = bench.size();
  std::vector<vpMatrix> PI(size), imA(size), imAt(size), kerAt(size);
  std::vector<vpColVector> sv(size);
  int ret = EXIT_SUCCESS;

  // test 0
  unsigned int test = 0;
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    PI[i] = bench[i].pseudoInverseGsl();
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 1
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseGsl(PI[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 2
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseGsl(PI[i], sv[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 3
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseGsl(PI[i], sv[i], 1e-6, imA[i], imAt[i], kerAt[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;

  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI, sv, imA, imAt, kerAt);
  }

  return ret;
}
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
int test_pseudo_inverse_opencv(bool verbose, const std::vector<vpMatrix> &bench, std::vector<double> &time)
{
  if (verbose)
    std::cout << "Test pseudo-inverse using OpenCV 3rd party" << std::endl;
  if (verbose)
    std::cout << "  Pseudo-inverse on a " << bench[0].getRows() << "x" << bench[0].getCols() << " matrix" << std::endl;

  size_t size = bench.size();
  std::vector<vpMatrix> PI(size), imA(size), imAt(size), kerAt(size);
  std::vector<vpColVector> sv(size);
  int ret = EXIT_SUCCESS;

  // test 0
  unsigned int test = 0;
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    PI[i] = bench[i].pseudoInverseOpenCV();
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 1
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseOpenCV(PI[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 2
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseOpenCV(PI[i], sv[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;
  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI);
  }

  // test 3
  test++;
  t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    bench[i].pseudoInverseOpenCV(PI[i], sv[i], 1e-6, imA[i], imAt[i], kerAt[i]);
  }
  time[test] = vpTime::measureTimeMs() - t;

  for (unsigned int i = 0; i < time.size(); i++) {
    ret += test_pseudo_inverse(bench, PI, sv, imA, imAt, kerAt);
  }

  return ret;
}
#endif

void save_time(const std::string &method, unsigned int nrows, unsigned int ncols, bool verbose, bool use_plot_file,
               std::ofstream &of, const std::vector<double> &time)
{
  for (size_t i = 0; i < time.size(); i++) {
    if (use_plot_file)
      of << time[i] << "\t";
    if (verbose) {
      std::cout << "  " << method << " svd(" << nrows << "x" << ncols << ")"
                << " test " << i << ": " << time[i] << std::endl;
    }
  }
}

int main(int argc, const char *argv[])
{
  try {
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK) || (VISP_HAVE_OPENCV_VERSION >= 0x020101) ||                \
    defined(VISP_HAVE_GSL)
    unsigned int nb_matrices = 10;
    unsigned int nb_iterations = 10;
    unsigned int nb_rows = 12;
    unsigned int nb_cols = 6;
    bool verbose = false;
    std::string plotfile("plot-pseudo-inv.csv");
    bool use_plot_file = false;
    std::ofstream of;

    unsigned int nb_svd_functions = 4;    // 4 tests for each existing vpMatrix::pseudoInverse(...) functions
    unsigned int nb_test_matrix_size = 3; // 3 tests: m > n, m = n, m < n
    std::vector<double> time(nb_svd_functions);
    std::vector<unsigned int> nrows(nb_test_matrix_size), ncols(nb_test_matrix_size);

    // Read the command line options
    if (getOptions(argc, argv, nb_matrices, nb_iterations, use_plot_file, plotfile, nb_rows, nb_cols, verbose) ==
        false) {
      exit(-1);
    }

    for (unsigned int s = 0; s < nb_test_matrix_size; s++) {
      // consider m > n, m = n, m < n
      if (s == 0) {
        nrows[s] = nb_rows;
        ncols[s] = nb_cols;
      } else if (s == 1) {
        nrows[s] = nb_cols;
        ncols[s] = nb_cols;
      } else {
        nrows[s] = nb_cols;
        ncols[s] = nb_rows;
      }
    }

    if (use_plot_file) {
      of.open(plotfile.c_str());
      of << "iter"
         << "\t";

      for (unsigned int s = 0; s < nb_test_matrix_size; s++) {
        for (unsigned int i = 0; i < nb_svd_functions; i++)
          of << "\"default " << nrows[s] << "x" << ncols[s] << " test " << i << "\""
             << "\t";

#if defined(VISP_HAVE_LAPACK)
        for (unsigned int i = 0; i < nb_svd_functions; i++)
          of << "\"Lapack " << nrows[s] << "x" << ncols[s] << " test " << i << "\""
             << "\t";
#endif
#if defined(VISP_HAVE_EIGEN3)
        for (unsigned int i = 0; i < nb_svd_functions; i++)
          of << "\"Eigen3 " << nrows[s] << "x" << ncols[s] << " test " << i << "\""
             << "\t";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
        for (unsigned int i = 0; i < nb_svd_functions; i++)
          of << "\"OpenCV " << nrows[s] << "x" << ncols[s] << " test " << i << "\""
             << "\t";
#endif
#if defined(VISP_HAVE_GSL)
        for (unsigned int i = 0; i < nb_svd_functions; i++)
          of << "\"GSL " << nrows[s] << "x" << ncols[s] << " test " << i << "\""
             << "\t";
#endif
      }
      of << std::endl;
    }

    int ret = EXIT_SUCCESS;
    for (unsigned int iter = 0; iter < nb_iterations; iter++) {

      if (use_plot_file)
        of << iter << "\t";

      for (unsigned int s = 0; s < nb_test_matrix_size; s++) {
        std::vector<vpMatrix> bench_random_matrices;
        create_bench_random_matrix(nb_matrices, nrows[s], ncols[s], verbose, bench_random_matrices);

        ret += test_pseudo_inverse_default(verbose, bench_random_matrices, time);
        save_time("default -", nrows[s], ncols[s], verbose, use_plot_file, of, time);

#if defined(VISP_HAVE_LAPACK)
        ret += test_pseudo_inverse_lapack(verbose, bench_random_matrices, time);
        save_time("Lapack -", nrows[s], ncols[s], verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_EIGEN3)
        ret += test_pseudo_inverse_eigen3(verbose, bench_random_matrices, time);
        save_time("Eigen3 -", nrows[s], ncols[s], verbose, use_plot_file, of, time);
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
        ret += test_pseudo_inverse_opencv(verbose, bench_random_matrices, time);
        save_time("OpenCV -", nrows[s], ncols[s], verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_GSL)
        ret += test_pseudo_inverse_gsl(verbose, bench_random_matrices, time);
        save_time("GSL -", nrows[s], ncols[s], verbose, use_plot_file, of, time);
#endif
      }
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
