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
 * Test various determinant computation methods.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testMatrixDeterminant.cpp
  \brief Test matrix determinant computation.
*/

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

void create_bench(unsigned int nb_matrices, unsigned int nb_rows, unsigned int nb_cols, bool verbose,
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

void test_det_default(bool verbose, const std::vector<vpMatrix> &bench, double &time, std::vector<double> &result)
{
  if (verbose)
    std::cout << "Test determinant using default method" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Matrix size: " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << std::endl;

  result.resize(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().det();
  }
  time = vpTime::measureTimeMs() - t;
}

#if defined(VISP_HAVE_EIGEN3)
void test_det_eigen3(bool verbose, const std::vector<vpMatrix> &bench, double &time, std::vector<double> &result)
{
  if (verbose)
    std::cout << "Test determinant using Eigen3 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Matrix size: " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << std::endl;

  result.resize(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().detByLUEigen3();
  }
  time = vpTime::measureTimeMs() - t;
}
#endif

#if defined(VISP_HAVE_GSL)
void test_det_gsl(bool verbose, const std::vector<vpMatrix> &bench, double &time, std::vector<double> &result)
{
  if (verbose)
    std::cout << "Test determinant using GSL 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Matrix size: " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << std::endl;

  result.resize(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().detByLUGsl();
  }
  time = vpTime::measureTimeMs() - t;
}
#endif

#if defined(VISP_HAVE_LAPACK)
void test_det_lapack(bool verbose, const std::vector<vpMatrix> &bench, double &time, std::vector<double> &result)
{
  if (verbose)
    std::cout << "Test determinant using Lapack 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Matrix size: " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << std::endl;

  result.resize(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().detByLULapack();
  }
  time = vpTime::measureTimeMs() - t;
}
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
void test_det_opencv(bool verbose, const std::vector<vpMatrix> &bench, double &time, std::vector<double> &result)
{
  if (verbose)
    std::cout << "Test determinant using OpenCV 3rd party" << std::endl;
  // Compute inverse
  if (verbose)
    std::cout << "  Matrix size: " << bench[0].AtA().getRows() << "x" << bench[0].AtA().getCols() << std::endl;

  result.resize(bench.size());
  double t = vpTime::measureTimeMs();
  for (unsigned int i = 0; i < bench.size(); i++) {
    result[i] = bench[i].AtA().detByLUOpenCV();
  }
  time = vpTime::measureTimeMs() - t;
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
#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_GSL) || defined(VISP_HAVE_LAPACK) ||                                \
    (VISP_HAVE_OPENCV_VERSION >= 0x020101)
    unsigned int nb_matrices = 1000;
    unsigned int nb_iterations = 10;
    unsigned int nb_rows = 6;
    unsigned int nb_cols = 6;
    bool verbose = false;
    std::string plotfile("plot-det.csv");
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

      of << "\"Determinant default\""
         << "\t";

#if defined(VISP_HAVE_LAPACK)
      of << "\"Determinant Lapack\""
         << "\t";
#endif
#if defined(VISP_HAVE_EIGEN3)
      of << "\"Determinant Eigen3\""
         << "\t";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      of << "\"Determinant OpenCV\""
         << "\t";
#endif
#if defined(VISP_HAVE_GSL)
      of << "\"Determinant GSL\""
         << "\t";
#endif
      of << std::endl;
    }

    int ret = EXIT_SUCCESS;
    for (unsigned int iter = 0; iter < nb_iterations; iter++) {
      std::vector<vpMatrix> bench;
      create_bench(nb_matrices, nb_rows, nb_cols, verbose, bench);

      if (use_plot_file)
        of << iter << "\t";

      double time;

      std::vector<double> result_default;
      test_det_default(verbose, bench, time, result_default);
      save_time("Determinant default: ", verbose, use_plot_file, of, time);

#if defined(VISP_HAVE_LAPACK)
      std::vector<double> result_lapack;
      test_det_lapack(verbose, bench, time, result_lapack);
      save_time("Determinant by Lapack: ", verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_EIGEN3)
      std::vector<double> result_eigen3;
      test_det_eigen3(verbose, bench, time, result_eigen3);
      save_time("Determinant by Eigen3: ", verbose, use_plot_file, of, time);
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      std::vector<double> result_opencv;
      test_det_opencv(verbose, bench, time, result_opencv);
      save_time("Determinant by OpenCV: ", verbose, use_plot_file, of, time);
#endif

#if defined(VISP_HAVE_GSL)
      std::vector<double> result_gsl;
      test_det_gsl(verbose, bench, time, result_gsl);
      save_time("Determinant by GSL: ", verbose, use_plot_file, of, time);
#endif

      if (use_plot_file)
        of << std::endl;

#if defined(VISP_HAVE_GSL) && (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_gsl[i] - result_opencv[i]) > 1e-6) {
          std::cout << "Determinant differ between GSL and OpenCV: " << result_gsl[i] << " " << result_opencv[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
#if defined(VISP_HAVE_GSL) && defined(VISP_HAVE_LAPACK)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_gsl[i] - result_lapack[i]) > 1e-6) {
          std::cout << "Determinant differ between GSL and Lapack: " << result_gsl[i] << " " << result_lapack[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
#if defined(VISP_HAVE_GSL) && defined(VISP_HAVE_EIGEN3)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_gsl[i] - result_eigen3[i]) > 1e-6) {
          std::cout << "Determinant differ between GSL and Eigen3: " << result_gsl[i] << " " << result_eigen3[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
#if defined(VISP_HAVE_LAPACK) && (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_lapack[i] - result_opencv[i]) > 1e-6) {
          std::cout << "Determinant differ between Lapack and OpenCV: " << result_lapack[i] << " " << result_opencv[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
#if defined(VISP_HAVE_EIGEN3) && (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_eigen3[i] - result_opencv[i]) > 1e-6) {
          std::cout << "Determinant differ between Eigen3 and OpenCV: " << result_eigen3[i] << " " << result_opencv[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
#if defined(VISP_HAVE_EIGEN3) && defined(VISP_HAVE_LAPACK)
      // Compare results
      for (unsigned int i = 0; i < bench.size(); i++) {
        if (std::fabs(result_eigen3[i] - result_lapack[i]) > 1e-6) {
          std::cout << "Determinant differ between Eigen3 and Lapack: " << result_eigen3[i] << " " << result_lapack[i]
                    << std::endl;
          ret = EXIT_FAILURE;
        }
      }
#endif
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
