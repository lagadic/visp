/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Filip Novotny
 *
 *****************************************************************************/


/*!
  \example testMatrixInverse.cpp
  \brief Test various matrix inversions.
*/



#include <visp3/core/vpTime.h>

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/io/vpParseArgv.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cmath>
// List of allowed command line options
#define GETOPTARGS	"cdn:i:pf:R:C:vh"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv,
                unsigned int& nb_matrices, unsigned int& nb_iterations,
                bool& use_plot_file, std::string& plotfile,
                unsigned int& nbrows, unsigned int& nbcols, bool& verbose);
void writeTime(const char *name, double time);
vpMatrix makeRandomMatrix(unsigned int nbrows, unsigned int nbcols);

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
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }

}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv,
                unsigned int& nb_matrices, unsigned int& nb_iterations,
                bool& use_plot_file, std::string& plotfile,
                unsigned int& nbrows, unsigned int& nbcols, bool& verbose)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;
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
      return false; break;
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

void writeTime(const char *name, double time)
{
  std::cout << name << " time=" << time << " ms." << std::endl;
}

vpMatrix makeRandomMatrix(unsigned int nbrows, unsigned int nbcols)
{
  vpMatrix A;
  A.resize(nbrows,nbcols);

  for (unsigned int i=0 ; i < A.getRows() ; i++)
    for  (unsigned int j=0 ; j < A.getCols() ; j++)
      A[i][j] =  (double)rand()/(double)RAND_MAX;
  return A;
}


int
main(int argc, const char ** argv)
{
#ifdef VISP_HAVE_LAPACK_C
  try {
    unsigned int nb_matrices=1000;
    unsigned int nb_iterations=10;
    unsigned int nb_rows = 6;
    unsigned int nb_cols = 6;
    bool verbose = false;
    std::string plotfile("plot.txt");
    bool use_plot_file=false;
    std::ofstream of;

    double t, qr_time, lu_time,pi_time,chol_time;
    // Read the command line options
    if (getOptions(argc, argv, nb_matrices,nb_iterations,use_plot_file,plotfile,nb_rows,nb_cols,verbose) == false) {
      exit (-1);
    }

    if(use_plot_file){
      of.open(plotfile.c_str());
    }

    for(unsigned int iter=0;iter<nb_iterations;iter++){
      std::vector<vpMatrix> benchQR;
      std::vector<vpMatrix> benchLU;
      std::vector<vpMatrix> benchCholesky;
      std::vector<vpMatrix> benchPseudoInverse;
      if(verbose)
        std::cout << "********* generating matrices for iteration " << iter << "." << std::endl;
      for(unsigned int i=0;i<nb_matrices;i++){
        vpMatrix cur;
        double det=0.;
        //don't put singular matrices in the benchmark
        for(cur=makeRandomMatrix(nb_rows,nb_cols);std::abs(det=cur.AtA().det())<.01;cur = makeRandomMatrix(nb_rows,nb_cols))
          if(verbose){
            std::cout << "Generated random matrix A*tA=" << std::endl << cur.AtA() << std::endl;
            std::cout << "generated random matrix not invertibleL: det="<<det<< ". Retrying..." << std::endl;
          }
        benchCholesky.push_back(cur);
        benchQR.push_back(cur);
        benchLU.push_back(cur);
        benchPseudoInverse.push_back(cur);
      }

      if(verbose)
        std::cout << "\t Inverting " << benchCholesky[0].AtA().getRows() << "x" << benchCholesky[0].AtA().getCols() << " matrix using cholesky decomposition." << std::endl;
      t = vpTime::measureTimeMs() ;
      for(unsigned int i=0;i<nb_matrices;i++){
        benchCholesky[i]=benchCholesky[i].AtA().inverseByCholesky()*benchCholesky[i].transpose();
      }
      chol_time = vpTime::measureTimeMs() - t ;

      if(verbose)
        std::cout << "\t Inverting " << benchLU[0].AtA().getRows() << "x" << benchLU[0].AtA().getCols() << " matrix using LU decomposition." << std::endl;
      t = vpTime::measureTimeMs() ;
      for(unsigned int i=0;i<nb_matrices;i++)
        benchLU[i] = benchLU[i].AtA().inverseByLU()*benchLU[i].transpose();
      lu_time = vpTime::measureTimeMs() -t ;

      if(verbose)
        std::cout << "\t Inverting " << benchQR[0].AtA().getRows() << "x" << benchQR[0].AtA().getCols() << " matrix using QR decomposition." << std::endl;
      t = vpTime::measureTimeMs() ;
      for(unsigned int i=0;i<nb_matrices;i++){
        benchQR[i]=benchQR[i].AtA().inverseByQR()*benchQR[i].transpose();
      }
      qr_time = vpTime::measureTimeMs() - t ;

      if(verbose)
        std::cout << "\t Inverting " << benchPseudoInverse[0].AtA().getRows() << "x" << benchPseudoInverse[0].AtA().getCols() << " matrix while computing pseudo-inverse." << std::endl;
      t = vpTime::measureTimeMs() ;
      for(unsigned int i=0;i<nb_matrices;i++){
        benchPseudoInverse[i]=benchPseudoInverse[i].pseudoInverse();
      }
      pi_time = vpTime::measureTimeMs() - t ;

      double avg_err_lu_qr=0.;
      double avg_err_lu_pi=0.;
      double avg_err_lu_chol=0.;
      double avg_err_qr_pi=0.;
      double avg_err_qr_chol=0.;
      double avg_err_pi_chol=0.;

      for(unsigned int i=0;i<nb_matrices;i++){
        avg_err_lu_qr+= (benchQR[i]-benchLU[i]).euclideanNorm();
        avg_err_lu_pi+= (benchPseudoInverse[i]-benchLU[i]).euclideanNorm();
        avg_err_qr_pi+= (benchPseudoInverse[i]-benchQR[i]).euclideanNorm();
        avg_err_qr_chol+= (benchCholesky[i]-benchQR[i]).euclideanNorm();
        avg_err_lu_chol+= (benchCholesky[i]-benchLU[i]).euclideanNorm();
        avg_err_pi_chol+= (benchCholesky[i]-benchPseudoInverse[i]).euclideanNorm();
      }

      avg_err_lu_qr/=nb_matrices;
      avg_err_lu_pi/=nb_matrices;
      avg_err_qr_pi/=nb_matrices;

      if(use_plot_file){
        of << iter << "\t" << lu_time << "\t" << qr_time << "\t" << pi_time << "\t" << chol_time << "\t" << avg_err_lu_qr << "\t" << avg_err_qr_pi << "\t" << avg_err_lu_pi << "\t" << avg_err_qr_chol << "\t" << avg_err_lu_chol << "\t" <<  avg_err_pi_chol << std::endl;
      }
      if(verbose || !use_plot_file){
        writeTime("LU",lu_time);
        writeTime("QR",qr_time);
        writeTime("Pseudo-inverse",pi_time);
        writeTime("Cholesky",chol_time);
      }
    }
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }

#else
  (void)argc;
  (void)argv;
  std::cout << "You don't have lapack installed" << std::endl;
  return 0;
#endif
}

