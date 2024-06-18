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
 * Homography estimation.
 */

 /*!
   \file vpHomographyDLT.cpp

   This file implements the functions related with the homography
   estimation using the DLT algorithm
 */

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/vision/vpHomography.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void vpHomography::hartleyNormalization(const std::vector<double> &x, const std::vector<double> &y,
                                        std::vector<double> &xn, std::vector<double> &yn, double &xg, double &yg,
                                        double &coef)
{
  if (x.size() != y.size()) {
    throw(vpException(vpException::dimensionError, "Hartley normalization require that x and y vector "
                      "have the same dimension"));
  }

  unsigned int n = static_cast<unsigned int>(x.size());
  if (xn.size() != n) {
    xn.resize(n);
  }
  if (yn.size() != n) {
    yn.resize(n);
  }

  xg = 0;
  yg = 0;

  for (unsigned int i = 0; i < n; ++i) {
    xg += x[i];
    yg += y[i];
  }
  xg /= n;
  yg /= n;

  // Changement d'origine : le centre de gravite doit correspondre
  // a l'origine des coordonnees
  double distance = 0;
  for (unsigned int i = 0; i < n; ++i) {
    double xni = x[i] - xg;
    double yni = y[i] - yg;
    xn[i] = xni;
    yn[i] = yni;
    distance += sqrt(vpMath::sqr(xni) + vpMath::sqr(yni));
  } // for translation sur tous les points

  // Changement d'echelle
  distance /= n;
  // calcul du coef de changement d'echelle
  // if(distance ==0)
  if (std::fabs(distance) <= std::numeric_limits<double>::epsilon()) {
    coef = 1;
  }
  else {
    coef = sqrt(2.0) / distance;
  }

  for (unsigned int i = 0; i < n; ++i) {
    xn[i] *= coef;
    yn[i] *= coef;
  }
}

void vpHomography::hartleyNormalization(unsigned int n, const double *x, const double *y, double *xn, double *yn,
                                        double &xg, double &yg, double &coef)
{
  unsigned int i;
  xg = 0;
  yg = 0;

  for (i = 0; i < n; ++i) {
    xg += x[i];
    yg += y[i];
  }
  xg /= n;
  yg /= n;

  // Changement d'origine : le centre de gravite doit correspondre
  // a l'origine des coordonnees
  double distance = 0;
  for (i = 0; i < n; ++i) {
    double xni = x[i] - xg;
    double yni = y[i] - yg;
    xn[i] = xni;
    yn[i] = yni;
    distance += sqrt(vpMath::sqr(xni) + vpMath::sqr(yni));
  } // for translation sur tous les points

  // Changement d'echelle
  distance /= n;
  // calcul du coef de changement d'echelle
  // if(distance ==0)
  if (std::fabs(distance) <= std::numeric_limits<double>::epsilon()) {
    coef = 1;
  }
  else {
    coef = sqrt(2.0) / distance;
  }

  for (i = 0; i < n; ++i) {
    xn[i] *= coef;
    yn[i] *= coef;
  }
}

//---------------------------------------------------------------------------------------

void vpHomography::hartleyDenormalization(vpHomography &aHbn, vpHomography &aHb, double xg1, double yg1, double coef1,
                                          double xg2, double yg2, double coef2)
{

  // calcul des transformations a appliquer sur M_norm pour obtenir M
  // en fonction des deux normalizations effectuees au debut sur
  // les points: aHb = T2^ aHbn T1
  vpMatrix T1(3, 3);
  vpMatrix T2(3, 3);
  vpMatrix T2T(3, 3);

  T1.eye();
  T2.eye();
  T2T.eye();

  T1[0][0] = (T1[1][1] = coef1);
  T1[0][2] = -coef1 * xg1;
  T1[1][2] = -coef1 * yg1;

  T2[0][0] = (T2[1][1] = coef2);
  T2[0][2] = -coef2 * xg2;
  T2[1][2] = -coef2 * yg2;

  T2T = T2.pseudoInverse(1e-16);

  vpMatrix aHbn_(3, 3);
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      aHbn_[i][j] = aHbn[i][j];
    }
  }

  vpMatrix maHb = T2T * aHbn_ * T1;

  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      aHb[i][j] = maHb[i][j];
    }
  }
}

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

void vpHomography::DLT(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                       const std::vector<double> &ya, vpHomography &aHb, bool normalization)
{
  unsigned int n = static_cast<unsigned int>(xb.size());
  if ((yb.size() != n) || (xa.size() != n) || (ya.size() != n)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for DLT homography estimation"));
  }

  // 4 point are required
  const unsigned int nbRequiredPoints = 4;
  if (n < nbRequiredPoints) {
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));
  }

  std::vector<double> xan, yan, xbn, ybn;

  double xg1 = 0., yg1 = 0., coef1 = 0., xg2 = 0., yg2 = 0., coef2 = 0.;

  vpHomography aHbn;

  if (normalization) {
    vpHomography::hartleyNormalization(xb, yb, xbn, ybn, xg1, yg1, coef1);
    vpHomography::hartleyNormalization(xa, ya, xan, yan, xg2, yg2, coef2);
  }
  else {
    xbn = xb;
    ybn = yb;
    xan = xa;
    yan = ya;
  }

  vpMatrix A(2 * n, 9);
  vpColVector h(9);
  vpColVector D(9);
  vpMatrix V(9, 9);

  // We need here to compute the SVD on a (n*2)*9 matrix (where n is
  // the number of points). if n == 4, the matrix has more columns
  // than rows. This kind of matrix is not supported by GSL for
  // SVD. The solution is to add an extra line with zeros
  if (n == 4) {
    A.resize((2 * n) + 1, 9);
  }

  // build matrix A
  for (unsigned int i = 0; i < n; ++i) {
    A[2 * i][0] = 0;
    A[2 * i][1] = 0;
    A[2 * i][2] = 0;
    A[2 * i][3] = -xbn[i];
    A[2 * i][4] = -ybn[i];
    A[2 * i][5] = -1;
    A[2 * i][6] = xbn[i] * yan[i];
    A[2 * i][7] = ybn[i] * yan[i];
    A[2 * i][8] = yan[i];

    A[(2 * i) + 1][0] = xbn[i];
    A[(2 * i) + 1][1] = ybn[i];
    A[(2 * i) + 1][2] = 1;
    A[(2 * i) + 1][3] = 0;
    A[(2 * i) + 1][4] = 0;
    A[(2 * i) + 1][5] = 0;
    A[(2 * i) + 1][6] = -xbn[i] * xan[i];
    A[(2 * i) + 1][7] = -ybn[i] * xan[i];
    A[(2 * i) + 1][8] = -xan[i];
  }

  // Add an extra line with zero.
  if (n == 4) {
    for (unsigned int i = 0; i < 9; ++i) {
      A[2 * n][i] = 0;
    }
  }

  // solve Ah = 0
  // SVD  Decomposition A = UDV^T (destructive wrt A)
  A.svd(D, V);

  // on en profite pour effectuer un controle sur le rang de la matrice :
  // pas plus de 2 valeurs singulieres quasi=0
  int rank = 0;
  for (unsigned int i = 0; i < 9; ++i) {
    if (D[i] > 1e-7) {
      ++rank;
    }
  }
  if (rank < 7) {
    throw(vpMatrixException(vpMatrixException::rankDeficient, "Matrix rank %d is deficient (should be 8)", rank));
  }
  // h = is the column of V associated with the smallest singular value of A

  // since  we are not sure that the svd implemented sort the
  // singular value... we seek for the smallest
  double smallestSv = 1e30;
  unsigned int indexSmallestSv = 0;
  for (unsigned int i = 0; i < 9; ++i) {
    if (D[i] < smallestSv) {
      smallestSv = D[i];
      indexSmallestSv = i;
    }
  }

  h = V.getCol(indexSmallestSv);

  // build the homography
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      aHbn[i][j] = h[(3 * i) + j];
    }
  }

  if (normalization) {
    // H after denormalization
    vpHomography::hartleyDenormalization(aHbn, aHb, xg1, yg1, coef1, xg2, yg2, coef2);
  }
  else {
    aHb = aHbn;
  }
}

END_VISP_NAMESPACE
