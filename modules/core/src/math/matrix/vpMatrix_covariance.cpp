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
 * Covariance matrix computation.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <cmath>  // std::fabs()
#include <limits> // numeric_limits

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpTranslationVector.h>

/*!
  Compute the covariance matrix of the parameters x from a least squares
  minimisation defined as: Ax = b

  \param A : Matrix A from Ax = b.

  \param x : Vector x from Ax = b corresponding to the parameters to estimate.

  \param b : Vector b from Ax = b.
*/
vpMatrix vpMatrix::computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b)
{
  //  double denom = ((double)(A.getRows()) - (double)(A.getCols())); // To
  //  consider OLS Estimate for sigma
  double denom = ((double)(A.getRows())); // To consider MLE Estimate for sigma

  if (denom <= std::numeric_limits<double>::epsilon())
    throw vpMatrixException(vpMatrixException::divideByZeroError,
                            "Impossible to compute covariance matrix: not enough data");

  //  double sigma2 = ( ((b.t())*b) - ( (b.t())*A*x ) ); // Should be
  //  equivalent to line bellow.
  double sigma2 = (b - (A * x)).t() * (b - (A * x));

  sigma2 /= denom;

  return (A.t() * A).pseudoInverse(A.getCols() * std::numeric_limits<double>::epsilon()) * sigma2;
}

/*!
  Compute the covariance matrix of the parameters x from a least squares
  minimisation defined as: WAx = Wb

  \param A : Matrix A from WAx = Wb.

  \param x : Vector x from WAx = Wb corresponding to the parameters to
  estimate.

  \param b : Vector b from WAx = Wb.

  \param W : Diagonal weigths matrix from WAx = Wb.
*/
vpMatrix vpMatrix::computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b,
                                           const vpMatrix &W)
{
  double denom = 0.0;
  vpMatrix W2(W.getCols(), W.getCols());
  for (unsigned int i = 0; i < W.getCols(); i++) {
    denom += W[i][i];
    W2[i][i] = W[i][i] * W[i][i];
  }

  if (denom <= std::numeric_limits<double>::epsilon())
    throw vpMatrixException(vpMatrixException::divideByZeroError,
                            "Impossible to compute covariance matrix: not enough data");

  //  double sigma2 = ( ((W*b).t())*W*b - ( ((W*b).t())*W*A*x ) ); // Should
  //  be equivalent to line bellow.
  double sigma2 = (W * b - (W * A * x)).t() * (W * b - (W * A * x));
  sigma2 /= denom;

  return (A.t() * (W2)*A).pseudoInverse(A.getCols() * std::numeric_limits<double>::epsilon()) * sigma2;
}

/*!
  Compute the covariance matrix of an image-based virtual visual servoing.
  This assumes the optimization has been done via v = Ls.pseudoInverse() *
  DeltaS.

  \param cMo : Pose matrix that has been computed with the v.

  \param deltaS : Error vector used in v = Ls.pseudoInverse() * DeltaS

  \param Ls : interaction matrix used in v = Ls.pseudoInverse() * DeltaS
*/
vpMatrix vpMatrix::computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS,
                                              const vpMatrix &Ls)
{
  vpMatrix Js;
  vpColVector deltaP;
  vpMatrix::computeCovarianceMatrixVVS(cMo, deltaS, Ls, Js, deltaP);

  return vpMatrix::computeCovarianceMatrix(Js, deltaP, deltaS);
}

/*!
  Compute the covariance matrix of an image-based virtual visual servoing.
  This assumes the optimization has been done via v = (W * Ls).pseudoInverse()
  * W * DeltaS.

  \param cMo : Pose matrix that has been computed with the v.

  \param deltaS : Error vector used in v = (W * Ls).pseudoInverse() * W *
  DeltaS.

  \param Ls : interaction matrix used in v = (W * Ls).pseudoInverse() * W *
  DeltaS.

  \param W : Weight matrix used in v = (W * Ls).pseudoInverse() * W * DeltaS.
*/
vpMatrix vpMatrix::computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS,
                                              const vpMatrix &Ls, const vpMatrix &W)
{
  vpMatrix Js;
  vpColVector deltaP;
  vpMatrix::computeCovarianceMatrixVVS(cMo, deltaS, Ls, Js, deltaP);

  return vpMatrix::computeCovarianceMatrix(Js, deltaP, deltaS, W);
}

void vpMatrix::computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS, const vpMatrix &Ls,
                                          vpMatrix &Js, vpColVector &deltaP)
{
  // building Lp
  vpMatrix LpInv(6, 6);
  LpInv = 0;
  LpInv[0][0] = -1.0;
  LpInv[1][1] = -1.0;
  LpInv[2][2] = -1.0;

  vpTranslationVector ctoInit;

  cMo.extract(ctoInit);
  vpMatrix ctoInitSkew = ctoInit.skew();

  vpThetaUVector thetau;
  cMo.extract(thetau);

  vpColVector tu(3);
  for (unsigned int i = 0; i < 3; i++)
    tu[i] = thetau[i];

  double theta = sqrt(tu.sumSquare());

  //    vpMatrix Lthetau(3,3);
  vpMatrix LthetauInvAnalytic(3, 3);
  vpMatrix I3(3, 3);
  I3.eye();
  //    Lthetau = -I3;
  LthetauInvAnalytic = -I3;

  if (theta / (2.0 * M_PI) > std::numeric_limits<double>::epsilon()) {
    // Computing [theta/2 u]_x
    vpColVector theta2u(3);
    for (unsigned int i = 0; i < 3; i++) {
      theta2u[i] = tu[i] / 2.0;
    }
    vpMatrix theta2u_skew = vpColVector::skew(theta2u);

    vpColVector u(3);
    for (unsigned int i = 0; i < 3; i++) {
      u[i] = tu[i] / theta;
    }
    vpMatrix u_skew = vpColVector::skew(u);

    //        Lthetau += (theta2u_skew -
    //        (1.0-vpMath::sinc(theta)/vpMath::sqr(vpMath::sinc(theta/2.0)))*u_skew*u_skew);
    LthetauInvAnalytic +=
        -(vpMath::sqr(vpMath::sinc(theta / 2.0)) * theta2u_skew - (1.0 - vpMath::sinc(theta)) * u_skew * u_skew);
  }

  //    vpMatrix LthetauInv = Lthetau.inverseByLU();

  ctoInitSkew = ctoInitSkew * LthetauInvAnalytic;

  for (unsigned int a = 0; a < 3; a++)
    for (unsigned int b = 0; b < 3; b++)
      LpInv[a][b + 3] = ctoInitSkew[a][b];

  for (unsigned int a = 0; a < 3; a++)
    for (unsigned int b = 0; b < 3; b++)
      LpInv[a + 3][b + 3] = LthetauInvAnalytic[a][b];

  // Building Js
  Js = Ls * LpInv;

  // building deltaP
  deltaP = (Js).pseudoInverse(Js.getRows() * std::numeric_limits<double>::epsilon()) * deltaS;
}
