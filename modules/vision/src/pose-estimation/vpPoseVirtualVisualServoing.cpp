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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpPoseVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach
*/

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpPose.h>

/*!
  \brief Compute the pose using virtual visual servoing approach

  This approach is described in \cite Marchand02c.

*/

void vpPose::poseVirtualVS(vpHomogeneousMatrix &cMo)
{
  try {

    double residu_1 = 1e8;
    double r = 1e8 - 1;

    // we stop the minimization when the error is bellow 1e-8

    int iter = 0;

    unsigned int nb = (unsigned int)listP.size();
    vpMatrix L(2 * nb, 6);
    vpColVector err(2 * nb);
    vpColVector sd(2 * nb), s(2 * nb);
    vpColVector v;

    vpPoint P;
    std::list<vpPoint> lP;

    // create sd
    unsigned int k = 0;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
      P = *it;
      sd[2 * k] = P.get_x();
      sd[2 * k + 1] = P.get_y();
      lP.push_back(P);
      k++;
    }

    vpHomogeneousMatrix cMoPrev = cMo;
    // while((int)((residu_1 - r)*1e12) !=0)
    //    while(std::fabs((residu_1 - r)*1e12) >
    //    std::numeric_limits<double>::epsilon())
    while (std::fabs(residu_1 - r) > vvsEpsilon) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      k = 0;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it) {
        P = *it;
        // forward projection of the 3D model for a given pose
        // change frame coordinates
        // perspective projection
        P.track(cMo);

        double x = s[2 * k] = P.get_x(); /* point projected from cMo */
        double y = s[2 * k + 1] = P.get_y();
        double Z = P.get_Z();
        L[2 * k][0] = -1 / Z;
        L[2 * k][1] = 0;
        L[2 * k][2] = x / Z;
        L[2 * k][3] = x * y;
        L[2 * k][4] = -(1 + x * x);
        L[2 * k][5] = y;

        L[2 * k + 1][0] = 0;
        L[2 * k + 1][1] = -1 / Z;
        L[2 * k + 1][2] = y / Z;
        L[2 * k + 1][3] = 1 + y * y;
        L[2 * k + 1][4] = -x * y;
        L[2 * k + 1][5] = -x;

        k += 1;
      }
      err = s - sd;

      // compute the residual
      r = err.sumSquare();

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      L.pseudoInverse(Lp, 1e-16);

      // compute the VVS control law
      v = -lambda * Lp * err;

      // std::cout << "r=" << r <<std::endl ;
      // update the pose

      cMoPrev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      if (iter++ > vvsIterMax) {
        break;
      }
    }

    if (computeCovariance)
      covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev, err, L);
  }

  catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }
}

/*!
  \brief Compute the pose using virtual visual servoing approach and
  a robust control law

  This approach is described in \cite Comport06b.

*/
void vpPose::poseVirtualVSrobust(vpHomogeneousMatrix &cMo)
{
  try {

    double residu_1 = 1e8;
    double r = 1e8 - 1;

    // we stop the minimization when the error is bellow 1e-8
    vpMatrix W;
    vpRobust robust((unsigned int)(2 * listP.size()));
    robust.setThreshold(0.0000);
    vpColVector w, res;

    unsigned int nb = (unsigned int)listP.size();
    vpMatrix L(2 * nb, 6);
    vpColVector error(2 * nb);
    vpColVector sd(2 * nb), s(2 * nb);
    vpColVector v;

    listP.front();
    vpPoint P;
    std::list<vpPoint> lP;

    // create sd
    unsigned int k_ = 0;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
      P = *it;
      sd[2 * k_] = P.get_x();
      sd[2 * k_ + 1] = P.get_y();
      lP.push_back(P);
      k_++;
    }
    int iter = 0;
    res.resize(s.getRows() / 2);
    w.resize(s.getRows() / 2);
    W.resize(s.getRows(), s.getRows());
    w = 1;

    // while((int)((residu_1 - r)*1e12) !=0)
    while (std::fabs((residu_1 - r) * 1e12) > std::numeric_limits<double>::epsilon()) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      k_ = 0;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it) {
        P = *it;
        // forward projection of the 3D model for a given pose
        // change frame coordinates
        // perspective projection
        P.track(cMo);

        double x = s[2 * k_] = P.get_x(); // point projected from cMo
        double y = s[2 * k_ + 1] = P.get_y();
        double Z = P.get_Z();
        L[2 * k_][0] = -1 / Z;
        L[2 * k_][1] = 0;
        L[2 * k_][2] = x / Z;
        L[2 * k_][3] = x * y;
        L[2 * k_][4] = -(1 + x * x);
        L[2 * k_][5] = y;

        L[2 * k_ + 1][0] = 0;
        L[2 * k_ + 1][1] = -1 / Z;
        L[2 * k_ + 1][2] = y / Z;
        L[2 * k_ + 1][3] = 1 + y * y;
        L[2 * k_ + 1][4] = -x * y;
        L[2 * k_ + 1][5] = -x;

        k_++;
      }
      error = s - sd;

      // compute the residual
      r = error.sumSquare();

      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        res[k] = vpMath::sqr(error[2 * k]) + vpMath::sqr(error[2 * k + 1]);
      }
      robust.setIteration(0);
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        W[2 * k][2 * k] = w[k];
        W[2 * k + 1][2 * k + 1] = w[k];
      }
      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      (W * L).pseudoInverse(Lp, 1e-6);

      // compute the VVS control law
      v = -lambda * Lp * W * error;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      ;
      if (iter++ > vvsIterMax)
        break;
    }

    if (computeCovariance)
      covarianceMatrix =
          vpMatrix::computeCovarianceMatrix(L, v, -lambda * error, W * W); // Remark: W*W = W*W.t() since the
                                                                           // matrix is diagonale, but using W*W
                                                                           // is more efficient.
  } catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }
}
