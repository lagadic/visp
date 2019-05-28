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
 * Homography transformation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

//#include <computeHomography.h>
//#include <utilsHomography.h>
#include <iostream>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpHomography.h>

const double vpHomography::threshold_rotation = 1e-7;
const double vpHomography::threshold_displacement = 1e-18;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

static void updatePoseRotation(vpColVector &dx, vpHomogeneousMatrix &mati)
{
  vpRotationMatrix rd;

  double s = sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
  if (s > 1.e-25) {
    double u[3];

    for (unsigned int i = 0; i < 3; i++)
      u[i] = dx[i] / s;
    double sinu = sin(s);
    double cosi = cos(s);
    double mcosi = 1 - cosi;
    rd[0][0] = cosi + mcosi * u[0] * u[0];
    rd[0][1] = -sinu * u[2] + mcosi * u[0] * u[1];
    rd[0][2] = sinu * u[1] + mcosi * u[0] * u[2];
    rd[1][0] = sinu * u[2] + mcosi * u[1] * u[0];
    rd[1][1] = cosi + mcosi * u[1] * u[1];
    rd[1][2] = -sinu * u[0] + mcosi * u[1] * u[2];
    rd[2][0] = -sinu * u[1] + mcosi * u[2] * u[0];
    rd[2][1] = sinu * u[0] + mcosi * u[2] * u[1];
    rd[2][2] = cosi + mcosi * u[2] * u[2];
  } else {
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++)
        rd[i][j] = 0.0;
      rd[i][i] = 1.0;
    }
  }

  vpHomogeneousMatrix Delta;
  Delta.insert(rd);

  mati = Delta.inverse() * mati;
}

double vpHomography::computeRotation(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpHomogeneousMatrix &c2Mc1,
                                     int userobust)
{
  vpColVector e(2);
  double r_1 = -1;

  vpColVector p2(3);
  vpColVector p1(3);
  vpColVector Hp2(3);
  vpColVector Hp1(3);

  vpMatrix H2(2, 3);
  vpColVector e2(2);
  vpMatrix H1(2, 3);
  vpColVector e1(2);

  bool only_1 = false;
  bool only_2 = false;
  int iter = 0;

  unsigned int n = 0;
  for (unsigned int i = 0; i < nbpoint; i++) {
    //    if ((c2P[i].get_x() !=-1) && (c1P[i].get_x() !=-1))
    if ((std::fabs(c2P[i].get_x() + 1) > std::fabs(vpMath::maximum(c2P[i].get_x(), 1.))) &&
        (std::fabs(c1P[i].get_x() + 1) > std::fabs(vpMath::maximum(c1P[i].get_x(), 1.)))) {
      n++;
    }
  }
  if ((!only_1) && (!only_2))
    n *= 2;

  vpRobust robust(n);
  vpColVector res(n);
  vpColVector w(n);
  w = 1;
  robust.setThreshold(0.0000);
  vpMatrix W(2 * n, 2 * n);
  W = 0;
  vpMatrix c2Rc1(3, 3);
  double r = 0;
  while (vpMath::equal(r_1, r, threshold_rotation) == false) {

    r_1 = r;
    // compute current position

    // Change frame (current)
    for (unsigned int i = 0; i < 3; i++)
      for (unsigned int j = 0; j < 3; j++)
        c2Rc1[i][j] = c2Mc1[i][j];

    vpMatrix L(2, 3), Lp;
    int k = 0;
    for (unsigned int i = 0; i < nbpoint; i++) {
      // if ((c2P[i].get_x() !=-1) && (c1P[i].get_x() !=-1))
      if ((std::fabs(c2P[i].get_x() + 1) > std::fabs(vpMath::maximum(c2P[i].get_x(), 1.))) &&
          (std::fabs(c1P[i].get_x() + 1) > std::fabs(vpMath::maximum(c1P[i].get_x(), 1.)))) {
        p2[0] = c2P[i].get_x();
        p2[1] = c2P[i].get_y();
        p2[2] = 1.0;
        p1[0] = c1P[i].get_x();
        p1[1] = c1P[i].get_y();
        p1[2] = 1.0;

        Hp2 = c2Rc1.t() * p2; // p2 = Hp1
        Hp1 = c2Rc1 * p1;     // p1 = Hp2

        Hp2 /= Hp2[2]; // normalisation
        Hp1 /= Hp1[2];

        // set up the interaction matrix
        double x = Hp2[0];
        double y = Hp2[1];

        H2[0][0] = x * y;
        H2[0][1] = -(1 + x * x);
        H2[0][2] = y;
        H2[1][0] = 1 + y * y;
        H2[1][1] = -x * y;
        H2[1][2] = -x;
        H2 *= -1;
        H2 = H2 * c2Rc1.t();

        // Set up the error vector
        e2[0] = Hp2[0] - c1P[i].get_x();
        e2[1] = Hp2[1] - c1P[i].get_y();

        // set up the interaction matrix
        x = Hp1[0];
        y = Hp1[1];

        H1[0][0] = x * y;
        H1[0][1] = -(1 + x * x);
        H1[0][2] = y;
        H1[1][0] = 1 + y * y;
        H1[1][1] = -x * y;
        H1[1][2] = -x;

        // Set up the error vector
        e1[0] = Hp1[0] - c2P[i].get_x();
        e1[1] = Hp1[1] - c2P[i].get_y();

        if (only_2) {
          if (k == 0) {
            L = H2;
            e = e2;
          } else {
            L = vpMatrix::stack(L, H2);
            e = vpColVector::stack(e, e2);
          }
        } else if (only_1) {
          if (k == 0) {
            L = H1;
            e = e1;
          } else {
            L = vpMatrix::stack(L, H1);
            e = vpColVector::stack(e, e1);
          }
        } else {
          if (k == 0) {
            L = H2;
            e = e2;
          } else {
            L = vpMatrix::stack(L, H2);
            e = vpColVector::stack(e, e2);
          }
          L = vpMatrix::stack(L, H1);
          e = vpColVector::stack(e, e1);
        }

        k++;
      }
    }

    if (userobust) {
      robust.setIteration(0);

      for (unsigned int l = 0; l < n; l++) {
        res[l] = vpMath::sqr(e[2 * l]) + vpMath::sqr(e[2 * l + 1]);
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int l = 0; l < n; l++) {
        W[2 * l][2 * l] = w[l];
        W[2 * l + 1][2 * l + 1] = w[l];
      }
    } else {
      for (unsigned int l = 0; l < 2 * n; l++)
        W[l][l] = 1;
    }
    // CreateDiagonalMatrix(w, W) ;
    (L).pseudoInverse(Lp, 1e-6);
    // Compute the camera velocity
    vpColVector c2rc1, v(6);

    c2rc1 = -2 * Lp * W * e;
    for (unsigned int i = 0; i < 3; i++)
      v[i + 3] = c2rc1[i];

    // only for simulation

    updatePoseRotation(c2rc1, c2Mc1);
    r = e.sumSquare();

    if ((W * e).sumSquare() < 1e-10)
      break;
    if (iter > 25)
      break;
    iter++; // std::cout <<  iter <<"  e=" <<(e).sumSquare() <<"  e="
            // <<(W*e).sumSquare() <<std::endl ;
  }

  //  std::cout << c2Mc1 <<std::endl ;
  return (W * e).sumSquare();
}

static void getPlaneInfo(vpPlane &oN, vpHomogeneousMatrix &cMo, vpColVector &cN, double &cd)
{
  double A1 = cMo[0][0] * oN.getA() + cMo[0][1] * oN.getB() + cMo[0][2] * oN.getC();
  double B1 = cMo[1][0] * oN.getA() + cMo[1][1] * oN.getB() + cMo[1][2] * oN.getC();
  double C1 = cMo[2][0] * oN.getA() + cMo[2][1] * oN.getB() + cMo[2][2] * oN.getC();
  double D1 = oN.getD() - (cMo[0][3] * A1 + cMo[1][3] * B1 + cMo[2][3] * C1);

  cN.resize(3);
  cN[0] = A1;
  cN[1] = B1;
  cN[2] = C1;
  cd = -D1;
}

double vpHomography::computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane &oN,
                                         vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust)
{
  vpColVector e(2);
  double r_1 = -1;

  vpColVector p2(3);
  vpColVector p1(3);
  vpColVector Hp2(3);
  vpColVector Hp1(3);

  vpMatrix H2(2, 6);
  vpColVector e2(2);
  vpMatrix H1(2, 6);
  vpColVector e1(2);

  bool only_1 = true;
  bool only_2 = false;
  int iter = 0;
  unsigned int n = 0;
  n = nbpoint;

  // next 2 lines are useless (detected by Coverity Scan)
  // if ( (! only_1) && (! only_2) )
  //  n *=2 ;

  vpRobust robust(n);
  vpColVector res(n);
  vpColVector w(n);
  w = 1;
  robust.setThreshold(0.0000);
  vpMatrix W(2 * n, 2 * n);
  W = 0;

  vpColVector N1(3), N2(3);
  double d1, d2;

  double r = 1e10;
  iter = 0;
  while (vpMath::equal(r_1, r, threshold_displacement) == false) {
    r_1 = r;
    // compute current position

    // Change frame (current)
    vpHomogeneousMatrix c1Mc2, c2Mo;
    vpRotationMatrix c1Rc2, c2Rc1;
    vpTranslationVector c1Tc2, c2Tc1;
    c1Mc2 = c2Mc1.inverse();
    c2Mc1.extract(c2Rc1);
    c2Mc1.extract(c2Tc1);
    c2Mc1.extract(c1Rc2);
    c1Mc2.extract(c1Tc2);

    c2Mo = c2Mc1 * c1Mo;

    getPlaneInfo(oN, c1Mo, N1, d1);
    getPlaneInfo(oN, c2Mo, N2, d2);

    vpMatrix L(2, 3), Lp;
    int k = 0;
    for (unsigned int i = 0; i < nbpoint; i++) {
      p2[0] = c2P[i].get_x();
      p2[1] = c2P[i].get_y();
      p2[2] = 1.0;
      p1[0] = c1P[i].get_x();
      p1[1] = c1P[i].get_y();
      p1[2] = 1.0;

      vpMatrix H(3, 3);

      Hp2 = ((vpMatrix)c1Rc2 + (c1Tc2 * N2.t()) / d2) * p2; // p2 = Hp1
      Hp1 = ((vpMatrix)c2Rc1 + (c2Tc1 * N1.t()) / d1) * p1; // p1 = Hp2

      Hp2 /= Hp2[2]; // normalisation
      Hp1 /= Hp1[2];

      // set up the interaction matrix
      double x = Hp2[0];
      double y = Hp2[1];
      double Z1;

      Z1 = (N1[0] * x + N1[1] * y + N1[2]) / d1; // 1/z

      H2[0][0] = -Z1;
      H2[0][1] = 0;
      H2[0][2] = x * Z1;
      H2[1][0] = 0;
      H2[1][1] = -Z1;
      H2[1][2] = y * Z1;
      H2[0][3] = x * y;
      H2[0][4] = -(1 + x * x);
      H2[0][5] = y;
      H2[1][3] = 1 + y * y;
      H2[1][4] = -x * y;
      H2[1][5] = -x;
      H2 *= -1;

      vpMatrix c1CFc2(6, 6);
      {
        vpMatrix sTR = c1Tc2.skew() * (vpMatrix)c1Rc2;
        for (unsigned int k_ = 0; k_ < 3; k_++)
          for (unsigned int l = 0; l < 3; l++) {
            c1CFc2[k_][l] = c1Rc2[k_][l];
            c1CFc2[k_ + 3][l + 3] = c1Rc2[k_][l];
            c1CFc2[k_][l + 3] = sTR[k_][l];
          }
      }
      H2 = H2 * c1CFc2;

      // Set up the error vector
      e2[0] = Hp2[0] - c1P[i].get_x();
      e2[1] = Hp2[1] - c1P[i].get_y();

      x = Hp1[0];
      y = Hp1[1];

      Z1 = (N2[0] * x + N2[1] * y + N2[2]) / d2; // 1/z

      H1[0][0] = -Z1;
      H1[0][1] = 0;
      H1[0][2] = x * Z1;
      H1[1][0] = 0;
      H1[1][1] = -Z1;
      H1[1][2] = y * Z1;
      H1[0][3] = x * y;
      H1[0][4] = -(1 + x * x);
      H1[0][5] = y;
      H1[1][3] = 1 + y * y;
      H1[1][4] = -x * y;
      H1[1][5] = -x;

      // Set up the error vector
      e1[0] = Hp1[0] - c2P[i].get_x();
      e1[1] = Hp1[1] - c2P[i].get_y();

      if (only_2) {
        if (k == 0) {
          L = H2;
          e = e2;
        } else {
          L = vpMatrix::stack(L, H2);
          e = vpColVector::stack(e, e2);
        }
      } else if (only_1) {
        if (k == 0) {
          L = H1;
          e = e1;
        } else {
          L = vpMatrix::stack(L, H1);
          e = vpColVector::stack(e, e1);
        }
      } else {
        if (k == 0) {
          L = H2;
          e = e2;
        } else {
          L = vpMatrix::stack(L, H2);
          e = vpColVector::stack(e, e2);
        }
        L = vpMatrix::stack(L, H1);
        e = vpColVector::stack(e, e1);
      }

      k++;
    }

    if (userobust) {
      robust.setIteration(0);
      for (unsigned int l = 0; l < n; l++) {
        res[l] = vpMath::sqr(e[2 * l]) + vpMath::sqr(e[2 * l + 1]);
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int l = 0; l < n; l++) {
        W[2 * l][2 * l] = w[l];
        W[2 * l + 1][2 * l + 1] = w[l];
      }
    } else {
      for (unsigned int l = 0; l < 2 * n; l++)
        W[l][l] = 1;
    }
    (W * L).pseudoInverse(Lp, 1e-16);
    // Compute the camera velocity
    vpColVector c2Tcc1;

    c2Tcc1 = -1 * Lp * W * e;

    // only for simulation

    c2Mc1 = vpExponentialMap::direct(c2Tcc1).inverse() * c2Mc1;
    ;
    //   UpdatePose2(c2Tcc1, c2Mc1) ;
    r = (W * e).sumSquare();

    if (r < 1e-15) {
      break;
    }
    if (iter > 1000) {
      break;
    }
    if (r > r_1) {
      break;
    }
    iter++;
  }

  return (W * e).sumSquare();
}

double vpHomography::computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane *oN,
                                         vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust)
{

  vpColVector e(2);
  double r_1 = -1;

  vpColVector p2(3);
  vpColVector p1(3);
  vpColVector Hp2(3);
  vpColVector Hp1(3);

  vpMatrix H2(2, 6);
  vpColVector e2(2);
  vpMatrix H1(2, 6);
  vpColVector e1(2);

  bool only_1 = true;
  bool only_2 = false;
  int iter = 0;
  unsigned int i;
  unsigned int n = 0;
  n = nbpoint;

  // next 2 lines are useless (detected by Coverity Scan)
  // if ( (! only_1) && (! only_2) )
  //  n *=2 ;

  vpRobust robust(n);
  vpColVector res(n);
  vpColVector w(n);
  w = 1;
  robust.setThreshold(0.0000);
  vpMatrix W(2 * n, 2 * n);
  W = 0;

  vpColVector N1(3), N2(3);
  double d1, d2;

  double r = 1e10;
  iter = 0;
  while (vpMath::equal(r_1, r, threshold_displacement) == false) {
    r_1 = r;
    // compute current position

    // Change frame (current)
    vpHomogeneousMatrix c1Mc2, c2Mo;
    vpRotationMatrix c1Rc2, c2Rc1;
    vpTranslationVector c1Tc2, c2Tc1;
    c1Mc2 = c2Mc1.inverse();
    c2Mc1.extract(c2Rc1);
    c2Mc1.extract(c2Tc1);
    c2Mc1.extract(c1Rc2);
    c1Mc2.extract(c1Tc2);

    c2Mo = c2Mc1 * c1Mo;

    vpMatrix L(2, 3), Lp;
    int k = 0;
    for (i = 0; i < nbpoint; i++) {
      getPlaneInfo(oN[i], c1Mo, N1, d1);
      getPlaneInfo(oN[i], c2Mo, N2, d2);
      p2[0] = c2P[i].get_x();
      p2[1] = c2P[i].get_y();
      p2[2] = 1.0;
      p1[0] = c1P[i].get_x();
      p1[1] = c1P[i].get_y();
      p1[2] = 1.0;

      vpMatrix H(3, 3);

      Hp2 = ((vpMatrix)c1Rc2 + (c1Tc2 * N2.t()) / d2) * p2; // p2 = Hp1
      Hp1 = ((vpMatrix)c2Rc1 + (c2Tc1 * N1.t()) / d1) * p1; // p1 = Hp2

      Hp2 /= Hp2[2]; // normalisation
      Hp1 /= Hp1[2];

      // set up the interaction matrix
      double x = Hp2[0];
      double y = Hp2[1];
      double Z1;

      Z1 = (N1[0] * x + N1[1] * y + N1[2]) / d1; // 1/z

      H2[0][0] = -Z1;
      H2[0][1] = 0;
      H2[0][2] = x * Z1;
      H2[1][0] = 0;
      H2[1][1] = -Z1;
      H2[1][2] = y * Z1;
      H2[0][3] = x * y;
      H2[0][4] = -(1 + x * x);
      H2[0][5] = y;
      H2[1][3] = 1 + y * y;
      H2[1][4] = -x * y;
      H2[1][5] = -x;
      H2 *= -1;

      vpMatrix c1CFc2(6, 6);
      {
        vpMatrix sTR = c1Tc2.skew() * (vpMatrix)c1Rc2;
        for (unsigned int k_ = 0; k_ < 3; k_++)
          for (unsigned int l = 0; l < 3; l++) {
            c1CFc2[k_][l] = c1Rc2[k_][l];
            c1CFc2[k_ + 3][l + 3] = c1Rc2[k_][l];
            c1CFc2[k_][l + 3] = sTR[k_][l];
          }
      }
      H2 = H2 * c1CFc2;

      // Set up the error vector
      e2[0] = Hp2[0] - c1P[i].get_x();
      e2[1] = Hp2[1] - c1P[i].get_y();

      x = Hp1[0];
      y = Hp1[1];

      Z1 = (N2[0] * x + N2[1] * y + N2[2]) / d2; // 1/z

      H1[0][0] = -Z1;
      H1[0][1] = 0;
      H1[0][2] = x * Z1;
      H1[1][0] = 0;
      H1[1][1] = -Z1;
      H1[1][2] = y * Z1;
      H1[0][3] = x * y;
      H1[0][4] = -(1 + x * x);
      H1[0][5] = y;
      H1[1][3] = 1 + y * y;
      H1[1][4] = -x * y;
      H1[1][5] = -x;

      // Set up the error vector
      e1[0] = Hp1[0] - c2P[i].get_x();
      e1[1] = Hp1[1] - c2P[i].get_y();

      if (only_2) {
        if (k == 0) {
          L = H2;
          e = e2;
        } else {
          L = vpMatrix::stack(L, H2);
          e = vpColVector::stack(e, e2);
        }
      } else if (only_1) {
        if (k == 0) {
          L = H1;
          e = e1;
        } else {
          L = vpMatrix::stack(L, H1);
          e = vpColVector::stack(e, e1);
        }
      } else {
        if (k == 0) {
          L = H2;
          e = e2;
        } else {
          L = vpMatrix::stack(L, H2);
          e = vpColVector::stack(e, e2);
        }
        L = vpMatrix::stack(L, H1);
        e = vpColVector::stack(e, e1);
      }

      k++;
    }

    if (userobust) {
      robust.setIteration(0);
      for (unsigned int k_ = 0; k_ < n; k_++) {
        res[k_] = vpMath::sqr(e[2 * k_]) + vpMath::sqr(e[2 * k_ + 1]);
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int k_ = 0; k_ < n; k_++) {
        W[2 * k_][2 * k_] = w[k_];
        W[2 * k_ + 1][2 * k_ + 1] = w[k_];
      }
    } else {
      for (unsigned int k_ = 0; k_ < 2 * n; k_++)
        W[k_][k_] = 1;
    }
    (W * L).pseudoInverse(Lp, 1e-16);
    // Compute the camera velocity
    vpColVector c2Tcc1;

    c2Tcc1 = -1 * Lp * W * e;

    // only for simulation

    c2Mc1 = vpExponentialMap::direct(c2Tcc1).inverse() * c2Mc1;
    ;
    //   UpdatePose2(c2Tcc1, c2Mc1) ;
    r = (W * e).sumSquare();

    if (r < 1e-15) {
      break;
    }
    if (iter > 1000) {
      break;
    }
    if (r > r_1) {
      break;
    }
    iter++;
  }

  return (W * e).sumSquare();
}

#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS
