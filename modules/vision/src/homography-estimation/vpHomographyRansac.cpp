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
 * Homography estimation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRansac.h>
#include <visp3/vision/vpHomography.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMeterPixelConversion.h>

#define vpEps 1e-6

/*!
  \file vpHomographyRansac.cpp
  \brief function used to estimate an homography using the Ransac algorithm
*/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

bool iscolinear(double *x1, double *x2, double *x3);
bool isColinear(vpColVector &p1, vpColVector &p2, vpColVector &p3);

bool iscolinear(double *x1, double *x2, double *x3)
{
  vpColVector p1(3), p2(3), p3(3);
  p1 << x1;
  p2 << x2;
  p3 << x3;
  // vpColVector v;
  // vpColVector::cross(p2-p1, p3-p1, v);
  // return (v.sumSquare() < vpEps);
  // Assume inhomogeneous coords, or homogeneous coords with equal
  // scale.
  return ((vpColVector::cross(p2 - p1, p3 - p1).sumSquare()) < vpEps);
}
bool isColinear(vpColVector &p1, vpColVector &p2, vpColVector &p3)
{
  return ((vpColVector::cross(p2 - p1, p3 - p1).sumSquare()) < vpEps);
}

bool vpHomography::degenerateConfiguration(vpColVector &x, unsigned int *ind, double threshold_area)
{

  unsigned int i, j, k;

  for (i = 1; i < 4; i++)
    for (j = 0; j < i; j++)
      if (ind[i] == ind[j])
        return true;

  unsigned int n = x.getRows() / 4;
  double pa[4][3];
  double pb[4][3];

  for (i = 0; i < 4; i++) {
    pb[i][0] = x[2 * ind[i]];
    pb[i][1] = x[2 * ind[i] + 1];
    pb[i][2] = 1;

    pa[i][0] = x[2 * n + 2 * ind[i]];
    pa[i][1] = x[2 * n + 2 * ind[i] + 1];
    pa[i][2] = 1;
  }

  i = 0, j = 1, k = 2;

  double area012 = (-pa[j][0] * pa[i][1] + pa[k][0] * pa[i][1] + pa[i][0] * pa[j][1] - pa[k][0] * pa[j][1] +
                    -pa[i][0] * pa[k][1] + pa[1][j] * pa[k][1]);

  i = 0;
  j = 1, k = 3;
  double area013 = (-pa[j][0] * pa[i][1] + pa[k][0] * pa[i][1] + pa[i][0] * pa[j][1] - pa[k][0] * pa[j][1] +
                    -pa[i][0] * pa[k][1] + pa[1][j] * pa[k][1]);

  i = 0;
  j = 2, k = 3;
  double area023 = (-pa[j][0] * pa[i][1] + pa[k][0] * pa[i][1] + pa[i][0] * pa[j][1] - pa[k][0] * pa[j][1] +
                    -pa[i][0] * pa[k][1] + pa[1][j] * pa[k][1]);

  i = 1;
  j = 2, k = 3;
  double area123 = (-pa[j][0] * pa[i][1] + pa[k][0] * pa[i][1] + pa[i][0] * pa[j][1] - pa[k][0] * pa[j][1] +
                    -pa[i][0] * pa[k][1] + pa[1][j] * pa[k][1]);

  double sum_area = area012 + area013 + area023 + area123;

  return ((sum_area < threshold_area) ||
          (iscolinear(pa[0], pa[1], pa[2]) || iscolinear(pa[0], pa[1], pa[3]) || iscolinear(pa[0], pa[2], pa[3]) ||
           iscolinear(pa[1], pa[2], pa[3]) || iscolinear(pb[0], pb[1], pb[2]) || iscolinear(pb[0], pb[1], pb[3]) ||
           iscolinear(pb[0], pb[2], pb[3]) || iscolinear(pb[1], pb[2], pb[3])));
}
/*
\brief
Function to determine if a set of 4 pairs of matched  points give rise
to a degeneracy in the calculation of a homography as needed by RANSAC.
This involves testing whether any 3 of the 4 points in each set is
colinear.

point are coded this way
x1b,y1b, x2b, y2b, ... xnb, ynb
x1a,y1a, x2a, y2a, ... xna, yna
leading to 2*2*n
*/
bool vpHomography::degenerateConfiguration(vpColVector &x, unsigned int *ind)
{
  for (unsigned int i = 1; i < 4; i++)
    for (unsigned int j = 0; j < i; j++)
      if (ind[i] == ind[j])
        return true;

  unsigned int n = x.getRows() / 4;
  double pa[4][3];
  double pb[4][3];
  unsigned int n2 = 2 * n;
  for (unsigned int i = 0; i < 4; i++) {
    unsigned int ind2 = 2 * ind[i];
    pb[i][0] = x[ind2];
    pb[i][1] = x[ind2 + 1];
    pb[i][2] = 1;

    pa[i][0] = x[n2 + ind2];
    pa[i][1] = x[n2 + ind2 + 1];
    pa[i][2] = 1;
  }
  return (iscolinear(pa[0], pa[1], pa[2]) || iscolinear(pa[0], pa[1], pa[3]) || iscolinear(pa[0], pa[2], pa[3]) ||
          iscolinear(pa[1], pa[2], pa[3]) || iscolinear(pb[0], pb[1], pb[2]) || iscolinear(pb[0], pb[1], pb[3]) ||
          iscolinear(pb[0], pb[2], pb[3]) || iscolinear(pb[1], pb[2], pb[3]));
}
bool vpHomography::degenerateConfiguration(const std::vector<double> &xb, const std::vector<double> &yb,
                                           const std::vector<double> &xa, const std::vector<double> &ya)
{
  unsigned int n = (unsigned int)xb.size();
  if (n < 4)
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));

  std::vector<vpColVector> pa(n), pb(n);
  for (unsigned i = 0; i < n; i++) {
    pa[i].resize(3);
    pa[i][0] = xa[i];
    pa[i][1] = ya[i];
    pa[i][2] = 1;
    pb[i].resize(3);
    pb[i][0] = xb[i];
    pb[i][1] = yb[i];
    pb[i][2] = 1;
  }

  for (unsigned int i = 0; i < n - 2; i++) {
    for (unsigned int j = i + 1; j < n - 1; j++) {
      for (unsigned int k = j + 1; k < n; k++) {
        if (isColinear(pa[i], pa[j], pa[k])) {
          return true;
        }
        if (isColinear(pb[i], pb[j], pb[k])) {
          return true;
        }
      }
    }
  }
  return false;
}
// Fit model to this random selection of data points.
void vpHomography::computeTransformation(vpColVector &x, unsigned int *ind, vpColVector &M)
{
  unsigned int n = x.getRows() / 4;
  std::vector<double> xa(4), xb(4);
  std::vector<double> ya(4), yb(4);
  unsigned int n2 = n * 2;
  for (unsigned int i = 0; i < 4; i++) {
    unsigned int ind2 = 2 * ind[i];
    xb[i] = x[ind2];
    yb[i] = x[ind2 + 1];

    xa[i] = x[n2 + ind2];
    ya[i] = x[n2 + ind2 + 1];
  }

  vpHomography aHb;
  try {
    vpHomography::HLM(xb, yb, xa, ya, true, aHb);
  } catch (...) {
    aHb.eye();
  }

  M.resize(9);
  for (unsigned int i = 0; i < 9; i++) {
    M[i] = aHb.data[i];
  }
  aHb /= aHb[2][2];
}

// Evaluate distances between points and model.
double vpHomography::computeResidual(vpColVector &x, vpColVector &M, vpColVector &d)
{
  unsigned int n = x.getRows() / 4;
  unsigned int n2 = n * 2;
  vpColVector *pa;
  vpColVector *pb;

  pa = new vpColVector[n];
  pb = new vpColVector[n];

  for (unsigned int i = 0; i < n; i++) {
    unsigned int i2 = 2 * i;
    pb[i].resize(3);
    pb[i][0] = x[i2];
    pb[i][1] = x[i2 + 1];
    pb[i][2] = 1;

    pa[i].resize(3);
    pa[i][0] = x[n2 + i2];
    pa[i][1] = x[n2 + i2 + 1];
    pa[i][2] = 1;
  }

  vpMatrix aHb(3, 3);

  for (unsigned int i = 0; i < 9; i++) {
    aHb.data[i] = M[i];
  }

  aHb /= aHb[2][2];

  d.resize(n);

  vpColVector Hpb;
  for (unsigned int i = 0; i < n; i++) {
    Hpb = aHb * pb[i];
    Hpb /= Hpb[2];
    d[i] = sqrt((pa[i] - Hpb).sumSquare());
  }

  delete[] pa;
  delete[] pb;

  return 0;
}
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS

void vpHomography::initRansac(unsigned int n, double *xb, double *yb, double *xa, double *ya, vpColVector &x)
{
  x.resize(4 * n);
  unsigned int n2 = n * 2;
  for (unsigned int i = 0; i < n; i++) {
    unsigned int i2 = 2 * i;
    x[i2] = xb[i];
    x[i2 + 1] = yb[i];
    x[n2 + i2] = xa[i];
    x[n2 + i2 + 1] = ya[i];
  }
}

/*!

  From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
  and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
  computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
  ^b{\bf p}\f$ using Ransac algorithm.

  \param xb, yb : Coordinates vector of matched points in image b. These
  coordinates are expressed in meters. \param xa, ya : Coordinates vector of
  matched points in image a. These coordinates are expressed in meters. \param
  aHb : Estimated homography that relies the transformation from image a to
  image b. \param inliers : Vector that indicates if a matched point is an
  inlier (true) or an outlier (false). \param residual : Global residual
  computed as \f$r = \sqrt{1/n \sum_{inliers} {\| {^a{\bf p} - {\hat{^a{\bf
  H}_b}} {^b{\bf p}}} \|}^{2}}\f$ with \f$n\f$ the number of inliers.

  \param nbInliersConsensus : Minimal number of points requested to fit the
  estimated homography.

  \param threshold : Threshold for outlier removing. A point is considered as
  an outlier if the reprojection error \f$\| {^a{\bf p} - {\hat{^a{\bf H}_b}}
  {^b{\bf p}}} \|\f$ is greater than this threshold.

  \param normalization : When set to true, the coordinates of the points are
  normalized. The normalization carried out is the one preconized by Hartley.

  \return true if the homography could be computed, false otherwise.

*/
bool vpHomography::ransac(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                          const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers,
                          double &residual, unsigned int nbInliersConsensus, double threshold, bool normalization)
{
  unsigned int n = (unsigned int)xb.size();
  if (yb.size() != n || xa.size() != n || ya.size() != n)
    throw(vpException(vpException::dimensionError, "Bad dimension for robust homography estimation"));

  // 4 point are required
  if (n < 4)
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));

  vpUniRand random((long)time(NULL));

  std::vector<unsigned int> best_consensus;
  std::vector<unsigned int> cur_consensus;
  std::vector<unsigned int> cur_outliers;
  std::vector<unsigned int> cur_randoms;

  std::vector<unsigned int> rand_ind;

  unsigned int nbMinRandom = 4;
  unsigned int ransacMaxTrials = 1000;
  unsigned int maxDegenerateIter = 1000;

  unsigned int nbTrials = 0;
  unsigned int nbDegenerateIter = 0;
  unsigned int nbInliers = 0;

  bool foundSolution = false;

  std::vector<double> xa_rand(nbMinRandom);
  std::vector<double> ya_rand(nbMinRandom);
  std::vector<double> xb_rand(nbMinRandom);
  std::vector<double> yb_rand(nbMinRandom);

  if (inliers.size() != n)
    inliers.resize(n);

  while (nbTrials < ransacMaxTrials && nbInliers < nbInliersConsensus) {
    cur_outliers.clear();
    cur_randoms.clear();

    bool degenerate = true;
    while (degenerate == true) {
      std::vector<bool> usedPt(n, false);

      rand_ind.clear();
      for (unsigned int i = 0; i < nbMinRandom; i++) {
        // Generate random indicies in the range 0..n
        unsigned int r = (unsigned int)ceil(random() * n) - 1;
        while (usedPt[r]) {
          r = (unsigned int)ceil(random() * n) - 1;
        }
        usedPt[r] = true;
        rand_ind.push_back(r);

        xa_rand[i] = xa[r];
        ya_rand[i] = ya[r];
        xb_rand[i] = xb[r];
        yb_rand[i] = yb[r];
      }

      try {
        if (!vpHomography::degenerateConfiguration(xb_rand, yb_rand, xa_rand, ya_rand)) {
          vpHomography::DLT(xb_rand, yb_rand, xa_rand, ya_rand, aHb, normalization);
          degenerate = false;
        }
      } catch (...) {
        degenerate = true;
      }

      nbDegenerateIter++;

      if (nbDegenerateIter > maxDegenerateIter) {
        vpERROR_TRACE("Unable to select a nondegenerate data set");
        throw(vpException(vpException::fatalError, "Unable to select a nondegenerate data set"));
      }
    }

    aHb /= aHb[2][2];

    // Computing Residual
    double r = 0;
    vpColVector a(3), b(3), c(3);
    for (unsigned int i = 0; i < nbMinRandom; i++) {
      a[0] = xa_rand[i];
      a[1] = ya_rand[i];
      a[2] = 1;
      b[0] = xb_rand[i];
      b[1] = yb_rand[i];
      b[2] = 1;

      c = aHb * b;
      c /= c[2];
      r += (a - c).sumSquare();
      // cout << "point " <<i << "  " << (a-c).sumSquare()  <<endl ;;
    }

    // Finding inliers & ouliers
    r = sqrt(r / nbMinRandom);
    // std::cout << "Candidate residual: " << r << std::endl;
    if (r < threshold) {
      unsigned int nbInliersCur = 0;
      for (unsigned int i = 0; i < n; i++) {
        a[0] = xa[i];
        a[1] = ya[i];
        a[2] = 1;
        b[0] = xb[i];
        b[1] = yb[i];
        b[2] = 1;

        c = aHb * b;
        c /= c[2];
        double error = sqrt((a - c).sumSquare());
        if (error <= threshold) {
          nbInliersCur++;
          cur_consensus.push_back(i);
          inliers[i] = true;
        } else {
          cur_outliers.push_back(i);
          inliers[i] = false;
        }
      }
      // std::cout << "nb inliers that matches: " << nbInliersCur <<
      // std::endl;
      if (nbInliersCur > nbInliers) {
        foundSolution = true;
        best_consensus = cur_consensus;
        nbInliers = nbInliersCur;
      }

      cur_consensus.clear();
    }

    nbTrials++;
    if (nbTrials >= ransacMaxTrials) {
      vpERROR_TRACE("Ransac reached the maximum number of trials");
      foundSolution = true;
    }
  }

  if (foundSolution) {
    if (nbInliers >= nbInliersConsensus) {
      std::vector<double> xa_best(best_consensus.size());
      std::vector<double> ya_best(best_consensus.size());
      std::vector<double> xb_best(best_consensus.size());
      std::vector<double> yb_best(best_consensus.size());

      for (unsigned i = 0; i < best_consensus.size(); i++) {
        xa_best[i] = xa[best_consensus[i]];
        ya_best[i] = ya[best_consensus[i]];
        xb_best[i] = xb[best_consensus[i]];
        yb_best[i] = yb[best_consensus[i]];
      }

      vpHomography::DLT(xb_best, yb_best, xa_best, ya_best, aHb, normalization);
      aHb /= aHb[2][2];

      residual = 0;
      vpColVector a(3), b(3), c(3);
      for (unsigned int i = 0; i < best_consensus.size(); i++) {
        a[0] = xa_best[i];
        a[1] = ya_best[i];
        a[2] = 1;
        b[0] = xb_best[i];
        b[1] = yb_best[i];
        b[2] = 1;

        c = aHb * b;
        c /= c[2];
        residual += (a - c).sumSquare();
      }

      residual = sqrt(residual / best_consensus.size());
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}
