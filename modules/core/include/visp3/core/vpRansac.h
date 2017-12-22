/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Ransac robust algorithm.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpRansac.h

  template class for
*/

#ifndef vpRANSAC_HH
#define vpRANSAC_HH

#include <ctime>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h> // debug and trace
#include <visp3/core/vpMath.h>
#include <visp3/core/vpUniRand.h> // random number generation
/*!
  \class vpRansac
  \ingroup group_core_robust

  \brief This class is a generic implementation of the Ransac algorithm. It
  cannot be used alone.

  RANSAC is described in \cite Fischler81 and \cite Hartley01a.

  The code of this class is inspired by :
  Peter Kovesi
  School of Computer Science & Software Engineering
  The University of Western Australia
  pk at csse uwa edu au
  http://www.csse.uwa.edu.au/~pk

  \sa vpHomography

 */
template <class vpTransformation> class vpRansac
{
public:
  static bool ransac(unsigned int npts, vpColVector &x, unsigned int s, double t, vpColVector &model,
                     vpColVector &inliers, int consensus = 1000, double not_used = 0.0,
                     const int maxNbumbersOfTrials = 10000);
};

/*!
  \brief
  RANSAC - Robustly fits a model to data with the RANSAC algorithm

  \param npts : The number of data points.

  \param x : Data sets to which we are seeking to fit a model M It is assumed
  that x is of size [d x Npts] where d is the dimensionality of the data and
  npts is the number of data points.

  \param s : The minimum number of samples from x required by fitting fn to
  fit a model. Value should be greater or equal to 4.

  \param t : The distance threshold between data point and the model used to
  decide whether a point is an inlier or not.

  \param M : The model having the greatest number of inliers.

  \param inliers :  An array of indices of the elements of x that were the
  inliers for the best model.

  \param consensus :  Consensus

  \param not_used : Unused parameter.

  \param maxNbumbersOfTrials : Maximum number of trials. Even if a solution is
  not found, the method is stopped.

*/

template <class vpTransformation>
bool vpRansac<vpTransformation>::ransac(unsigned int npts, vpColVector &x, unsigned int s, double t, vpColVector &M,
                                        vpColVector &inliers, int consensus, double not_used,
                                        const int maxNbumbersOfTrials)
{
  /*   bool isplanar; */
  /*   if (s == 4) isplanar = true; */
  /*   else isplanar = false; */
  (void)not_used;
  double eps = 1e-6;
  double p = 0.99; // Desired probability of choosing at least one sample
  // free from outliers

  int maxTrials = maxNbumbersOfTrials; // Maximum number of trials before we give up.
  int maxDataTrials = 1000;            // Max number of attempts to select a non-degenerate
  // data set.

  if (s < 4)
    s = 4;

  // Sentinel value allowing detection of solution failure.
  bool solutionFind = false;
  vpColVector bestM;
  int trialcount = 0;
  int bestscore = -1;
  double N = 1; // Dummy initialisation for number of trials.

  vpUniRand random((const long)time(NULL));
  vpColVector bestinliers;
  unsigned int *ind = new unsigned int[s];
  int numiter = 0;
  int ninliers = 0;
  double residual = 0.0;
  while ((N > trialcount) && (consensus > bestscore)) {
    // Select at random s datapoints to form a trial model, M.
    // In selecting these points we have to check that they are not in
    // a degenerate configuration.

    bool degenerate = true;
    int count = 1;

    while (degenerate == true) {
      // Generate s random indicies in the range 1..npts
      for (unsigned int i = 0; i < s; i++)
        ind[i] = (unsigned int)ceil(random() * npts) - 1;

      // Test that these points are not a degenerate configuration.
      degenerate = vpTransformation::degenerateConfiguration(x, ind);
      //   degenerate = feval(degenfn, x(:,ind));

      // Safeguard against being stuck in this loop forever
      count = count + 1;

      if (count > maxDataTrials) {
        delete[] ind;
        vpERROR_TRACE("Unable to select a nondegenerate data set");
        throw(vpException(vpException::fatalError, "Unable to select a nondegenerate data set"));
        // return false; //Useless after a throw() function
      }
    }
    // Fit model to this random selection of data points.
    vpTransformation::computeTransformation(x, ind, M);

    vpColVector d;
    // Evaluate distances between points and model.
    vpTransformation::computeResidual(x, M, d);

    // Find the indices of points that are inliers to this model.
    residual = 0.0;
    ninliers = 0;
    for (unsigned int i = 0; i < npts; i++) {
      double resid = fabs(d[i]);
      if (resid < t) {
        inliers[i] = 1;
        ninliers++;
        residual += fabs(d[i]);
      } else
        inliers[i] = 0;
    }

    if (ninliers > bestscore) // Largest set of inliers so far...
    {
      bestscore = ninliers; // Record data for this model
      bestinliers = inliers;
      bestM = M;
      solutionFind = true;

      // Update estimate of N, the number of trials to ensure we pick,
      // with probability p, a data set with no outliers.

      double fracinliers = (double)ninliers / (double)npts;

      double pNoOutliers = 1 - pow(fracinliers, static_cast<int>(s));

      pNoOutliers = vpMath::maximum(eps, pNoOutliers);     // Avoid division by -Inf
      pNoOutliers = vpMath::minimum(1 - eps, pNoOutliers); // Avoid division by 0.
      N = (log(1 - p) / log(pNoOutliers));
    }

    trialcount = trialcount + 1;
    // Safeguard against being stuck in this loop forever
    if (trialcount > maxTrials) {
      vpTRACE("ransac reached the maximum number of %d trials", maxTrials);
      break;
    }
    numiter++;
  }

  if (solutionFind == true) // We got a solution
  {
    M = bestM;
    inliers = bestinliers;
  } else {
    vpTRACE("ransac was unable to find a useful solution");
    M = 0;
  }

  if (ninliers > 0)
    residual /= ninliers;
  delete[] ind;

  return true;
}

#endif
