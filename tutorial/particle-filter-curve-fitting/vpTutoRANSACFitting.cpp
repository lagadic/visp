/****************************************************************************
 *
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
*****************************************************************************/

#include "vpTutoRANSACFitting.h"

#include <limits>

namespace tutorial
{

vpTutoRANSACFitting::vpTutoRANSACFitting(const unsigned int &n, const unsigned int &k, const float &thresh, const float &ratioInliers,
                                         const unsigned int &degree)
  : m_degree(degree)
  , m_bestModel(degree)
  , m_bestError(std::numeric_limits<float>::max())
  , m_n(n)
  , m_k(k)
  , m_thresh(thresh)
  , m_ratioInliers(ratioInliers)
{

}

void vpTutoRANSACFitting::fit(const std::vector<vpImagePoint> &pts)
{
  m_bestError = std::numeric_limits<float>::max();
  unsigned int nbIter = 0;
  m_bestModel.reinit();
  unsigned int bestNbInliers = 0;
  unsigned int nbPts = pts.size();
  unsigned int minNbInliers = std::floor(m_ratioInliers * static_cast<float>(nbPts));
  bool hasFirstModel = false;
  std::vector<vpImagePoint> maybeInliers(m_n); // Candidate points to create a new parabola model
  while (nbIter < m_k) {
    std::vector<vpImagePoint> randomizedInput = vpUniRand::shuffleVector<vpImagePoint>(pts); // Candidate points to create a new parabola model
    for (unsigned int i = 0; i < m_n; ++i) {
      maybeInliers[i] = randomizedInput[i];
    }
    vpTutoMeanSquareFitting maybeModel(m_degree); // Candidate model
    maybeModel.fit(maybeInliers); // Compute the candidate model
    std::vector<vpImagePoint> confirmedInliers; // Confirmed inliers corresponding to the candidate model
    // Evaluate all the points to see if they can match the candidate model
    for (unsigned int i = 0; i < nbPts; ++i) {
      float squareError = maybeModel.evaluate(pts[i]);
      if (squareError < m_thresh) {
        confirmedInliers.push_back(pts[i]);
      }
    }
    unsigned int candidateNbInliers = confirmedInliers.size();
    if (confirmedInliers.size() > minNbInliers) {
      // We have a good model
      // Fitting all the confirmed inliers into a refined model
      vpTutoMeanSquareFitting refinedModel(m_degree);
      refinedModel.fit(confirmedInliers);
      // Testing if it is better than our best model
      float refinedError = refinedModel.evaluate(confirmedInliers);
      if (hasFirstModel) {
        if ((refinedError < m_bestError) && (candidateNbInliers > bestNbInliers)) {
          m_bestError = refinedError;
          m_bestModel = refinedModel;
        }
      }
      else {
        m_bestError = refinedError;
        m_bestModel = refinedModel;
        bestNbInliers = candidateNbInliers;
        hasFirstModel = true;
      }
    }
    ++nbIter;
  }
}
}
