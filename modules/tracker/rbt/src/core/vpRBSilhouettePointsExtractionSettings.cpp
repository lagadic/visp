
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

#include <stdlib.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>

#include <visp3/rbt/vpRBSilhouettePointsExtractionSettings.h>
#include <visp3/rbt/vpRBSilhouettePoint.h>

BEGIN_VISP_NAMESPACE

vpSilhouettePointsExtractionSettings::vpSilhouettePointsExtractionSettings()
{
  m_depthThreshold = 0.1;
  m_thresholdIsRelative = false;
  m_preferPreviousPoints = false;
  m_sampleStep = 5;
  m_maxNumPoints = 0;
  m_border = 10;
}

vpSilhouettePointsExtractionSettings::vpSilhouettePointsExtractionSettings(const vpSilhouettePointsExtractionSettings &rend)
{
  *this = rend;
}

const vpSilhouettePointsExtractionSettings &vpSilhouettePointsExtractionSettings::operator=(const vpSilhouettePointsExtractionSettings &rend)
{
  m_depthThreshold = rend.m_depthThreshold;
  m_thresholdIsRelative = rend.m_thresholdIsRelative;
  m_sampleStep = rend.m_sampleStep;
  m_maxNumPoints = rend.m_maxNumPoints;
  m_preferPreviousPoints = rend.m_preferPreviousPoints;
  m_border = rend.m_border;
  return *this;
}

std::vector<std::pair<unsigned int, unsigned int>> vpSilhouettePointsExtractionSettings::getSilhouetteCandidates(
 const vpImage<unsigned char> &validSilhouette, const vpImage<float> &renderDepth,
 const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp,
 const std::vector<vpRBSilhouettePoint> &previousPoints, long randomSeed) const
{
  const unsigned int rows = validSilhouette.getHeight();
  const unsigned int cols = validSilhouette.getWidth();

  std::vector<std::pair<unsigned int, unsigned int>> finalCandidates;
  std::vector<std::pair<unsigned int, unsigned int>> candidates;
  if (m_maxNumPoints) {
    finalCandidates.reserve(m_maxNumPoints);
    candidates.reserve(m_maxNumPoints);
  }
  if (m_preferPreviousPoints) {
    for (const vpRBSilhouettePoint &p: previousPoints) {
      double x = 0.0, y = 0.0;
      vpPixelMeterConversion::convertPoint(cam, p.j, p.i, x, y);
      vpColVector cpX({ x * p.Z, y * p.Z, p.Z, 1.0 });
      vpColVector cX = cTcp * cpX;
      cX /= cX[3];
      vpMeterPixelConversion::convertPoint(cam, cX[0] / cX[2], cX[1] / cX[2], x, y);

      unsigned nu = static_cast<unsigned int>(round(x)), nv = static_cast<unsigned int>(round(y));
      if (nu > 0 && nv > 0 && nv < rows && nu < cols) {
        if (validSilhouette[nv][nu] > 0 && fabs((renderDepth[nv][nu] / p.Z) - 1.0) < 0.01) {
          finalCandidates.push_back(std::make_pair(nv, nu));
        }

      }
    }
  }
  if (m_maxNumPoints > 0 && finalCandidates.size() >= static_cast<unsigned int>(m_maxNumPoints)) {
    return finalCandidates;
  }

  for (unsigned int n = m_border; n < rows - m_border; n += m_sampleStep) {
    for (unsigned int m = m_border; m < cols - m_border; m += m_sampleStep) {
      //std::cout << "n = " << n << ", m = " << m << ", h = " << rows << ", w = " << cols <<  std::endl;
      //std::cout << "m = " << m << ", n = " << n << ", s = " << (int)(validSilhouette[n][m]) << std::endl;
      if (validSilhouette[n][m] > 0) {
        candidates.push_back(std::make_pair(n, m));
      }
    }
  }

  if (m_maxNumPoints > 0) {
    vpUniRand random(randomSeed);
    std::vector<size_t> indices(m_maxNumPoints - finalCandidates.size());
    sampleWithoutReplacement(m_maxNumPoints - finalCandidates.size(), candidates.size(), indices, random);
    for (unsigned int i = 0; i < indices.size(); ++i) {
      finalCandidates.push_back(candidates[indices[i]]);
    }
  }
  else {
    for (unsigned int i = 0; i < candidates.size(); ++i) {
      finalCandidates.push_back(candidates[i]);
    }
  }
  return finalCandidates;
}

END_VISP_NAMESPACE
