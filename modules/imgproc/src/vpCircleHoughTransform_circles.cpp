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
 */

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageMorphology.h>

#include <visp3/imgproc/vpCircleHoughTransform.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{
#if (VISP_CXX_STANDARD == VISP_CXX_STANDARD_98)
float computeEffectiveRadius(const float &votes, const float &weigthedSumRadius)
{
  float r_effective = -1.f;
  if (votes > std::numeric_limits<float>::epsilon()) {
    r_effective = weigthedSumRadius / votes;
  }
  return r_effective;
}
#endif

typedef struct vpDataUpdateRadAccum
{
  int m_nbBins;
  std::pair<float, float> m_centerCandidate;
  std::pair<unsigned int, unsigned int> m_edgePoint;
  const vpImage<float> &m_dIx;
  const vpImage<float> &m_dIy;
  float m_rx;
  float m_ry;
  float m_circlePerfectness2;
  float m_r2;
  float m_minRadius;
  float m_mergingRadiusDiffThresh;
  bool m_recordVotingPoints;

  vpDataUpdateRadAccum(const vpImage<float> &dIx, const vpImage<float> &dIy)
    : m_dIx(dIx)
    , m_dIy(dIy)
  { }
} vpDataUpdateRadAccum;

void
updateRadiusAccumulator(const vpDataUpdateRadAccum &data, std::vector<float> &radiusAccumList,
                        std::vector<float> &radiusActualValueList,
                        std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &votingPoints)
{
  float gx = data.m_dIx[data.m_edgePoint.first][data.m_edgePoint.second];
  float gy = data.m_dIy[data.m_edgePoint.first][data.m_edgePoint.second];
  float grad2 = (gx * gx) + (gy * gy);
  float scalProd = (data.m_rx * gx) + (data.m_ry * gy);
  float scalProd2 = scalProd * scalProd;
  if (scalProd2 >= (data.m_circlePerfectness2 * data.m_r2 * grad2)) {
    // Look for the Radius Candidate Bin RCB_k to which d_ij is "the closest" will have an additional vote
    float r = static_cast<float>(std::sqrt(data.m_r2));
    int r_bin = static_cast<int>(std::floor((r - data.m_minRadius) / data.m_mergingRadiusDiffThresh));
    r_bin = std::min<int>(r_bin, data.m_nbBins - 1);
    if ((r < (data.m_minRadius + (data.m_mergingRadiusDiffThresh * 0.5f)))
        || (r >(data.m_minRadius + (data.m_mergingRadiusDiffThresh * (static_cast<float>(data.m_nbBins - 1) + 0.5f))))) {
      // If the radius is at the very beginning of the allowed radii or at the very end, we do not span the vote
      radiusAccumList[r_bin] += 1.f;
      radiusActualValueList[r_bin] += r;
      if (data.m_recordVotingPoints) {
        votingPoints[r_bin].push_back(data.m_edgePoint);
      }
    }
    else {
      float midRadiusPrevBin = data.m_minRadius + (data.m_mergingRadiusDiffThresh * ((r_bin - 1.f) + 0.5f));
      float midRadiusCurBin = data.m_minRadius + (data.m_mergingRadiusDiffThresh * (r_bin + 0.5f));
      float midRadiusNextBin = data.m_minRadius + (data.m_mergingRadiusDiffThresh * (r_bin + 1.f + 0.5f));

      if ((r >= midRadiusCurBin) && (r <= midRadiusNextBin)) {
        // The radius is at  the end of the current bin or beginning of the next, we span the vote with the next bin
        float voteCurBin = (midRadiusNextBin - r) / data.m_mergingRadiusDiffThresh; // If the difference is big, it means that we are closer to the current bin
        float voteNextBin = 1.f - voteCurBin;
        radiusAccumList[r_bin] += voteCurBin;
        radiusActualValueList[r_bin] += r * voteCurBin;
        radiusAccumList[r_bin + 1] += voteNextBin;
        radiusActualValueList[r_bin + 1] += r * voteNextBin;
        if (data.m_recordVotingPoints) {
          votingPoints[r_bin].push_back(data.m_edgePoint);
          votingPoints[r_bin + 1].push_back(data.m_edgePoint);
        }
      }
      else {
        // The radius is at the end of the previous bin or beginning of the current, we span the vote with the previous bin
        float votePrevBin = (r - midRadiusPrevBin) / data.m_mergingRadiusDiffThresh; // If the difference is big, it means that we are closer to the previous bin
        float voteCurBin = 1.f - votePrevBin;
        radiusAccumList[r_bin] += voteCurBin;
        radiusActualValueList[r_bin] += r * voteCurBin;
        radiusAccumList[r_bin - 1] += votePrevBin;
        radiusActualValueList[r_bin - 1] += r * votePrevBin;
        if (data.m_recordVotingPoints) {
          votingPoints[r_bin].push_back(data.m_edgePoint);
          votingPoints[r_bin - 1].push_back(data.m_edgePoint);
        }
      }
    }
  }
}
}
#endif

void
vpCircleHoughTransform::computeCircleCandidates()
{
  size_t nbCenterCandidates = m_centerCandidatesList.size();
  int nbBins = static_cast<int>(((m_algoParams.m_maxRadius - m_algoParams.m_minRadius) + 1) / m_algoParams.m_mergingRadiusDiffThresh);
  nbBins = std::max<int>(static_cast<int>(1), nbBins); // Avoid having 0 bins, which causes segfault
  std::vector<float> radiusAccumList; // Radius accumulator for each center candidates.
  std::vector<float> radiusActualValueList; // Vector that contains the actual distance between the edge points and the center candidates.

  float rmin2 = m_algoParams.m_minRadius * m_algoParams.m_minRadius;
  float rmax2 = m_algoParams.m_maxRadius * m_algoParams.m_maxRadius;
  float circlePerfectness2 = m_algoParams.m_circlePerfectness * m_algoParams.m_circlePerfectness;

  vpDataUpdateRadAccum data(m_dIx, m_dIy);
  data.m_nbBins = nbBins;
  data.m_circlePerfectness2 = circlePerfectness2;
  data.m_minRadius = m_algoParams.m_minRadius;
  data.m_mergingRadiusDiffThresh = m_algoParams.m_mergingRadiusDiffThresh;
  data.m_recordVotingPoints = m_algoParams.m_recordVotingPoints;

  for (size_t i = 0; i < nbCenterCandidates; ++i) {
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > votingPoints(nbBins); // Vectors that contain the points voting for each radius bin
    std::pair<float, float> centerCandidate = m_centerCandidatesList[i];
    // Initialize the radius accumulator of the candidate with 0s
    radiusAccumList.clear();
    radiusAccumList.resize(nbBins, 0);
    radiusActualValueList.clear();
    radiusActualValueList.resize(nbBins, 0.);

    const unsigned int nbEdgePoints = static_cast<unsigned int>(m_edgePointsList.size());
    for (unsigned int e = 0; e < nbEdgePoints; ++e) {
      const std::pair<unsigned int, unsigned int> &edgePoint = m_edgePointsList[e];

      // For each center candidate CeC_i, compute the distance with each edge point EP_j d_ij = dist(CeC_i; EP_j)
      float rx = edgePoint.second - centerCandidate.second;
      float ry = edgePoint.first - centerCandidate.first;
      float r2 = (rx * rx) + (ry * ry);
      if ((r2 > rmin2) && (r2 < rmax2)) {
        data.m_centerCandidate = centerCandidate;
        data.m_edgePoint = edgePoint;
        data.m_rx = rx;
        data.m_ry = ry;
        data.m_r2 = r2;
        updateRadiusAccumulator(data, radiusAccumList, radiusActualValueList, votingPoints);
      }
    }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    // Lambda to compute the effective radius (i.e. barycenter) of each radius bin
    auto computeEffectiveRadius = [](const float &votes, const float &weigthedSumRadius) {
      float r_effective = -1.f;
      if (votes > std::numeric_limits<float>::epsilon()) {
        r_effective = weigthedSumRadius / votes;
      }
      return r_effective;
      };
#endif

    // Merging similar candidates
    std::vector<float> v_r_effective; // Vector of radius of each candidate after the merge step
    std::vector<float> v_votes_effective; // Vector of number of votes of each candidate after the merge step
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > v_votingPoints_effective; // Vector of voting points after the merge step
    std::vector<bool> v_hasMerged_effective; // Vector indicating if merge has been performed for the different candidates
    for (int idBin = 0; idBin < nbBins; ++idBin) {
      float r_effective = computeEffectiveRadius(radiusAccumList[idBin], radiusActualValueList[idBin]);
      float votes_effective = radiusAccumList[idBin];
      std::vector<std::pair<unsigned int, unsigned int> > votingPoints_effective = votingPoints[idBin];
      bool is_r_effective_similar = (r_effective > 0.f);
      // Looking for potential similar radii in the following bins
      // If so, compute the barycenter radius between them
      int idCandidate = idBin + 1;
      bool hasMerged = false;
      while ((idCandidate < nbBins) && is_r_effective_similar) {
        float r_effective_candidate = computeEffectiveRadius(radiusAccumList[idCandidate], radiusActualValueList[idCandidate]);
        if (std::abs(r_effective_candidate - r_effective) < m_algoParams.m_mergingRadiusDiffThresh) {
          r_effective = ((r_effective * votes_effective) + (r_effective_candidate * radiusAccumList[idCandidate])) / (votes_effective + radiusAccumList[idCandidate]);
          votes_effective += radiusAccumList[idCandidate];
          radiusAccumList[idCandidate] = -.1f;
          radiusActualValueList[idCandidate] = -1.f;
          is_r_effective_similar = true;
          if (m_algoParams.m_recordVotingPoints) {
            // Move elements from votingPoints[idCandidate] to votingPoints_effective.
            // votingPoints[idCandidate] is left in undefined but safe-to-destruct state.
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
            votingPoints_effective.insert(
              votingPoints_effective.end(),
              std::make_move_iterator(votingPoints[idCandidate].begin()),
              std::make_move_iterator(votingPoints[idCandidate].end())
            );
#else
            votingPoints_effective.insert(
              votingPoints_effective.end(),
              votingPoints[idCandidate].begin(),
              votingPoints[idCandidate].end()
            );
#endif
            hasMerged = true;
          }
        }
        else {
          is_r_effective_similar = false;
        }
        ++idCandidate;
      }

      if ((votes_effective > m_algoParams.m_centerMinThresh) && (votes_effective >= (m_algoParams.m_circleVisibilityRatioThresh * 2.f * M_PI_FLOAT * r_effective))) {
        // Only the circles having enough votes and being visible enough are considered
        v_r_effective.push_back(r_effective);
        v_votes_effective.push_back(votes_effective);
        if (m_algoParams.m_recordVotingPoints) {
          v_votingPoints_effective.push_back(votingPoints_effective);
          v_hasMerged_effective.push_back(hasMerged);
        }
      }
    }

    unsigned int nbCandidates = static_cast<unsigned int>(v_r_effective.size());
    for (unsigned int idBin = 0; idBin < nbCandidates; ++idBin) {
      // If the circle of center CeC_i  and radius RCB_k has enough votes, it is added to the list
      // of Circle Candidates
      float r_effective = v_r_effective[idBin];
      vpImageCircle candidateCircle(vpImagePoint(centerCandidate.first, centerCandidate.second), r_effective);
      float proba = computeCircleProbability(candidateCircle, static_cast<unsigned int>(v_votes_effective[idBin]));
      if (proba > m_algoParams.m_circleProbaThresh) {
        m_circleCandidates.push_back(candidateCircle);
        m_circleCandidatesProbabilities.push_back(proba);
        m_circleCandidatesVotes.push_back(static_cast<unsigned int>(v_votes_effective[idBin]));
        if (m_algoParams.m_recordVotingPoints) {
          if (v_hasMerged_effective[idBin]) {
            // Remove potential duplicated points
            std::sort(v_votingPoints_effective[idBin].begin(), v_votingPoints_effective[idBin].end());
            v_votingPoints_effective[idBin].erase(std::unique(v_votingPoints_effective[idBin].begin(), v_votingPoints_effective[idBin].end()), v_votingPoints_effective[idBin].end());
          }
          // Save the points
          m_circleCandidatesVotingPoints.push_back(v_votingPoints_effective[idBin]);
        }
      }
    }
  }
}

float
vpCircleHoughTransform::computeCircleProbability(const vpImageCircle &circle, const unsigned int &nbVotes)
{
  float proba(0.f);
  float visibleArc(static_cast<float>(nbVotes));
  float theoreticalLenght;
  if (mp_mask != nullptr) {
    theoreticalLenght = static_cast<float>(circle.computePixelsInMask(*mp_mask));
  }
  else {
    theoreticalLenght = circle.computeArcLengthInRoI(vpRect(vpImagePoint(0, 0), m_edgeMap.getWidth(), m_edgeMap.getHeight()));
  }
  if (theoreticalLenght < std::numeric_limits<float>::epsilon()) {
    proba = 0.f;
  }
  else {
    proba = std::min(visibleArc / theoreticalLenght, 1.f);
  }
  return proba;
}

void
vpCircleHoughTransform::mergeCircleCandidates()
{
  std::vector<vpImageCircle> circleCandidates = m_circleCandidates;
  std::vector<unsigned int> circleCandidatesVotes = m_circleCandidatesVotes;
  std::vector<float> circleCandidatesProba = m_circleCandidatesProbabilities;
  std::vector<std::vector<std::pair<unsigned int, unsigned int> > > circleCandidatesVotingPoints = m_circleCandidatesVotingPoints;
  // First iteration of merge
  mergeCandidates(circleCandidates, circleCandidatesVotes, circleCandidatesProba, circleCandidatesVotingPoints);

  // Second iteration of merge
  mergeCandidates(circleCandidates, circleCandidatesVotes, circleCandidatesProba, circleCandidatesVotingPoints);

  // Saving the results
  m_finalCircles = circleCandidates;
  m_finalCircleVotes = circleCandidatesVotes;
  m_finalCirclesProbabilities = circleCandidatesProba;
  m_finalCirclesVotingPoints = circleCandidatesVotingPoints;
}

void
vpCircleHoughTransform::mergeCandidates(std::vector<vpImageCircle> &circleCandidates, std::vector<unsigned int> &circleCandidatesVotes,
                                        std::vector<float> &circleCandidatesProba, std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &votingPoints)
{
  size_t nbCandidates = circleCandidates.size();
  size_t i = 0;
  while (i < nbCandidates) {
    vpImageCircle cic_i = circleCandidates[i];
    bool hasPerformedMerge = false;
    // // For each other circle candidate CiC_j do:
    size_t j = i + 1;
    while (j < nbCandidates) {
      vpImageCircle cic_j = circleCandidates[j];
      // // // Compute the similarity between CiC_i and CiC_j
      double distanceBetweenCenters = vpImagePoint::distance(cic_i.getCenter(), cic_j.getCenter());
      double radiusDifference = std::abs(cic_i.getRadius() - cic_j.getRadius());
      bool areCirclesSimilar = ((distanceBetweenCenters < m_algoParams.m_centerMinDist)
                                && (radiusDifference < m_algoParams.m_mergingRadiusDiffThresh)
                                );

      if (areCirclesSimilar) {
        hasPerformedMerge = true;
        // // // If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
        unsigned int totalVotes = circleCandidatesVotes[i] + circleCandidatesVotes[j];
        float totalProba = circleCandidatesProba[i] + circleCandidatesProba[j];
        float newProba = 0.5f * totalProba;
        float newRadius = ((cic_i.getRadius() * circleCandidatesProba[i]) + (cic_j.getRadius() * circleCandidatesProba[j])) / totalProba;
        vpImagePoint newCenter = ((cic_i.getCenter() * circleCandidatesProba[i]) + (cic_j.getCenter() * circleCandidatesProba[j])) / totalProba;
        cic_i = vpImageCircle(newCenter, newRadius);
        circleCandidates[j] = circleCandidates[nbCandidates - 1];
        const unsigned int var2 = 2;
        circleCandidatesVotes[i] = totalVotes / var2; // Compute the mean vote
        circleCandidatesVotes[j] = circleCandidatesVotes[nbCandidates - 1];
        circleCandidatesProba[i] = newProba;
        circleCandidatesProba[j] = circleCandidatesProba[nbCandidates - 1];
        if (m_algoParams.m_recordVotingPoints) {
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
          votingPoints[i].insert(
            votingPoints[i].end(),
            std::make_move_iterator(votingPoints[j].begin()),
            std::make_move_iterator(votingPoints[j].end())
          );
#else
          votingPoints[i].insert(
            votingPoints[i].end(),
            votingPoints[j].begin(),
            votingPoints[j].end()
          );
#endif
          votingPoints.pop_back();
        }
        circleCandidates.pop_back();
        circleCandidatesVotes.pop_back();
        circleCandidatesProba.pop_back();
        --nbCandidates;
        // We do not update j because the new j-th candidate has not been evaluated yet
      }
      else {
        // We will evaluate the next candidate
        ++j;
      }
    }
    // // Add the circle candidate CiC_i, potentially merged with other circle candidates, to the final list of detected circles
    circleCandidates[i] = cic_i;
    if (hasPerformedMerge && m_algoParams.m_recordVotingPoints) {
      // Remove duplicated points
      std::sort(votingPoints[i].begin(), votingPoints[i].end());
      votingPoints[i].erase(std::unique(votingPoints[i].begin(), votingPoints[i].end()), votingPoints[i].end());
    }
    ++i;
  }
}

END_VISP_NAMESPACE
