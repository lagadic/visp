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

namespace
{
  /**
 * \brief Data required to update the center candidates accumulator along the
 * gradient direction.
 */
typedef struct vpDataForAccumLoop
{
  unsigned int r; /*!< The row of the edge-point of interest.*/
  unsigned int c; /*!< The column of the edge-point of interest.*/
  float minRadius; /*!< The minimum radius of the searched circle.s.*/
  float maxRadius; /*!< The maximum radius of the searched circle.s.*/
  float minimumXpositionFloat; /*!< The minimum x-axis position of the center in the image.*/
  float minimumYpositionFloat; /*!< The minimum y-axis position of the center in the image.*/
  float maximumXpositionFloat; /*!< The maximum x-axis position of the center in the image.*/
  float maximumYpositionFloat; /*!< The maximum y-axis position of the center in the image.*/
  int offsetX; /*!< The offset to map the accumulator indices with the minimum x-axis value.*/
  int offsetY; /*!< The offset to map the accumulator indices with the minimum y-axis value.*/
  int accumulatorWidth; /*!< The width of the accumulator.*/
  int accumulatorHeight; /*!< The height of the accumulator.*/
}vpDataForAccumLoop;

/**
 * \brief Store the coordinates for single step of update of the center candidates accumulator.
 */
typedef struct vpCoordinatesForAccumStep
{
  float x_orig;
  float y_orig;
  int x;
  int y;
}vpCoordinatesForAccumStep;

#if (VISP_CXX_STANDARD == VISP_CXX_STANDARD_98)
void updateAccumulator(const vpCoordinatesForAccumStep &coord,
                       const vpDataForAccumLoop &data,
                       vpImage<float> &accum, bool &hasToStop)
{
  if (((coord.x - data.offsetX) < 0) ||
      ((coord.x - data.offsetX) >= data.accumulatorWidth) ||
      ((coord.y - data.offsetY) < 0) ||
      ((coord.y - data.offsetY) >= data.accumulatorHeight)
      ) {
    hasToStop = true;
  }
  else {
    float dx = (coord.x_orig - static_cast<float>(coord.x));
    float dy = (coord.y_orig - static_cast<float>(coord.y));
    accum[coord.y - data.offsetY][coord.x - data.offsetX] += std::abs(dx) + std::abs(dy);
  }
}

bool sortingCenters(const vpCircleHoughTransform::vpCenterVotes &position_vote_a,
                    const vpCircleHoughTransform::vpCenterVotes &position_vote_b)
{
  return position_vote_a.m_votes > position_vote_b.m_votes;
}
#endif

/**
 * \brief Update the center accumulator along the positive and negative gradient direction
 * starting from an edge-point of interest.
 *
 * \param[in] data The data required for the algorithm.
 * \param[out] sx The gradient along x.
 * \param[out] sy The gradient along y.
 * \param[out] centersAccum The center candidates accumulator.
 */
void
updateAccumAlongGradientDir(const vpDataForAccumLoop &data, float &sx, float &sy, vpImage<float> &centersAccum)
{
  static const int nbDirections = 2;
  for (int k1 = 0; k1 < nbDirections; ++k1) {
    bool hasToStopLoop = false;
    int x_low_prev = std::numeric_limits<int>::max(), y_low_prev, y_high_prev;
    int x_high_prev = (y_low_prev = (y_high_prev = x_low_prev));

    float rstart = data.minRadius, rstop = data.maxRadius;
    float min_minus_c = data.minimumXpositionFloat - static_cast<float>(data.c);
    float min_minus_r = data.minimumYpositionFloat - static_cast<float>(data.r);
    float max_minus_c = data.maximumXpositionFloat - static_cast<float>(data.c);
    float max_minus_r = data.maximumYpositionFloat - static_cast<float>(data.r);
    if (sx > 0) {
      float rmin = min_minus_c / sx;
      rstart = std::max<float>(rmin, data.minRadius);
      float rmax = max_minus_c / sx;
      rstop = std::min<float>(rmax, data.maxRadius);
    }
    else if (sx < 0) {
      float rmin = max_minus_c / sx;
      rstart = std::max<float>(rmin, data.minRadius);
      float rmax = min_minus_c / sx;
      rstop = std::min<float>(rmax, data.maxRadius);
    }

    if (sy > 0) {
      float rmin = min_minus_r / sy;
      rstart = std::max<float>(rmin, rstart);
      float rmax = max_minus_r / sy;
      rstop = std::min<float>(rmax, rstop);
    }
    else if (sy < 0) {
      float rmin = max_minus_r / sy;
      rstart = std::max<float>(rmin, rstart);
      float rmax = min_minus_r / sy;
      rstop = std::min<float>(rmax, rstop);
    }

    float deltar_x = 1.f / std::abs(sx), deltar_y = 1.f / std::abs(sy);
    float deltar = std::min<float>(deltar_x, deltar_y);

    float rad = rstart;
    while ((rad <= rstop) && (!hasToStopLoop)) {
      float x1 = static_cast<float>(data.c) + (rad * sx);
      float y1 = static_cast<float>(data.r) + (rad * sy);
      rad += deltar; // Update rad that is not used below not to forget it

      bool xOutsideRoI = (x1 < data.minimumXpositionFloat) || (x1 > data.maximumXpositionFloat);
      bool yOutsideRoI = (y1 < data.minimumYpositionFloat) || (y1 > data.maximumYpositionFloat);
      // Continue only if the center is inside the search region.
      if (!(xOutsideRoI || yOutsideRoI)) {
        int x_low, x_high, y_low, y_high;

        if (x1 > 0.) {
          x_low = static_cast<int>(std::floor(x1));
          x_high = static_cast<int>(std::ceil(x1));
        }
        else {
          x_low = -(static_cast<int>(std::ceil(-x1)));
          x_high = -(static_cast<int>(std::floor(-x1)));
        }

        if (y1 > 0.) {
          y_low = static_cast<int>(std::floor(y1));
          y_high = static_cast<int>(std::ceil(y1));
        }
        else {
          y_low = -(static_cast<int>(std::ceil(-1. * y1)));
          y_high = -(static_cast<int>(std::floor(-1. * y1)));
        }

        bool xHasNotChanged = (x_low_prev == x_low) && (x_high_prev == x_high);
        bool yHasNotChanged = (y_low_prev == y_low) && (y_high_prev == y_high);

        // Avoid duplicated votes to the same center candidate
        if (!(xHasNotChanged && yHasNotChanged)) {
          x_low_prev = x_low;
          x_high_prev = x_high;
          y_low_prev = y_low;
          y_high_prev = y_high;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
          auto updateAccumulator =
            [](const vpCoordinatesForAccumStep &coord,
                       const vpDataForAccumLoop &data,
                       vpImage<float> &accum, bool &hasToStop)
            {
              if (((coord.x - data.offsetX) < 0) ||
                  ((coord.x - data.offsetX) >= data.accumulatorWidth) ||
                  ((coord.y - data.offsetY) < 0) ||
                  ((coord.y - data.offsetY) >= data.accumulatorHeight)
                  ) {
                hasToStop = true;
              }
              else {
                float dx = (coord.x_orig - static_cast<float>(coord.x));
                float dy = (coord.y_orig - static_cast<float>(coord.y));
                accum[coord.y - data.offsetY][coord.x - data.offsetX] += std::abs(dx) + std::abs(dy);
              }
        };
#endif
          vpCoordinatesForAccumStep coords;
          coords.x_orig = x1;
          coords.y_orig = y1;
          coords.x = x_low;
          coords.y = y_low;
          updateAccumulator(coords, data, centersAccum, hasToStopLoop);

          coords.x = x_high;
          coords.y = y_high;
          updateAccumulator(coords, data, centersAccum, hasToStopLoop);
      }
    }
  }
    sx = -sx;
    sy = -sy;
}
}
}


// Static variables
const unsigned char vpCircleHoughTransform::edgeMapOn = 255;
const unsigned char vpCircleHoughTransform::edgeMapOff = 0;

void
vpCircleHoughTransform::computeGradients(const vpImage<unsigned char> &I)
{
  if ((m_algoParams.m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
      || (m_algoParams.m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING)) {
    // Computing the Gaussian blurr
    vpImage<float> Iblur, GIx;
    vpImageFilter::filterX(I, GIx, m_fg.data, m_algoParams.m_gaussianKernelSize, mp_mask);
    vpImageFilter::filterY(GIx, Iblur, m_fg.data, m_algoParams.m_gaussianKernelSize, mp_mask);

    // Computing the gradients
    vpImageFilter::filter(Iblur, m_dIx, m_gradientFilterX, true, mp_mask);
    vpImageFilter::filter(Iblur, m_dIy, m_gradientFilterY, true, mp_mask);
  }
  else {
    std::string errMsg("[computeGradients] The filtering + gradient operators \"");
    errMsg += vpImageFilter::vpCannyFiltAndGradTypeToStr(m_algoParams.m_filteringAndGradientType);
    errMsg += "\" is not implemented (yet).";
    throw(vpException(vpException::notImplementedError, errMsg));
  }
}

void
vpCircleHoughTransform::edgeDetection(const vpImage<unsigned char> &I)
{
  if (m_algoParams.m_cannyBackendType == vpImageFilter::CANNY_VISP_BACKEND) {
    // This is done to increase the time performances, because it avoids to
    // recompute the gradient in the vpImageFilter::canny method
    m_cannyVisp.setFilteringAndGradientType(m_algoParams.m_filteringAndGradientType);
    m_cannyVisp.setCannyThresholds(m_algoParams.m_lowerCannyThresh, m_algoParams.m_upperCannyThresh);
    m_cannyVisp.setCannyThresholdsRatio(m_algoParams.m_lowerCannyThreshRatio, m_algoParams.m_upperCannyThreshRatio);
    m_cannyVisp.setGradients(m_dIx, m_dIy);
    m_cannyVisp.setMask(mp_mask);
    m_edgeMap = m_cannyVisp.detect(I);
  }
  else {
    if (mp_mask != nullptr) {
      // Delete pixels that fall outside the mask
      vpImage<unsigned char> I_masked(I);
      unsigned int nbRows = I_masked.getHeight();
      unsigned int nbCols = I_masked.getWidth();
      for (unsigned int r = 0; r < nbRows; ++r) {
        for (unsigned int c = 0; c < nbCols; ++c) {
          if (!((*mp_mask)[r][c])) {
            I_masked[r][c] = 0;
          }
        }
      }

      // We will have to recompute the gradient in the desired backend format anyway so we let
      // the vpImageFilter::canny method take care of it
      vpImageFilter::canny(I_masked, m_edgeMap, m_algoParams.m_gaussianKernelSize, m_algoParams.m_lowerCannyThresh,
                           m_algoParams.m_upperCannyThresh, m_algoParams.m_gradientFilterKernelSize, m_algoParams.m_gaussianStdev,
                           m_algoParams.m_lowerCannyThreshRatio, m_algoParams.m_upperCannyThreshRatio, true,
                           m_algoParams.m_cannyBackendType, m_algoParams.m_filteringAndGradientType);
    }
    else {
      vpImageFilter::canny(I, m_edgeMap, m_algoParams.m_gaussianKernelSize, m_algoParams.m_lowerCannyThresh,
                           m_algoParams.m_upperCannyThresh, m_algoParams.m_gradientFilterKernelSize, m_algoParams.m_gaussianStdev,
                           m_algoParams.m_lowerCannyThreshRatio, m_algoParams.m_upperCannyThreshRatio, true,
                           m_algoParams.m_cannyBackendType, m_algoParams.m_filteringAndGradientType);
    }
  }

  for (int i = 0; i < m_algoParams.m_edgeMapFilteringNbIter; ++i) {
    filterEdgeMap();
  }
}

void
vpCircleHoughTransform::filterEdgeMap()
{
  vpImage<unsigned char> J = m_edgeMap;
  const unsigned int height = J.getHeight();
  const unsigned int width = J.getWidth();
  const int minNbContiguousPts = 2;

  for (unsigned int i = 1; i < (height - 1); ++i) {
    for (unsigned int j = 1; j < (width - 1); ++j) {
      if (J[i][j] == vpCircleHoughTransform::edgeMapOn) {
        // Consider 8 neighbors
        int topLeftPixel = static_cast<int>(J[i - 1][j - 1]);
        int topPixel = static_cast<int>(J[i - 1][j]);
        int topRightPixel = static_cast<int>(J[i - 1][j + 1]);
        int botLeftPixel = static_cast<int>(J[i + 1][j - 1]);
        int bottomPixel = static_cast<int>(J[i + 1][j]);
        int botRightPixel = static_cast<int>(J[i + 1][j + 1]);
        int leftPixel = static_cast<int>(J[i][j - 1]);
        int rightPixel = static_cast<int>(J[i][j + 1]);
        if ((topLeftPixel + topPixel + topRightPixel
             + botLeftPixel + bottomPixel + botRightPixel
             + leftPixel + rightPixel
             ) >= (minNbContiguousPts * static_cast<int>(vpCircleHoughTransform::edgeMapOn))) {
          // At least minNbContiguousPts of the 8-neighbor points are also an edge point
          // so we keep the edge point
          m_edgeMap[i][j] = vpCircleHoughTransform::edgeMapOn;
        }
        else {
          // The edge point is isolated => we erase it
          m_edgeMap[i][j] = vpCircleHoughTransform::edgeMapOff;
        }
      }
    }
  }
}

void
vpCircleHoughTransform::computeCenterCandidates()
{
  // For each edge point EP_i, check the image gradient at EP_i
  // Then, for each image point in the direction of the gradient,
  // increment the accumulator
  // We can perform bilinear interpolation in order not to vote for a "line" of
  // points, but for an "area" of points
  unsigned int nbRows = m_edgeMap.getRows(), nbCols = m_edgeMap.getCols();

  // Computing the minimum and maximum horizontal position of the center candidates
  // The miminum horizontal position of the center is at worst -maxRadius outside the image
  // The maxinum horizontal position of the center is at worst +maxRadiusoutside the image
  // The width of the accumulator is the difference between the max and the min
  int minimumXposition = std::max<int>(m_algoParams.m_centerXlimits.first, -1 * static_cast<int>(m_algoParams.m_maxRadius));
  int maximumXposition = std::min<int>(m_algoParams.m_centerXlimits.second, static_cast<int>(m_algoParams.m_maxRadius + nbCols));
  minimumXposition = std::min<int>(minimumXposition, maximumXposition - 1);
  float minimumXpositionFloat = static_cast<float>(minimumXposition);
  float maximumXpositionFloat = static_cast<float>(maximumXposition);
  int offsetX = minimumXposition;
  int accumulatorWidth = (maximumXposition - minimumXposition) + 1;
  if (accumulatorWidth <= 0) {
    throw(vpException(vpException::dimensionError, "[vpCircleHoughTransform::computeCenterCandidates] Accumulator width <= 0!"));
  }

  // Computing the minimum and maximum vertical position of the center candidates
  // The miminum vertical position of the center is at worst -maxRadius outside the image
  // The maxinum vertical position of the center is at worst +maxRadiusoutside the image
  // The height of the accumulator is the difference between the max and the min
  int minimumYposition = std::max<int>(m_algoParams.m_centerYlimits.first, -1 * static_cast<int>(m_algoParams.m_maxRadius));
  int maximumYposition = std::min<int>(m_algoParams.m_centerYlimits.second, static_cast<int>(m_algoParams.m_maxRadius + nbRows));
  minimumYposition = std::min<int>(minimumYposition, maximumYposition - 1);
  float minimumYpositionFloat = static_cast<float>(minimumYposition);
  float maximumYpositionFloat = static_cast<float>(maximumYposition);
  int offsetY = minimumYposition;
  int accumulatorHeight = (maximumYposition - minimumYposition) + 1;
  if (accumulatorHeight <= 0) {
    throw(vpException(vpException::dimensionError, "[vpCircleHoughTransform::computeCenterCandidates] Accumulator height <= 0!"));
  }

  vpImage<float> centersAccum(accumulatorHeight, accumulatorWidth + 1, 0.); /*!< Votes for the center candidates.*/

  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      if (m_edgeMap[r][c] == vpCircleHoughTransform::edgeMapOn) {
        // Voting for points in both direction of the gradient
        // Step from min_radius to max_radius in both directions of the gradient
        float mag = std::sqrt((m_dIx[r][c] * m_dIx[r][c]) + (m_dIy[r][c] * m_dIy[r][c]));

        float sx = 0.f, sy = 0.f;
        if (std::abs(mag) >= std::numeric_limits<float>::epsilon()) {
          sx = m_dIx[r][c] / mag;
          sy = m_dIy[r][c] / mag;

                  // Saving the edge point for further use
          m_edgePointsList.push_back(std::pair<unsigned int, unsigned int>(r, c));

          vpDataForAccumLoop data;
          data.accumulatorHeight = accumulatorHeight;
          data.accumulatorWidth = accumulatorWidth;
          data.c = c;
          data.maximumXpositionFloat = maximumXpositionFloat;
          data.maximumYpositionFloat = maximumYpositionFloat;
          data.maxRadius = m_algoParams.m_maxRadius;
          data.minimumXpositionFloat = minimumXpositionFloat;
          data.minimumYpositionFloat = minimumYpositionFloat;
          data.minRadius = m_algoParams.m_minRadius;
          data.offsetX = offsetX;
          data.offsetY = offsetY;
          data.r = r;
          updateAccumAlongGradientDir(data, sx, sy, centersAccum);
        }
      }
    }
  }

  // Use dilatation with large kernel in order to determine the
  // accumulator maxima
  vpImage<float> centerCandidatesMaxima = centersAccum;
  int dilatationKernelSize = std::max<int>(m_algoParams.m_dilatationKernelSize, 3); // Ensure at least a 3x3 dilatation operation is performed
  vpImageMorphology::dilatation(centerCandidatesMaxima, dilatationKernelSize);

  // Look for the image points that correspond to the accumulator maxima
  // These points will become the center candidates
  // find the possible circle centers
  int nbColsAccum = centersAccum.getCols();
  int nbRowsAccum = centersAccum.getRows();
  int nbVotes = -1;
  std::vector<vpCenterVotes> peak_positions_votes;

  for (int y = 0; y < nbRowsAccum; ++y) {
    int left = -1;
    for (int x = 0; x < nbColsAccum; ++x) {
      if ((centersAccum[y][x] >= m_algoParams.m_centerMinThresh)
          && (vpMath::equal(centersAccum[y][x], centerCandidatesMaxima[y][x]))
          && (centersAccum[y][x] > centersAccum[y][x + 1])
          ) {
        if (left < 0) {
          left = x;
        }
        nbVotes = std::max<int>(nbVotes, static_cast<int>(centersAccum[y][x]));
      }
      else if (left >= 0) {
        int cx = static_cast<int>(((left + x) - 1) * 0.5f);
        float sumVotes = 0., x_avg = 0., y_avg = 0.;
        int averagingWindowHalfSize = m_algoParams.m_averagingWindowSize / 2;
        int startingRow = std::max<int>(0, y - averagingWindowHalfSize);
        int startingCol = std::max<int>(0, cx - averagingWindowHalfSize);
        int endRow = std::min<int>(accumulatorHeight, y + averagingWindowHalfSize + 1);
        int endCol = std::min<int>(accumulatorWidth, cx + averagingWindowHalfSize + 1);
        for (int r = startingRow; r < endRow; ++r) {
          for (int c = startingCol; c < endCol; ++c) {
            sumVotes += centersAccum[r][c];
            x_avg += centersAccum[r][c] * c;
            y_avg += centersAccum[r][c] * r;
          }
        }
        float avgVotes = sumVotes / static_cast<float>(m_algoParams.m_averagingWindowSize * m_algoParams.m_averagingWindowSize);
        if (avgVotes > m_algoParams.m_centerMinThresh) {
          x_avg /= static_cast<float>(sumVotes);
          y_avg /= static_cast<float>(sumVotes);
          std::pair<float, float> position(y_avg + static_cast<float>(offsetY), x_avg + static_cast<float>(offsetX));
          vpCenterVotes position_vote;
          position_vote.m_position = position;
          position_vote.m_votes = avgVotes;
          peak_positions_votes.push_back(position_vote);
        }
        if (nbVotes < 0) {
          std::stringstream errMsg;
          errMsg << "nbVotes (" << nbVotes << ") < 0, thresh = " << m_algoParams.m_centerMinThresh;
          throw(vpException(vpException::badValue, errMsg.str()));
        }
        left = -1;
        nbVotes = -1;
      }
    }
  }
  filterCenterCandidates(peak_positions_votes);
}

void
vpCircleHoughTransform::filterCenterCandidates(const std::vector<vpCenterVotes> &peak_positions_votes)
{
  unsigned int nbPeaks = static_cast<unsigned int>(peak_positions_votes.size());
  if (nbPeaks > 0) {
    std::vector<bool> has_been_merged(nbPeaks, false);
    std::vector<vpCenterVotes> merged_peaks_position_votes;
    float squared_distance_max = m_algoParams.m_centerMinDist * m_algoParams.m_centerMinDist;
    for (unsigned int idPeak = 0; idPeak < nbPeaks; ++idPeak) {
      float votes = peak_positions_votes[idPeak].m_votes;
      // Ignoring peak that has already been merged
      if (!has_been_merged[idPeak]) {
        if (votes < m_algoParams.m_centerMinThresh) {
        // Ignoring peak whose number of votes is lower than the threshold
          has_been_merged[idPeak] = true;
        }
        else {
          vpCentersBarycenter barycenter = mergeSimilarCenters(idPeak, nbPeaks, squared_distance_max, peak_positions_votes, has_been_merged);

          float avg_votes = barycenter.m_totalVotes / barycenter.m_nbElectors;
          // Only the centers having enough votes are considered
          if (avg_votes > m_algoParams.m_centerMinThresh) {
            barycenter.m_position.first /= barycenter.m_totalVotes;
            barycenter.m_position.second /= barycenter.m_totalVotes;
            vpCenterVotes barycenter_votes;
            barycenter_votes.m_position = barycenter.m_position;
            barycenter_votes.m_votes = avg_votes;
            merged_peaks_position_votes.push_back(barycenter_votes);
          }
        }
      }
    }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    auto sortingCenters = [](const vpCenterVotes &position_vote_a,
                             const vpCenterVotes &position_vote_b) {
                               return position_vote_a.m_votes > position_vote_b.m_votes;
      };
#endif

    std::sort(merged_peaks_position_votes.begin(), merged_peaks_position_votes.end(), sortingCenters);

    nbPeaks = static_cast<unsigned int>(merged_peaks_position_votes.size());
    int nbPeaksToKeep = (m_algoParams.m_expectedNbCenters > 0 ? m_algoParams.m_expectedNbCenters : static_cast<int>(nbPeaks));
    nbPeaksToKeep = std::min<int>(nbPeaksToKeep, static_cast<int>(nbPeaks));
    for (int i = 0; i < nbPeaksToKeep; ++i) {
      m_centerCandidatesList.push_back(merged_peaks_position_votes[i].m_position);
      m_centerVotes.push_back(static_cast<int>(merged_peaks_position_votes[i].m_votes));
    }
          }
        }

vpCircleHoughTransform::vpCentersBarycenter
vpCircleHoughTransform::mergeSimilarCenters(const unsigned int &idPeak, const unsigned int &nbPeaks, const float &squared_distance_max, const std::vector<vpCenterVotes> &peak_positions_votes, std::vector<bool> &has_been_merged)
{
  std::pair<float, float> position = peak_positions_votes[idPeak].m_position;
  vpCentersBarycenter barycenter;
  barycenter.m_position.first = position.first * peak_positions_votes[idPeak].m_votes;
  barycenter.m_position.second = position.second * peak_positions_votes[idPeak].m_votes;
  barycenter.m_totalVotes = peak_positions_votes[idPeak].m_votes;
  barycenter.m_nbElectors = 1.f;
  // Looking for potential similar peak in the following peaks
  for (unsigned int idCandidate = idPeak + 1; idCandidate < nbPeaks; ++idCandidate) {
    float votes_candidate = peak_positions_votes[idCandidate].m_votes;
    // Ignoring peaks that have already been merged
    if (!has_been_merged[idCandidate]) {
      if (votes_candidate < m_algoParams.m_centerMinThresh) {
      // Ignoring peak whose number of votes is lower than the threshold
        has_been_merged[idCandidate] = true;
      }
      else {
        // Computing the distance with the peak of insterest
        std::pair<float, float> position_candidate = peak_positions_votes[idCandidate].m_position;
        float squared_distance = ((position.first - position_candidate.first) * (position.first - position_candidate.first))
          + ((position.second - position_candidate.second) * (position.second - position_candidate.second));

        // If the peaks are similar, update the barycenter peak between them and corresponding votes
        if (squared_distance < squared_distance_max) {
          barycenter.m_position.first += position_candidate.first * votes_candidate;
          barycenter.m_position.second += position_candidate.second * votes_candidate;
          barycenter.m_totalVotes += votes_candidate;
          barycenter.m_nbElectors += 1.f;
          has_been_merged[idCandidate] = true;
        }
      }
    }
  }
  return barycenter;
}

END_VISP_NAMESPACE
