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

#include <visp3/rbt/vpColorHistogramMask.h>

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE


class vpProbaComputer {
  public:
  vpProbaComputer(const vpColorHistogram& object, const vpColorHistogram& bg) {
    unsigned int N = object.getBinNumber();
    m_probas.resize(N * N * N);
    unsigned int increment = 256 / N;
    unsigned int half = increment / 2;
    vpRGBa c;
    unsigned int index = 0;
    for(unsigned int r = half; r < 256; r += increment) {
      c.R = r;
      for(unsigned int g = half; g < 256; g += increment) {
        c.G = g;
        for(unsigned int b = half; b < 256; b += increment) {
          c.B = b;
          const float pbg = bg.probability(c);
          const float pObject = object.probability(c);
          if (pbg == 0.f) {
            m_probas[index] = pObject > 0.f ? 1.f : 0.f;
          }
          else {
            float logOdds = log(pObject / pbg);
            const float score = 1 / (1 + exp(-logOdds));
            m_probas[index] = score;
          }
          ++index;

        }
      }
    }

  }

  float operator()(unsigned int colorIndex) {
    return m_probas[colorIndex];
  }

  private:
  std::vector<float> m_probas;
};

vpColorHistogramMask::vpColorHistogramMask() : m_depthErrorTolerance(0.01), m_objectUpdateRate(0.1), m_backgroundUpdateRate(0.1), m_threshold(2.f), m_computeOnBBOnly(false) { }

void vpColorHistogramMask::updateMask(const vpRBFeatureTrackerInput &frame,
                                      const vpRBFeatureTrackerInput &previousFrame,
                                      vpImage<float> &mask)
{
  // Prefer the last frame:
  // we have updated the render to match the pose so we should get better object and background histogram separation.
  const vpImage<vpRGBa> &rgb = previousFrame.IRGB.getSize() == 0 ? frame.IRGB : previousFrame.IRGB;

  const int height = static_cast<int>(rgb.getHeight()), width = static_cast<int>(rgb.getWidth());
  m_mask.resize(height, width, false);
  const vpRect renderBB = frame.renders.boundingBox;
  const int top = static_cast<int>(renderBB.getTop());
  const int left = static_cast<int>(renderBB.getLeft());
  const int bottom = std::min(height - 1, static_cast<int>(renderBB.getBottom()));
  const int right = std::min(width - 1, static_cast<int>(renderBB.getRight()));

  const vpImage<float> &renderDepth = frame.renders.depth;
  const vpImage<float> &depth = previousFrame.depth.getSize() == 0 ? frame.depth : previousFrame.depth;
  if (depth.getSize() > 0 && m_depthErrorTolerance > 0.f) {
    #pragma omp parallel for
    for (unsigned int i = top; i <= static_cast<unsigned int>(bottom); ++i) {
      for (unsigned int j = left; j <= static_cast<unsigned int>(right); ++j) {
        m_mask[i][j] = renderDepth[i][j] > 0.f && (depth[i][j] > 0.f && fabs(renderDepth[i][j] - depth[i][j]) <= m_depthErrorTolerance);
      }
    }
  }
  else {
    #pragma omp parallel for
    for (unsigned int i = top; i <= static_cast<unsigned int>(bottom); ++i) {
      for (unsigned int j = left; j <= static_cast<unsigned int>(right); ++j) {
        m_mask[i][j] = renderDepth[i][j] > 0.f;
      }
    }
  }
  vpColorHistogram::computeSplitHistograms(rgb, m_mask, renderBB, m_histObjectFrame, m_histBackgroundFrame);
  const float numPxTotal = static_cast<float>(m_histObjectFrame.getNumPixels() + m_histBackgroundFrame.getNumPixels());

  const float pObject = static_cast<float>(m_histObjectFrame.getNumPixels()) / numPxTotal;
  const float pBackground = static_cast<float>(m_histBackgroundFrame.getNumPixels()) / numPxTotal;
  {
    {
      if (pObject != 0.f) {
        m_histObject.merge(m_histObjectFrame, m_objectUpdateRate);
      }
    }
    {
      if (pBackground != 0.f) {
        m_histBackground.merge(m_histBackgroundFrame, m_backgroundUpdateRate);
      }
    }
  }
  vpProbaComputer probas(m_histObject, m_histBackground);

  if (m_computeOnBBOnly) {
    mask.resize(height, width, 0.f);
#pragma omp parallel for
    for (int i = top; i <= bottom; ++i) {
      for (int j = left; j <= right; ++j) {
        unsigned int index = m_histObject.colorToIndex(frame.IRGB[i][j]);
        mask[i][j] = probas(index);
      }
    }

  }
  else {
    mask.resize(height, width);
    #pragma omp parallel for
    for (unsigned int i = 0; i < mask.getSize(); ++i) {
      unsigned int index = m_histObject.colorToIndex(frame.IRGB.bitmap[i]);
      mask.bitmap[i] = probas(index);
    }
    // if (maxValue > 0.0) {
    //   for (unsigned int i = 0; i < mask.getSize(); ++i) {
    //     mask.bitmap[i] /= maxValue;
    //   }
    // }
  }
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpColorHistogramMask::loadJsonConfiguration(const nlohmann::json &json)
{
  setBinNumber(json.at("bins"));
  m_backgroundUpdateRate = json.at("backgroundUpdateRate");
  m_objectUpdateRate = json.at("objectUpdateRate");
  m_depthErrorTolerance = json.at("maxDepthError");
  m_computeOnBBOnly = json.value("computeOnlyOnBoundingBox", m_computeOnBBOnly);
  m_threshold = json.value("likelihoodRatioThreshold", m_threshold);
}
#endif

END_VISP_NAMESPACE
