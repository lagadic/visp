/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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

#include <visp3/rbt/vpDepthMask.h>

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

void vpDepthMask::updateMask(const vpRBFeatureTrackerInput &frame,
                             const vpRBFeatureTrackerInput &/* previousFrame */,
                             vpImage<float> &mask)
{
  if (!frame.hasDepth()) {
    throw vpException(vpException::badValue, "This segmentation method requires a depth image");
  }


  // const float radius = frame.renders.objectDiameter / 2.f;

  const double radius = std::max(m_minRadiusFactor * (frame.renders.objectDiameter / 2.f), (frame.renders.zFar - frame.renders.zNear) / 2.f);
  const double Zo = (frame.renders.cMo * frame.renders.objectCenter)[2];

  const float Zmi = std::min(Zo - radius, frame.renders.zNear);
  const float Zma = std::max(Zo + radius, frame.renders.zFar);
  const float tol = radius * m_falloffSmoothingFactor;
  const float fac = 1.f / (tol * sqrtf(2.f));

  const auto getProba = [Zmi, Zma, fac](double Z) -> float {
    // Missing depth value => Consider likely
    if (Z == 0.f) {
      return 1.f;
    }
    else {
      // Depth values that are close enough to the object's center depth value are correct
      if (Z > Zmi && Z < Zma) {
        return 1.f;
      }
      // Otherwise, use a gradual fallof based on a Gaussian distribution with sigma based on tol
      else {
        if (Z <= Zmi) {
          return erfc((Zmi - Z) * fac);
        }
        else {
          return erfc((Z - Zma) * fac);
        }
      }
    }};

  if (!m_computeOnBBOnly) {
    mask.resize(frame.depth.getHeight(), frame.depth.getWidth());
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(frame.depth.getSize()); ++i) {
      const float Z = frame.depth.bitmap[i];
      getProba(Z);
    }
  }
  else {
    mask.resize(frame.depth.getHeight(), frame.depth.getWidth(), 0.f);
    const vpRect renderBB = frame.renders.boundingBox;
    const int top = static_cast<int>(renderBB.getTop());
    const int left = static_cast<int>(renderBB.getLeft());
    const int bottom = std::min(static_cast<int>(frame.depth.getHeight()) - 1, static_cast<int>(renderBB.getBottom()));
    const int right = std::min(static_cast<int>(frame.depth.getWidth()) - 1, static_cast<int>(renderBB.getRight()));
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = top; i <= bottom; ++i) {
      float *const maskRow = mask[i];
      const float *const depthRow = frame.depth[i];
      for (unsigned int j = left; j <= static_cast<unsigned int>(right); ++j) {
        maskRow[j] = getProba(depthRow[j]);
      }
    }
  }
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpDepthMask::loadJsonConfiguration(const nlohmann::json &json)
{
  m_minRadiusFactor = json.value("minRadiusFactor", m_minRadiusFactor);
  m_falloffSmoothingFactor = json.value("falloffRadiusFactor", m_falloffSmoothingFactor);
  m_computeOnBBOnly = json.value("computeOnlyOnBoundingBox", m_computeOnBBOnly);
}
#endif

END_VISP_NAMESPACE
