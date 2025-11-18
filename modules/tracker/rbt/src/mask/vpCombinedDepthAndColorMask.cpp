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

#include <visp3/rbt/vpCombinedDepthAndColorMask.h>

#include <visp3/rbt/vpRBFeatureTrackerInput.h>
#include <visp3/core/vpImageFilter.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

void vpCombinedDepthAndColorMask::updateMask(const vpRBFeatureTrackerInput &frame,
                                      const vpRBFeatureTrackerInput &previousFrame,
                                      vpImage<float> &mask)
{
  m_colorMask.updateMask(frame, previousFrame, m_color);
  m_depthMask.updateMask(frame, previousFrame, m_depth);

  bool computeOnlyOnBB = m_colorMask.isComputedOnlyOnBoundingBox() && m_depthMask.isComputedOnlyOnBoundingBox();
  if (!computeOnlyOnBB) {
    mask.resize(m_color.getHeight(), m_color.getWidth());
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < m_color.getSize(); ++i) {
      mask.bitmap[i] = std::min(m_color.bitmap[i], m_depth.bitmap[i]);
    }
  }
  else {
    mask.resize(m_color.getHeight(), m_color.getWidth(), 0.f);

    const vpRect renderBB = frame.renders.boundingBox;
    const int top = static_cast<int>(renderBB.getTop());
    const int left = static_cast<int>(renderBB.getLeft());
    const int bottom = std::min(static_cast<int>(m_color.getHeight()) - 1, static_cast<int>(renderBB.getBottom()));
    const int right = std::min(static_cast<int>(m_color.getWidth()) - 1, static_cast<int>(renderBB.getRight()));
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = top; i <= bottom; ++i) {
      const float *const colorProbaRow = m_color[i];
      const float *const depthProbaRow = m_depth[i];
      float *const maskRow = mask[i];
      for (unsigned int j = left; j <= static_cast<unsigned int>(right); ++j) {
        maskRow[j] = std::min(colorProbaRow[j], depthProbaRow[j]);
      }
    }
  }
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpCombinedDepthAndColorMask::loadJsonConfiguration(const nlohmann::json &json)
{
  m_colorMask.loadJsonConfiguration(json["color"]);
  m_depthMask.loadJsonConfiguration(json["depth"]);
}
#endif

END_VISP_NAMESPACE
