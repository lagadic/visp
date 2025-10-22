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

/*!
  \file vpCombinedDepthAndColorMask.h
  \brief Object mask segmentation based on combining depth and color modalities
*/
#ifndef VP_COMBINED_DEPTH_AND_COLOR_MASK_H
#define VP_COMBINED_DEPTH_AND_COLOR_MASK_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/rbt/vpObjectMask.h>
#include <visp3/rbt/vpDepthMask.h>
#include <visp3/rbt/vpColorHistogramMask.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

class vpRBFeatureTrackerInput;

/**
 * \brief An segmentation method based on combining masks that are separately computed on depth and color modalities.
 * The final computed probability of a pixel is the minimum of the probabilities computed from either modalities.
 * \ingroup group_rbt_mask
*/
class VISP_EXPORT vpCombinedDepthAndColorMask : public vpObjectMask
{
public:
  vpCombinedDepthAndColorMask() { }
  virtual ~vpCombinedDepthAndColorMask() = default;

  void updateMask(const vpRBFeatureTrackerInput &frame,
                  const vpRBFeatureTrackerInput &previousFrame,
                  vpImage<float> &mask) VP_OVERRIDE;

  virtual void reset() VP_OVERRIDE
  {
    m_colorMask.reset();
    m_depthMask.reset();
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadJsonConfiguration(const nlohmann::json &json) VP_OVERRIDE;
#endif

private:
  vpColorHistogramMask m_colorMask;
  vpDepthMask m_depthMask;

  vpImage<float> m_depth, m_color;
};

END_VISP_NAMESPACE

#endif
