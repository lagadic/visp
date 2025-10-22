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
  \file vpDepthMask.h
  \brief Object mask segmentation based on comparing pixel depth values with object depth values.
*/
#ifndef VP_DEPTH_MASK_H
#define VP_DEPTH_MASK_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/rbt/vpObjectMask.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

class vpRBFeatureTrackerInput;

/**
 * \brief A mask computation algorithm based on depth values.
 * \ingroup group_rbt_mask
*/
class VISP_EXPORT vpDepthMask : public vpObjectMask
{
public:
  vpDepthMask() : m_minRadiusFactor(0.0), m_falloffSmoothingFactor(0.5) { }
  virtual ~vpDepthMask() = default;

  void updateMask(const vpRBFeatureTrackerInput &frame,
                  const vpRBFeatureTrackerInput &previousFrame,
                  vpImage<float> &mask) VP_OVERRIDE;

  virtual void reset() VP_OVERRIDE
  { }

  /**
   * \brief Retrieve the value of the minimum depth tolerated error.
   * It is expressed as a factor of the object's diameter.
   *
   * By default, the accepted depth range  error when computing the mask is computed
   * using the rendered object's clipping planes.
   * If these values are too close to the object's center
   *
   * \return double
  */
  double getMinRadiusFactor() const { return m_minRadiusFactor; }
  void setMinRadiusMeters(double minRadius) { m_minRadiusFactor = minRadius; }

  /**
   * \brief Get the Falloff smoothing factor (of the depth range) strength of the depth probability distribution.
   * It corresponds to the standard deviation of a gaussian distribution,
   * which is used to compute the probability of a depth values when it is not in the accepted depth range.
   * The accepted depth range is computed using the object radius and clipping planes.
   *
   * \return double
  */
  double getFalloffRadiusFactor() const { return m_falloffSmoothingFactor; }
  void setFalloffRadiusFactor(double factor) { m_falloffSmoothingFactor = factor; }


#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadJsonConfiguration(const nlohmann::json &json) VP_OVERRIDE;
#endif

private:
  double m_minRadiusFactor;
  double m_falloffSmoothingFactor;
};

END_VISP_NAMESPACE

#endif
