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

/*!
  \file vpColorHistogramMask.h
  \brief Object mask estimation through global foreground/background color histogram representations
*/
#ifndef VP_COLOR_HISTOGRAM_MASK_H
#define VP_COLOR_HISTOGRAM_MASK_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/rbt/vpColorHistogram.h>
#include <visp3/rbt/vpObjectMask.h>
#include <visp3/core/vpImage.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json_fwd.hpp>
#endif

BEGIN_VISP_NAMESPACE

class vpRBFeatureTrackerInput;

/**
 * \brief A color histogram based segmentation algorithm.
 * \ingroup group_rbt_mask
*/
class VISP_EXPORT vpColorHistogramMask : public vpObjectMask
{
public:
  vpColorHistogramMask();
  virtual ~vpColorHistogramMask() = default;

  void updateMask(const vpRBFeatureTrackerInput &frame,
                  const vpRBFeatureTrackerInput &previousFrame,
                  vpImage<float> &mask) VP_OVERRIDE;


  /**
   * \name Histogram settings
   * @{
   */
  void setBinNumber(unsigned int N)
  {
    m_histBackground.setBinNumber(N);
    m_histBackgroundFrame.setBinNumber(N);
    m_histObject.setBinNumber(N);
    m_histObjectFrame.setBinNumber(N);
  }

  float getDepthErrorTolerance() const { return m_depthErrorTolerance; }
  void setDepthErrorTolerance(float errorMax)
  {
    if (errorMax < 0.f) {
      throw vpException(vpException::badValue, "Depth error tolerance in histogram computation should be > 0");
    }
    m_depthErrorTolerance = errorMax;
  }

  float getObjectUpdateRate() const { return m_objectUpdateRate; }
  void setObjectUpdateRate(float updateRate)
  {
    if (updateRate < 0.f || updateRate > 1.f) {
      throw vpException(vpException::badValue, "Histogram update rate should be between 0 and 1 (included)");
    }
    m_objectUpdateRate = updateRate;
  }

  float getBackgroundUpdateRate() const { return m_backgroundUpdateRate; }
  void setBackgroundUpdateRate(float updateRate)
  {
    if (updateRate < 0.f || updateRate > 1.f) {
      throw vpException(vpException::badValue, "Histogram update rate should be between 0 and 1 (included)");
    }
    m_backgroundUpdateRate = updateRate;
  }
  /**
   * @}
   */

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadJsonConfiguration(const nlohmann::json &json) VP_OVERRIDE;
#endif

private:
  vpColorHistogram m_histObject, m_histBackground, m_histObjectFrame, m_histBackgroundFrame;
  float m_depthErrorTolerance;
  float m_objectUpdateRate, m_backgroundUpdateRate;

  vpImage<bool> m_mask;
  vpImage<float> m_probaObject, m_probaBackground;
};

END_VISP_NAMESPACE

#endif
