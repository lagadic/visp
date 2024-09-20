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
  \file vpRBDenseDepthTracker.h
  \brief Dense depth features for render-based tracking
*/

#ifndef VP_RB_DENSE_DEPTH_TRACKER_H
#define VP_RB_DENSE_DEPTH_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>

// #if defined(VISP_HAVE_SIMDLIB)
// #include <Simd/SimdLib.h>
// #endif
#include <visp3/rbt/vpRBFeatureTracker.h>

#include <vector>
#include <iostream>
#include <algorithm>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/**
 * @brief A tracker based on dense depth point-plane alignement
 *
 * \ingroup group_rbt_trackers
 *
*/
class VISP_EXPORT vpRBDenseDepthTracker : public vpRBFeatureTracker
{
public:

  vpRBDenseDepthTracker() : vpRBFeatureTracker(), m_step(2), m_useMask(false), m_minMaskConfidence(0.f) { }

  virtual ~vpRBDenseDepthTracker() = default;

  bool requiresRGB() const VP_OVERRIDE { return false; }
  bool requiresDepth() const VP_OVERRIDE { return true; }
  bool requiresSilhouetteCandidates() const VP_OVERRIDE { return false; }

  /**
   * \name Settings
   * @{
   */
  unsigned int getStep() const { return m_step; }
  void setStep(unsigned int step)
  {
    if (step == 0) {
      throw vpException(vpException::badValue, "Step should be greater than 0");
    }
    m_step = step;
  }

  /**
   * \brief Returns whether the tracking algorithm should filter out points that are unlikely to be on the object according to the mask.
   * If the mask is not computed beforehand, then it has no effect
   */
  bool shouldUseMask() const { return m_useMask; }
  void setShouldUseMask(bool useMask) { m_useMask = useMask; }

  /**
   * \brief Returns the minimum mask confidence that a pixel linked to depth point should have if it should be kept during tracking.
   *
   * This value is between 0 and 1
   */
  float getMinimumMaskConfidence() const { return m_minMaskConfidence; }
  void setMinimumMaskConfidence(float confidence)
  {
    if (confidence > 1.f || confidence < 0.f) {
      throw vpException(vpException::badValue, "Mask confidence should be between 0 and 1");
    }
    m_minMaskConfidence = confidence;
  }

  /**
   * \name Settings
   * @}
   */

  /**
   * @brief Method called when starting a tracking iteration
   */
  void onTrackingIterStart() VP_OVERRIDE { }
  void onTrackingIterEnd() VP_OVERRIDE { }

  double getVVSTrackerWeight() const VP_OVERRIDE { return m_userVvsWeight / (m_error.size()); }

  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  void trackFeatures(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type) const VP_OVERRIDE;

  struct vpDepthPoint
  {

    vpDepthPoint() : currentPoint(3), cameraNormal(3), objectNormal(3)
    {

    }

    inline void update(const vpHomogeneousMatrix &cMo, const vpRotationMatrix &cRo)
    {
      oP.changeFrame(cMo);
      oP.projection();
      cameraNormal = cRo * objectNormal;
      // f.buildFrom(oP.get_x(), oP.get_y(), oP.get_Z(), log(oP.get_Z() / currentPoint.get_Z()));
    }

    inline void error(vpColVector &e, unsigned i) const
    {
      double D = -((cameraNormal[0] * oP.get_X()) + (cameraNormal[1] * oP.get_Y()) + (cameraNormal[2] * oP.get_Z()));
      double projNormal = cameraNormal[0] * currentPoint[0] + cameraNormal[1] * currentPoint[1] + cameraNormal[2] * currentPoint[2];

      e[i] = D + projNormal;
      //e[i] = f.get_LogZoverZstar();
    }

    inline void interaction(vpMatrix &L, unsigned i)
    {
      const double X = currentPoint[0], Y = currentPoint[1], Z = currentPoint[2];
      const double nx = cameraNormal[0], ny = cameraNormal[1], nz = cameraNormal[2];
      L[i][0] = nx;
      L[i][1] = ny;
      L[i][2] = nz;
      L[i][3] = nz * Y - ny * Z;
      L[i][4] = nx * Z - nz * X;
      L[i][5] = ny * X - nx * Y;
      // vpMatrix LL = f.interaction();
      // for (unsigned int j = 0; j < 6; ++j) {
      //   L[i][j] = LL[0][j];
      // }
    }

  public:
    vpPoint oP;
    vpColVector currentPoint;
    vpColVector cameraNormal;
    vpColVector objectNormal;
    vpImagePoint pixelPos;
    //vpFeatureDepth f;
  };

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);
    m_step = j.value("step", m_step);
    m_useMask = j.value("useMask", m_useMask);
    m_minMaskConfidence = j.value("minMaskConfidence", m_minMaskConfidence);
  }

#endif

protected:

  std::vector<vpDepthPoint> m_depthPoints;
  vpRobust m_robust;
  unsigned int m_step;
  bool m_useMask;
  float m_minMaskConfidence;
};

END_VISP_NAMESPACE

#endif
