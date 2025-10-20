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
  \file vpRBPhotometricTracker.h
  \brief Photometric based feature for Render based tracking
*/

#ifndef VP_RB_PHOTOMETRIC_TRACKER_H
#define VP_RB_PHOTOMETRIC_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/rbt/vpRBFeatureTracker.h>

#include <vector>
#include <iostream>
#include <algorithm>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

/**
 * @brief A tracker based on dense depth point-plane alignment.
 *
 * \ingroup group_rbt_trackers
 *
*/
class VISP_EXPORT vpRBPhotometricTracker : public vpRBFeatureTracker
{
public:

  enum vpDisplayType
  {
    DT_SIMPLE = 0,
    DT_WEIGHT = 1,
    DT_ERROR = 2,
    DT_WEIGHT_AND_ERROR = 3,
    DT_INVALID = 4
  };

  vpRBPhotometricTracker() : vpRBFeatureTracker(), m_step(2), m_useMask(false), m_minMaskConfidence(0.f), m_displayType(vpDisplayType::DT_SIMPLE) { }

  virtual ~vpRBPhotometricTracker() = default;

  bool requiresRGB() const VP_OVERRIDE { return false; }
  bool requiresDepth() const VP_OVERRIDE { return false; }
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

  vpDisplayType getDisplayType() const { return m_displayType; }

  void setDisplayType(vpDisplayType type)
  {
    if (type == DT_INVALID) {
      throw vpException(vpException::badValue, "Depth tracker display type is invalid");
    }
    m_displayType = type;
  }

  /**
   * \name Settings
   * @}
   */

  /**
   * @brief Method called when starting a tracking iteration
   */
  void onTrackingIterStart(const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }
  void onTrackingIterEnd(const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }

  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  void trackFeatures(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth) const VP_OVERRIDE;

  struct vpSurfacePoint
  {
    vpSurfacePoint() { }
  public:
    std::array<double, 3> oX;
    double targetLuminance;
    std::array<int, 2> pixelPos;

  };

  class vpSurfacePointSet
  {
  public:
    vpSurfacePointSet()
    {

    }
    void build(const std::vector<vpSurfacePoint> &points)
    {
      unsigned int numPoints = points.size();
      std::vector<vpMatrix *> matrices3 = { &m_oXt, &m_cXt };
      for (vpMatrix *m: matrices3) {
        m->resize(3, numPoints, false, false);
      }
      std::vector<vpMatrix *> matrices2 = { &m_dI, &m_xy };
      for (vpMatrix *m: matrices2) {
        m->resize(numPoints, 2, false, false);
      }
      std::vector<vpColVector *> vectors = { &m_observations, &m_targetLuminance };
      for (vpColVector *m: vectors) {
        m->resize(numPoints, false);
      }
      m_valid.resize(numPoints);

      for (unsigned int i = 0; i < numPoints; ++i) {
        m_targetLuminance[i] = points[i].targetLuminance;
        for (unsigned int j = 0; j < 3; ++j) {
          m_oXt[j][i] = points[i].oX[j];
        }
      }
    }

    inline unsigned int getNumValidPoints() const
    {
      unsigned int res = 0;
      for (unsigned int i = 0; i < m_valid.size(); ++i) {
        if (m_valid[i]) {
          ++res;
        }
      }
      return res;
    }

    inline void update(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpCameraParameters &cam)
    {
      const vpRotationMatrix cRo = cMo.getRotationMatrix();
      const vpTranslationVector t = cMo.getTranslationVector();
      vpMatrix::mult2Matrices(cRo, m_oXt, m_cXt);

      for (unsigned int i = 0; i < m_cXt.getCols(); ++i) {
        m_valid[i] = true;
        for (unsigned int j = 0; j < 3; ++j) {
          m_cXt[j][i] += t[j];
        }
        m_xy[i][0] = m_cXt[0][i] / m_cXt[2][i];
        m_xy[i][1] = m_cXt[1][i] / m_cXt[2][i];
        double u, v;
        vpMeterPixelConversion::convertPointWithoutDistortion(cam, m_xy[i][0], m_xy[i][1], u, v);
        if (u < 5 || v < 5 || u >(I.getWidth() - 5) || v >(I.getHeight() - 5)) { // Check against border. If u or v is negative, we can't cast to uint!
          m_valid[i] = false;
          continue;
        }
        unsigned int ui = static_cast<unsigned int>(u), vi = static_cast<unsigned int>(v);
        float observation = I[vi][ui];
        m_observations[i] = static_cast<double>(observation);
        double px = cam.get_px(), py = cam.get_py();
        m_dI[i][0] = px * vpImageFilter::derivativeFilterX(I, vi, ui);
        m_dI[i][1] = py * vpImageFilter::derivativeFilterY(I, vi, ui);
      }
    }

    inline void errorAndInteraction(vpColVector &e, vpMatrix &L) const
    {
      const unsigned int numPoints = m_oXt.getCols();
      e.resize(numPoints, false);
      L.resize(numPoints, 6, false, false);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < static_cast<int>(numPoints); ++i) {
        const double Z = m_cXt[2][i];
        const double Zinv = 1.0 / Z;
        const double Ix = m_dI[i][0], Iy = m_dI[i][1];
        const double x = m_xy[i][0], y = m_xy[i][1];
        if (m_valid[i]) {
          e[i] = m_observations[i] -  m_targetLuminance[i];

          L[i][0] = Ix * Zinv;
          L[i][1] = Iy * Zinv;
          L[i][2] = -(x * Ix + y * Iy) * Zinv;
          L[i][3] = -Ix * x * y - (1 + y * y) * Iy;
          L[i][4] = (1 + x * x) * Ix + Iy * x * y;
          L[i][5] = Iy * x - Ix * y;
        }
        else {
          e[i] = 0.0;
          for (unsigned int j = 0; j < 6; ++j) {
            L[i][j] = 0.0;
          }
        }
      }
    }

    const vpMatrix &getPointsObject() const { return m_oXt; }
    const vpMatrix &getPointsCamera() const { return m_cXt; }

  private:
    std::vector<bool> m_valid;
    vpColVector m_observations; // Vector containing the observed luminance in the current frame
    vpMatrix m_xy; //! Pixel position of the 3d model point
    vpColVector m_targetLuminance; // Vector containing the model luminance acquired from either the previous frame or the render
    vpMatrix m_dI; // 2XN
    vpMatrix m_oXt; // a 3xN matrix containing the 3D points in the object frame
    vpMatrix m_cXt; // a 3xN matrix containing the 3D points in the camera frame
  };

#if defined(VISP_HAVE_NLOHMANN_JSON)

#if defined(__clang__)
// Mute warning : declaration requires an exit-time destructor [-Wexit-time-destructors]
// message : expanded from macro 'NLOHMANN_JSON_SERIALIZE_ENUM'
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

  NLOHMANN_JSON_SERIALIZE_ENUM(vpRBPhotometricTracker::vpDisplayType, {
      {vpRBPhotometricTracker::vpDisplayType::DT_INVALID, nullptr},
      {vpRBPhotometricTracker::vpDisplayType::DT_SIMPLE, "simple"},
      {vpRBPhotometricTracker::vpDisplayType::DT_WEIGHT, "weight"},
      {vpRBPhotometricTracker::vpDisplayType::DT_ERROR, "error"},
      {vpRBPhotometricTracker::vpDisplayType::DT_WEIGHT_AND_ERROR, "weightAndError"}
    });

  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);
    setStep(j.value("step", m_step));
    setShouldUseMask(j.value("useMask", m_useMask));
    setMinimumMaskConfidence(j.value("minMaskConfidence", m_minMaskConfidence));
    setDisplayType(j.value("displayType", m_displayType));
  }

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif

#endif

protected:

  std::vector<vpSurfacePoint> m_surfacePoints;
  vpSurfacePointSet m_surfacePointsSet;
  vpRobust m_robust;
  unsigned int m_step;
  bool m_useMask;
  float m_minMaskConfidence;
  vpDisplayType m_displayType;
};

END_VISP_NAMESPACE

#endif
