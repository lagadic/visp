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
  \file vpRBDenseDepthTracker.h
  \brief Dense depth features for render-based tracking
*/

#ifndef VP_RB_DENSE_DEPTH_TRACKER_H
#define VP_RB_DENSE_DEPTH_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>

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
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

/**
 * @brief A tracker based on dense depth point-plane alignment.
 *
 * \ingroup group_rbt_trackers
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you want to have an in-depth presentation of the Render-Based Tracker (RBT), you may have a look at:</span><br>
 *
 * - \ref tutorial-tracking-rbt
*/
class VISP_EXPORT vpRBDenseDepthTracker : public vpRBFeatureTracker
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

  vpRBDenseDepthTracker() : vpRBFeatureTracker(), m_step(2), m_maxFeatures(0), m_useMask(false), m_minMaskConfidence(0.f), m_displayType(vpDisplayType::DT_SIMPLE) { }

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

  unsigned int getMaxNumFeatures() const { return m_maxFeatures; }
  void setMaxNumFeatures(unsigned int num)
  {
    m_maxFeatures = num;
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
  void onTrackingIterStart(const vpRBFeatureTrackerInput &, const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }
  void onTrackingIterEnd(const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }

  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  void trackFeatures(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/) VP_OVERRIDE { }
  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth) const VP_OVERRIDE;

  struct vpDepthPoint
  {
    vpDepthPoint() { }
  public:
    std::array<double, 3> oX, observation, objectNormal;
    std::array<double, 2> pixelPos;

  };

  class vpDepthPointSet
  {
  public:
    vpDepthPointSet()
    {

    }
    void build(const std::vector<vpDepthPoint> &points)
    {
      unsigned int numPoints = static_cast<unsigned int>(points.size());
      std::vector<vpMatrix *> matrices = { &m_oXt, &m_oNt, &m_cXt, &m_cNt, &m_observations };
      for (vpMatrix *m: matrices) {
        m->resize(3, numPoints, false, false);
      }
      m_valid.resize(numPoints);

      for (unsigned int i = 0; i < numPoints; ++i) {
        m_valid[i] = true;
        for (unsigned int j = 0; j < 3; ++j) {
          m_oXt[j][i] = points[i].oX[j];
          m_oNt[j][i] = points[i].objectNormal[j];
          m_observations[j][i] = points[i].observation[j];
        }
      }
    }
    inline void updateAndErrorAndInteractionMatrix(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, const vpImage<float> &depth, vpColVector &e, vpMatrix &L)
    {
      const vpRotationMatrix cRo = cMo.getRotationMatrix();
      const vpTranslationVector t = cMo.getTranslationVector();
      vpMatrix::mult2Matrices(cRo, m_oXt, m_cXt);
      vpMatrix::mult2Matrices(cRo, m_oNt, m_cNt);

      const unsigned int numPoints = m_oXt.getCols();
      e.resize(numPoints, false);
      L.resize(numPoints, 6, false, false);

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < static_cast<int>(numPoints); ++i) {

        //Step 1: update and filter out points that are no longer valid
        {

          for (unsigned int j = 0; j < 3; ++j) {
            m_cXt[j][i] += t[j];
          }
          // Plane points away from the camera: this surface is no longer visible due to rotation
          if (m_cNt[2][i] >= 0.0) {
            m_valid[i] = false;
            continue;
          }
          double x, y, u, v;
          x = m_cXt[0][i] / m_cXt[2][i];
          y = m_cXt[1][i] / m_cXt[2][i];

          vpMeterPixelConversion::convertPointWithoutDistortion(cam, x, y, u, v);
          // Point is no longer in image: depth value cannot be sampled
          if (u < 0 || v < 0 || u >= depth.getWidth() || v >= depth.getHeight()) {
            m_valid[i] = false;
            continue;
          }
          const double Z = depth[static_cast<unsigned int>(v)][static_cast<unsigned int>(u)];
          // Z value in the depth image from the camera is invalid
          if (Z <= 0.0) {
            m_valid[i] = false;
            continue;
          }

          m_valid[i] = true;
          m_observations[0][i] = x * Z;
          m_observations[1][i] = y * Z;
          m_observations[2][i] = Z;
        }
        // Step 2: update Jacobian and error for valid points
        {
          const double X = m_cXt[0][i], Y = m_cXt[1][i], Z = m_cXt[2][i];
          const double nX = m_cNt[0][i], nY = m_cNt[1][i], nZ = m_cNt[2][i];

          const double D = -((nX * X) + (nY  * Y) + (nZ * Z));
          double projNormal = nX * m_observations[0][i] + nY  * m_observations[1][i] + nZ * m_observations[2][i];

          e[i] = D + projNormal;

          L[i][0] = nX;
          L[i][1] = nY;
          L[i][2] = nZ;
          L[i][3] = nZ * Y - nY * Z;
          L[i][4] = nX * Z - nZ * X;
          L[i][5] = nY * X - nX * Y;

        }
      }

      // Disable invalid points
      for (unsigned int i = 0; i < numPoints; ++i) {
        if (!m_valid[i]) {
          e[i] = 0.0;

          L[i][0] = 0;
          L[i][1] = 0;
          L[i][2] = 0;
          L[i][3] = 0;
          L[i][4] = 0;
          L[i][5] = 0;
        }
      }
    }

    const vpMatrix &getPointsObject() const { return m_oXt; }
    const vpMatrix &getNormalsObject() const { return m_oNt; }
    const vpMatrix &getPointsCamera() const { return m_cXt; }
    const vpMatrix &getNormalsCamera() const { return m_cNt; }

  private:
    vpMatrix m_observations; // A 3xN matrix containing the observed 3D points from the camera
    vpMatrix m_oXt; // a 3xN matrix containing the 3D points in object frame
    vpMatrix m_oNt; // a 3xN matrix containing the 3D normals on the object
    vpMatrix m_cXt;
    vpMatrix m_cNt;
    std::vector<bool> m_valid;

  };

#if defined(VISP_HAVE_NLOHMANN_JSON)

#if defined(__clang__)
// Mute warning : declaration requires an exit-time destructor [-Wexit-time-destructors]
// message : expanded from macro 'NLOHMANN_JSON_SERIALIZE_ENUM'
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

  NLOHMANN_JSON_SERIALIZE_ENUM(vpRBDenseDepthTracker::vpDisplayType, {
      {vpRBDenseDepthTracker::vpDisplayType::DT_INVALID, nullptr},
      {vpRBDenseDepthTracker::vpDisplayType::DT_SIMPLE, "simple"},
      {vpRBDenseDepthTracker::vpDisplayType::DT_WEIGHT, "weight"},
      {vpRBDenseDepthTracker::vpDisplayType::DT_ERROR, "error"},
      {vpRBDenseDepthTracker::vpDisplayType::DT_WEIGHT_AND_ERROR, "weightAndError"}
    });

  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);
    setMaxNumFeatures(j.value("maxNumFeatures", m_maxFeatures));
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

  std::vector<vpDepthPoint> m_depthPoints;
  vpDepthPointSet m_depthPointSet;
  vpRobust m_robust;
  unsigned int m_step;
  unsigned int m_maxFeatures;
  bool m_useMask;
  float m_minMaskConfidence;
  vpDisplayType m_displayType;
};

END_VISP_NAMESPACE

#endif
