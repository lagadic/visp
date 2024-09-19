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
  \file vpRBKltTracker.h
  \brief KLT features in the context of render based tracking
*/
#ifndef VP_RB_KLT_TRACKER_H
#define VP_RB_KLT_TRACKER_H

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO))
#define VP_HAVE_RB_KLT_TRACKER
#endif

#if defined(VP_HAVE_RB_KLT_TRACKER)
#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpRobust.h>
#include <visp3/klt/vpKltOpencv.h>

#include <opencv2/core/mat.hpp>

/**
 * \brief KLT-Based features
 *
 * \ingroup group_rbt_trackers
 */
class VISP_EXPORT vpRBKltTracker : public vpRBFeatureTracker
{
public:
  vpRBKltTracker();

  bool requiresRGB() const VP_OVERRIDE { return false; }

  bool requiresDepth() const VP_OVERRIDE { return false; }

  bool requiresSilhouetteCandidates() const VP_OVERRIDE { return false; }

  void onTrackingIterStart() VP_OVERRIDE { }

  void onTrackingIterEnd() VP_OVERRIDE { }

  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void initVVS(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type) const VP_OVERRIDE;


  /**
   * \name Settings
   * @{
   */

  /**
   * \brief Get the minimum acceptable number of points that should be tracked. If KLT tracking has less than this number of points
   * The KLT tracking will be fully reinitialized.
   */
  unsigned int getMinimumNumberOfPoints() const { return m_numPointsReinit; }
  void setMinimumNumberOfPoints(unsigned int points) { m_numPointsReinit = points; }

  /**
   * \brief Get the minimum distance that a candidate point should have to every other tracked point if it should be added.
   *
   * During tracking, KLT points are frequently sampled. This settings used to ensure that multiple klt points do not track the same 3D points
   */
  double getMinimumDistanceNewPoints() const { return m_newPointsDistanceThreshold; }
  void setMinimumDistanceNewPoints(double distance) { m_newPointsDistanceThreshold = distance; }

  /**
   * \brief Return the number of pixels in the image border where points should not be tracked.
   * Points that are near image borders are likely to be lost in the future.
   */
  unsigned int getFilteringBorderSize() const { return m_border; }
  void setFilteringBorderSize(unsigned int border) { m_border = border; }

  /**
   * \brief Get the maximum reprojection error, in pixels, for a point to be considered as outlier.
   * This reprojection error is computed between the tracked klt position in the image and the reprojection of the associated 3D point.
   * If a point goes above this threshold, it is removed from tracking
   *
   * \return double
   */
  double getFilteringMaxReprojectionError() const { return m_maxErrorOutliersPixels; }
  void setFilteringMaxReprojectionError(double maxError) { m_maxErrorOutliersPixels = maxError; }

  /**
   * \brief Returns whether the tracking algorithm should filter out points that are unlikely to be on the object according to the mask.
   * If the mask is not computed beforehand, then it has no effect
   */
  bool shouldUseMask() const { return m_useMask; }
  void setShouldUseMask(bool useMask) { m_useMask = useMask; }

  /**
   * \brief Returns the minimum mask confidence that a pixel should have if it should be kept during tracking.
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
   * \brief Get the underlying KLT tracker. Use this to read its settings.
   */
  const vpKltOpencv &getKltTracker() const { return m_klt; }
  /**
   * \brief Get the underlying KLT tracker. Use this to modify its settings.
   *
   * \warning Only modify its tracking settings, not its state.
   */
  vpKltOpencv &getKltTracker() { return m_klt; }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j)
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);

    m_klt = j;

    setMinimumNumberOfPoints(j.value("minimumNumPoints", m_numPointsReinit));
    setMinimumDistanceNewPoints(j.value("newPointsMinPixelDistance", m_newPointsDistanceThreshold));
    setFilteringMaxReprojectionError(j.value("maxReprojectionErrorPixels", m_maxErrorOutliersPixels));
    setShouldUseMask(j.value("useMask", m_useMask));
    setMinimumMaskConfidence(j.value("minMaskConfidence", m_minMaskConfidence));
  }
#endif

  /**
   * @}
   *
   */

  struct vpTrackedKltPoint
  {
  public:
    vpHomogeneousMatrix cTo0; //! Initial pose of the object in the camera frame, acquired when the tracked point was first constructed
    vpPoint oX; //! Tracked 3D point
    vpColVector normal; //! Surface normal at this point, in the object frame
    vpImagePoint currentPos; //! Current image coordinates, in normalized image coordinates

    inline double rotationDifferenceToInitial(const vpHomogeneousMatrix &oMc)
    {
      const vpHomogeneousMatrix cinitTc = cTo0 * oMc;
      return cinitTc.getThetaUVector().getTheta();
    }

    inline double normalDotProd(const vpHomogeneousMatrix &cMo)
    {
      vpColVector cameraNormal = cMo.getRotationMatrix() * normal;
      oX.changeFrame(cMo);
      vpColVector dir({ -oX.get_X(), -oX.get_Y(), -oX.get_Z() });
      dir.normalize();
      return dir * cameraNormal;
    }

    inline void update(const vpHomogeneousMatrix &cMo)
    {
      oX.changeFrame(cMo);
      oX.projection();
    }

    inline void error(vpColVector &e, unsigned i) const
    {
      e[i * 2] = oX.get_x() - currentPos.get_u();
      e[i * 2 + 1] = oX.get_y() - currentPos.get_v();
    }

    inline double distance(const vpTrackedKltPoint &other) const
    {
      const double d = sqrt(std::pow(oX.get_oX() - other.oX.get_oX(), 2) +
        std::pow(oX.get_oY() - other.oX.get_oY(), 2) +
        std::pow(oX.get_oZ() - other.oX.get_oZ(), 2));
      return d;
    }

    inline void interaction(vpMatrix &L, unsigned i) const
    {
      double x = oX.get_x(), y = oX.get_y();
      double xy = x * y;
      double Zinv = 1.0 / oX.get_Z();
      L[i * 2][0] = -Zinv;
      L[i * 2][1] = 0.0;
      L[i * 2][2] = x * Zinv;
      L[i * 2][3] = xy;
      L[i * 2][4] = -(1.0 + x * x);
      L[i * 2][5] = y;

      L[i * 2 + 1][0] = 0.0;
      L[i * 2 + 1][1] = -Zinv;
      L[i * 2 + 1][2] = y * Zinv;
      L[i * 2 + 1][3] = 1.0 + y * y;
      L[i * 2 + 1][4] = -xy;
      L[i * 2 + 1][5] = -x;
    }
  };


private:

  void tryAddNewPoint(const vpRBFeatureTrackerInput &frame, std::map<long, vpTrackedKltPoint> &points,
                      long id, const float u, const float v, const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc);

  vpRobust m_robust;

  cv::Mat m_I, m_Iprev;
  vpKltOpencv m_klt;

  unsigned int m_numPointsReinit; //! Minimum number of KLT points required to avoid performing reinitialization
  double m_newPointsDistanceThreshold; //! Minimum distance (to the other tracked points) threshold for a new detected klt point to be considered as novel. In Pixels
  unsigned int m_border; //! Image border size, where points should not be considered

  double m_maxErrorOutliersPixels; //! Max 3D reprojection error before a point is considered an outlier and rejected from tracking. In meters

  /*!
   * Reject points where the render normals's dot product
   * with the inverse camera vector is above this angle threshold.
   * Helps removing uncertain keypoints or keypoints that may disappear in the next frame.
   */
  double m_normalAcceptanceThresholdDeg;

  std::map<long, vpTrackedKltPoint> m_points;

  bool m_useMask;
  float m_minMaskConfidence;

};
#endif
#endif
