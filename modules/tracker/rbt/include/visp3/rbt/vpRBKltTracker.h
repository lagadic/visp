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

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpRobust.h>
#include <visp3/klt/vpKltOpencv.h>

#include <opencv2/core/mat.hpp>



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


#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j)
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);

    m_klt.setMaxFeatures(j.value("maxFeatures", 10000));
    m_klt.setWindowSize(j.value("windowSize", 5));
    m_klt.setQuality(j.value("quality", 0.01));
    m_klt.setMinDistance(j.value("minDistance", 5));
    m_klt.setHarrisFreeParameter(j.value("harris", 0.01));
    m_klt.setBlockSize(j.value("blockSize", 3));
    m_klt.setPyramidLevels(j.value("pyramidLevels", 3));

    m_numPointsReinit = j.value("minimumNumPoints", m_numPointsReinit);
    m_newPointsDistanceThreshold = j.value("newPointsMinPixelDistance", m_newPointsDistanceThreshold);
    m_maxErrorOutliersPixels = j.value("maxReprojectionErrorPixels", m_maxErrorOutliersPixels);

  }
#endif


  struct vpTrackedKltPoint
  {
  public:
    vpHomogeneousMatrix cTo0;
    vpPoint oX;
    vpColVector normal;
    vpImagePoint currentPos;

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

    inline double weight(const vpHomogeneousMatrix &cMo)
    {
      //return static_cast<double>(validAndInlierCount) / static_cast<double>(validCount);
      return 1.0;
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

};
#endif
