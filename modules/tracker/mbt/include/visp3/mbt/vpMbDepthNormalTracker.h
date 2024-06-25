/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Description:
 * Model-based tracker using depth normal features.
 */

#ifndef _vpMbDepthNormalTracker_h_
#define _vpMbDepthNormalTracker_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>

#if DEBUG_DISPLAY_DEPTH_NORMAL
#include <visp3/core/vpDisplay.h>
#endif

BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpMbDepthNormalTracker : public virtual vpMbTracker
{
public:
  vpMbDepthNormalTracker();
  virtual ~vpMbDepthNormalTracker() VP_OVERRIDE;

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  virtual inline vpMbtFaceDepthNormal::vpFeatureEstimationType getDepthFeatureEstimationMethod() const
  {
    return m_depthNormalFeatureEstimationMethod;
  }

  virtual inline vpColVector getError() const VP_OVERRIDE { return m_error_depthNormal; }

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
                                                               const vpHomogeneousMatrix &cMo,
                                                               const vpCameraParameters &cam,
                                                               bool displayFullModel = false) VP_OVERRIDE;

  virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w_depthNormal; }

  virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE;

  virtual void loadConfigFile(const std::string &configFile, bool verbose = true) VP_OVERRIDE;

  void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo,
                   bool verbose = false);
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  void reInitModel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const std::string &cad_name,
                   const vpHomogeneousMatrix &cMo, bool verbose = false);
#endif

  virtual void resetTracker() VP_OVERRIDE;

  virtual void setCameraParameters(const vpCameraParameters &camera) VP_OVERRIDE;

  virtual void setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method);

  virtual void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method);

  virtual void setDepthNormalPclPlaneEstimationMethod(int method);

  virtual void setDepthNormalPclPlaneEstimationRansacMaxIter(int maxIter);

  virtual void setDepthNormalPclPlaneEstimationRansacThreshold(double thresold);

  virtual void setDepthNormalSamplingStep(unsigned int stepX, unsigned int stepY);

  //  virtual void setDepthNormalUseRobust(bool use);

  virtual void setOgreVisibilityTest(const bool &v) VP_OVERRIDE;

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void setPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const vpHomogeneousMatrix &cdMo);
#endif

  virtual void setScanLineVisibilityTest(const bool &v) VP_OVERRIDE;

  void setUseDepthNormalTracking(const std::string &name, const bool &useDepthNormalTracking);

  virtual void testTracking() VP_OVERRIDE;

  virtual void track(const vpImage<unsigned char> &) VP_OVERRIDE;
  virtual void track(const vpImage<vpRGBa> &I_color) VP_OVERRIDE;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void track(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
  virtual void track(const std::vector<vpColVector> &point_cloud, unsigned int width, unsigned int height);

protected:
  //! Method to estimate the desired features
  vpMbtFaceDepthNormal::vpFeatureEstimationType m_depthNormalFeatureEstimationMethod;
  //! Set of faces describing the object used only for display with scan line.
  vpMbHiddenFaces<vpMbtPolygon> m_depthNormalHiddenFacesDisplay;
  //! List of current active (visible and with features extracted) faces
  std::vector<vpMbtFaceDepthNormal *> m_depthNormalListOfActiveFaces;
  //! List of desired features
  std::vector<vpColVector> m_depthNormalListOfDesiredFeatures;
  //! List of faces
  std::vector<vpMbtFaceDepthNormal *> m_depthNormalFaces;
  //! PCL plane estimation method
  int m_depthNormalPclPlaneEstimationMethod;
  //! PCL RANSAC maximum number of iterations
  int m_depthNormalPclPlaneEstimationRansacMaxIter;
  //! PCL RANSAC threshold
  double m_depthNormalPclPlaneEstimationRansacThreshold;
  //! Sampling step in x-direction
  unsigned int m_depthNormalSamplingStepX;
  //! Sampling step in y-direction
  unsigned int m_depthNormalSamplingStepY;
  //! If true, use Tukey robust M-Estimator
  bool m_depthNormalUseRobust;
  //! (s - s*)
  vpColVector m_error_depthNormal;
  //! Display features
  std::vector<std::vector<double> > m_featuresToBeDisplayedDepthNormal;
  //! Interaction matrix
  vpMatrix m_L_depthNormal;
  //! Robust
  vpRobust m_robust_depthNormal;
  //! Robust weights
  vpColVector m_w_depthNormal;
  //! Weighted error
  vpColVector m_weightedError_depthNormal;
#if DEBUG_DISPLAY_DEPTH_NORMAL
  vpDisplay *m_debugDisp_depthNormal;
  vpImage<unsigned char> m_debugImage_depthNormal;
#endif

  void addFace(vpMbtPolygon &polygon, bool alreadyClose);

  void computeVisibility(unsigned int width, unsigned int height);

  void computeVVS();
  virtual void computeVVSInit() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;

  virtual std::vector<std::vector<double> > getFeaturesForDisplayDepthNormal();

  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int idFace = 0,
                          const std::string &name = "") VP_OVERRIDE;

  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace = 0,
                            const std::string &name = "") VP_OVERRIDE;

  virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE;

  virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE;

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
  void segmentPointCloud(const std::vector<vpColVector> &point_cloud, unsigned int width, unsigned int height);
  void segmentPointCloud(const vpMatrix &point_cloud, unsigned int width, unsigned int height);
};
END_VISP_NAMESPACE
#endif
