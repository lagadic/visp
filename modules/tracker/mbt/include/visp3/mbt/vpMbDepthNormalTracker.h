/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 *****************************************************************************/

#ifndef __vpMbDepthNormalTracker_h_
#define __vpMbDepthNormalTracker_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>

#if DEBUG_DISPLAY_DEPTH_NORMAL
#include <visp3/core/vpDisplay.h>
#endif

class VISP_EXPORT vpMbDepthNormalTracker : public virtual vpMbTracker
{
public:
  vpMbDepthNormalTracker();
  virtual ~vpMbDepthNormalTracker();

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual inline vpMbtFaceDepthNormal::vpFeatureEstimationType getDepthFeatureEstimationMethod() const
  {
    return m_depthNormalFeatureEstimationMethod;
  }

  virtual inline vpColVector getError() const { return m_error_depthNormal; }

  virtual inline vpColVector getRobustWeights() const { return m_w_depthNormal; }

  virtual void init(const vpImage<unsigned char> &I);

  virtual void loadConfigFile(const std::string &configFile);

  void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_,
                   const bool verbose = false);
#if defined(VISP_HAVE_PCL)
  void reInitModel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const std::string &cad_name,
                   const vpHomogeneousMatrix &cMo_, const bool verbose = false);
#endif

  virtual void resetTracker();

  virtual void setCameraParameters(const vpCameraParameters &camera);

  virtual void setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method);

  virtual void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method);

  virtual void setDepthNormalPclPlaneEstimationMethod(const int method);

  virtual void setDepthNormalPclPlaneEstimationRansacMaxIter(const int maxIter);

  virtual void setDepthNormalPclPlaneEstimationRansacThreshold(const double thresold);

  virtual void setDepthNormalSamplingStep(const unsigned int stepX, const unsigned int stepY);

  //  virtual void setDepthNormalUseRobust(const bool use);

  virtual void setOgreVisibilityTest(const bool &v);

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo);
#if defined(VISP_HAVE_PCL)
  virtual void setPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const vpHomogeneousMatrix &cdMo);
#endif

  virtual void setScanLineVisibilityTest(const bool &v);

  virtual void testTracking();

  virtual void track(const vpImage<unsigned char> &);
#if defined(VISP_HAVE_PCL)
  virtual void track(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
  virtual void track(const std::vector<vpColVector> &point_cloud, const unsigned int width, const unsigned int height);

protected:
  //! Method to estimate the desired features
  vpMbtFaceDepthNormal::vpFeatureEstimationType m_depthNormalFeatureEstimationMethod;
  //! Set of faces describing the object used only for display with scan line.
  vpMbHiddenFaces<vpMbtPolygon> m_depthNormalHiddenFacesDisplay;
  //! Dummy image used to compute the visibility
  vpImage<unsigned char> m_depthNormalI_dummyVisibility;
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

  void addFace(vpMbtPolygon &polygon, const bool alreadyClose);

  void computeVisibility(const unsigned int width, const unsigned int height);

  void computeVVS();
  virtual void computeVVSInit();
  virtual void computeVVSInteractionMatrixAndResidu();

  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                          const int idFace = 0, const std::string &name = "");

  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, const double radius, const int idFace = 0,
                            const std::string &name = "");

  virtual void initFaceFromCorners(vpMbtPolygon &polygon);

  virtual void initFaceFromLines(vpMbtPolygon &polygon);

#ifdef VISP_HAVE_PCL
  void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
  void segmentPointCloud(const std::vector<vpColVector> &point_cloud, const unsigned int width,
                         const unsigned int height);
};
#endif
