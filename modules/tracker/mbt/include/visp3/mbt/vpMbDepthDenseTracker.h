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
 * Model-based tracker using depth dense features.
 */

#ifndef _vpMbDepthDenseTracker_h_
#define _vpMbDepthDenseTracker_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtFaceDepthDense.h>
#include <visp3/mbt/vpMbtTukeyEstimator.h>

#if DEBUG_DISPLAY_DEPTH_DENSE
#include <visp3/core/vpDisplay.h>
#endif

BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpMbDepthDenseTracker : public virtual vpMbTracker
{
public:
  vpMbDepthDenseTracker();
  virtual ~vpMbDepthDenseTracker() VP_OVERRIDE;

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  virtual inline vpColVector getError() const VP_OVERRIDE { return m_error_depthDense; }

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
                                                               const vpHomogeneousMatrix &cMo,
                                                               const vpCameraParameters &cam,
                                                               bool displayFullModel = false) VP_OVERRIDE;

  virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w_depthDense; }

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

  virtual void setDepthDenseFilteringMaxDistance(double maxDistance);
  virtual void setDepthDenseFilteringMethod(int method);
  virtual void setDepthDenseFilteringMinDistance(double minDistance);
  virtual void setDepthDenseFilteringOccupancyRatio(double occupancyRatio);

  inline void setDepthDenseSamplingStep(unsigned int stepX, unsigned int stepY)
  {
    if (stepX == 0 || stepY == 0) {
      std::cerr << "stepX and stepY must be greater than zero!" << std::endl;
      return;
    }

    m_depthDenseSamplingStepX = stepX;
    m_depthDenseSamplingStepY = stepY;
  }

  virtual void setOgreVisibilityTest(const bool &v) VP_OVERRIDE;

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void setPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud, const vpHomogeneousMatrix &cdMo);
#endif

  virtual void setScanLineVisibilityTest(const bool &v) VP_OVERRIDE;

  void setUseDepthDenseTracking(const std::string &name, const bool &useDepthDenseTracking);

  virtual void testTracking() VP_OVERRIDE;

  virtual void track(const vpImage<unsigned char> &) VP_OVERRIDE;
  virtual void track(const vpImage<vpRGBa> &) VP_OVERRIDE;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void track(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
  virtual void track(const std::vector<vpColVector> &point_cloud, unsigned int width, unsigned int height);

protected:
  //! Set of faces describing the object used only for display with scan line.
  vpMbHiddenFaces<vpMbtPolygon> m_depthDenseHiddenFacesDisplay;
  //! List of current active (visible and features extracted) faces
  std::vector<vpMbtFaceDepthDense *> m_depthDenseListOfActiveFaces;
  //! Nb features
  unsigned int m_denseDepthNbFeatures;
  //! List of faces
  std::vector<vpMbtFaceDepthDense *> m_depthDenseFaces;
  //! Sampling step in x-direction
  unsigned int m_depthDenseSamplingStepX;
  //! Sampling step in y-direction
  unsigned int m_depthDenseSamplingStepY;
  //! (s - s*)
  vpColVector m_error_depthDense;
  //! Interaction matrix
  vpMatrix m_L_depthDense;
  //! Tukey M-Estimator
  vpMbtTukeyEstimator<double> m_robust_depthDense;
  //! Robust weights
  vpColVector m_w_depthDense;
  //! Weighted error
  vpColVector m_weightedError_depthDense;
#if DEBUG_DISPLAY_DEPTH_DENSE
  vpDisplay *m_debugDisp_depthDense;
  vpImage<unsigned char> m_debugImage_depthDense;
#endif

  void addFace(vpMbtPolygon &polygon, bool alreadyClose);

  void computeVisibility(unsigned int width, unsigned int height);

  void computeVVS();
  virtual void computeVVSInit() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;
  virtual void computeVVSWeights();
  using vpMbTracker::computeVVSWeights;

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
