/*
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
 */

/*!
  \file vpRBTracker.h
  \brief Render-Based Tracker
*/
#ifndef VP_RB_TRACKER_H
#define VP_RB_TRACKER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/rbt/vpRBSilhouettePointsExtractionSettings.h>
#include <visp3/rbt/vpPanda3DDepthFilters.h>
#include <visp3/rbt/vpObjectCentricRenderer.h>
#include <visp3/rbt/vpRBTrackingResult.h>
#include <visp3/rbt/vpRBADDSMetric.h>
#include <visp3/rbt/vpRBInitializationHelper.h>
#include <visp3/core/vpDisplay.h>

#include <ostream>
#include <type_traits>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

class vpObjectMask;
class vpRBDriftDetector;
class vpRBVisualOdometry;

/**
 * \brief
 *
 * \ingroup group_rbt_core
*/
class VISP_EXPORT vpRBTracker
{
public:

  vpRBTracker();

  ~vpRBTracker() = default;

  /**
   * \name Information retrieval
   * @{
   */
  void getPose(vpHomogeneousMatrix &cMo) const;
  void setPose(const vpHomogeneousMatrix &cMo);
  vpObjectCentricRenderer &getRenderer();
  const vpRBFeatureTrackerInput &getMostRecentFrame() const { return m_currentFrame; }

  vpMatrix getCovariance() const;

  /**
   * @}
   */

  /**
   * \name Settings
   * @{
   */
  void addTracker(std::shared_ptr<vpRBFeatureTracker> tracker);
  void setupRenderer(const std::string &file);
  inline std::string getModelPath() const { return m_modelPath; }
  void setModelPath(const std::string &path);

  inline std::vector<std::shared_ptr<vpRBFeatureTracker>> getFeatureTrackers() const { return m_trackers; }

  vpCameraParameters getCameraParameters() const;
  void setCameraParameters(const vpCameraParameters &cam, unsigned h, unsigned w);

  inline unsigned int getImageWidth() const { return m_imageWidth; }
  inline unsigned int getImageHeight() const { return m_imageHeight; }

  inline vpSilhouettePointsExtractionSettings getSilhouetteExtractionParameters() const
  {
    return m_depthSilhouetteSettings;
  }

  void setSilhouetteExtractionParameters(const vpSilhouettePointsExtractionSettings &settings);

  double getOptimizationGain() const { return m_lambda; }
  inline void setOptimizationGain(double lambda)
  {
    if (lambda < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater to zero");
    }
    m_lambda = lambda;
  }
  unsigned int getMaxOptimizationIters() const { return m_vvsIterations; }
  inline void setMaxOptimizationIters(unsigned int iters)
  {
    if (iters == 0) {
      throw vpException(vpException::badValue, "Max number of iterations must be greater than zero");
    }
    m_vvsIterations = iters;
  }

  double getOptimizationInitialMu() const { return m_muInit; }
  inline void setOptimizationInitialMu(double mu)
  {
    if (mu < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater or equal to zero");
    }
    m_muInit = mu;
  }

  double getOptimizationMuIterFactor() const { return m_muIterFactor; }
  inline void setOptimizationMuIterFactor(double factor)
  {
    if (factor < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater or equal to zero");
    }
    m_muIterFactor = factor;
  }

  bool scaleInvariantRegularization() const { return m_scaleInvariantOptim; }
  inline void setScaleInvariantRegularization(bool invariant)
  {

    m_scaleInvariantOptim = invariant;
  }

  std::shared_ptr<vpRBDriftDetector> getDriftDetector() const { return m_driftDetector; }
  inline void setDriftDetector(const std::shared_ptr<vpRBDriftDetector> &detector)
  {
    m_driftDetector = detector;
  }

  std::shared_ptr<vpObjectMask> getObjectSegmentationMethod() const { return m_mask; }
  inline void setObjectSegmentationMethod(const std::shared_ptr<vpObjectMask> &mask)
  {
    m_mask = mask;
  }

  std::shared_ptr<vpRBVisualOdometry> getOdometryMethod() const { return m_odometry; }
  inline void setOdometryMethod(const std::shared_ptr<vpRBVisualOdometry> &odometry)
  {
    m_odometry = odometry;
  }

  /*!
   * Get verbosity mode.
   * \return true when verbosity is enabled, false otherwise.
   */
  inline bool getVerbose() const
  {
    return m_verbose;
  }

  /*!
   * Enable/disable verbosity mode.
   * \param verbose : When true verbose mode is enabled. When false verbosity is disabled.
   */
  inline void setVerbose(bool verbose)
  {
    m_verbose = verbose;
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadConfigurationFile(const std::string &filename);
  void loadConfiguration(const nlohmann::json &j);
#endif

  /**
   * @}
   */

  void reset();

  /**
   * \name Tracking
   * @{
   */
  void startTracking();
  vpRBTrackingResult track(const vpImage<unsigned char> &I);
  vpRBTrackingResult track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB);
  vpRBTrackingResult track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);
  /**
   * @}
   */

  double score(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);

  /**
   * \name Display
   * @{
   */
  void displayMask(vpImage<unsigned char> &Imask) const;
  void display(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth);
  /**
   * @}
   */

#ifdef VISP_HAVE_MODULE_GUI
  template <typename ImageType>
  typename std::enable_if<std::is_same<ImageType, unsigned char>::value || std::is_same<ImageType, vpRGBa>::value, void >::type initClick(const vpImage<ImageType> &I, const std::string &initFile, bool displayHelp)
  {
    vpRBInitializationHelper initializer;
    initializer.setCameraParameters(m_cam);
    initializer.initClick(I, initFile, displayHelp, *this);
    m_cMo = initializer.getPose();
  }
#endif

  friend vpRBInitializationHelper;
protected:

  vpRBTrackingResult track(vpRBFeatureTrackerInput &input);
  void updateRender(vpRBFeatureTrackerInput &frame);
  void updateRender(vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo);

  template <typename T>
  void displaySilhouette(const vpImage<T> &I, const vpRBFeatureTrackerInput &frame)
  {
    if (shouldRenderSilhouette()) {
      const vpImage<unsigned char> &Isilhouette = frame.renders.isSilhouette;
      const vpRect bb = m_renderer.getBoundingBox();
      for (unsigned int r = std::max(bb.getTop(), 0.); (r < bb.getBottom()) &&(r < I.getRows()); ++r) {
        for (unsigned int c = std::max(bb.getLeft(), 0.); (c < bb.getRight()) && (c < I.getCols()); ++c) {
          if (Isilhouette[r][c] != 0) {
            vpDisplay::displayPoint(I, vpImagePoint(r, c), vpColor::green);
          }
        }
      }
    }
  }

  std::vector<vpRBSilhouettePoint> extractSilhouettePoints(
    const vpImage<vpRGBf> &Inorm, const vpImage<float> &Idepth,
    const vpImage<vpRGBf> &Ior, const vpImage<unsigned char> &Ivalid,
    const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp);

  template<typename T>
  void checkDimensionsOrThrow(const vpImage<T> &I, const std::string &imgType) const
  {
    if (I.getRows() != m_imageHeight || I.getCols() != m_imageWidth) {
      std::stringstream ss;
      ss << "vpRBTracker: dimension error: expected " << imgType;
      ss << " image to have the following resolution " << m_imageWidth << " x " << m_imageHeight;
      ss << ", but got " << I.getCols() << " x " << I.getRows();
      throw vpException(vpException::dimensionError, ss.str());
    }
  }

  bool shouldRenderSilhouette()
  {
    return m_renderer.getRenderer<vpPanda3DDepthCannyFilter>() != nullptr;
  }

  bool m_firstIteration; //! Whether this is the first iteration

  std::vector<std::shared_ptr<vpRBFeatureTracker>> m_trackers; //! List of trackers

  vpRBFeatureTrackerInput m_currentFrame;
  vpRBFeatureTrackerInput m_previousFrame;

  std::string m_modelPath;
  vpHomogeneousMatrix m_cMo;
  vpHomogeneousMatrix m_cMoPrev;
  vpCameraParameters m_cam;

  double m_lambda; //! VVS gain
  unsigned m_vvsIterations; //! Max number of VVS iterations
  double m_muInit; //! Initial mu value for Levenberg-Marquardt
  double m_muIterFactor; //! Factor with which to multiply mu at every iteration during VVS.
  bool m_scaleInvariantOptim;

  vpSilhouettePointsExtractionSettings m_depthSilhouetteSettings;
  vpPanda3DRenderParameters m_rendererSettings;
  vpObjectCentricRenderer m_renderer;

  unsigned m_imageHeight, m_imageWidth; //! Color and render image dimensions

  bool m_verbose;

  std::shared_ptr<vpObjectMask> m_mask;
  std::shared_ptr<vpRBDriftDetector> m_driftDetector;
  std::shared_ptr<vpRBVisualOdometry> m_odometry;

  vpRBADDSMetric m_convergenceMetric;
  double m_convergedMetricThreshold;
  double m_updateRenderThreshold;

  bool m_displaySilhouette; //! Whether a call to the display function should draw a silhouette outline

};

END_VISP_NAMESPACE

#endif
#endif
