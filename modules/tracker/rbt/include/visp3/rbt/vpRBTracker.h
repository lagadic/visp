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
#include <visp3/rbt/vpRBTrackerLogger.h>

#include <visp3/core/vpDisplay.h>


#include <ostream>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json_fwd.hpp>
#endif

BEGIN_VISP_NAMESPACE

class vpObjectMask;
class vpRBDriftDetector;

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
  const vpRBTrackerLogger &getLogger() const { return m_logger; }

  /**
   * @}
   */

  /**
   * \name Settings
   * @{
   */
  void addTracker(std::shared_ptr<vpRBFeatureTracker> tracker);
  void loadObjectModel(const std::string &file);

  vpCameraParameters getCameraParameters() const;
  void setCameraParameters(const vpCameraParameters &cam, unsigned h, unsigned w);

  vpSilhouettePointsExtractionSettings getSilhouetteExtractionParameters() const
  {
    return m_depthSilhouetteSettings;
  }

  void setSilhouetteExtractionParameters(const vpSilhouettePointsExtractionSettings &settings);

  double getOptimizationGain() const { return m_lambda; }
  void setOptimizationGain(double lambda)
  {
    if (lambda < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater to zero");
    }
    m_lambda = lambda;
  }
  unsigned int getMaxOptimizationIters() const { return m_vvsIterations; }
  void setMaxOptimizationIters(unsigned int iters) { m_vvsIterations = iters; }

  double getOptimizationInitialMu() const { return m_muInit; }
  void setOptimizationInitialMu(double mu)
  {
    if (mu < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater to zero");
    }
    m_muInit = mu;
  }

  double getOptimizationMuIterFactor() const { return m_muIterFactor; }
  void setOptimizationMuIterFactor(double factor)
  {
    if (factor < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater to zero");
    }
    m_muIterFactor = factor;
  }

  std::shared_ptr<vpRBDriftDetector> getDriftDetector() const { return m_driftDetector; }
  void setDriftDetector(const std::shared_ptr<vpRBDriftDetector> &detector)
  {
    m_driftDetector = detector;
  }

  std::shared_ptr<vpObjectMask> getObjectSegmentationMethod() const { return m_mask; }
  void setObjectSegmentationMethod(const std::shared_ptr<vpObjectMask> &mask)
  {
    m_mask = mask;
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
  void track(const vpImage<unsigned char> &I);
  void track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB);
  void track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);
  /**
   * @}
   */

  /**
   * \name Display
   * @{
   */
  void displayMask(vpImage<unsigned char> &Imask) const;
  void display(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type);
  /**
   * @}
   */

#ifdef VISP_HAVE_MODULE_GUI
  void initClick(const vpImage<unsigned char> &I, const std::string &initFile, bool displayHelp);
#endif

protected:

  void track(vpRBFeatureTrackerInput &input);
  void updateRender(vpRBFeatureTrackerInput &frame);

  std::vector<vpRBSilhouettePoint> extractSilhouettePoints(
    const vpImage<vpRGBf> &Inorm, const vpImage<float> &Idepth,
    const vpImage<vpRGBf> &Ior, const vpImage<unsigned char> &Ivalid,
    const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp);

  vpMatrix getCovariance()
  {
    throw vpException(vpException::notImplementedError);
  }

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

  bool m_firstIteration; //! Whether this is the first iteration

  std::vector<std::shared_ptr<vpRBFeatureTracker>> m_trackers; //! List of trackers

  vpRBFeatureTrackerInput m_currentFrame;
  vpRBFeatureTrackerInput m_previousFrame;

  vpHomogeneousMatrix m_cMo;
  vpHomogeneousMatrix m_cMoPrev;
  vpCameraParameters m_cam;

  double m_lambda; //! VVS gain
  unsigned m_vvsIterations; //! Max number of VVS iterations
  double m_muInit; //! Initial mu value for Levenberg-Marquardt
  double m_muIterFactor; //! Factor with which to multiply mu at every iteration during VVS.

  vpSilhouettePointsExtractionSettings m_depthSilhouetteSettings;
  vpPanda3DRenderParameters m_rendererSettings;
  vpObjectCentricRenderer m_renderer;

  unsigned m_imageHeight, m_imageWidth; //! Color and render image dimensions

  vpRBTrackerLogger m_logger;

  std::shared_ptr<vpObjectMask> m_mask;
  std::shared_ptr<vpRBDriftDetector> m_driftDetector;

  // vpRBTrackerFilter m_filter;
};

END_VISP_NAMESPACE


#endif
#endif
