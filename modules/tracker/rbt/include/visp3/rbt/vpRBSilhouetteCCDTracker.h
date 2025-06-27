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
  \file vpRBSilhouetteCCDTracker.h
  \brief Color silhouette features
*/
#ifndef VP_SILHOUETTE_CCD_TRACKER_H
#define VP_SILHOUETTE_CCD_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpUniRand.h>

// #if defined(VISP_HAVE_SIMDLIB)
// #include <Simd/SimdLib.h>
// #endif
#include <visp3/rbt/vpRBSilhouetteControlPoint.h>
#include <visp3/rbt/vpRBFeatureTracker.h>

#include <vector>
#include <iostream>
#include <algorithm>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpCCDParameters
{
public:
  vpCCDParameters() :
    gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3),
    alpha(1.3), beta(0.06), kappa(0.5),
    covarianceIterDecreaseFactor(0.25),
    h(40), delta_h(1), min_h(4), start_h(40), start_delta_h(1), phi_dim(6)
  { }

  ~vpCCDParameters() = default;
  /**
   * \brief Curve uncertainty computation hyperparameter.
   * Recommended to leave fixed.
   */
  double gamma_1;
  /**
   * \brief Curve uncertainty computation hyperparameter.
   * Recommended to leave fixed.
   */
  double gamma_2;
  /**
   * \brief Curve uncertainty computation hyperparameter.
   * Recommended to leave fixed.
   */
  double gamma_3;
  /**
   * \brief Curve uncertainty computation hyperparameter.
   * Recommended to leave fixed.
   */
  double gamma_4;
  double alpha;
  double beta;

  /**
   * \brief Bias to the diagonal of the covariance of the color statistics of a single pixel.
   * Used to avoid singularities and degenerate cases
   *
   * The final pixel color covariance will be kappa * I(3) + covariance.
   */
  double kappa;
  /**
   *
   * \brief From the CCD paper:
   * maximum decrease of the covariance within one iteration step. Between 0 and 1
   * If c2 is too high, the covariance declines slowly. Hence, a small number of iterations is
   * necessary. If c2 is too small, the CCD algorithm may converge to a wrong solution.
   * it is recommended to leave this value fixed
   */
  double covarianceIterDecreaseFactor;
  /**
   * \brief Size of the vicinity that is used to compute statistics and error.
   * Length of the line along the normal (and the opposite direction). To subsample the line, set delta_h > 1.
   * Number of pixels used is computed as 2 * floor(h/delta_h). If you expect large motions, set a large value.
   * If you want to reduce computation time, decrease this value or increase delta_h
   * Recommended value: 4 or above (this is dependent on image resolution)
   */
  int h;
  /**
   * \brief Sample step when computing statistics and errors.
   * Increase this value to decrease computation time, at the risk of obtaining inaccurate statistics.
   */
  int delta_h;
  int min_h;
  int start_h;
  int start_delta_h;
  /**
   * \brief Number of parameters estimated by CCD. Either 6 or 8.
   * Leave this fixed
   */
  int phi_dim;
};

#if defined(VISP_HAVE_NLOHMANN_JSON)
inline void from_json(const nlohmann::json &j, vpCCDParameters &ccdParameters)
{
  ccdParameters.alpha = j.value("alpha", ccdParameters.alpha);
  ccdParameters.beta = j.value("beta", ccdParameters.beta);
  ccdParameters.kappa = j.value("kappa", ccdParameters.kappa);
  ccdParameters.covarianceIterDecreaseFactor = j.value("covarianceIterDecreaseFactor",
                                                      ccdParameters.covarianceIterDecreaseFactor);
  ccdParameters.h = j.value("h", ccdParameters.h);
  ccdParameters.start_h = ccdParameters.h;
  ccdParameters.delta_h = j.value("delta_h", ccdParameters.delta_h);
  ccdParameters.start_delta_h = ccdParameters.delta_h;
  if (j.contains("min_h")) {
    ccdParameters.min_h = j.value("min_h", ccdParameters.min_h);
  }
  else {
    ccdParameters.min_h = ccdParameters.h;
  }

  ccdParameters.phi_dim = j.value("phi_dim", ccdParameters.phi_dim);
  if (j.contains("gamma")) {
    nlohmann::json gammaj = j["gamma"];
    if (!gammaj.is_array() || gammaj.size() != 4) {
      throw vpException(vpException::ioError, "CCD parameters: tried to read gamma values from something that is not a 4-sized float array");
    }
    ccdParameters.gamma_1 = gammaj[0];
    ccdParameters.gamma_2 = gammaj[1];
    ccdParameters.gamma_3 = gammaj[2];
    ccdParameters.gamma_4 = gammaj[3];
  }
}
#endif

class VISP_EXPORT vpCCDStatistics
{
public:
  vpMatrix vic; //! Vicinity data
  vpMatrix mean_vic; //! Mean
  vpMatrix cov_vic; //! Covariance
  vpMatrix nv; //! Normal vector
  vpMatrix imgPoints; //! Img pixels
  vpMatrix weight; //! Whether this pixel is the object

  void reinit(int resolution, unsigned normalPointsNumber)
  {
    nv.resize(resolution, 2, false, false);
    mean_vic.resize(resolution, 6, false, false);
    cov_vic.resize(resolution, 18, false, false);
    vic.resize(resolution, 20 * normalPointsNumber, false, false);
    imgPoints.resize(resolution, 2 * 3 * normalPointsNumber, false, false);
    weight.resize(resolution, 2 * normalPointsNumber, false, false);
  }
  void zero()
  {
    nv = 0.0;
    mean_vic = 0.0;
    cov_vic = 0.0;
    vic = 0.0;
    imgPoints = 0.0;
    weight = 0.0;
  }
};

/**
 * \brief Tracking based on the Contracting Curve Density algorithm.
 *
 * \ingroup group_rbt_trackers
 */
class VISP_EXPORT vpRBSilhouetteCCDTracker : public vpRBFeatureTracker
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

  vpRBSilhouetteCCDTracker();
  virtual ~vpRBSilhouetteCCDTracker() = default;

  bool requiresRGB() const VP_OVERRIDE { return true; }
  bool requiresDepth() const VP_OVERRIDE { return false; }
  bool requiresSilhouetteCandidates() const VP_OVERRIDE { return true; }

  /**
   * \name Settings
   * @{
   */
  void setCCDParameters(const vpCCDParameters &parameters) { m_ccdParameters = parameters; }
  vpCCDParameters getCCDParameters() const { return m_ccdParameters; }
  //void computeMask(const vpImage<vpRGBa> &render, vpCCDStatistics &stats);

  /**
   * \brief Returns the amount of temporal smoothing applied when computing the tracking error and its jacobian.
   * This factor is used to interpolate with the error computed on the previous frame for the features selected at the
   * current iteration.
   * Temporal smoothing may help smooth out the motion and reduce jitter.
   */
  double getTemporalSmoothingFactor() const { return m_temporalSmoothingFac; }

  /**
   * \brief Sets the temporal smoothing factor.
   *
   * \see getTemporalSmoothingFactor
   *
   * @param factor the new temporal smoothing factor. Should be greater than 0
   */
  void setTemporalSmoothingFactor(double factor)
  {
    if (factor < 0.0) {
      throw vpException(vpException::badValue, "Temporal smoothing factor should be equal to or greater than 0");
    }
    m_temporalSmoothingFac = factor;
  }

  /**
   * \brief Returns whether the tracking algorithm should filter out points that are unlikely to be on the object
   * according to the mask.
   * If the mask is not computed beforehand, then it has no effect.
   */
  bool shouldUseMask() const { return m_useMask; }
  void setShouldUseMask(bool useMask) { m_useMask = useMask; }

  /**
   * \brief Returns the minimum mask gradient required for a silhouette point to be considered.
   *
   * This value is between 0 and 1.
   */
  double getMinimumMaskConfidence() const { return m_minMaskConfidence; }
  void setMinimumMaskConfidence(double confidence)
  {

    m_minMaskConfidence = confidence;
  }
  /**
   * \brief Get the maximum number of silhouette control points that will be used by the tracker at a given iteration.
   * If there are more control points on the silhouette than getMaxNumPoints(), they will be subsampled randomly.
   * If maxNumPoints is zero, then all points are used.
  */
  unsigned int getMaxNumPoints() const { return m_maxPoints; }
  void setMaxNumPoints(unsigned int maxPoints) { m_maxPoints = maxPoints; }

  void setDisplayType(vpDisplayType type)
  {
    if (type == DT_INVALID) {
      throw vpException(vpException::badValue, "CCD tracker display type is invalid");
    }
    m_displayType = type;
  }

  /**
   * @}
   */

  void onTrackingIterStart(const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }
  void onTrackingIterEnd(const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }

  double getVVSTrackerWeight(double optimizationProgress) const VP_OVERRIDE
  {
    return m_weighting->weight(optimizationProgress) / (100 * m_error.size());
  }

  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  void trackFeatures(const vpRBFeatureTrackerInput & /*frame*/, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix & /*cMo*/) VP_OVERRIDE { }

  void initVVS(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;
  void updateCovariance(const double /*lambda*/) VP_OVERRIDE
  {
    m_cov = m_sigma;
  }

  void changeScale();

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth) const VP_OVERRIDE;

#if defined(VISP_HAVE_NLOHMANN_JSON)

#if defined(__clang__)
// Mute warning : declaration requires an exit-time destructor [-Wexit-time-destructors]
// message : expanded from macro 'NLOHMANN_JSON_SERIALIZE_ENUM'
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

  NLOHMANN_JSON_SERIALIZE_ENUM(vpRBSilhouetteCCDTracker::vpDisplayType, {
        {vpRBSilhouetteCCDTracker::vpDisplayType::DT_INVALID, nullptr},
        {vpRBSilhouetteCCDTracker::vpDisplayType::DT_SIMPLE, "simple"},
        {vpRBSilhouetteCCDTracker::vpDisplayType::DT_WEIGHT, "weight"},
        {vpRBSilhouetteCCDTracker::vpDisplayType::DT_ERROR, "error"},
        {vpRBSilhouetteCCDTracker::vpDisplayType::DT_WEIGHT_AND_ERROR, "weightAndError"}
      });
  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);

    m_vvsConvergenceThreshold = j.value("convergenceThreshold", m_vvsConvergenceThreshold);
    setTemporalSmoothingFactor(j.value("temporalSmoothing", m_temporalSmoothingFac));
    setShouldUseMask(j.value("useMask", m_useMask));
    setMinimumMaskConfidence(j.value("minMaskConfidence", m_minMaskConfidence));
    setMaxNumPoints(j.value("maxNumPoints", m_maxPoints));

    setDisplayType(j.value("displayType", m_displayType));
    m_ccdParameters = j.value("ccd", m_ccdParameters);
  }

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif

#endif

protected:
  void updateCCDPoints(const vpHomogeneousMatrix &cMo);
  void computeLocalStatistics(const vpImage<vpRGBa> &I, vpCCDStatistics &stats);
  void computeErrorAndInteractionMatrix();

  vpCCDParameters m_ccdParameters;

  std::vector<vpRBSilhouetteControlPoint> m_controlPoints; //! Silhouette points where to compute CCD statistics
  vpRobust m_robust;

  vpCCDStatistics m_stats;
  vpCCDStatistics m_prevStats;

  vpMatrix m_sigma;

  double m_vvsConvergenceThreshold;
  double tol;

  std::vector<double> m_gradientData;
  std::vector<double> m_hessianData;

  std::vector<vpColVector> m_gradients;
  std::vector<vpMatrix> m_hessians;
  vpColVector m_gradient; //! Sum of local gradients
  vpMatrix m_hessian; //! Sum of local hessians
  double m_temporalSmoothingFac; //! Smoothing factor used to integrate data from the previous frame.

  bool m_useMask;
  double m_minMaskConfidence;

  unsigned int m_maxPoints;

  vpUniRand m_random;

  vpDisplayType m_displayType;
  const vpRBFeatureTrackerInput *m_previousFrame;
};

END_VISP_NAMESPACE

#endif
