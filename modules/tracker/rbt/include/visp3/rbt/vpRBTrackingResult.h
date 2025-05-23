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
  \file vpRBTrackingResult.h
  \brief Structure containing information about the tracking process for a given iteration
*/
#ifndef VP_RB_TRACKING_RESULT_H
#define VP_RB_TRACKING_RESULT_H

#include <visp3/core/vpConfig.h>


#include <vector>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#include <visp3/rbt/vpRBTrackingTimings.h>
#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpRBFeatureResult
{

public:
  vpRBFeatureResult() = default;
  void onIter(vpRBFeatureTracker &tracker)
  {
    m_numFeatures.push_back(tracker.getNumFeatures());
    double w = tracker.getVVSTrackerWeight();
    if (!vpMath::isFinite(w)) {
      w = 0.0;
    }
    m_overallWeight.push_back(w);
    m_error.push_back(tracker.getWeightedError());
    m_JTJ.push_back(tracker.getLTL());
    m_JTR.push_back(tracker.getLTR());
  }

  std::vector<unsigned int> getNumFeatures() const { return m_numFeatures; }
  std::vector<double> getWeight() const { return m_overallWeight; }

  std::vector<vpColVector> getErrors() const { return m_error; }
  std::vector<vpMatrix> getJTJs() const { return m_JTJ; }
  std::vector<vpColVector> getJTRs() const { return m_JTR; }


#ifdef VISP_HAVE_NLOHMANN_JSON
  inline friend void from_json(const nlohmann::json &j, vpRBFeatureResult &result);
  inline friend void to_json(nlohmann::json &j, const vpRBFeatureResult &result);
#endif

private:
  std::vector<unsigned int> m_numFeatures;
  std::vector<double> m_overallWeight;

  std::vector<vpColVector> m_error;
  std::vector<vpMatrix> m_JTJ;
  std::vector<vpColVector> m_JTR;
};


enum vpRBTrackingStoppingReason
{
  MAX_ITERS = 0, // Reached maximum number of iterations
  CONVERGENCE_CRITERION = 1, //! Convergence criterion was met, early stopping
  OBJECT_NOT_IN_IMAGE = 2, //! Object is not in image, thus tracking was not performed
  NOT_ENOUGH_FEATURES = 3, //! Not enough features to correctly track the object
  EXCEPTION = 4,
  INVALID_REASON = 5 //! This should not happen
};



class VISP_EXPORT vpRBTrackingResult
{
public:

  vpRBTrackingResult() : m_odometryMetric(0.0), m_odometryThreshold(0.0)
  {
    m_odometryMotion = vpHomogeneousMatrix();
  }

  unsigned int getNumIterations() const { return static_cast<unsigned int>(m_cMos.size()); }
  const std::vector<vpHomogeneousMatrix> &getPoses() const { return m_cMos; }
  vpHomogeneousMatrix getPoseBeforeTracking() const { return m_cMoBeforeTracking; }

  const std::vector<vpColVector> &getVelocities() const { return m_velocities; }
  const std::vector<double> &getConvergenceMetricValues() const { return m_convergenceMetric; }

  vpRBTrackingTimings &timer() { return m_timings; }

  vpRBTrackingStoppingReason getStoppingReason() const { return m_stopReason; }
  void setStoppingReason(vpRBTrackingStoppingReason reason) { m_stopReason = reason; }

  vpHomogeneousMatrix getOdometryMotion() { return m_odometryMotion; }

  vpHomogeneousMatrix getPoseBeforeOdometry() { return m_cMoBeforeOdometry; }
  vpHomogeneousMatrix getPoseAfterOdometry() { return m_cMoAfterOdometry; }

  void setOdometryMotion(const vpHomogeneousMatrix &cMo_before, const vpHomogeneousMatrix &cMcp, const vpHomogeneousMatrix &cMo_after)
  {
    m_odometryMotion = cMcp;
    m_cMoBeforeOdometry = cMo_before;
    m_cMoAfterOdometry = cMo_after;
  }

  void setOdometryMetricAndThreshold(const double metricValue, const double metricThreshold)
  {
    m_odometryMetric = metricValue;
    m_odometryThreshold = metricThreshold;
  }

  void beforeIter(const vpHomogeneousMatrix &cMo)
  {
    m_cMoBeforeTracking = cMo;
  }

  std::vector<vpRBFeatureResult> getFeatureData() const { return m_featureData; }


  inline void onEndIter(const vpHomogeneousMatrix &cMo, const vpColVector &v, const double convergenceMetric, const vpMatrix &JTJ, const vpColVector &JTR, double mu)
  {
    m_cMos.push_back(cMo);
    m_velocities.push_back(v);
    m_convergenceMetric.push_back(convergenceMetric);
    m_JTJ.push_back(JTJ);
    m_JTR.push_back(JTR);
    m_mus.push_back(mu);

  }

  inline void logFeatures(const std::vector<std::shared_ptr<vpRBFeatureTracker>> &features)
  {
    if (m_featureData.size() == 0) {
      m_featureData.resize(features.size());
    }
    else if (m_featureData.size() != features.size()) {
      throw vpException(vpException::dimensionError, "Wrong number of features were logged");
    }

    for (unsigned int i = 0; i < features.size(); ++i) {
      m_featureData[i].onIter(*features[i]);
    }
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  void saveToFile(const std::string &path) const;
  static vpRBTrackingResult readFromJsonFile(const std::string &path);
  inline friend void from_json(const nlohmann::json &j, vpRBTrackingResult &result);
  inline friend void to_json(nlohmann::json &j, const vpRBTrackingResult &result);
#endif

private:
  vpRBTrackingStoppingReason m_stopReason;
  vpRBTrackingTimings m_timings;
  std::vector<vpHomogeneousMatrix> m_cMos;
  vpHomogeneousMatrix m_cMoBeforeTracking;

  std::vector<vpColVector> m_velocities;
  std::vector<double> m_convergenceMetric;
  std::vector<double> m_mus;
  std::vector<vpMatrix> m_JTJ;
  std::vector<vpColVector> m_JTR;

  vpHomogeneousMatrix m_odometryMotion;
  vpHomogeneousMatrix m_cMoBeforeOdometry, m_cMoAfterOdometry;

  double m_odometryMetric, m_odometryThreshold;
  std::vector<vpRBFeatureResult> m_featureData;
};


#ifdef VISP_HAVE_NLOHMANN_JSON

#if defined(__clang__)
// Mute warning : declaration requires an exit-time destructor [-Wexit-time-destructors]
// message : expanded from macro 'NLOHMANN_JSON_SERIALIZE_ENUM'
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

NLOHMANN_JSON_SERIALIZE_ENUM(vpRBTrackingStoppingReason, {
        {vpRBTrackingStoppingReason::INVALID_REASON, nullptr},
        {vpRBTrackingStoppingReason::MAX_ITERS, "maxIterations"},
        {vpRBTrackingStoppingReason::CONVERGENCE_CRITERION, "converged"},
        {vpRBTrackingStoppingReason::OBJECT_NOT_IN_IMAGE, "objectNotInImage"},
        {vpRBTrackingStoppingReason::NOT_ENOUGH_FEATURES, "notEnoughFeatures"},
        {vpRBTrackingStoppingReason::EXCEPTION, "exception"}
});

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif

inline void from_json(const nlohmann::json &j, vpRBFeatureResult &result)
{
  result.m_numFeatures = j.at("numFeatures").get<std::vector<unsigned int>>();
  result.m_overallWeight = j.at("weight").get<std::vector<double>>();
  result.m_error = j.at("error");
  result.m_JTJ = j.at("JTJ");
  result.m_JTR = j.at("JTR");
}
inline void to_json(nlohmann::json &j, const vpRBFeatureResult &result)
{
  j["numFeatures"] = result.m_numFeatures;
  j["weight"] = result.m_overallWeight;
  j["error"] = result.m_error;
  j["JTJ"] = result.m_JTJ;
  j["JTR"] = result.m_JTR;
}

inline void from_json(const nlohmann::json &j, vpRBTrackingResult &result)
{
  result.m_stopReason = j.at("stopReason");
  result.m_timings = j.at("timings");
  result.m_cMos = j.at("cMos");
  result.m_cMoBeforeTracking = j.at("cMoBeforeTracking");

  result.m_velocities = j.at("velocities");
  result.m_mus = j.at("mus").get<std::vector<double>>();
  result.m_JTJ = j.at("JTJ");
  result.m_JTR = j.at("JTR");

  result.m_convergenceMetric = j.at("convergenceMetric").get<std::vector<double>>();
  result.m_odometryMotion = j.at("odometryDisplacement");
  result.m_cMoBeforeOdometry = j.at("cMoBeforeOdometry");
  result.m_cMoAfterOdometry = j.at("cMoAfterOdometry");


  result.m_odometryMetric = j.at("odometryMetric");
  result.m_odometryThreshold = j.at("odometryThreshold");

  result.m_featureData = j.at("features");
}
inline void to_json(nlohmann::json &j, const vpRBTrackingResult &result)
{
  j["stopReason"] = result.m_stopReason;
  j["timings"] = result.m_timings;
  j["cMos"] = result.m_cMos;
  j["cMoBeforeTracking"] = result.m_cMoBeforeTracking;
  j["velocities"] = result.m_velocities;
  j["mus"] = result.m_mus;
  j["JTJ"] = result.m_JTJ;
  j["JTR"] = result.m_JTR;

  j["convergenceMetric"] = result.m_convergenceMetric;
  j["odometryDisplacement"] = result.m_odometryMotion;
  j["cMoBeforeOdometry"] = result.m_cMoBeforeOdometry;
  j["cMoAfterOdometry"] = result.m_cMoAfterOdometry;
  j["odometryMetric"] = result.m_odometryMetric;
  j["odometryThreshold"] = result.m_odometryThreshold;

  j["features"] = result.m_featureData;
}

#endif


END_VISP_NAMESPACE

#endif
