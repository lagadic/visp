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
  \file vpRBTrackingTimings.h
  \brief Information storage for render based tracking process.
*/
#ifndef VP_RB_TRACKING_TIMINGS_H
#define VP_RB_TRACKING_TIMINGS_H

#include <iomanip>
#include <map>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>


#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE
/*!
  \brief Information storage for render based tracking process.

  \ingroup group_rbt_core
*/

class vpRBTrackingTimings;

std::ostream &operator<<(std::ostream &s, const vpRBTrackingTimings &I);

class VISP_EXPORT vpRBTrackingTimings
{
public:
  inline void reset()
  {
    m_renderTime = 0.0;
    m_silhouetteExtractionTime = 0.0;
    m_odometryTime = 0.0;
    m_driftTime = 0.0;

    m_trackerIterStartTime.clear();
    m_trackerFeatureExtractionTime.clear();

    m_trackerFeatureTrackingTime.clear();
    m_trackerVVSIterTimes.clear();
  }

  friend std::ostream &operator<<(std::ostream &, const vpRBTrackingTimings &);

  void startTimer() { m_startTime = vpTime::measureTimeMs(); }
  inline double endTimer()
  {
    if (m_startTime < 0.f) throw vpException(vpException::notInitialized, "Tried to query timer without starting it.");
    double elapsed = vpTime::measureTimeMs() - m_startTime;
    m_startTime = -1.f;
    return elapsed;
  }

  inline void setRenderTime(double elapsed) { m_renderTime = elapsed; }
  inline void setSilhouetteTime(double elapsed) { m_silhouetteExtractionTime = elapsed; }
  inline void setMaskTime(double elapsed) { m_maskTime = elapsed; }

  inline void insertTrackerTime(std::map<int, std::vector<double>> &map, int id, double elapsed)
  {
    if (map.find(id) == map.end()) {
      map.insert(std::make_pair(id, std::vector<double>()));
    }
    map.find(id)->second.push_back(elapsed);
  }
  inline void addTrackerVVSTime(int id, double elapsed)
  {
    insertTrackerTime(m_trackerVVSIterTimes, id, elapsed);
  }

  inline void setTrackerIterStartTime(int id, double elapsed)
  {
    m_trackerIterStartTime[id] = elapsed;
  }

  inline void setTrackerFeatureExtractionTime(int id, double elapsed)
  {
    m_trackerFeatureExtractionTime[id] = elapsed;
  }

  inline void setTrackerFeatureTrackingTime(int id, double elapsed)
  {
    m_trackerFeatureTrackingTime[id] = elapsed;
  }

  inline void setInitVVSTime(int id, double elapsed)
  {
    m_trackerInitVVSTime[id] = elapsed;
  }

  inline void setDriftDetectionTime(double elapsed)
  {
    m_driftTime = elapsed;
  }

  inline void setOdometryTime(double elapsed)
  {
    m_odometryTime = elapsed;
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  inline friend void from_json(const nlohmann::json &j, vpRBTrackingTimings &result);
  inline friend void to_json(nlohmann::json &j, const vpRBTrackingTimings &result);

#endif

private:
  double m_startTime;

  double m_renderTime;
  double m_silhouetteExtractionTime;
  double m_maskTime;
  double m_driftTime;
  double m_odometryTime;

  std::map<int, std::vector<double>> m_trackerVVSIterTimes;
  std::map<int, double> m_trackerIterStartTime;
  std::map<int, double> m_trackerFeatureExtractionTime;
  std::map<int, double> m_trackerFeatureTrackingTime;
  std::map<int, double> m_trackerInitVVSTime;

};

inline std::ostream &operator<<(std::ostream &out, const vpRBTrackingTimings &timer)
{
  const auto default_precision { out.precision() };
  auto flags = out.flags();
  out << std::setprecision(2) << std::fixed;
  out << "====================================================" << std::endl;
  out << "Render: " << timer.m_renderTime << "ms" << std::endl;
  out << "Mask: " << timer.m_maskTime << "ms" << std::endl;
  out << "Drift: " << timer.m_driftTime << "ms" << std::endl;
  out << "Odometry: " << timer.m_odometryTime << "ms" << std::endl;
  out << "Silhouette extraction: " << timer.m_silhouetteExtractionTime << "ms" << std::endl;

  out << "Trackers: " << std::endl;
  for (const std::pair<const int, std::vector<double>> &vvsIterData : timer.m_trackerVVSIterTimes) {
    double trackingStartTime = timer.m_trackerIterStartTime.find(vvsIterData.first)->second;
    double featTrackTime = timer.m_trackerFeatureTrackingTime.find(vvsIterData.first)->second;
    double featExtractionTime = timer.m_trackerFeatureExtractionTime.find(vvsIterData.first)->second;
    double initVVSTime = timer.m_trackerInitVVSTime.find(vvsIterData.first)->second;

    double ttVVSIter = 0.f;
    for (double v : vvsIterData.second) {
      ttVVSIter += v;
    }
    out << "\t" << vvsIterData.first << std::endl;
    out << "\t" << "\t" << "Tracking initialization: " << trackingStartTime << "ms" << std::endl;
    out << "\t" << "\t" << "Feature extraction: " << featExtractionTime << "ms" << std::endl;
    out << "\t" << "\t" << "Feature tracking: " << featTrackTime << "ms" << std::endl;
    out << "\t" << "\t" << "VVS init: " << initVVSTime << "ms" << std::endl;
    out << "\t" << "\t" << "VVS:      " << ttVVSIter << "ms (" << vpMath::getMean(vvsIterData.second) << "ms"
      << "+-" << vpMath::getStdev(vvsIterData.second) << "ms)" << std::endl;
  }
  out << "====================================================" << std::endl;
  out.flags(flags);
  out << std::setprecision(default_precision); // restore defaults
  return out;
}

#ifdef VISP_HAVE_NLOHMANN_JSON
inline void from_json(const nlohmann::json &j, vpRBTrackingTimings &result)
{
  result.m_renderTime = j.at("render");
  result.m_silhouetteExtractionTime = j.at("silhouetteExtraction");
  result.m_maskTime = j.at("mask");
  result.m_driftTime = j.at("drift");
  result.m_odometryTime = j.at("odometry");

  nlohmann::json jf = j.at("features");
  result.m_trackerVVSIterTimes = jf.at("vvs");
  result.m_trackerIterStartTime = jf.at("start");
  result.m_trackerFeatureExtractionTime = jf.at("extraction");
  result.m_trackerFeatureTrackingTime = jf.at("tracking");
  result.m_trackerInitVVSTime = jf.at("vvsInit");
}
inline void to_json(nlohmann::json &j, const vpRBTrackingTimings &result)
{
  j["render"] = result.m_renderTime;
  j["silhouetteExtraction"] = result.m_silhouetteExtractionTime;
  j["mask"] = result.m_maskTime;
  j["drift"] = result.m_driftTime;
  j["odometry"] = result.m_odometryTime;
  nlohmann::json jf;
  jf["vvs"] = result.m_trackerVVSIterTimes;
  jf["start"] = result.m_trackerIterStartTime;
  jf["extraction"] = result.m_trackerFeatureExtractionTime;
  jf["tracking"] = result.m_trackerFeatureTrackingTime;
  jf["vvsInit"] = result.m_trackerInitVVSTime;
  j["features"] = jf;
}
#endif


END_VISP_NAMESPACE

#endif
