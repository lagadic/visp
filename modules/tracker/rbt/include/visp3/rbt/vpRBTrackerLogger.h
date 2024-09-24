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
  \file vpRBTrackerLogger.h
  \brief Information storage for render based tracking process.
*/
#ifndef VP_RB_TRACKER_LOGGER_H
#define VP_RB_TRACKER_LOGGER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \brief Information storage for render based tracking process.

  \ingroup group_rbt_core
*/

class vpRBTrackerLogger;

std::ostream &operator<<(std::ostream &s, const vpRBTrackerLogger &I);

class VISP_EXPORT vpRBTrackerLogger
{
public:
  void reset()
  {
    m_renderTime = 0.0;
    m_silhouetteExtractionTime = 0.0;
    m_trackerIterStartTime.clear();
    m_trackerFeatureExtractionTime.clear();

    m_trackerFeatureTrackingTime.clear();
    m_trackerVVSIterTimes.clear();
  }

  friend std::ostream &operator<<(std::ostream &, const vpRBTrackerLogger &);

  void startTimer() { m_startTime = vpTime::measureTimeMs(); }
  double endTimer()
  {
    if (m_startTime < 0.f) throw vpException(vpException::notInitialized, "Tried to query timer without starting it.");
    double elapsed = vpTime::measureTimeMs() - m_startTime;
    m_startTime = -1.f;
    return elapsed;
  }

  void setRenderTime(double elapsed) { m_renderTime = elapsed; }
  void setSilhouetteTime(double elapsed) { m_silhouetteExtractionTime = elapsed; }
  void setMaskTime(double elapsed) { m_maskTime = elapsed; }


  void insertTrackerTime(std::map<int, std::vector<double>> &map, int id, double elapsed)
  {
    if (map.find(id) == map.end()) {
      map.insert(std::make_pair(id, std::vector<double>()));
    }
    map.find(id)->second.push_back(elapsed);
  }
  void addTrackerVVSTime(int id, double elapsed)
  {
    insertTrackerTime(m_trackerVVSIterTimes, id, elapsed);
  }

  void setTrackerIterStartTime(int id, double elapsed)
  {
    m_trackerIterStartTime[id] = elapsed;
  }

  void setTrackerFeatureExtractionTime(int id, double elapsed)
  {
    m_trackerFeatureExtractionTime[id] = elapsed;
  }

  void setTrackerFeatureTrackingTime(int id, double elapsed)
  {
    m_trackerFeatureTrackingTime[id] = elapsed;
  }

  void setInitVVSTime(int id, double elapsed)
  {
    m_trackerInitVVSTime[id] = elapsed;
  }

  void setDriftDetectionTime(double elapsed)
  {
    m_driftTime = elapsed;
  }

private:
  double m_startTime;
  double m_renderTime;
  double m_silhouetteExtractionTime;
  double m_maskTime;
  double m_driftTime;
  std::map<int, std::vector<double>> m_trackerVVSIterTimes;

  std::map<int, double> m_trackerIterStartTime;

  std::map<int, double> m_trackerFeatureExtractionTime;

  std::map<int, double> m_trackerFeatureTrackingTime;
  std::map<int, double> m_trackerInitVVSTime;
  std::map<int, int> m_trackerNumFeatures;

};

std::ostream &operator<<(std::ostream &out, const vpRBTrackerLogger &timer)
{
  ssize_t ss = out.precision();
  out << std::setprecision(2) << std::fixed;
  out << "====================================================" << std::endl;
  out << "Render: " << timer.m_renderTime << "ms" << std::endl;
  out << "Mask: " << timer.m_maskTime << "ms" << std::endl;
  out << "Drift: " << timer.m_driftTime << "ms" << std::endl;
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
  out << std::setprecision(ss);
  return out;
}

END_VISP_NAMESPACE
#endif
