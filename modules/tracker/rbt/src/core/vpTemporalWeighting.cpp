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

#include <visp3/rbt/vpTemporalWeighting.h>

#include <cmath>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

#if defined(VISP_HAVE_NLOHMANN_JSON)
std::shared_ptr<vpTemporalWeighting> vpTemporalWeighting::parseTemporalWeighting(const nlohmann::json &j)
{
  if (j.is_number()) {
    return std::make_shared<vpFixedTemporalWeighting>(j.get<double>());
  }
  else if (j.is_object()) {
    bool slopeIncreasing = j.value("increasing", true);
    double slopePower = j.value("slopePower", 1.0); // Linear increase

    if ((!slopeIncreasing && slopePower > 0.0) || (slopeIncreasing && slopePower < 0.0)) {
      slopePower = -slopePower;
    }
    return std::make_shared<vpSigmoidTemporalWeighting>(
      j.value("minWeight", 0.0),
      j.value("maxWeight", 1.0),
      j.value("midpointLocation", 0.5),
      slopePower
    );
  }
  else {
    throw vpException(vpException::badValue, "Wrong weighting type");
  }
}
std::shared_ptr<vpTemporalWeighting> vpTemporalWeighting::parseTemporalWeightingRawJson(const std::string &s)
{
  nlohmann::json j = nlohmann::json::parse(s);
  return parseTemporalWeighting(j);
}

#endif

double vpFixedTemporalWeighting::weight(const double /*progress*/) const
{
  return m_weight;
}

double vpSigmoidTemporalWeighting::weight(const double progress) const
{
  double w = 1.0;
  if (progress == 0.0 || m_location == 1.0) {
    if (m_power > 0.0) {
      w = 0.0;
    }
    else {
      w = 1.0;
    }
  }
  else {
    double f1 = m_location / progress;
    double f2 = (1.0 - progress) / (1.0 - m_location);

    w = (1.0 / (1.0 + (std::pow(f1 * f2, m_power))));
  }
  return m_minWeight + w * (m_maxWeight - m_minWeight);
}

END_VISP_NAMESPACE
