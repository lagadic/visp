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
#ifndef VP_TEMPORAL_WEIGHTING_H
#define VP_TEMPORAL_WEIGHTING_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpTemporalWeighting
{
public:
  vpTemporalWeighting() = default;
  virtual double weight(const double progress) const = 0;
  virtual ~vpTemporalWeighting() = default;
#if defined(VISP_HAVE_NLOHMANN_JSON)
  static std::shared_ptr<vpTemporalWeighting> parseTemporalWeighting(const nlohmann::json &j);
  static std::shared_ptr<vpTemporalWeighting> parseTemporalWeightingRawJson(const std::string &j);

#endif
};

class VISP_EXPORT vpFixedTemporalWeighting : public vpTemporalWeighting
{
public:
  vpFixedTemporalWeighting(double weight) : m_weight(weight) { }

  double weight(const double /*progress*/) const VP_OVERRIDE;

  double getWeight() const { return m_weight; }
  void setWeight(double weight) { m_weight = weight; }
private:
  double m_weight;
};

class VISP_EXPORT vpSigmoidTemporalWeighting : public vpTemporalWeighting
{
public:
  vpSigmoidTemporalWeighting(double minWeight, double maxWeight, double location, double power)
  {
    setLocation(location);
    setSlopePower(power);
    setMinimumWeight(minWeight);
    setMaximumWeight(maxWeight);

    if (maxWeight < minWeight) {
      throw vpException(vpException::badValue, "Maximum weight should be equal or greater than the minimum weight");
    }
  }

  double getLocation() const { return m_location; }
  void setLocation(double location)
  {
    if (location < 0 || location > 1) {
      throw vpException(vpException::badValue, "Location parameter needs to be between 0 and 1");
    }
    m_location = location;
  }

  double weight(const double progress) const VP_OVERRIDE;

  double getSlopePower() const { return m_power; }
  void setSlopePower(double power) { m_power = power; }

  double getMinimumWeight() const { return m_minWeight; }
  void setMinimumWeight(double w) { m_minWeight = w; }

  double getMaximumWeight() const { return m_maxWeight; }
  void setMaximumWeight(double w) { m_maxWeight = w; }


private:
  double m_location;
  double m_power;
  double m_minWeight;
  double m_maxWeight;

};

END_VISP_NAMESPACE

#endif
