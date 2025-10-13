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

#ifndef VP_RB_CONVERGENCE_METRIC_H
#define VP_RB_CONVERGENCE_METRIC_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpUniRand.h>

#include <visp3/rbt/vpPointMap.h>
#include <visp3/rbt/vpObjectCentricRenderer.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpRBConvergenceMetric
{
public:
  vpRBConvergenceMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed);
  virtual ~vpRBConvergenceMetric() = default;
  bool shouldUpdateRender(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2)
  {
    return (*this)(cam, cTo1, cTo2) > m_rerenderThreshold;
  }
  bool hasConverged(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2)
  {
    return (*this)(cam, cTo1, cTo2) < m_convergedThreshold;
  }

  double getUpdateRenderThreshold() const { return m_rerenderThreshold; }
  double getConvergenceThreshold() const { return m_convergedThreshold; }


  void sampleObject(vpObjectCentricRenderer &renderer);

  virtual double operator()(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2) = 0;

#ifdef VISP_HAVE_NLOHMANN_JSON
  static std::shared_ptr<vpRBConvergenceMetric> loadFromJSON(const nlohmann::json &j);
#endif

protected:
  unsigned int m_seed;
  vpPointMap m_map;
  vpUniRand m_random;
  vpArray2D<int> m_indices;

  double m_rerenderThreshold;
  double m_convergedThreshold;
};

class VISP_EXPORT vpRBConvergenceADDMetric : public vpRBConvergenceMetric
{
public:
  vpRBConvergenceADDMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed);
  double operator()(const vpCameraParameters & /*cam*/, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2) VP_OVERRIDE;
};

class VISP_EXPORT vpRBConvergenceReprojectionMetric : public vpRBConvergenceMetric
{
public:
  vpRBConvergenceReprojectionMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed);
  double operator()(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2) VP_OVERRIDE;
};

END_VISP_NAMESPACE

#endif
