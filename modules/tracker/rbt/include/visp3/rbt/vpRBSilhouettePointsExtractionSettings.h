/****************************************************************************
 *
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
 *
*****************************************************************************/

/*!
  \file vpRBSilhouettePointsExtractionSettings.h
  \brief Silhouette point extraction settings
*/

#ifndef VP_RB_SILHOUETTE_POINTS_EXTRACTION_SETTINGS_H
#define VP_RB_SILHOUETTE_POINTS_EXTRACTION_SETTINGS_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpUniRand.h>


#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif


BEGIN_VISP_NAMESPACE

class vpRBSilhouettePoint;
/*!
  \brief Silhouette point extraction settings
  \ingroup group_rbt_core
*/
class VISP_EXPORT vpSilhouettePointsExtractionSettings
{

private:
  unsigned int m_sampleStep; //! Step with which to sample the image to acquire candidates
  int m_maxNumPoints; //! Max number of points to keep
  unsigned int m_border; //! Border rejection parameter: do not seek candidates that are too close to image border

  double m_depthThreshold;
  bool m_thresholdIsRelative;
  bool m_preferPreviousPoints;


  void sampleWithoutReplacement(unsigned int count, unsigned int vectorSize, std::vector<size_t> &indices, vpUniRand &random) const
  {
    count = std::min(count, vectorSize);
    indices.resize(count);
    unsigned int added = 0;
    for (unsigned i = 0; i < vectorSize; ++i) {
      double randomVal = random.uniform(0.0, 1.0);
      if ((vectorSize - i) * randomVal < (count - added)) {
        indices[added++] = i;
      }
      if (added == count) {
        break;
      }
    }
  }

public:

  vpSilhouettePointsExtractionSettings();
  vpSilhouettePointsExtractionSettings(const vpSilhouettePointsExtractionSettings &rend);
  ~vpSilhouettePointsExtractionSettings() = default;
  const vpSilhouettePointsExtractionSettings &operator=(const vpSilhouettePointsExtractionSettings &rend);

  void setThreshold(double lambda) { m_depthThreshold = lambda; }
  double getThreshold() const { return m_depthThreshold; }
  void setThresholdIsRelative(bool isRelative) { m_thresholdIsRelative = isRelative; }
  bool thresholdIsRelative() const { return m_thresholdIsRelative; }

  void setMaxCandidates(int maxCandidates) { m_maxNumPoints = maxCandidates; }
  void setSampleStep(unsigned int a)
  {
    if (m_sampleStep == 0) {
      throw vpException(vpException::badValue, "Sample step should be greater than 0");
    }
    m_sampleStep = a;
  }

  std::vector<std::pair<unsigned int, unsigned int>> getSilhouetteCandidates(
    const vpImage<unsigned char> &validSilhouette, const vpImage<float> &renderDepth,
    const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp,
    const std::vector<vpRBSilhouettePoint> &previousPoints, long randomSeed = 41) const;


#if defined(VISP_HAVE_NLOHMANN_JSON)
  inline friend void from_json(const nlohmann::json &j, vpSilhouettePointsExtractionSettings &settings);
#endif

};

#if defined(VISP_HAVE_NLOHMANN_JSON)
inline void from_json(const nlohmann::json &j, vpSilhouettePointsExtractionSettings &settings)
{
  nlohmann::json thresholdSettings = j.at("threshold");
  std::string thresholdType = thresholdSettings.at("type");
  settings.m_thresholdIsRelative = thresholdType == "relative";
  settings.m_preferPreviousPoints = thresholdSettings.at("reusePreviousPoints");

  settings.m_depthThreshold = thresholdSettings.at("value");

  nlohmann::json samplingSettings = j.at("sampling");
  settings.m_maxNumPoints = samplingSettings.at("numPoints");
  settings.m_sampleStep = samplingSettings.at("samplingRate");

}
#endif

END_VISP_NAMESPACE


#endif
#endif
