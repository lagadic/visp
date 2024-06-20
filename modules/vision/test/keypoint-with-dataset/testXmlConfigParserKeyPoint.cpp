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
 *
 * Description:
 * Test vpXmlConfigParserKeyPoint parse / save.
 */

/*!
  \file testXmlConfigParserKeyPoint.cpp

  Test vpXmlConfigParserKeyPoint parse / save.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpXmlConfigParserKeyPoint.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_PUGIXML)
  std::string visp_images_dir = vpIoTools::getViSPImagesDataPath();
  if (vpIoTools::checkDirectory(visp_images_dir + "/xml")) {
    double eps = std::numeric_limits<double>::epsilon();
    {
      std::string filename = visp_images_dir + "/xml/detection-config.xml";
      vpXmlConfigParserKeyPoint xml;
      xml.parse(filename);

      if (xml.getDetectorName() != "FAST" || xml.getExtractorName() != "ORB" ||
          xml.getMatcherName() != "BruteForce-Hamming" ||
          xml.getMatchingMethod() != vpXmlConfigParserKeyPoint::ratioDistanceThreshold ||
          !vpMath::equal(xml.getMatchingRatioThreshold(), 0.8, eps) || !xml.getUseRansacVVSPoseEstimation() ||
          !xml.getUseRansacConsensusPercentage() || !vpMath::equal(xml.getRansacConsensusPercentage(), 20.0, eps) ||
          xml.getNbRansacIterations() != 200 || !vpMath::equal(xml.getRansacThreshold(), 0.005, eps)) {
        std::cerr << "Issue when parsing xml: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::string filename = visp_images_dir + "/xml/detection-config-SIFT.xml";
      vpXmlConfigParserKeyPoint xml;
      xml.parse(filename);

      if (xml.getDetectorName() != "SIFT" || xml.getExtractorName() != "SIFT" || xml.getMatcherName() != "BruteForce" ||
          xml.getMatchingMethod() != vpXmlConfigParserKeyPoint::ratioDistanceThreshold ||
          !vpMath::equal(xml.getMatchingRatioThreshold(), 0.8, eps) || !xml.getUseRansacVVSPoseEstimation() ||
          !xml.getUseRansacConsensusPercentage() || !vpMath::equal(xml.getRansacConsensusPercentage(), 20.0, eps) ||
          xml.getNbRansacIterations() != 200 || !vpMath::equal(xml.getRansacThreshold(), 0.005, eps)) {
        std::cerr << "Issue when parsing xml: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }
#endif
  return EXIT_SUCCESS;
}
