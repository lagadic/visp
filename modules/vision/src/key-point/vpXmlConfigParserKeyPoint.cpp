/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * XML parser to load configuration for vpKeyPoint class.
 */

/*!
  \file vpXmlConfigParserKeyPoint.cpp
  \brief Definition of the vpXmlConfigParserKeyPoint class member functions.
  Class vpXmlConfigParserKeyPoint permits to load configuration defined in a
  XML file for vpKeyPoint class.
*/

#include <iostream>

#include <visp3/vision/vpXmlConfigParserKeyPoint.h>

#include <map>
#if defined(VISP_HAVE_PUGIXML)
#include <pugixml.hpp>

#include <visp3/core/vpException.h>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpXmlConfigParserKeyPoint::Impl
{
public:
  Impl()
    : m_detectorName("ORB"), m_extractorName("ORB"), m_matcherName("BruteForce-Hamming"),
    m_matchingFactorThreshold(2.0), m_matchingMethod(ratioDistanceThreshold), m_matchingRatioThreshold(0.85),
    m_nbRansacIterations(200), m_nbRansacMinInlierCount(100), m_ransacConsensusPercentage(20.0),
    m_ransacReprojectionError(6.0), m_ransacThreshold(0.01), m_useRansacConsensusPercentage(false),
    m_useRansacVVS(true)
  {
    init();
  }

  /*!
    Initialize the nodeMap for the node parsing.
  */
  void init()
  {
    m_nodeMap["conf"] = conf;
    m_nodeMap["detector"] = detector;
    m_nodeMap["extractor"] = extractor;
    m_nodeMap["matcher"] = matcher;
    m_nodeMap["name"] = name;
    m_nodeMap["matching_method"] = matching_method;
    m_nodeMap["constantFactorDistanceThreshold"] = constant_factor_distance_threshold;
    m_nodeMap["stdDistanceThreshold"] = std_distance_threshold;
    m_nodeMap["ratioDistanceThreshold"] = ratio_distance_threshold;
    m_nodeMap["stdAndRatioDistanceThreshold"] = std_and_ratio_distance_threshold;
    m_nodeMap["noFilterMatching"] = no_filter_matching;
    m_nodeMap["matchingFactorThreshold"] = matching_factor_threshold;
    m_nodeMap["matchingRatioThreshold"] = matching_ratio_threshold;
    m_nodeMap["ransac"] = ransac;
    m_nodeMap["useRansacVVS"] = use_ransac_vvs;
    m_nodeMap["useRansacConsensusPercentage"] = use_ransac_consensus_percentage;
    m_nodeMap["nbRansacIterations"] = nb_ransac_iterations;
    m_nodeMap["ransacReprojectionError"] = ransac_reprojection_error;
    m_nodeMap["nbRansacMinInlierCount"] = nb_ransac_min_inlier_count;
    m_nodeMap["ransacThreshold"] = ransac_threshold;
    m_nodeMap["ransacConsensusPercentage"] = ransac_consensus_percentage;
  }

  void parse(const std::string &filename)
  {
    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
      throw vpException(vpException::ioError, "Cannot open file: %s", filename.c_str());
    }

    bool detector_node = false;
    bool extractor_node = false;
    bool matcher_node = false;

    pugi::xml_node root_node = doc.document_element();
    for (pugi::xml_node dataNode = root_node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case detector:
            read_detector(dataNode);
            detector_node = true;
            break;

          case extractor:
            read_extractor(dataNode);
            extractor_node = true;
            break;

          case matcher:
            read_matcher(dataNode);
            matcher_node = true;
            break;

          case ransac:
            read_ransac(dataNode);
            break;

          default:
            break;
          }
        }
      }
    }

    if (!detector_node) {
      std::cout << "detector: name: " << m_detectorName << " (default)" << std::endl;
    }

    if (!extractor_node) {
      std::cout << "extractor: name: " << m_extractorName << " (default)" << std::endl;
    }

    if (!matcher_node) {
      std::cout << "matcher: name: " << m_matcherName << " (default)" << std::endl;
    }
  }

  /*!
    Parse detector tag part.

    \param node : Detector node.
  */
  void read_detector(const pugi::xml_node &node)
  {
    bool detector_name_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case name:
            m_detectorName = dataNode.text().as_string();
            detector_name_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (!detector_name_node)
      std::cout << "detector : Name : " << m_detectorName << " (default)" << std::endl;
    else
      std::cout << "detector : Name : " << m_detectorName << std::endl;
  }

  /*!
    Parse extractor tag part.

    \param node : Extractor node.
  */
  void read_extractor(const pugi::xml_node &node)
  {
    bool extractor_name_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case name:
            m_extractorName = dataNode.text().as_string();
            extractor_name_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (!extractor_name_node)
      std::cout << "extractor : Name : " << m_extractorName << " (default)" << std::endl;
    else
      std::cout << "extractor : Name : " << m_extractorName << std::endl;
  }

  /*!
    Parse matcher tag part.

    \param node : Matcher node.
  */
  void read_matcher(const pugi::xml_node &node)
  {
    bool matcher_name_node = false;
    bool matching_method_node = false;
    std::string matchingMethodName = "ratioDistanceThreshold";
    bool matching_factor_threshold_node = false;
    bool matching_ratio_threshold_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case name:
            m_matcherName = dataNode.text().as_string();
            matcher_name_node = true;
            break;

          case matching_method: {
            matchingMethodName = dataNode.text().as_string();

            std::map<std::string, int>::iterator iter_data2 = m_nodeMap.find(matchingMethodName);
            if (iter_data2 != m_nodeMap.end()) {
              matching_method_node = true;
              switch (iter_data2->second) {
              case constant_factor_distance_threshold:
                m_matchingMethod = constantFactorDistanceThreshold;
                break;

              case std_distance_threshold:
                m_matchingMethod = stdDistanceThreshold;
                break;

              case ratio_distance_threshold:
                m_matchingMethod = ratioDistanceThreshold;
                break;

              case std_and_ratio_distance_threshold:
                m_matchingMethod = stdAndRatioDistanceThreshold;
                break;

              case no_filter_matching:
                m_matchingMethod = noFilterMatching;
                break;

              default:
                matching_method_node = false;
                break;
              }
            }
            break;
          }

          case matching_factor_threshold:
            m_matchingFactorThreshold = dataNode.text().as_double();
            matching_factor_threshold_node = true;
            break;

          case matching_ratio_threshold:
            m_matchingRatioThreshold = dataNode.text().as_double();
            matching_ratio_threshold_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (!matcher_name_node)
      std::cout << "matcher : Name : " << m_matcherName << " (default)" << std::endl;
    else
      std::cout << "matcher : Name : " << m_matcherName << std::endl;

    if (!matching_method_node)
      std::cout << "matcher : Filter method : " << matchingMethodName << " (default)" << std::endl;
    else
      std::cout << "matcher : Filter method : " << matchingMethodName << std::endl;

    if (!matching_factor_threshold_node)
      std::cout << "matcher : matching factor threshold : " << m_matchingFactorThreshold << " (default)" << std::endl;
    else
      std::cout << "matcher : matching factor threshold : " << m_matchingFactorThreshold << std::endl;

    if (!matching_ratio_threshold_node)
      std::cout << "matcher : matching ratio threshold : " << m_matchingRatioThreshold << " (default)" << std::endl;
    else
      std::cout << "matcher : matching ratio threshold : " << m_matchingRatioThreshold << std::endl;
  }

  /*!
    Parse ransac tag part.

    \param node : Ransac node.
  */
  void read_ransac(const pugi::xml_node &node)
  {
    bool use_ransac_vvs_node = false;
    bool use_ransac_consensus_percentage_node = false;
    bool nb_ransac_iterations_node = false;
    bool ransac_reprojection_error_node = false;
    bool nb_ransac_min_inlier_count_node = false;
    bool ransac_threshold_node = false;
    bool ransac_consensus_percentage_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case use_ransac_vvs:
            m_useRansacVVS = dataNode.text().as_int() != 0;
            use_ransac_vvs_node = true;
            break;

          case use_ransac_consensus_percentage:
            m_useRansacConsensusPercentage = dataNode.text().as_int() != 0;
            use_ransac_consensus_percentage_node = true;
            break;

          case nb_ransac_iterations:
            m_nbRansacIterations = dataNode.text().as_int();
            nb_ransac_iterations_node = true;
            break;

          case ransac_reprojection_error:
            m_ransacReprojectionError = dataNode.text().as_double();
            ransac_reprojection_error_node = true;
            break;

          case nb_ransac_min_inlier_count:
            m_nbRansacMinInlierCount = dataNode.text().as_int();
            nb_ransac_min_inlier_count_node = true;
            break;

          case ransac_threshold:
            m_ransacThreshold = dataNode.text().as_double();
            ransac_threshold_node = true;
            break;

          case ransac_consensus_percentage:
            m_ransacConsensusPercentage = dataNode.text().as_double();
            ransac_consensus_percentage_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (!use_ransac_vvs_node)
      std::cout << "ransac: use ransac vvs pose estimation: " << m_useRansacVVS << " (default)" << std::endl;
    else
      std::cout << "ransac: use ransac vvs pose estimation: " << m_useRansacVVS << std::endl;

    if (!use_ransac_consensus_percentage_node)
      std::cout << "ransac: use consensus percentage: " << m_useRansacConsensusPercentage << " (default)" << std::endl;
    else
      std::cout << "ransac: use consensus percentage: " << m_useRansacConsensusPercentage << std::endl;

    if (!nb_ransac_iterations_node)
      std::cout << "ransac: nb ransac iterations: " << m_nbRansacIterations << " (default)" << std::endl;
    else
      std::cout << "ransac: nb ransac iterations: " << m_nbRansacIterations << std::endl;

    if (!ransac_reprojection_error_node)
      std::cout << "ransac: ransac reprojection error in pixel (for OpenCV "
      "function): "
      << m_ransacReprojectionError << " (default)" << std::endl;
    else
      std::cout << "ransac: ransac reprojection error in pixel (for OpenCV "
      "function): "
      << m_ransacReprojectionError << std::endl;

    if (!nb_ransac_min_inlier_count_node)
      std::cout << "ransac: nb ransac min inlier count: " << m_nbRansacMinInlierCount << " (default)" << std::endl;
    else
      std::cout << "ransac: nb ransac min inlier count: " << m_nbRansacMinInlierCount << std::endl;

    if (!ransac_threshold_node)
      std::cout << "ransac: ransac threshold in meter (for ViSP function): " << m_ransacThreshold << " (default)"
      << std::endl;
    else
      std::cout << "ransac: ransac threshold in meter (for ViSP function): " << m_ransacThreshold << std::endl;

    if (!ransac_consensus_percentage_node)
      std::cout << "ransac: consensus percentage: " << m_ransacConsensusPercentage << " (default)" << std::endl;
    else
      std::cout << "ransac: consensus percentage: " << m_ransacConsensusPercentage << std::endl;
  }

  std::string getDetectorName() const { return m_detectorName; }
  std::string getExtractorName() const { return m_extractorName; }
  std::string getMatcherName() const { return m_matcherName; }

  double getMatchingFactorThreshold() const { return m_matchingFactorThreshold; }
  vpMatchingMethodEnum getMatchingMethod() const { return m_matchingMethod; }
  double getMatchingRatioThreshold() const { return m_matchingRatioThreshold; }

  int getNbRansacIterations() const { return m_nbRansacIterations; }
  int getNbRansacMinInlierCount() const { return m_nbRansacMinInlierCount; }
  double getRansacConsensusPercentage() const { return m_ransacConsensusPercentage; }
  double getRansacReprojectionError() const { return m_ransacReprojectionError; }
  double getRansacThreshold() const { return m_ransacThreshold; }
  bool getUseRansacConsensusPercentage() const { return m_useRansacConsensusPercentage; }
  bool getUseRansacVVSPoseEstimation() const { return m_useRansacVVS; }

protected:
  /*! Predefined xml node identifier. */
  enum vpNodeIdentifier
  {
    conf,                               /*!< Identifier associated to the root tag. */
    detector,                           /*!< Identifier associated to the detector tag. */
    extractor,                          /*!< Identifier associated to the extractor tag. */
    matcher,                            /*!< Identifier associated to the matcher tag. */
    name,                               /*!< Identifier associated to the name tag. */
    matching_method,                    /*!< Identifier associated to the matching_method tag. */
    constant_factor_distance_threshold, /*!< Identifier associated to the
                                           constant_factor_distance_threshold
                                           tag. */
    std_distance_threshold,             /*!< Identifier associated to the
                                           std_distance_threshold tag. */
    ratio_distance_threshold,           /*!< Identifier associated to the
                                           ratio_distance_threshold tag. */
    std_and_ratio_distance_threshold,   /*!< Identifier associated to the
                                           std_and_ratio_distance_threshold tag.
                                         */
    no_filter_matching,                 /*!< Identifier associated to the no_filter_matching
                                           tag. */
    matching_factor_threshold,          /*!< Identifier associated to the
                                           matching_factor_threshold tag. */
    matching_ratio_threshold,           /*!< Identifier associated to the
                                           matching_ratio_threshold tag. */
    ransac,                             /*!< Identifier associated to the ransac tag. */
    use_ransac_vvs,                     /*!< Identifier associated to the use_ransac_vvs tag. */
    use_ransac_consensus_percentage,    /*!< Identifier associated to the
                                           use_ransac_consensus_percentage tag.
                                         */
    nb_ransac_iterations,               /*!< Identifier associated to the
                                           nb_ransac_iterations tag. */
    ransac_reprojection_error,          /*!< Identifier associated to the
                                           ransac_reprojection_error tag. */
    nb_ransac_min_inlier_count,         /*!< Identifier associated to the
                                           nb_ransac_min_inlier_count tag. */
    ransac_threshold,                   /*!< Identifier associated to the ransac_threshold tag.
                                         */
    ransac_consensus_percentage         /*!< Identifier associated to the
                                           ransac_consensus_percentage tag. */
  };

  //! Name of the keypoint detector.
  std::string m_detectorName;
  //! Name of the keypoint extractor.
  std::string m_extractorName;
  //! Name of the keypoint matcher.
  std::string m_matcherName;
  //! Factor value for filtering method: constantFactorDistanceThreshold.
  double m_matchingFactorThreshold;
  //! Filtering method.
  vpMatchingMethodEnum m_matchingMethod;
  //! Ratio value for filtering method: ratioDistanceThreshold.
  double m_matchingRatioThreshold;
  //! Maximum number of iterations for the Ransac method.
  int m_nbRansacIterations;
  //! Minimum number of inliers for the Ransac method.
  int m_nbRansacMinInlierCount;
  //! Percentage value of inliers compared to outliers for the Ransac method.
  double m_ransacConsensusPercentage;
  //! Maximum reprojection error (in pixel for OpenCV method) to consider a
  //! point as an inlier.
  double m_ransacReprojectionError;
  //! Maximum error (in meter for ViSP method) to consider a point as an
  //! inlier.
  double m_ransacThreshold;
  //! If true, the cardinality of the consensus is based upon a percentage,
  //! otherwise
  // it is based on a fixed number.
  bool m_useRansacConsensusPercentage;
  //! If true, use ViSP Ransac VVS pose estimation method, otherwise use
  //! OpenCV method.
  bool m_useRansacVVS;
  std::map<std::string, int> m_nodeMap;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpXmlConfigParserKeyPoint::vpXmlConfigParserKeyPoint() : m_impl(new Impl()) { }

vpXmlConfigParserKeyPoint::~vpXmlConfigParserKeyPoint() { delete m_impl; }

void vpXmlConfigParserKeyPoint::parse(const std::string &filename) { m_impl->parse(filename); }

std::string vpXmlConfigParserKeyPoint::getDetectorName() const { return m_impl->getDetectorName(); }

std::string vpXmlConfigParserKeyPoint::getExtractorName() const { return m_impl->getExtractorName(); }

std::string vpXmlConfigParserKeyPoint::getMatcherName() const { return m_impl->getMatcherName(); }

double vpXmlConfigParserKeyPoint::getMatchingFactorThreshold() const { return m_impl->getMatchingFactorThreshold(); }

vpXmlConfigParserKeyPoint::vpMatchingMethodEnum vpXmlConfigParserKeyPoint::getMatchingMethod() const
{
  return m_impl->getMatchingMethod();
}

double vpXmlConfigParserKeyPoint::getMatchingRatioThreshold() const { return m_impl->getMatchingRatioThreshold(); }

int vpXmlConfigParserKeyPoint::getNbRansacIterations() const { return m_impl->getNbRansacIterations(); }

int vpXmlConfigParserKeyPoint::getNbRansacMinInlierCount() const { return m_impl->getNbRansacMinInlierCount(); }

double vpXmlConfigParserKeyPoint::getRansacConsensusPercentage() const
{
  return m_impl->getRansacConsensusPercentage();
}

double vpXmlConfigParserKeyPoint::getRansacReprojectionError() const { return m_impl->getRansacReprojectionError(); }

double vpXmlConfigParserKeyPoint::getRansacThreshold() const { return m_impl->getRansacThreshold(); }

bool vpXmlConfigParserKeyPoint::getUseRansacConsensusPercentage() const
{
  return m_impl->getUseRansacConsensusPercentage();
}

bool vpXmlConfigParserKeyPoint::getUseRansacVVSPoseEstimation() const
{
  return m_impl->getUseRansacVVSPoseEstimation();
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpXmlConfigParserKeyPoint.cpp.o) has no symbols
void dummy_vpXmlConfigParserKeyPoint() { };

#endif
