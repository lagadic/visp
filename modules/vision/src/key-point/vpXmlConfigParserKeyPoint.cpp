/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpXmlConfigParserKeyPoint.cpp
  \brief Definition of the vpXmlConfigParserKeyPoint class member functions.
  Class vpXmlConfigParserKeyPoint permits to load configuration defined in a
  XML file for vpKeyPoint class.

*/

#include <iostream>

#include <visp3/vision/vpXmlConfigParserKeyPoint.h>

#ifdef VISP_HAVE_XML2

vpXmlConfigParserKeyPoint::vpXmlConfigParserKeyPoint()
  : m_detectorName("ORB"), m_extractorName("ORB"), m_matcherName("BruteForce-Hamming"), m_matchingFactorThreshold(2.0),
    m_matchingMethod(ratioDistanceThreshold), m_matchingRatioThreshold(0.85), m_nbRansacIterations(200),
    m_nbRansacMinInlierCount(100), m_ransacConsensusPercentage(20.0), m_ransacReprojectionError(6.0),
    m_ransacThreshold(0.01), m_useRansacConsensusPercentage(false), m_useRansacVVS(true)
{
  init();
}

/*!
  Initialize the nodeMap for the node parsing.
*/
void vpXmlConfigParserKeyPoint::init()
{
  setMainTag("conf");

  nodeMap["conf"] = conf;
  nodeMap["detector"] = detector;
  nodeMap["extractor"] = extractor;
  nodeMap["matcher"] = matcher;
  nodeMap["name"] = name;
  nodeMap["matching_method"] = matching_method;
  nodeMap["constantFactorDistanceThreshold"] = constant_factor_distance_threshold;
  nodeMap["stdDistanceThreshold"] = std_distance_threshold;
  nodeMap["ratioDistanceThreshold"] = ratio_distance_threshold;
  nodeMap["stdAndRatioDistanceThreshold"] = std_and_ratio_distance_threshold;
  nodeMap["noFilterMatching"] = no_filter_matching;
  nodeMap["matchingFactorThreshold"] = matching_factor_threshold;
  nodeMap["matchingRatioThreshold"] = matching_ratio_threshold;
  nodeMap["ransac"] = ransac;
  nodeMap["useRansacVVS"] = use_ransac_vvs;
  nodeMap["useRansacConsensusPercentage"] = use_ransac_consensus_percentage;
  nodeMap["nbRansacIterations"] = nb_ransac_iterations;
  nodeMap["ransacReprojectionError"] = ransac_reprojection_error;
  nodeMap["nbRansacMinInlierCount"] = nb_ransac_min_inlier_count;
  nodeMap["ransacThreshold"] = ransac_threshold;
  nodeMap["ransacConsensusPercentage"] = ransac_consensus_percentage;
}

/*!
  Parse an XML file to load configuration for vpKeyPoint class.
  \param filename : filename of the XML file to parse.
*/
void vpXmlConfigParserKeyPoint::parse(const std::string &filename) { vpXmlParser::parse(filename); }

/*!
  Read the parameters of the class from the file given by its document pointer
  and by its root node.

  \param doc : Document to parse.
  \param node : Root node.
*/
void vpXmlConfigParserKeyPoint::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  bool detector_node = false;
  bool extractor_node = false;
  bool matcher_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case detector:
          this->read_detector(doc, dataNode);
          detector_node = true;
          break;

        case extractor:
          this->read_extractor(doc, dataNode);
          extractor_node = true;
          break;

        case matcher:
          this->read_matcher(doc, dataNode);
          matcher_node = true;
          break;

        case ransac:
          this->read_ransac(doc, dataNode);
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

  \param doc : Document to parse.
  \param node : Detector node.
*/
void vpXmlConfigParserKeyPoint::read_detector(xmlDocPtr doc, xmlNodePtr node)
{
  bool detector_name_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case name:
          m_detectorName = xmlReadStringChild(doc, dataNode);
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

  \param doc : Document to parse.
  \param node : Extractor node.
*/
void vpXmlConfigParserKeyPoint::read_extractor(xmlDocPtr doc, xmlNodePtr node)
{
  bool extractor_name_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case name:
          m_extractorName = xmlReadStringChild(doc, dataNode);
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

  \param doc : Document to parse.
  \param node : Matcher node.
*/
void vpXmlConfigParserKeyPoint::read_matcher(xmlDocPtr doc, xmlNodePtr node)
{
  bool matcher_name_node = false;
  bool matching_method_node = false;
  std::string matchingMethodName = "ratioDistanceThreshold";
  bool matching_factor_threshold_node = false;
  bool matching_ratio_threshold_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case name:
          m_matcherName = xmlReadStringChild(doc, dataNode);
          matcher_name_node = true;
          break;

        case matching_method: {
          matchingMethodName = xmlReadStringChild(doc, dataNode);

          std::map<std::string, int>::iterator iter_data2 = nodeMap.find(matchingMethodName);
          if (iter_data2 != nodeMap.end()) {
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
          m_matchingFactorThreshold = xmlReadDoubleChild(doc, dataNode);
          matching_factor_threshold_node = true;
          break;

        case matching_ratio_threshold:
          m_matchingRatioThreshold = xmlReadDoubleChild(doc, dataNode);
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

  \param doc : Document to parse.
  \param node : Ransac node.
*/
void vpXmlConfigParserKeyPoint::read_ransac(xmlDocPtr doc, xmlNodePtr node)
{
  bool use_ransac_vvs_node = false;
  bool use_ransac_consensus_percentage_node = false;
  bool nb_ransac_iterations_node = false;
  bool ransac_reprojection_error_node = false;
  bool nb_ransac_min_inlier_count_node = false;
  bool ransac_threshold_node = false;
  bool ransac_consensus_percentage_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case use_ransac_vvs:
          m_useRansacVVS = xmlReadIntChild(doc, dataNode) != 0;
          use_ransac_vvs_node = true;
          break;

        case use_ransac_consensus_percentage:
          m_useRansacConsensusPercentage = xmlReadIntChild(doc, dataNode) != 0;
          use_ransac_consensus_percentage_node = true;
          break;

        case nb_ransac_iterations:
          m_nbRansacIterations = xmlReadIntChild(doc, dataNode);
          nb_ransac_iterations_node = true;
          break;

        case ransac_reprojection_error:
          m_ransacReprojectionError = xmlReadDoubleChild(doc, dataNode);
          ransac_reprojection_error_node = true;
          break;

        case nb_ransac_min_inlier_count:
          m_nbRansacMinInlierCount = xmlReadIntChild(doc, dataNode);
          nb_ransac_min_inlier_count_node = true;
          break;

        case ransac_threshold:
          m_ransacThreshold = xmlReadDoubleChild(doc, dataNode);
          ransac_threshold_node = true;
          break;

        case ransac_consensus_percentage:
          m_ransacConsensusPercentage = xmlReadDoubleChild(doc, dataNode);
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

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_vision.a(vpXmlConfigParserKeyPoint.cpp.o) has no symbols
void dummy_vpXmlConfigParserKeyPoint(){};
#endif // VISP_HAVE_XML2
