/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
  Class vpXmlConfigParserKeyPoint allows to load configuration defined  in a
  XML file for vpKeyPoint class.

*/

#ifndef __vpXmlConfigParserKeyPoint_h__
#define __vpXmlConfigParserKeyPoint_h__

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <iostream>
#include <stdlib.h>

#include <libxml/xmlmemory.h> // Functions of libxml.

#include <visp3/core/vpXmlParser.h>

/*!
  \class vpXmlConfigParserKeyPoint
  \ingroup group_vision_keypoints

  \warning This class is only available if libxml2 is installed and detected
  by ViSP. Installation instructions are provided here
  https://visp.inria.fr/3rd_xml2.

*/
class VISP_EXPORT vpXmlConfigParserKeyPoint : public vpXmlParser
{

public:
  /*! Predefined xml node identifier. */
  typedef enum {
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
  } vpNodeIdentifier;

  /*! Enumerator for the different filtering matching method. */
  typedef enum {
    constantFactorDistanceThreshold, /*!< Keep all the points below a constant
                                        factor threshold. */
    stdDistanceThreshold,            /*!< Keep all the points below a minimal distance +
                                        the standard deviation. */
    ratioDistanceThreshold,          /*!< Keep all the points enough discriminated. */
    stdAndRatioDistanceThreshold,    /*!< Keep all the points which fall with the
                                        two conditions. */
    noFilterMatching                 /*!< No filtering. */
  } vpMatchingMethodEnum;

private:
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

public:
  vpXmlConfigParserKeyPoint();
  //! Default destructor.
  virtual ~vpXmlConfigParserKeyPoint(){};

  /*!
    Get the detector name.

    \return The detector name.
  */
  inline std::string getDetectorName() const { return m_detectorName; }

  /*!
    Get the extractor name.

    \return The extractor name.
  */
  inline std::string getExtractorName() const { return m_extractorName; }
  /*!
    Get the matcher name.

    \return The detector name.
  */
  inline std::string getMatcherName() const { return m_matcherName; }
  /*!
    Get the factor value.

    \return The factor value for the filtering method:
    constantFactorDistanceThreshold.
  */
  inline double getMatchingFactorThreshold() const { return m_matchingFactorThreshold; }

  /*!
    Get the filtering method.

    \return The filtering method.
  */
  inline vpMatchingMethodEnum getMatchingMethod() const { return m_matchingMethod; }

  /*!
    Get the ratio value.

    \return The factor value for the filtering method: ratioDistanceThreshold.
  */
  inline double getMatchingRatioThreshold() const { return m_matchingRatioThreshold; }

  /*!
    Get the maximum number of iterations for the Ransac method.

    \return The maximum number of iterations for the Ransac method.
  */
  inline int getNbRansacIterations() const { return m_nbRansacIterations; }

  /*!
    Get the minimum number of inliers for the Ransac method.

    \return The minimum number of inliers for the Ransac method.
  */
  inline int getNbRansacMinInlierCount() const { return m_nbRansacMinInlierCount; }

  /*!
    Get the percentage value of inliers for the Ransac method.

    \return The percentage value of inliers for the Ransac method.
  */
  inline double getRansacConsensusPercentage() const { return m_ransacConsensusPercentage; }

  /*!
    Get the maximum reprojection error for a candidate inlier for the Ransac
    method.

    \return The maximum reprojection error for the Ransac method.
  */
  inline double getRansacReprojectionError() const { return m_ransacReprojectionError; }

  /*!
    Get the maximum error for a candidate inlier for the Ransac method.

    \return The maximum error for the Ransac method.
  */
  inline double getRansacThreshold() const { return m_ransacThreshold; }

  /*!
    Get the flag state to choose between a percentage of inliers or a fixed
    number.

    \return True to use a percentage value for inliers, false otherwise.
  */
  inline bool getUseRansacConsensusPercentage() const { return m_useRansacConsensusPercentage; }

  /*!
    Get the flag state to choose between OpenCV Ransac pose estimation or ViSP
    Ransac VVS pose estimation.

    \return True to use ViSP method, false otherwise.
  */
  inline bool getUseRansacVVSPoseEstimation() const { return m_useRansacVVS; }

  /*!
    Parse an XML file to get configuration value for vpKeyPoint class.

    \param filename : filename of the XML file to parse
  */
  void parse(const std::string &filename);

private:
  void init();
  void read_detector(xmlDocPtr doc, xmlNodePtr node);
  void read_extractor(xmlDocPtr doc, xmlNodePtr node);
  void read_matcher(xmlDocPtr doc, xmlNodePtr node);
  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_ransac(xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass(xmlNodePtr){};
};
#endif // VISP_HAVE_XML2
#endif
