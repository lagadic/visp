/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
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
  Class vpXmlConfigParserKeyPoint allows to load configuration defined  in a XML file for vpKeyPoint class.

*/

#ifndef __vpXmlConfigParserKeyPoint_h__
#define __vpXmlConfigParserKeyPoint_h__

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <iostream>
#include <stdlib.h>

#include <libxml/xmlmemory.h>      // Functions of libxml.

#include <visp/vpXmlParser.h>


class VISP_EXPORT vpXmlConfigParserKeyPoint: public vpXmlParser
{

public:

  typedef enum
  {
    conf,
    detector,
    extractor,
    matcher,
    name,
    matching_method,
    constant_factor_distance_threshold,
    std_distance_threshold,
    ratio_distance_threshold,
    std_and_ratio_distance_threshold,
    no_filter_matching,
    matching_factor_threshold,
    matching_ratio_threshold,
    ransac,
    use_ransac_vvs,
    use_ransac_consensus_percentage,
    nb_ransac_iterations,
    ransac_reprojection_error,
    nb_ransac_min_inlier_count,
    ransac_threshold,
    ransac_consensus_percentage
  } dataToParse;

  typedef enum
  {
    constantFactorDistanceThreshold,
    stdDistanceThreshold,
    ratioDistanceThreshold,
    stdAndRatioDistanceThreshold,
    noFilterMatching
  } matchingMethodEnum;

private :

  std::string m_detectorName;
  std::string m_extractorName;
  std::string m_matcherName;
  matchingMethodEnum m_matchingMethod;
  double m_matchingFactorThreshold;
  double m_matchingRatioThreshold;

  bool m_useRansacVVS;
  bool m_useRansacConsensusPercentage;
  int m_nbRansacIterations;
  double m_ransacReprojectionError;
  int m_nbRansacMinInlierCount;
  double m_ransacThreshold;
  double m_ransacConsensusPercentage;


public:

  vpXmlConfigParserKeyPoint();

  // get/set functions
  inline std::string getDetectorName() const {
    return m_detectorName;
  }

  inline std::string getExtractorName() const {
    return m_extractorName;
  }

  inline std::string getMatcherName() const {
    return m_matcherName;
  }

  inline double getMatchingFactorThreshold() const {
    return m_matchingFactorThreshold;
  }

  inline matchingMethodEnum getMatchingMethod() const {
    return m_matchingMethod;
  }

  inline double getMatchingRatioThreshold() const {
    return m_matchingRatioThreshold;
  }

  inline int getNbRansacIterations() const {
    return m_nbRansacIterations;
  }

  inline int getNbRansacMinInlierCount() const {
    return m_nbRansacMinInlierCount;
  }

  inline double getRansacConsensusPercentage() const {
    return m_ransacConsensusPercentage;
  }

  inline double getRansacReprojectionError() const {
    return m_ransacReprojectionError;
  }

  inline double getRansacThreshold() const {
    return m_ransacThreshold;
  }

  inline bool getUseRansacConsensusPercentage() const {
    return m_useRansacConsensusPercentage;
  }

  inline bool getUseRansacVVSPoseEstimation() const {
    return m_useRansacVVS;
  }

  void parse(const std::string &filename);


private:

  void init();
  void read_detector(xmlDocPtr doc, xmlNodePtr node);
  void read_extractor(xmlDocPtr doc, xmlNodePtr node);
  void read_matcher(xmlDocPtr doc, xmlNodePtr node);
  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_ransac(xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass(xmlNodePtr ){};
  
};
#endif //VISP_HAVE_XML2
#endif
