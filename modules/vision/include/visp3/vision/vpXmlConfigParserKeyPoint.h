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
  Class vpXmlConfigParserKeyPoint allows to load configuration defined  in a
  XML file for vpKeyPoint class.

*/

#ifndef _vpXmlConfigParserKeyPoint_h_
#define _vpXmlConfigParserKeyPoint_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_PUGIXML

#include <string>

/*!
  \class vpXmlConfigParserKeyPoint
  \ingroup group_vision_keypoints

  \warning This class is only available if pugixml is successfully built.

*/
class VISP_EXPORT vpXmlConfigParserKeyPoint
{
public:
  /*! Enumerator for the different filtering matching method. */
  enum vpMatchingMethodEnum {
    constantFactorDistanceThreshold, /*!< Keep all the points below a constant
                                        factor threshold. */
    stdDistanceThreshold,            /*!< Keep all the points below a minimal distance +
                                        the standard deviation. */
    ratioDistanceThreshold,          /*!< Keep all the points enough discriminated. */
    stdAndRatioDistanceThreshold,    /*!< Keep all the points which fall with the
                                        two conditions. */
    noFilterMatching                 /*!< No filtering. */
  };

  vpXmlConfigParserKeyPoint();
  ~vpXmlConfigParserKeyPoint();

  std::string getDetectorName() const;
  std::string getExtractorName() const;
  std::string getMatcherName() const;

  double getMatchingFactorThreshold() const;
  vpMatchingMethodEnum getMatchingMethod() const;
  double getMatchingRatioThreshold() const;

  int getNbRansacIterations() const;
  int getNbRansacMinInlierCount() const;
  double getRansacConsensusPercentage() const;
  double getRansacReprojectionError() const;
  double getRansacThreshold() const;
  bool getUseRansacConsensusPercentage() const;
  bool getUseRansacVVSPoseEstimation() const;

  void parse(const std::string &filename);

private:
  vpXmlConfigParserKeyPoint(const vpXmlConfigParserKeyPoint &);            // noncopyable
  vpXmlConfigParserKeyPoint &operator=(const vpXmlConfigParserKeyPoint &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
#endif //VISP_HAVE_PUGIXML
#endif
