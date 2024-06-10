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
  Class vpXmlConfigParserKeyPoint allows to load configuration defined  in a
  XML file for vpKeyPoint class.

*/

#ifndef _vpXmlConfigParserKeyPoint_h_
#define _vpXmlConfigParserKeyPoint_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML)
#include <string>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpXmlConfigParserKeyPoint
 * \ingroup group_vision_keypoints
 *
 * Class vpXmlConfigParserKeyPoint allows to load configuration defined  in a
 * XML file for vpKeyPoint class.
 *
 * \warning This class is only available if pugixml is successfully built.
*/
class VISP_EXPORT vpXmlConfigParserKeyPoint
{
public:
  /*! Enumerator for the different filtering matching method. */
  enum vpMatchingMethodEnum
  {
    constantFactorDistanceThreshold, /*!< Keep all the points below a constant
                                        factor threshold. */
    stdDistanceThreshold,            /*!< Keep all the points below a minimal distance +
                                        the standard deviation. */
    ratioDistanceThreshold,          /*!< Keep all the points enough discriminated. */
    stdAndRatioDistanceThreshold,    /*!< Keep all the points which fall with the
                                        two conditions. */
    noFilterMatching                 /*!< No filtering. */
  };

  /*!
   * Default constructor.
   */
  vpXmlConfigParserKeyPoint();

  /*!
   * Default destructor.
   */
  ~vpXmlConfigParserKeyPoint();

  /*!
   * Get the detector name.
   *
   * \return The detector name.
   */
  std::string getDetectorName() const;

  /*!
   * Get the extractor name.
   *
   * \return The extractor name.
   */
  std::string getExtractorName() const;

  /*!
   * Get the matcher name.
   *
   * \return The detector name.
   */
  std::string getMatcherName() const;

  /*!
   * Get the factor value.
   *
   * \return The factor value for the filtering method: constantFactorDistanceThreshold.
   */
  double getMatchingFactorThreshold() const;

  /*!
   * Get the filtering method.
   *
   * \return The filtering method.
   */
  vpMatchingMethodEnum getMatchingMethod() const;

  /*!
   * Get the ratio value.
   *
   * \return The factor value for the filtering method: ratioDistanceThreshold.
   */
  double getMatchingRatioThreshold() const;

  /*!
   * Get the maximum number of iterations for the Ransac method.
   *
   * \return The maximum number of iterations for the Ransac method.
   */
  int getNbRansacIterations() const;

  /*!
   * Get the minimum number of inliers for the Ransac method.
   *
   * \return The minimum number of inliers for the Ransac method.
   */
  int getNbRansacMinInlierCount() const;

  /*!
   * Get the percentage value of inliers for the Ransac method.
   *
   * \return The percentage value of inliers for the Ransac method.
   */
  double getRansacConsensusPercentage() const;

  /*!
   * Get the maximum reprojection error for a candidate inlier for the Ransac
   * method.
   *
   * \return The maximum reprojection error for the Ransac method.
   */
  double getRansacReprojectionError() const;

  /*!
   * Get the maximum error for a candidate inlier for the Ransac method.
   *
   * \return The maximum error for the Ransac method.
   */
  double getRansacThreshold() const;

  /*!
   * Get the flag state to choose between a percentage of inliers or a fixed
   * number.
   *
   * \return True to use a percentage value for inliers, false otherwise.
   */
  bool getUseRansacConsensusPercentage() const;

  /*!
   * Get the flag state to choose between OpenCV Ransac pose estimation or ViSP
   * Ransac VVS pose estimation.
   *
   * \return True to use ViSP method, false otherwise.
   */
  bool getUseRansacVVSPoseEstimation() const;

  /*!
   * Parse an XML file to load configuration for vpKeyPoint class.
   * \param filename : filename of the XML file to parse.
   */
  void parse(const std::string &filename);

private:
  /*!
   * Non copyable constructor.
   */
  vpXmlConfigParserKeyPoint(const vpXmlConfigParserKeyPoint &);

  /*!
   * Non copyable operator.
   */
  vpXmlConfigParserKeyPoint &operator=(const vpXmlConfigParserKeyPoint &);

  //! PIMPL idiom
  class Impl;
  //! Pointer to implementation
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
