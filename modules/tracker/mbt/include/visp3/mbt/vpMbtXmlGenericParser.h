/****************************************************************************
 *
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
 * Load XML Parameter for Model Based Tracker.
 *
*****************************************************************************/

/*!
 * \file vpMbtXmlGenericParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig
 * object.
 */

#ifndef _vpMbtXmlGenericParser_h_
#define _vpMbtXmlGenericParser_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>
#include <visp3/me/vpMe.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpMbtXmlGenericParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig
  object.
  \ingroup group_mbt_xml_parser

  Data parser for the model-based tracker.

*/
class VISP_EXPORT vpMbtXmlGenericParser
{
public:
  enum vpParserType
  {
    EDGE_PARSER = 1 << 0,         /*!< Parser for model-based tracking using moving
                                     edges features. */
    KLT_PARSER = 1 << 1,          /*!< Parser for model-based tracking using KLT features. */
    DEPTH_NORMAL_PARSER = 1 << 2, /*!< Parser for model-based tracking using
                                     depth normal features. */
    DEPTH_DENSE_PARSER = 1 << 3,  /*!< Parser for model-based tracking using
                                     depth dense features. */
    PROJECTION_ERROR_PARSER = 0   /*!< Parser for projection error computation parameters. */
  };

public:
  vpMbtXmlGenericParser(int type = EDGE_PARSER);
  virtual ~vpMbtXmlGenericParser();

  double getAngleAppear() const;
  double getAngleDisappear() const;

  void getCameraParameters(vpCameraParameters &cam) const;

  void getEdgeMe(vpMe &ecm) const;

  unsigned int getDepthDenseSamplingStepX() const;
  unsigned int getDepthDenseSamplingStepY() const;

  vpMbtFaceDepthNormal::vpFeatureEstimationType getDepthNormalFeatureEstimationMethod() const;
  int getDepthNormalPclPlaneEstimationMethod() const;
  int getDepthNormalPclPlaneEstimationRansacMaxIter() const;
  double getDepthNormalPclPlaneEstimationRansacThreshold() const;
  unsigned int getDepthNormalSamplingStepX() const;
  unsigned int getDepthNormalSamplingStepY() const;

  double getFarClippingDistance() const;
  bool getFovClipping() const;

  unsigned int getKltBlockSize() const;
  double getKltHarrisParam() const;
  unsigned int getKltMaskBorder() const;
  unsigned int getKltMaxFeatures() const;
  double getKltMinDistance() const;
  unsigned int getKltPyramidLevels() const;
  double getKltQuality() const;
  unsigned int getKltWindowSize() const;

  bool getLodState() const;
  double getLodMinLineLengthThreshold() const;
  double getLodMinPolygonAreaThreshold() const;

  double getNearClippingDistance() const;

  void getProjectionErrorMe(vpMe &me) const;

  unsigned int getProjectionErrorKernelSize() const;

  bool hasFarClippingDistance() const;
  bool hasNearClippingDistance() const;

  void parse(const std::string &filename);

  void setAngleAppear(const double &aappear);
  void setAngleDisappear(const double &adisappear);

  void setCameraParameters(const vpCameraParameters &cam);

  void setDepthDenseSamplingStepX(unsigned int stepX);
  void setDepthDenseSamplingStepY(unsigned int stepY);

  void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method);
  void setDepthNormalPclPlaneEstimationMethod(int method);
  void setDepthNormalPclPlaneEstimationRansacMaxIter(int maxIter);
  void setDepthNormalPclPlaneEstimationRansacThreshold(double threshold);
  void setDepthNormalSamplingStepX(unsigned int stepX);
  void setDepthNormalSamplingStepY(unsigned int stepY);

  void setEdgeMe(const vpMe &ecm);

  void setFarClippingDistance(const double &fclip);

  void setKltBlockSize(const unsigned int &bs);
  void setKltHarrisParam(const double &hp);
  void setKltMaskBorder(const unsigned int &mb);
  void setKltMaxFeatures(const unsigned int &mF);
  void setKltMinDistance(const double &mD);
  void setKltPyramidLevels(const unsigned int &pL);
  void setKltQuality(const double &q);
  void setKltWindowSize(const unsigned int &w);

  void setNearClippingDistance(const double &nclip);

  void setProjectionErrorMe(const vpMe &me);
  void setProjectionErrorKernelSize(const unsigned int &size);

  void setVerbose(bool verbose);

private:
  vpMbtXmlGenericParser(const vpMbtXmlGenericParser &);            // noncopyable
  vpMbtXmlGenericParser &operator=(const vpMbtXmlGenericParser &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
