/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Load XML Parameter for Model Based Tracker.
 *
 *****************************************************************************/

/*!
 * \file vpMbtXmlGenericParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
*/

#ifndef __vpMbtXmlGenericParser_h_
#define __vpMbtXmlGenericParser_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h>

#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/me/vpMe.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>

/*!
  \class vpMbtXmlGenericParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
  \ingroup group_mbt_xml_parser

  Data parser for the model-based tracker.

 */
class VISP_EXPORT vpMbtXmlGenericParser: public vpXmlParser
{
public:
  enum vpParserType {
    EDGE_PARSER         = 1 << 0,    /*!< Parser for model-based tracking using moving edges features. */
    KLT_PARSER          = 1 << 1,    /*!< Parser for model-based tracking using KLT features. */
    DEPTH_NORMAL_PARSER = 1 << 2,    /*!< Parser for model-based tracking using depth normal features. */
    DEPTH_DENSE_PARSER  = 1 << 3     /*!< Parser for model-based tracking using depth dense features. */
  };

protected:
  //! Parser type
  vpParserType m_parserType;
  //! Camera parameters.
  vpCameraParameters m_cam;
  //! Angle to determine if a face appeared
  double m_angleAppear;
  //! Angle to determine if a face disappeared
  double m_angleDisappear;
  //! Is near clipping distance specified?
  bool m_hasNearClipping;
  //! Near clipping distance
  double m_nearClipping;
  //! Is far clipping distance specified?
  bool m_hasFarClipping;
  //! Near clipping distance
  double m_farClipping;
  //! Fov Clipping
  bool m_fovClipping;
  //LOD
  //! If true, the LOD is enabled, otherwise it is not
  bool m_useLod;
  //! Minimum line length to track a segment when LOD is enabled
  double m_minLineLengthThreshold;
  //! Minimum polygon area to track a face when LOD is enabled
  double m_minPolygonAreaThreshold;
  //Edge
  //! Moving edges parameters.
  vpMe m_ecm;
  //KLT
  //! Border of the mask used on Klt points
  unsigned int m_kltMaskBorder;
  //! Maximum of Klt features
  unsigned int m_kltMaxFeatures;
  //! Windows size
  unsigned int m_kltWinSize;
  //! Quality of the Klt points
  double m_kltQualityValue;
  //! Minimum distance between klt points
  double m_kltMinDist;
  //! Harris free parameters
  double m_kltHarrisParam;
  //! Block size
  unsigned int m_kltBlockSize;
  //! Number of pyramid levels
  unsigned int m_kltPyramidLevels;
  //Depth normal
  //! Feature estimation method
  vpMbtFaceDepthNormal::vpFeatureEstimationType m_depthNormalFeatureEstimationMethod;
  //! PCL plane estimation method
  int m_depthNormalPclPlaneEstimationMethod;
  //! PCL RANSAC maximum number of iterations
  int m_depthNormalPclPlaneEstimationRansacMaxIter;
  //! PCL RANSAC threshold
  double m_depthNormalPclPlaneEstimationRansacThreshold;
  //! Sampling step in X
  unsigned int m_depthNormalSamplingStepX;
  //! Sampling step in Y
  unsigned int m_depthNormalSamplingStepY;
  //Depth dense
  //! Sampling step in X
  unsigned int m_depthDenseSamplingStepX;
  //! Sampling step in Y
  unsigned int m_depthDenseSamplingStepY;

  enum vpDataToParseMb {
    //<conf>
    conf,
    //<face>
    face,
    angle_appear,
    angle_disappear,
    near_clipping,
    far_clipping,
    fov_clipping,
    //<camera>
    camera,
    height,
    width,
    u0,
    v0,
    px,
    py,
    lod,
    use_lod,
    min_line_length_threshold,
    min_polygon_area_threshold,
    //<ecm>
    ecm,
    mask,
    size,
    nb_mask,
    range,
    tracking,
    contrast,
    edge_threshold,
    mu1,
    mu2,
    sample,
    step,
    //<klt>
    klt,
    mask_border,
    max_features,
    window_size,
    quality,
    min_distance,
    harris,
    size_block,
    pyramid_lvl,
    //<depth_normal>
    depth_normal,
    feature_estimation_method,
    PCL_plane_estimation,
    PCL_plane_estimation_method,
    PCL_plane_estimation_ransac_max_iter,
    PCL_plane_estimation_ransac_threshold,
    depth_sampling_step,
    depth_sampling_step_X,
    depth_sampling_step_Y,
    //<depth_dense>
    depth_dense,
    depth_dense_sampling_step,
    depth_dense_sampling_step_X,
    depth_dense_sampling_step_Y
  };

public:
  /** @name Public Member Functions Inherited from vpMbtXmlGenericParser */
  //@{
  vpMbtXmlGenericParser(const vpParserType &type=EDGE_PARSER);
  virtual ~vpMbtXmlGenericParser();

  /*!
    Get the angle to determine if a face appeared.
  */
  inline double getAngleAppear() const {
    return m_angleAppear;
  }
  
  /*!
    Get the angle to determine if a face disappeared.
  */
  inline double getAngleDisappear() const {
    return m_angleDisappear;
  }
  
  void getCameraParameters(vpCameraParameters& _cam) const {
    _cam = m_cam;
  }

  /*!
    Get moving edge parameters.
  */
  void getEdgeMe(vpMe& _ecm) const {
    _ecm = m_ecm;
  }

  /*!
    Get depth dense sampling step in X.
  */
  inline unsigned int getDepthDenseSamplingStepX() const {
    return m_depthDenseSamplingStepX;
  }

  /*!
    Get depth dense sampling step in Y.
  */
  inline unsigned int getDepthDenseSamplingStepY() const {
    return m_depthDenseSamplingStepY;
  }

  /*!
    Get depth normal feature estimation method.
  */
  vpMbtFaceDepthNormal::vpFeatureEstimationType getDepthNormalFeatureEstimationMethod() const {
    return m_depthNormalFeatureEstimationMethod;
  }

  /*!
    Get depth normal PCL plane estimation method.
  */
  inline int getDepthNormalPclPlaneEstimationMethod() const {
    return m_depthNormalPclPlaneEstimationMethod;
  }

  /*!
    Get depth normal PCL maximum number of iterations.
  */
  inline int getDepthNormalPclPlaneEstimationRansacMaxIter() const {
    return m_depthNormalPclPlaneEstimationRansacMaxIter;
  }

  /*!
    Get depth normal PCL RANSAC threshold.
  */
  inline double getDepthNormalPclPlaneEstimationRansacThreshold() const {
    return m_depthNormalPclPlaneEstimationRansacThreshold;
  }

  /*!
    Get depth normal sampling step in X.
  */
  inline unsigned int getDepthNormalSamplingStepX() const {
    return m_depthNormalSamplingStepX;
  }

  /*!
    Get depth normal sampling step in Y.
  */
  inline unsigned int getDepthNormalSamplingStepY() const {
    return m_depthNormalSamplingStepY;
  }
  
  /*!
    Get the far clipping distance.
  */
  inline double getFarClippingDistance() const {
    return m_farClipping;
  }
  
  /*!
    Get if FOV clipping should be used or not.
  */
  inline bool getFovClipping() const {
    return m_fovClipping;
  }

  /*!
    Get the size of a block.
  */
  inline unsigned int getKltBlockSize() const {
    return m_kltBlockSize;
  }

  /*!
    Get the Harris free parameter.
  */
  inline double getKltHarrisParam() const {
    return m_kltHarrisParam;
  }

  /*!
    Get the Border of the mask.
  */
  inline unsigned int getKltMaskBorder() const {
    return m_kltMaskBorder;
  }

  /*!
    Get the maximum number of features for the KLT.
  */
  inline unsigned int getKltMaxFeatures() const {
    return m_kltMaxFeatures;
  }

  /*!
    Get the minimum distance between KLT points.
  */
  inline double getKltMinDistance() const {
    return m_kltMinDist;
  }

  /*!
    Get the number of pyramid levels
  */
  inline unsigned int getKltPyramidLevels() const {
    return m_kltPyramidLevels;
  }

  /*!
    Get the quality of the KLT.
  */
  inline double getKltQuality() const {
    return m_kltQualityValue;
  }

  /*!
    Get the size of the window used in the KLT tracker.
  */
  inline unsigned int getKltWindowSize() const {
    return m_kltWinSize;
  }

  /*!
    Get the state of LOD setting.
  */
  inline bool getLodState() const {
    return m_useLod;
  }

  /*!
    Get the minimum line length to track a segment when LOD is enabled.
  */
  inline double getLodMinLineLengthThreshold() const {
    return m_minLineLengthThreshold;
  }

  /*!
    Get the minimum polygon area to track a face when LOD is enabled.
  */
  inline double getLodMinPolygonAreaThreshold() const {
    return m_minPolygonAreaThreshold;
  }

  /*!
    Get the near clipping distance.
  */
  inline double getNearClippingDistance() const {
    return m_nearClipping;
  }

  /*!
    Has Far clipping been specified?

    \return True if yes, False otherwise.
  */
  inline bool hasFarClippingDistance() const {
    return m_hasFarClipping;
  }

  /*!
    Has Near clipping been specified?

    \return True if yes, False otherwise.
  */
  inline bool hasNearClippingDistance() const {
    return m_hasNearClipping;
  }
  
  void parse(const std::string &filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);

  
  /*!
    Set the angle to determine if a face appeared.

    \param aappear : New angleAppear
  */
  inline void setAngleAppear(const double &aappear) {
    m_angleAppear = aappear;
  }
  
  /*!
    Set the angle to determine if a face disappeared.

    \param adisappear : New angleDisappear
  */
  inline void setAngleDisappear(const double &adisappear) {
    m_angleDisappear = adisappear;
  }
  
  /*!
    Set camera parameters.

    \param _cam : New camera parameters
  */
  inline void setCameraParameters(const vpCameraParameters &_cam) {
    m_cam = _cam;
  }

  /*!
    Set depth dense sampling step in X.

    \param stepX : New sampling step
  */
  inline void setDepthDenseSamplingStepX(const unsigned int stepX) {
    m_depthDenseSamplingStepX = stepX;
  }

  /*!
    Set depth dense sampling step in Y.

    \param stepY : New sampling step
  */
  inline void setDepthDenseSamplingStepY(const unsigned int stepY) {
    m_depthDenseSamplingStepY = stepY;
  }

  /*!
    Set depth normal feature estimation method.

    \param method : New feature estimation method
  */
  inline void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method) {
    m_depthNormalFeatureEstimationMethod = method;
  }

  /*!
    Set depth normal PCL plane estimation method.

    \param method : New PCL plane estimation method
  */
  inline void setDepthNormalPclPlaneEstimationMethod(const int method) {
    m_depthNormalPclPlaneEstimationMethod = method;
  }

  /*!
    Set depth normal PCL RANSAC maximum number of iterations.

    \param maxIter : New maximum number of iterations
  */
  inline void setDepthNormalPclPlaneEstimationRansacMaxIter(const int maxIter) {
    m_depthNormalPclPlaneEstimationRansacMaxIter = maxIter;
  }

  /*!
    Set depth normal PCL RANSAC threshold.

    \param threshold : New RANSAC threshold
  */
  inline void setDepthNormalPclPlaneEstimationRansacThreshold(const double threshold) {
    m_depthNormalPclPlaneEstimationRansacThreshold = threshold;
  }

  /*!
    Set depth normal sampling step in X.

    \param stepX : New sampling step
  */
  inline void setDepthNormalSamplingStepX(const unsigned int stepX) {
    m_depthNormalSamplingStepX = stepX;
  }

  /*!
    Set depth normal sampling step in Y.

    \param stepY : New sampling step
  */
  inline void setDepthNormalSamplingStepY(const unsigned int stepY) {
    m_depthNormalSamplingStepY = stepY;
  }

  /*!
    Set moving edge parameters.

    \param _ecm : New moving edge parameters
  */
  inline void setEdgeMe(const vpMe &_ecm) {
    m_ecm = _ecm;
  }
  
  /*!
    Set the far clipping distance.

    \param fclip : New farClipping
  */
  inline void setFarClippingDistance(const double &fclip) {
    m_farClipping = fclip;
  }

  /*!
    Set the size of a block.

    \param bs : New blockSize
  */
  inline void setKltBlockSize(const unsigned int &bs) {
    m_kltBlockSize = bs;
  }

  /*!
    Set the Harris free parameter.

    \param hp : New harrisParam
  */
  inline void setKltHarrisParam(const double &hp) {
    m_kltHarrisParam = hp;
  }

  /*!
    Set the Border of the mask.

    \param mb = new maskBorder
  */
  inline void setKltMaskBorder(const unsigned int &mb) {
    m_kltMaskBorder = mb;
  }

  /*!
    Set the maximum number of features for the KLT.

    \param mF : New maxFeatures
  */
  inline void setKltMaxFeatures(const unsigned int &mF) {
    m_kltMaxFeatures = mF;
  }

  /*!
    Set the minimum distance between KLT points.

    \param mD : New minDist
  */
  inline void setKltMinDistance(const double &mD) {
    m_kltMinDist = mD;
  }

  /*!
    Set the number of pyramid levels

    \param pL : New pyramidLevels
  */
  inline void setKltPyramidLevels(const unsigned int &pL) {
    m_kltPyramidLevels = pL;
  }

  /*!
    Set the quality of the KLT.

    \param q : New quality
  */
  inline void setKltQuality(const double &q) {
    m_kltQualityValue = q;
  }

  /*!
    Set the size of the window used in the KLT tracker.

    \param w : New winSize
  */
  inline void setKltWindowSize(const unsigned int &w) {
    m_kltWinSize = w;
  }

  /*!
    Set the near clipping distance.

    \param nclip : New nearClipping
  */
  inline void setNearClippingDistance(const double &nclip) {
    m_nearClipping = nclip;
  }

  void writeMainClass(xmlNodePtr node);
  //@}

protected:
  void init();

  void read_camera(xmlDocPtr doc, xmlNodePtr node);
  void read_face(xmlDocPtr doc, xmlNodePtr node);
  void read_lod(xmlDocPtr doc, xmlNodePtr node);

  //Edge
  void read_ecm(xmlDocPtr doc, xmlNodePtr node);
  void read_ecm_sample(xmlDocPtr doc, xmlNodePtr node);
  void read_sample_deprecated(xmlDocPtr doc, xmlNodePtr node);
  void read_ecm_mask(xmlDocPtr doc, xmlNodePtr node);
  void read_ecm_range(xmlDocPtr doc, xmlNodePtr node);
  void read_ecm_contrast(xmlDocPtr doc, xmlNodePtr node);

  //KLT
  void read_klt(xmlDocPtr doc, xmlNodePtr node);

  //Depth normal
  void read_depth_normal(xmlDocPtr doc, xmlNodePtr node);
  void read_depth_normal_PCL(xmlDocPtr doc, xmlNodePtr node);
  void read_depth_normal_sampling_step(xmlDocPtr doc, xmlNodePtr node);

  //Depth dense
  void read_depth_dense(xmlDocPtr doc, xmlNodePtr node);
  void read_depth_dense_sampling_step(xmlDocPtr doc, xmlNodePtr node);
};

#endif

#endif
