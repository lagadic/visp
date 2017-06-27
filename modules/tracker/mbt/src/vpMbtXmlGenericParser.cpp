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
#include <visp3/core/vpConfig.h>


#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h>

#include <visp3/mbt/vpMbtXmlGenericParser.h>


vpMbtXmlGenericParser::vpMbtXmlGenericParser(const vpParserType &type) :
    m_parserType(type),
    //<camera>
    m_cam(),
    //<face>
    m_angleAppear(70), m_angleDisappear(80),
    m_hasNearClipping(false), m_nearClipping(false), m_hasFarClipping(false), m_farClipping(false), m_fovClipping(false),
    //<lod>
    m_useLod(false), m_minLineLengthThreshold(50.0), m_minPolygonAreaThreshold(2500.0),
    //<ecm>
    m_ecm(),
    //<klt>
    m_kltMaskBorder(0), m_kltMaxFeatures(0), m_kltWinSize(0), m_kltQualityValue(0.), m_kltMinDist(0.),
    m_kltHarrisParam(0.), m_kltBlockSize(0), m_kltPyramidLevels(0),
    //<depth_normal>
    m_depthNormalFeatureEstimationMethod(vpMbtFaceDepthNormal::ROBUST_FEATURE_ESTIMATION),
    m_depthNormalPclPlaneEstimationMethod(2), m_depthNormalPclPlaneEstimationRansacMaxIter(200),
    m_depthNormalPclPlaneEstimationRansacThreshold(0.001), m_depthNormalSamplingStepX(2), m_depthNormalSamplingStepY(2),
    //<depth_dense>
    m_depthDenseSamplingStepX(2), m_depthDenseSamplingStepY(2)
{
  init();
}

vpMbtXmlGenericParser::~vpMbtXmlGenericParser()
{
}

/*!
  Initialise internal variables (including the map).
*/
void vpMbtXmlGenericParser::init() {
  setMainTag("conf");

  //<conf>
  nodeMap["conf"] = conf;
  //<face>
  nodeMap["face"] = face;
  nodeMap["angle_appear"] = angle_appear;
  nodeMap["angle_disappear"] = angle_disappear;
  nodeMap["near_clipping"] = near_clipping;
  nodeMap["far_clipping"] = far_clipping;
  nodeMap["fov_clipping"] = fov_clipping;
  //<camera>
  nodeMap["camera"] = camera;
  nodeMap["height"] = height;
  nodeMap["width"] = width;
  nodeMap["u0"] = u0;
  nodeMap["v0"] = v0;
  nodeMap["px"] = px;
  nodeMap["py"] = py;
  //<lod>
  nodeMap["lod"] = lod;
  nodeMap["use_lod"] = use_lod;
  nodeMap["min_line_length_threshold"] = min_line_length_threshold;
  nodeMap["min_polygon_area_threshold"] = min_polygon_area_threshold;
  //<ecm>
  nodeMap["ecm"] = ecm;
  nodeMap["mask"] = mask;
  nodeMap["size"] = size;
  nodeMap["nb_mask"] = nb_mask;
  nodeMap["range"] = range;
  nodeMap["tracking"] = tracking;
  nodeMap["contrast"] = contrast;
  nodeMap["edge_threshold"] = edge_threshold;
  nodeMap["mu1"] = mu1;
  nodeMap["mu2"] = mu2;
  nodeMap["sample"] = sample;
  nodeMap["step"] = step;
  //<klt>
  nodeMap["klt"] = klt;
  nodeMap["mask_border"] = mask_border;
  nodeMap["max_features"] = max_features;
  nodeMap["window_size"] = window_size;
  nodeMap["quality"] = quality;
  nodeMap["min_distance"] = min_distance;
  nodeMap["harris"] = harris;
  nodeMap["size_block"] = size_block;
  nodeMap["pyramid_lvl"] = pyramid_lvl;
  //<depth_normal>
  nodeMap["depth_normal"] = depth_normal;
  nodeMap["feature_estimation_method"] = feature_estimation_method;
  nodeMap["PCL_plane_estimation"] = PCL_plane_estimation;
  nodeMap["method"] = PCL_plane_estimation_method;
  nodeMap["ransac_max_iter"] = PCL_plane_estimation_ransac_max_iter;
  nodeMap["ransac_threshold"] = PCL_plane_estimation_ransac_threshold;
  nodeMap["sampling_step"] = depth_sampling_step;
  nodeMap["step_X"] = depth_sampling_step_X;
  nodeMap["step_Y"] = depth_sampling_step_Y;
  //<depth_dense>
  nodeMap["depth_dense"] = depth_dense;
  nodeMap["sampling_step"] = depth_dense_sampling_step;
  nodeMap["step_X"] = depth_dense_sampling_step_X;
  nodeMap["step_Y"] = depth_dense_sampling_step_Y;
}

/*!
  Parse the file in parameters.
  This method is deprecated, use parse() instead.
  
  \param filename : File to parse.
*/
void vpMbtXmlGenericParser::parse(const std::string &filename) {
  vpXmlParser::parse(filename);
}

/*!
  Write info to file.
  
  \warning Useless, so not yet implemented => Throw exception.
*/
void vpMbtXmlGenericParser::writeMainClass(xmlNodePtr /*node*/) {
  throw vpException(vpException::notImplementedError, "Not implemented." );
}

/*!
  Read the parameters of the class from the file given by its document pointer
  and by its root node.
  
  \param doc : Document to parse.
  \param node : Root node.
*/
void vpMbtXmlGenericParser::readMainClass(xmlDocPtr doc, xmlNodePtr node) {
  bool camera_node = false;
  bool face_node = false;
  bool ecm_node = false;
  bool klt_node = false;
  bool lod_node = false;
  bool depth_normal_node = false;
  bool depth_dense_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case camera:
            read_camera(doc, dataNode);
            camera_node = true;
            break;

          case face:
            read_face(doc, dataNode);
            face_node = true;
            break;

          case lod:
            read_lod(doc, dataNode);
            lod_node = true;
            break;

          case ecm:
            if (m_parserType & EDGE_PARSER) {
              read_ecm(doc, dataNode);
              ecm_node = true;
            }
            break;

          case sample:
            if (m_parserType & EDGE_PARSER)
              read_sample_deprecated(doc, dataNode);
            break;

          case klt:
            if (m_parserType & KLT_PARSER) {
              read_klt(doc, dataNode);
              klt_node = true;
            }
            break;

          case depth_normal:
            if (m_parserType & DEPTH_NORMAL_PARSER) {
              read_depth_normal(doc, dataNode);
              depth_normal_node = true;
            }
            break;

          case depth_dense:
            if (m_parserType & DEPTH_DENSE_PARSER) {
              read_depth_dense(doc, dataNode);
              depth_dense_node = true;
            }
            break;

          default:
            break;
        }
      }
    }
  }

  if (!camera_node) {
    std::cout << "camera : u0 : " << m_cam.get_u0() << " (default)" << std::endl;
    std::cout << "camera : v0 : " << m_cam.get_v0() << " (default)" << std::endl;
    std::cout << "camera : px : " << m_cam.get_px() << " (default)" << std::endl;
    std::cout << "camera : py : " << m_cam.get_py() << " (default)" << std::endl;
  }

  if (!face_node) {
    std::cout << "face : Angle Appear : "    << m_angleAppear << " (default)" << std::endl;
    std::cout << "face : Angle Disappear : " << m_angleDisappear << " (default)" << std::endl;
  }

  if (!lod_node) {
    std::cout << "lod : use lod : "                    << m_useLod << " (default)" << std::endl;
    std::cout << "lod : min line length threshold : "  << m_minLineLengthThreshold << " (default)" << std::endl;
    std::cout << "lod : min polygon area threshold : " << m_minPolygonAreaThreshold << " (default)" << std::endl;
  }

  if (!ecm_node && (m_parserType & EDGE_PARSER)) {
    std::cout << "ecm : mask : size : "          << m_ecm.getMaskSize() << " (default)" << std::endl;
    std::cout << "ecm : mask : nb_mask : "       << m_ecm.getMaskNumber() << " (default)" << std::endl;
    std::cout << "ecm : range : tracking : "     << m_ecm.getRange() << " (default)" << std::endl;
    std::cout << "ecm : contrast : threshold : " << m_ecm.getThreshold() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu1 : "       << m_ecm.getMu1() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu2 : "       << m_ecm.getMu2() << " (default)" << std::endl;
    std::cout << "ecm : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
  }

  if (!klt_node && (m_parserType & KLT_PARSER)) {
    std::cout << "klt : Mask Border : "      << m_kltMaskBorder << " (default)" << std::endl;
    std::cout << "klt : Max Features : "     << m_kltMaxFeatures << " (default)" << std::endl;
    std::cout << "klt : Windows Size : "     << m_kltWinSize << " (default)" << std::endl;
    std::cout << "klt : Quality : "          << m_kltQualityValue << " (default)" << std::endl;
    std::cout << "klt : Min Distance : "     << m_kltMinDist << " (default)" << std::endl;
    std::cout << "klt : Harris Parameter : " << m_kltHarrisParam << " (default)" << std::endl;
    std::cout << "klt : Block Size : "       << m_kltBlockSize << " (default)" << std::endl;
    std::cout << "klt : Pyramid Levels : "   << m_kltPyramidLevels << " (default)" << std::endl;
  }

  if (!depth_normal_node && (m_parserType & DEPTH_NORMAL_PARSER)) {
    std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod << " (default)" << std::endl;
    std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod << " (default)" << std::endl;
    std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter << " (default)" << std::endl;
    std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : " << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
    std::cout << "depth normal : sampling_step : step_X " << m_depthNormalSamplingStepX << " (default)" << std::endl;
    std::cout << "depth normal : sampling_step : step_Y " << m_depthNormalSamplingStepY << " (default)" << std::endl;
  }

  if (!depth_dense_node && (m_parserType & DEPTH_DENSE_PARSER)) {
    std::cout << "depth dense : sampling_step : step_X " << m_depthDenseSamplingStepX << " (default)" << std::endl;
    std::cout << "depth dense : sampling_step : step_Y " << m_depthDenseSamplingStepY << " (default)" << std::endl;
  }
}

/*!
  Read camera information.
  
  \throw vpException::fatalError if there was an unexpected number of data.
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void vpMbtXmlGenericParser::read_camera(xmlDocPtr doc, xmlNodePtr node) {
  bool u0_node = false;
  bool v0_node = false;
  bool px_node = false;
  bool py_node = false;
  
  // current data values.
  double d_u0 = m_cam.get_u0();
  double d_v0 = m_cam.get_v0();
  double d_px = m_cam.get_px();
  double d_py = m_cam.get_py();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case u0:
            d_u0 = xmlReadDoubleChild(doc, dataNode);
            u0_node = true;
            break;

          case v0:
            d_v0 = xmlReadDoubleChild(doc, dataNode);
            v0_node = true;
            break;

          case px:
            d_px = xmlReadDoubleChild(doc, dataNode);
            px_node = true;
            break;

          case py:
            d_py = xmlReadDoubleChild(doc, dataNode);
            py_node = true;
            break;

          default:
            break;
        }
      }
    }
  }
  
  m_cam.initPersProjWithoutDistortion(d_px, d_py, d_u0, d_v0);
  
  if (!u0_node)
    std::cout << "camera : u0 : " << m_cam.get_u0() << " (default)" << std::endl;
  else
    std::cout << "camera : u0 : " << m_cam.get_u0() << std::endl;
  
  if (!v0_node)
    std::cout << "camera : v0 : " << m_cam.get_v0() << " (default)" << std::endl;
  else
    std::cout << "camera : v0 : " << m_cam.get_v0() << std::endl;
  
  if (!px_node)
    std::cout << "camera : px : " << m_cam.get_px() << " (default)" << std::endl;
  else
    std::cout << "camera : px : " << m_cam.get_px() << std::endl;

  if (!py_node)
    std::cout << "camera : py : " << m_cam.get_py() << " (default)" << std::endl;
  else
    std::cout << "camera : py : " << m_cam.get_py() << std::endl;
}

/*!
  Read depth normal information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node information.
*/
void vpMbtXmlGenericParser::read_depth_normal(xmlDocPtr doc, xmlNodePtr node) {
  bool feature_estimation_method_node = false;
  bool PCL_plane_estimation_node = false;
  bool sampling_step_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case feature_estimation_method:
            m_depthNormalFeatureEstimationMethod = (vpMbtFaceDepthNormal::vpFeatureEstimationType) xmlReadIntChild(doc, dataNode);
            feature_estimation_method_node = true;
            break;

          case PCL_plane_estimation:
            read_depth_normal_PCL(doc, dataNode);
            PCL_plane_estimation_node = true;
            break;

          case depth_sampling_step:
            read_depth_normal_sampling_step(doc, dataNode);
            sampling_step_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!feature_estimation_method_node)
    std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod << " (default)" << std::endl;
  else
    std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod << std::endl;

  if (!PCL_plane_estimation_node) {
    std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod << " (default)" << std::endl;
    std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter << " (default)" << std::endl;
    std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : " << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
  }

  if (!sampling_step_node) {
    std::cout << "depth normal : sampling_step : step_X " << m_depthNormalSamplingStepX << " (default)" << std::endl;
    std::cout << "depth normal : sampling_step : step_Y " << m_depthNormalSamplingStepY << " (default)" << std::endl;
  }
}

/*!
  Read depth normal PCL properties.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node information.
*/
void vpMbtXmlGenericParser::read_depth_normal_PCL(xmlDocPtr doc, xmlNodePtr node) {
  bool PCL_plane_estimation_method_node = false;
  bool PCL_plane_estimation_ransac_max_iter_node = false;
  bool PCL_plane_estimation_ransac_threshold_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data= nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case PCL_plane_estimation_method:
            m_depthNormalPclPlaneEstimationMethod = xmlReadIntChild(doc, dataNode);
            PCL_plane_estimation_method_node = true;
            break;

          case PCL_plane_estimation_ransac_max_iter:
            m_depthNormalPclPlaneEstimationRansacMaxIter = xmlReadIntChild(doc, dataNode);
            PCL_plane_estimation_ransac_max_iter_node = true;
            break;

          case PCL_plane_estimation_ransac_threshold:
            m_depthNormalPclPlaneEstimationRansacThreshold = xmlReadDoubleChild(doc, dataNode);
            PCL_plane_estimation_ransac_threshold_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!PCL_plane_estimation_method_node)
    std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod << " (default)" << std::endl;
  else
    std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod << std::endl;

  if (!PCL_plane_estimation_ransac_max_iter_node)
    std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter << " (default)" << std::endl;
  else
    std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter << std::endl;

  if (!PCL_plane_estimation_ransac_threshold_node)
    std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : " << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
  else
    std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : " << m_depthNormalPclPlaneEstimationRansacThreshold << std::endl;
}

/*!
  Read depth normal sampling step.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node information.
*/
void vpMbtXmlGenericParser::read_depth_normal_sampling_step(xmlDocPtr doc, xmlNodePtr node) {
  bool sampling_step_X_node = false;
  bool sampling_step_Y_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data= nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case depth_sampling_step_X:
            m_depthNormalSamplingStepX = xmlReadUnsignedIntChild(doc, dataNode);
            sampling_step_X_node = true;
            break;

          case depth_sampling_step_Y:
            m_depthNormalSamplingStepY = xmlReadUnsignedIntChild(doc, dataNode);
            sampling_step_Y_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!sampling_step_X_node)
    std::cout << "depth normal : sampling_step : step_X : " << m_depthNormalSamplingStepX << " (default)" << std::endl;
  else
    std::cout << "depth normal : sampling_step : step_X : " << m_depthNormalSamplingStepX << std::endl;

  if (!sampling_step_Y_node)
    std::cout << "depth normal : sampling_step : step_Y : " << m_depthNormalSamplingStepY << " (default)" << std::endl;
  else
    std::cout << "depth normal : sampling_step : step_Y : " << m_depthNormalSamplingStepY << std::endl;
}

/*!
  Read depth dense information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the ecm information.
*/
void vpMbtXmlGenericParser::read_depth_dense(xmlDocPtr doc, xmlNodePtr node) {
  bool sampling_step_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case depth_dense_sampling_step:
            read_depth_dense_sampling_step(doc, dataNode);
            sampling_step_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!sampling_step_node) {
    std::cout << "depth dense : sampling_step : step_X " << m_depthDenseSamplingStepX << " (default)" << std::endl;
    std::cout << "depth dense : sampling_step : step_Y " << m_depthDenseSamplingStepY << " (default)" << std::endl;
  }
}

/*!
  Read depth dense sampling step.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the range information.
*/
void vpMbtXmlGenericParser::read_depth_dense_sampling_step(xmlDocPtr doc, xmlNodePtr node) {
  bool sampling_step_X_node = false;
  bool sampling_step_Y_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data= nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case depth_dense_sampling_step_X:
            m_depthDenseSamplingStepX = xmlReadUnsignedIntChild(doc, dataNode);
            sampling_step_X_node = true;
            break;

          case depth_dense_sampling_step_Y:
            m_depthDenseSamplingStepY = xmlReadUnsignedIntChild(doc, dataNode);
            sampling_step_Y_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!sampling_step_X_node)
    std::cout << "depth dense : sampling_step : step_X : " << m_depthDenseSamplingStepX << " (default)" << std::endl;
  else
    std::cout << "depth dense : sampling_step : step_X : " << m_depthDenseSamplingStepX << std::endl;

  if (!sampling_step_Y_node)
    std::cout << "depth dense : sampling_step : step_Y : " << m_depthDenseSamplingStepY << " (default)" << std::endl;
  else
    std::cout << "depth dense : sampling_step : step_Y : " << m_depthDenseSamplingStepY << std::endl;
}

/*!
  Read ecm information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the ecm information.
*/
void vpMbtXmlGenericParser::read_ecm(xmlDocPtr doc, xmlNodePtr node) {
  bool mask_node = false;
  bool range_node = false;
  bool contrast_node = false;
  bool sample_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case mask:
            read_ecm_mask(doc, dataNode);
            mask_node = true;
            break;

          case range:
            read_ecm_range(doc, dataNode);
            range_node = true;
            break;

          case contrast:
            read_ecm_contrast(doc, dataNode);
            contrast_node = true;
            break;

          case sample:
            read_ecm_sample(doc, dataNode);
            sample_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!mask_node) {
    std::cout << "ecm : mask : size : "     << m_ecm.getMaskSize() << " (default)" << std::endl;
    std::cout << "ecm : mask : nb_mask : "  << m_ecm.getMaskNumber() << " (default)" << std::endl;
  }

  if (!range_node) {
    std::cout << "ecm : range : tracking : " << m_ecm.getRange() << " (default)" << std::endl;
  }

  if (!contrast_node) {
    std::cout << "ecm : contrast : threshold " << m_ecm.getThreshold() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu1 "       << m_ecm.getMu1() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu2 "       << m_ecm.getMu2() << " (default)" << std::endl;
  }

  if (!sample_node) {
    std::cout << "ecm : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
  }
}

/*!
  Read the contrast information from the xml file.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the contrast information.
*/
void vpMbtXmlGenericParser::read_ecm_contrast(xmlDocPtr doc, xmlNodePtr node) {
  bool edge_threshold_node = false;
  bool mu1_node = false;
  bool mu2_node = false;

  // current data values.
  double d_edge_threshold = m_ecm.getThreshold();
  double d_mu1 = m_ecm.getMu1();
  double d_mu2 = m_ecm.getMu2();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data= nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case edge_threshold:
            d_edge_threshold = xmlReadDoubleChild(doc, dataNode);
            edge_threshold_node = true;
            break;

          case mu1:
            d_mu1 = xmlReadDoubleChild(doc, dataNode);
            mu1_node = true;
            break;

          case mu2:
            d_mu2 = xmlReadDoubleChild(doc, dataNode);
            mu2_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  m_ecm.setMu1(d_mu1);
  m_ecm.setMu2(d_mu2);
  m_ecm.setThreshold(d_edge_threshold);

  if (!edge_threshold_node)
    std::cout << "ecm : contrast : threshold " << m_ecm.getThreshold() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : threshold " << m_ecm.getThreshold() << std::endl;

  if( !mu1_node)
    std::cout << "ecm : contrast : mu1 " << m_ecm.getMu1() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : mu1 " << m_ecm.getMu1() << std::endl;

  if (!mu2_node)
    std::cout << "ecm : contrast : mu2 " << m_ecm.getMu2() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : mu2 " << m_ecm.getMu2() << std::endl;
}

/*!
  Read mask information for the vpMeSite.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the mask information.
*/
void vpMbtXmlGenericParser::read_ecm_mask(xmlDocPtr doc, xmlNodePtr node) {
  bool size_node = false;
  bool nb_mask_node = false;

  // current data values.
  unsigned int d_size = m_ecm.getMaskSize();
  unsigned int d_nb_mask = m_ecm.getMaskNumber();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case size:
            d_size = xmlReadUnsignedIntChild(doc, dataNode);
            size_node = true;
            break;

          case nb_mask:
            d_nb_mask = xmlReadUnsignedIntChild(doc, dataNode);
            nb_mask_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  m_ecm.setMaskSize(d_size);

  // Check to ensure that d_nb_mask > 0
  if (d_nb_mask == 0)
    throw(vpException(vpException::badValue, "Model-based tracker mask size parameter should be different from zero in xml file"));
  m_ecm.setMaskNumber(d_nb_mask);

  if (!size_node)
    std::cout << "ecm : mask : size : " << m_ecm.getMaskSize() << " (default)" << std::endl;
  else
    std::cout << "ecm : mask : size : " << m_ecm.getMaskSize() << std::endl;

  if (!nb_mask_node)
    std::cout << "ecm : mask : nb_mask : " << m_ecm.getMaskNumber() << " (default)" << std::endl;
  else
    std::cout << "ecm : mask : nb_mask : " << m_ecm.getMaskNumber() << std::endl;
}

/*!
  Read range information for the vpMeSite.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the range information.
*/
void vpMbtXmlGenericParser::read_ecm_range(xmlDocPtr doc, xmlNodePtr node) {
  bool tracking_node = false;

  // current data values.
  unsigned int m_range_tracking = m_ecm.getRange();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data= nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case tracking:
            m_range_tracking = xmlReadUnsignedIntChild(doc, dataNode);
            tracking_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  m_ecm.setRange(m_range_tracking);

  if (!tracking_node)
    std::cout << "ecm : range : tracking : " << m_ecm.getRange() << " (default)" << std::endl;
  else
    std::cout << "ecm : range : tracking : " << m_ecm.getRange() << std::endl;
}

/*!
  Read sample information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample information.
*/
void vpMbtXmlGenericParser::read_ecm_sample(xmlDocPtr doc, xmlNodePtr node) {
  bool step_node = false;

  // current data values.
  double d_stp = m_ecm.getSampleStep();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case step:
            d_stp = xmlReadIntChild(doc, dataNode);
            step_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  m_ecm.setSampleStep(d_stp);

  if (!step_node)
    std::cout << "ecm : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
  else
    std::cout << "ecm : sample : sample_step : " << m_ecm.getSampleStep() << std::endl;
}

/*!
  Read face information.
  
  \throw vpException::fatalError if there was an unexpected number of data.
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void vpMbtXmlGenericParser::read_face(xmlDocPtr doc, xmlNodePtr node) {
  bool angle_appear_node = false;
  bool angle_disappear_node = false;
  bool near_clipping_node = false;
  bool far_clipping_node = false;
  bool fov_clipping_node = false;
  m_hasNearClipping = false;
  m_hasFarClipping = false;
  
  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case angle_appear:
            m_angleAppear = xmlReadDoubleChild(doc, dataNode);
            angle_appear_node = true;
            break;

          case angle_disappear:
            m_angleDisappear = xmlReadDoubleChild(doc, dataNode);
            angle_disappear_node = true;
            break;

          case near_clipping:
            m_nearClipping = xmlReadDoubleChild(doc, dataNode);
            m_hasNearClipping = true;
            near_clipping_node = true;
            break;

          case far_clipping:
            m_farClipping = xmlReadDoubleChild(doc, dataNode);
            m_hasFarClipping = true;
            far_clipping_node = true;
            break;

          case fov_clipping:
            if (xmlReadIntChild(doc, dataNode))
              m_fovClipping = true;
            else
              m_fovClipping = false;
            fov_clipping_node = true;
            break;

          default:
            break;
        }
      }
    }
  }
  
  if (!angle_appear_node)
    std::cout << "face : Angle Appear : " << m_angleAppear << " (default)" << std::endl;
  else
    std::cout << "face : Angle Appear : " << m_angleAppear << std::endl;
  
  if (!angle_disappear_node)
    std::cout << "face : Angle Disappear : " << m_angleDisappear << " (default)" << std::endl;
  else
    std::cout << "face : Angle Disappear : " << m_angleDisappear << std::endl;
  
  if (near_clipping_node)
    std::cout << "face : Near Clipping : " << m_nearClipping << std::endl;
  
  if (far_clipping_node)
    std::cout << "face : Far Clipping : " << m_farClipping << std::endl;
  
  if (fov_clipping_node) {
    if(m_fovClipping)
      std::cout << "face : Fov Clipping : True" << std::endl;
    else
      std::cout << "face : Fov Clipping : False" << std::endl;
  }
}

/*!
  Read klt information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void vpMbtXmlGenericParser::read_klt(xmlDocPtr doc, xmlNodePtr node) {
  bool mask_border_node = false;
  bool max_features_node = false;
  bool window_size_node = false;
  bool quality_node = false;
  bool min_distance_node = false;
  bool harris_node = false;
  bool size_block_node = false;
  bool pyramid_lvl_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case mask_border:
            m_kltMaskBorder = xmlReadUnsignedIntChild(doc, dataNode);
            mask_border_node = true;
            break;

          case max_features:
            m_kltMaxFeatures = xmlReadUnsignedIntChild(doc, dataNode);
            max_features_node = true;
            break;

          case window_size:
            m_kltWinSize = xmlReadUnsignedIntChild(doc, dataNode);
            window_size_node = true;
            break;

          case quality:
            m_kltQualityValue = xmlReadDoubleChild(doc, dataNode);
            quality_node = true;
            break;

          case min_distance:
            m_kltMinDist = xmlReadDoubleChild(doc, dataNode);
            min_distance_node = true;
            break;

          case harris:
            m_kltHarrisParam = xmlReadDoubleChild(doc, dataNode);
            harris_node = true;
            break;

          case size_block:
            m_kltBlockSize = xmlReadUnsignedIntChild(doc, dataNode);
            size_block_node = true;
            break;

          case pyramid_lvl:
            m_kltPyramidLevels = xmlReadUnsignedIntChild(doc, dataNode);
            pyramid_lvl_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!mask_border_node)
    std::cout << "klt : Mask Border : " << m_kltMaskBorder << " (default)" << std::endl;
  else
    std::cout << "klt : Mask Border : " << m_kltMaskBorder << std::endl;

  if (!max_features_node)
    std::cout << "klt : Max Features : " << m_kltMaxFeatures << " (default)" << std::endl;
  else
    std::cout << "klt : Max Features : " << m_kltMaxFeatures << std::endl;

  if (!window_size_node)
    std::cout << "klt : Windows Size : " << m_kltWinSize << " (default)" << std::endl;
  else
    std::cout << "klt : Windows Size : " << m_kltWinSize << std::endl;

  if (!quality_node)
    std::cout << "klt : Quality : " << m_kltQualityValue << " (default)" << std::endl;
  else
    std::cout << "klt : Quality : " << m_kltQualityValue << std::endl;

  if (!min_distance_node)
    std::cout << "klt : Min Distance : " << m_kltMinDist << " (default)" << std::endl;
  else
    std::cout << "klt : Min Distance : " << m_kltMinDist << std::endl;

  if (!harris_node)
    std::cout << "klt : Harris Parameter : " << m_kltHarrisParam << " (default)" << std::endl;
  else
    std::cout << "klt : Harris Parameter : " << m_kltHarrisParam << std::endl;

  if (!size_block_node)
    std::cout << "klt : Block Size : " << m_kltBlockSize << " (default)" << std::endl;
  else
    std::cout << "klt : Block Size : " << m_kltBlockSize << std::endl;

  if (!pyramid_lvl_node)
    std::cout << "klt : Pyramid Levels : " << m_kltPyramidLevels << " (default)" << std::endl;
  else
    std::cout << "klt : Pyramid Levels : " << m_kltPyramidLevels << std::endl;
}

void vpMbtXmlGenericParser::read_lod (xmlDocPtr doc, xmlNodePtr node) {
  bool use_lod_node = false;
  bool min_line_length_threshold_node = false;
  bool min_polygon_area_threshold_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second){
          case use_lod:
            m_useLod = (xmlReadIntChild(doc, dataNode) != 0);
            use_lod_node = true;
            break;

          case min_line_length_threshold:
            m_minLineLengthThreshold = xmlReadDoubleChild(doc, dataNode);
            min_line_length_threshold_node = true;
            break;

          case min_polygon_area_threshold:
            m_minPolygonAreaThreshold = xmlReadDoubleChild(doc, dataNode);
            min_polygon_area_threshold_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  if (!use_lod_node)
    std::cout << "lod : use lod : " << m_useLod << " (default)" << std::endl;
  else
    std::cout << "lod : use lod : " << m_useLod << std::endl;

  if (!min_line_length_threshold_node)
    std::cout << "lod : min line length threshold : " << m_minLineLengthThreshold << " (default)" << std::endl;
  else
    std::cout << "lod : min line length threshold : " << m_minLineLengthThreshold << std::endl;

  if (!min_polygon_area_threshold_node)
    std::cout << "lod : min polygon area threshold : " << m_minPolygonAreaThreshold << " (default)" << std::endl;
  else
    std::cout << "lod : min polygon area threshold : " << m_minPolygonAreaThreshold << std::endl;
}

/*!
  Read sample information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample information.
*/
void vpMbtXmlGenericParser::read_sample_deprecated(xmlDocPtr doc, xmlNodePtr node) {
  bool step_node = false;
  //bool nb_sample_node = false;

  // current data values.
  double d_stp = m_ecm.getSampleStep();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::const_iterator iter_data = nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
          case step:
            d_stp = xmlReadIntChild(doc, dataNode);
            step_node = true;
            break;

          default:
            break;
        }
      }
    }
  }

  m_ecm.setSampleStep(d_stp);

  if(!step_node)
    std::cout << "[DEPRECATED] sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
  else
    std::cout << "[DEPRECATED] sample : sample_step : " << m_ecm.getSampleStep() << std::endl;

  std::cout << "  WARNING : This node (sample) is deprecated." << std::endl;
  std::cout << "  It should be moved in the ecm node (ecm : sample)." << std::endl;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbtXmlGenericParser.cpp.o) has no symbols
void dummy_vpMbtXmlGenericParser() {};
#endif

