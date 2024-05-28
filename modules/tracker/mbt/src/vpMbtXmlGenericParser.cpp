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
#include <visp3/core/vpConfig.h>

#include <clocale>
#include <iostream>
#include <map>

#include <visp3/mbt/vpMbtXmlGenericParser.h>

#if defined(VISP_HAVE_PUGIXML)
#include <pugixml.hpp>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS

class vpMbtXmlGenericParser::Impl
{
public:
  Impl(int type = EDGE_PARSER)
    : m_parserType(type),
    //<camera>
    m_cam(),
    //<face>
    m_angleAppear(70), m_angleDisappear(80), m_hasNearClipping(false), m_nearClipping(false), m_hasFarClipping(false),
    m_farClipping(false), m_fovClipping(false),
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
    m_depthNormalPclPlaneEstimationRansacThreshold(0.001), m_depthNormalSamplingStepX(2),
    m_depthNormalSamplingStepY(2),
    //<depth_dense>
    m_depthDenseSamplingStepX(2), m_depthDenseSamplingStepY(2),
    //<projection_error>
    m_projectionErrorMe(), m_projectionErrorKernelSize(2), // 5x5
    m_nodeMap(), m_verbose(true)
  {
    // std::setlocale() is not thread safe and need to be called once
    // https://stackoverflow.com/questions/41117179/undefined-behavior-with-setlocale-and-multithreading
    if (m_call_setlocale) {
      // https://pugixml.org/docs/manual.html#access.attrdata
      // https://en.cppreference.com/w/cpp/locale/setlocale
      // When called from Java binding, the locale seems to be changed to the default system locale
      // It thus mess with the parsing of numbers with pugixml and comma decimal separator environment
      if (std::setlocale(LC_ALL, "C") == nullptr) {
        std::cerr << "Cannot set locale to C" << std::endl;
      }
      m_call_setlocale = false;
    }
    init();
  }

  void parse(const std::string &filename)
  {
    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
      throw vpException(vpException::ioError, "Cannot open file: %s", filename.c_str());
    }

    bool camera_node = false;
    bool face_node = false;
    bool ecm_node = false;
    bool klt_node = false;
    bool lod_node = false;
    bool depth_normal_node = false;
    bool depth_dense_node = false;
    bool projection_error_node = false;

    pugi::xml_node root_node = doc.document_element();
    for (pugi::xml_node dataNode = root_node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case camera:
            if (m_parserType != PROJECTION_ERROR_PARSER) {
              read_camera(dataNode);
              camera_node = true;
            }
            break;

          case face:
            if (m_parserType != PROJECTION_ERROR_PARSER) {
              read_face(dataNode);
              face_node = true;
            }
            break;

          case lod:
            if (m_parserType != PROJECTION_ERROR_PARSER) {
              read_lod(dataNode);
              lod_node = true;
            }
            break;

          case ecm:
            if (m_parserType & EDGE_PARSER) {
              read_ecm(dataNode);
              ecm_node = true;
            }
            break;

          case sample:
            if (m_parserType & EDGE_PARSER)
              read_sample_deprecated(dataNode);
            break;

          case klt:
            if (m_parserType & KLT_PARSER) {
              read_klt(dataNode);
              klt_node = true;
            }
            break;

          case depth_normal:
            if (m_parserType & DEPTH_NORMAL_PARSER) {
              read_depth_normal(dataNode);
              depth_normal_node = true;
            }
            break;

          case depth_dense:
            if (m_parserType & DEPTH_DENSE_PARSER) {
              read_depth_dense(dataNode);
              depth_dense_node = true;
            }
            break;

          case projection_error:
            if (m_parserType == PROJECTION_ERROR_PARSER) {
              read_projection_error(dataNode);
              projection_error_node = true;
            }
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (m_parserType == PROJECTION_ERROR_PARSER) {
        if (!projection_error_node) {
          std::cout << "projection_error : sample_step : " << m_projectionErrorMe.getSampleStep() << " (default)"
            << std::endl;
          std::cout << "projection_error : kernel_size : " << m_projectionErrorKernelSize * 2 + 1 << "x"
            << m_projectionErrorKernelSize * 2 + 1 << " (default)" << std::endl;
        }
      }
      else {
        if (!camera_node) {
          std::cout << "camera : u0 : " << m_cam.get_u0() << " (default)" << std::endl;
          std::cout << "camera : v0 : " << m_cam.get_v0() << " (default)" << std::endl;
          std::cout << "camera : px : " << m_cam.get_px() << " (default)" << std::endl;
          std::cout << "camera : py : " << m_cam.get_py() << " (default)" << std::endl;
        }

        if (!face_node) {
          std::cout << "face : Angle Appear : " << m_angleAppear << " (default)" << std::endl;
          std::cout << "face : Angle Disappear : " << m_angleDisappear << " (default)" << std::endl;
        }

        if (!lod_node) {
          std::cout << "lod : use lod : " << m_useLod << " (default)" << std::endl;
          std::cout << "lod : min line length threshold : " << m_minLineLengthThreshold << " (default)" << std::endl;
          std::cout << "lod : min polygon area threshold : " << m_minPolygonAreaThreshold << " (default)" << std::endl;
        }

        if (!ecm_node && (m_parserType & EDGE_PARSER)) {
          std::cout << "me : mask : size : " << m_ecm.getMaskSize() << " (default)" << std::endl;
          std::cout << "me : mask : nb_mask : " << m_ecm.getMaskNumber() << " (default)" << std::endl;
          std::cout << "me : range : tracking : " << m_ecm.getRange() << " (default)" << std::endl;
          std::cout << "me : contrast : threshold type : " << m_ecm.getLikelihoodThresholdType() << " (default)" << std::endl;
          std::cout << "me : contrast : threshold : " << m_ecm.getThreshold() << " (default)" << std::endl;
          std::cout << "me : contrast : mu1 : " << m_ecm.getMu1() << " (default)" << std::endl;
          std::cout << "me : contrast : mu2 : " << m_ecm.getMu2() << " (default)" << std::endl;
          std::cout << "me : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
        }

        if (!klt_node && (m_parserType & KLT_PARSER)) {
          std::cout << "klt : Mask Border : " << m_kltMaskBorder << " (default)" << std::endl;
          std::cout << "klt : Max Features : " << m_kltMaxFeatures << " (default)" << std::endl;
          std::cout << "klt : Windows Size : " << m_kltWinSize << " (default)" << std::endl;
          std::cout << "klt : Quality : " << m_kltQualityValue << " (default)" << std::endl;
          std::cout << "klt : Min Distance : " << m_kltMinDist << " (default)" << std::endl;
          std::cout << "klt : Harris Parameter : " << m_kltHarrisParam << " (default)" << std::endl;
          std::cout << "klt : Block Size : " << m_kltBlockSize << " (default)" << std::endl;
          std::cout << "klt : Pyramid Levels : " << m_kltPyramidLevels << " (default)" << std::endl;
        }

        if (!depth_normal_node && (m_parserType & DEPTH_NORMAL_PARSER)) {
          std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod
            << " (default)" << std::endl;
          std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod
            << " (default)" << std::endl;
          std::cout << "depth normal : PCL_plane_estimation : max_iter : "
            << m_depthNormalPclPlaneEstimationRansacMaxIter << " (default)" << std::endl;
          std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : "
            << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
          std::cout << "depth normal : sampling_step : step_X " << m_depthNormalSamplingStepX << " (default)"
            << std::endl;
          std::cout << "depth normal : sampling_step : step_Y " << m_depthNormalSamplingStepY << " (default)"
            << std::endl;
        }

        if (!depth_dense_node && (m_parserType & DEPTH_DENSE_PARSER)) {
          std::cout << "depth dense : sampling_step : step_X " << m_depthDenseSamplingStepX << " (default)"
            << std::endl;
          std::cout << "depth dense : sampling_step : step_Y " << m_depthDenseSamplingStepY << " (default)"
            << std::endl;
        }
      }
    }
  }

  /*!
    Read camera information.

    \param node : Pointer to the node of the camera information.
  */
  void read_camera(const pugi::xml_node &node)
  {
    bool u0_node = false;
    bool v0_node = false;
    bool px_node = false;
    bool py_node = false;

    // current data values.
    double d_u0 = m_cam.get_u0();
    double d_v0 = m_cam.get_v0();
    double d_px = m_cam.get_px();
    double d_py = m_cam.get_py();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case u0:
            d_u0 = dataNode.text().as_double();
            u0_node = true;
            break;

          case v0:
            d_v0 = dataNode.text().as_double();
            v0_node = true;
            break;

          case px:
            d_px = dataNode.text().as_double();
            px_node = true;
            break;

          case py:
            d_py = dataNode.text().as_double();
            py_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    m_cam.initPersProjWithoutDistortion(d_px, d_py, d_u0, d_v0);

    if (m_verbose) {
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
  }

  /*!
    Read depth normal information.

    \param node : Pointer to the node information.
  */
  void read_depth_normal(const pugi::xml_node &node)
  {
    bool feature_estimation_method_node = false;
    bool PCL_plane_estimation_node = false;
    bool sampling_step_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case feature_estimation_method:
            m_depthNormalFeatureEstimationMethod =
              (vpMbtFaceDepthNormal::vpFeatureEstimationType)dataNode.text().as_int();
            feature_estimation_method_node = true;
            break;

          case PCL_plane_estimation:
            read_depth_normal_PCL(dataNode);
            PCL_plane_estimation_node = true;
            break;

          case depth_sampling_step:
            read_depth_normal_sampling_step(dataNode);
            sampling_step_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (!feature_estimation_method_node)
        std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod
        << " (default)" << std::endl;
      else
        std::cout << "depth normal : feature_estimation_method : " << m_depthNormalFeatureEstimationMethod << std::endl;

      if (!PCL_plane_estimation_node) {
        std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod
          << " (default)" << std::endl;
        std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter
          << " (default)" << std::endl;
        std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : "
          << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
      }

      if (!sampling_step_node) {
        std::cout << "depth normal : sampling_step : step_X " << m_depthNormalSamplingStepX << " (default)"
          << std::endl;
        std::cout << "depth normal : sampling_step : step_Y " << m_depthNormalSamplingStepY << " (default)"
          << std::endl;
      }
    }
  }

  /*!
    Read depth normal PCL properties.

    \param node : Pointer to the node information.
  */
  void read_depth_normal_PCL(const pugi::xml_node &node)
  {
    bool PCL_plane_estimation_method_node = false;
    bool PCL_plane_estimation_ransac_max_iter_node = false;
    bool PCL_plane_estimation_ransac_threshold_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case PCL_plane_estimation_method:
            m_depthNormalPclPlaneEstimationMethod = dataNode.text().as_int();
            PCL_plane_estimation_method_node = true;
            break;

          case PCL_plane_estimation_ransac_max_iter:
            m_depthNormalPclPlaneEstimationRansacMaxIter = dataNode.text().as_int();
            PCL_plane_estimation_ransac_max_iter_node = true;
            break;

          case PCL_plane_estimation_ransac_threshold:
            m_depthNormalPclPlaneEstimationRansacThreshold = dataNode.text().as_double();
            PCL_plane_estimation_ransac_threshold_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (!PCL_plane_estimation_method_node)
        std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod
        << " (default)" << std::endl;
      else
        std::cout << "depth normal : PCL_plane_estimation : method : " << m_depthNormalPclPlaneEstimationMethod
        << std::endl;

      if (!PCL_plane_estimation_ransac_max_iter_node)
        std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter
        << " (default)" << std::endl;
      else
        std::cout << "depth normal : PCL_plane_estimation : max_iter : " << m_depthNormalPclPlaneEstimationRansacMaxIter
        << std::endl;

      if (!PCL_plane_estimation_ransac_threshold_node)
        std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : "
        << m_depthNormalPclPlaneEstimationRansacThreshold << " (default)" << std::endl;
      else
        std::cout << "depth normal : PCL_plane_estimation : ransac_threshold : "
        << m_depthNormalPclPlaneEstimationRansacThreshold << std::endl;
    }
  }

  /*!
    Read depth normal sampling step.

    \param node : Pointer to the node information.
  */
  void read_depth_normal_sampling_step(const pugi::xml_node &node)
  {
    bool sampling_step_X_node = false;
    bool sampling_step_Y_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case depth_sampling_step_X:
            m_depthNormalSamplingStepX = dataNode.text().as_uint();
            sampling_step_X_node = true;
            break;

          case depth_sampling_step_Y:
            m_depthNormalSamplingStepY = dataNode.text().as_uint();
            sampling_step_Y_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (!sampling_step_X_node)
        std::cout << "depth normal : sampling_step : step_X : " << m_depthNormalSamplingStepX << " (default)"
        << std::endl;
      else
        std::cout << "depth normal : sampling_step : step_X : " << m_depthNormalSamplingStepX << std::endl;

      if (!sampling_step_Y_node)
        std::cout << "depth normal : sampling_step : step_Y : " << m_depthNormalSamplingStepY << " (default)"
        << std::endl;
      else
        std::cout << "depth normal : sampling_step : step_Y : " << m_depthNormalSamplingStepY << std::endl;
    }
  }

  /*!
    Read depth dense information.

    \param node : Pointer to the node of the ecm information.
  */
  void read_depth_dense(const pugi::xml_node &node)
  {
    bool sampling_step_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case depth_dense_sampling_step:
            read_depth_dense_sampling_step(dataNode);
            sampling_step_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (!sampling_step_node && m_verbose) {
      std::cout << "depth dense : sampling_step : step_X " << m_depthDenseSamplingStepX << " (default)" << std::endl;
      std::cout << "depth dense : sampling_step : step_Y " << m_depthDenseSamplingStepY << " (default)" << std::endl;
    }
  }

  /*!
    Read depth dense sampling step.

    \param node : Pointer to the node of the range information.
  */
  void read_depth_dense_sampling_step(const pugi::xml_node &node)
  {
    bool sampling_step_X_node = false;
    bool sampling_step_Y_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case depth_dense_sampling_step_X:
            m_depthDenseSamplingStepX = dataNode.text().as_uint();
            sampling_step_X_node = true;
            break;

          case depth_dense_sampling_step_Y:
            m_depthDenseSamplingStepY = dataNode.text().as_uint();
            sampling_step_Y_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (!sampling_step_X_node)
        std::cout << "depth dense : sampling_step : step_X : " << m_depthDenseSamplingStepX << " (default)"
        << std::endl;
      else
        std::cout << "depth dense : sampling_step : step_X : " << m_depthDenseSamplingStepX << std::endl;

      if (!sampling_step_Y_node)
        std::cout << "depth dense : sampling_step : step_Y : " << m_depthDenseSamplingStepY << " (default)"
        << std::endl;
      else
        std::cout << "depth dense : sampling_step : step_Y : " << m_depthDenseSamplingStepY << std::endl;
    }
  }

  /*!
    Read ecm information.

    \param node : Pointer to the node of the ecm information.
  */
  void read_ecm(const pugi::xml_node &node)
  {
    bool mask_node = false;
    bool range_node = false;
    bool contrast_node = false;
    bool sample_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case mask:
            read_ecm_mask(dataNode);
            mask_node = true;
            break;

          case range:
            read_ecm_range(dataNode);
            range_node = true;
            break;

          case contrast:
            read_ecm_contrast(dataNode);
            contrast_node = true;
            break;

          case sample:
            read_ecm_sample(dataNode);
            sample_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
      if (!mask_node) {
        std::cout << "me : mask : size : " << m_ecm.getMaskSize() << " (default)" << std::endl;
        std::cout << "me : mask : nb_mask : " << m_ecm.getMaskNumber() << " (default)" << std::endl;
      }

      if (!range_node) {
        std::cout << "me : range : tracking : " << m_ecm.getRange() << " (default)" << std::endl;
      }

      if (!contrast_node) {
        std::cout << "me : contrast : threshold type " << m_ecm.getLikelihoodThresholdType() << " (default)" << std::endl;
        std::cout << "me : contrast : threshold " << m_ecm.getThreshold() << " (default)" << std::endl;
        std::cout << "me : contrast : mu1 " << m_ecm.getMu1() << " (default)" << std::endl;
        std::cout << "me : contrast : mu2 " << m_ecm.getMu2() << " (default)" << std::endl;
      }

      if (!sample_node) {
        std::cout << "me : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
      }
    }
  }

  /*!
    Read the contrast information from the xml file.

    \param node : Pointer to the node of the contrast information.
  */
  void read_ecm_contrast(const pugi::xml_node &node)
  {
    bool edge_threshold_type_node = false;
    bool edge_threshold_node = false;
    bool mu1_node = false;
    bool mu2_node = false;

    // current data values.
    vpMe::vpLikelihoodThresholdType d_edge_threshold_type = m_ecm.getLikelihoodThresholdType();
    double d_edge_threshold = m_ecm.getThreshold();
    double d_mu1 = m_ecm.getMu1();
    double d_mu2 = m_ecm.getMu2();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case edge_threshold_type:
            d_edge_threshold_type = static_cast<vpMe::vpLikelihoodThresholdType>(dataNode.text().as_int());
            edge_threshold_type_node = true;
            break;

          case edge_threshold:
            d_edge_threshold = dataNode.text().as_int();
            edge_threshold_node = true;
            break;

          case mu1:
            d_mu1 = dataNode.text().as_double();
            mu1_node = true;
            break;

          case mu2:
            d_mu2 = dataNode.text().as_double();
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
    m_ecm.setLikelihoodThresholdType(d_edge_threshold_type);
    m_ecm.setThreshold(d_edge_threshold);

    if (m_verbose) {
      if (!edge_threshold_type_node)
        std::cout << "me : contrast : threshold type " << m_ecm.getLikelihoodThresholdType() << " (default)" << std::endl;
      else
        std::cout << "me : contrast : threshold type " << m_ecm.getLikelihoodThresholdType() << std::endl;

      if (!edge_threshold_node)
        std::cout << "me : contrast : threshold " << m_ecm.getThreshold() << " (default)" << std::endl;
      else
        std::cout << "me : contrast : threshold " << m_ecm.getThreshold() << std::endl;

      if (!mu1_node)
        std::cout << "me : contrast : mu1 " << m_ecm.getMu1() << " (default)" << std::endl;
      else
        std::cout << "me : contrast : mu1 " << m_ecm.getMu1() << std::endl;

      if (!mu2_node)
        std::cout << "me : contrast : mu2 " << m_ecm.getMu2() << " (default)" << std::endl;
      else
        std::cout << "me : contrast : mu2 " << m_ecm.getMu2() << std::endl;
    }
  }

  /*!
    Read mask information for the vpMeSite.

    \param node : Pointer to the node of the mask information.
  */
  void read_ecm_mask(const pugi::xml_node &node)
  {
    bool size_node = false;
    bool nb_mask_node = false;

    // current data values.
    unsigned int d_size = m_ecm.getMaskSize();
    unsigned int d_nb_mask = m_ecm.getMaskNumber();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case size:
            d_size = dataNode.text().as_uint();
            size_node = true;
            break;

          case nb_mask:
            d_nb_mask = dataNode.text().as_uint();
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
      throw(vpException(vpException::badValue, "Model-based tracker mask size "
                        "parameter should be different "
                        "from zero in xml file"));
    m_ecm.setMaskNumber(d_nb_mask);

    if (m_verbose) {
      if (!size_node)
        std::cout << "me : mask : size : " << m_ecm.getMaskSize() << " (default)" << std::endl;
      else
        std::cout << "me : mask : size : " << m_ecm.getMaskSize() << std::endl;

      if (!nb_mask_node)
        std::cout << "me : mask : nb_mask : " << m_ecm.getMaskNumber() << " (default)" << std::endl;
      else
        std::cout << "me : mask : nb_mask : " << m_ecm.getMaskNumber() << std::endl;
    }
  }

  /*!
    Read range information for the vpMeSite.

    \param node : Pointer to the node of the range information.
  */
  void read_ecm_range(const pugi::xml_node &node)
  {
    bool tracking_node = false;

    // current data values.
    unsigned int m_range_tracking = m_ecm.getRange();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case tracking:
            m_range_tracking = dataNode.text().as_uint();
            tracking_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    m_ecm.setRange(m_range_tracking);

    if (m_verbose) {
      if (!tracking_node)
        std::cout << "me : range : tracking : " << m_ecm.getRange() << " (default)" << std::endl;
      else
        std::cout << "me : range : tracking : " << m_ecm.getRange() << std::endl;
    }
  }

  /*!
    Read sample information.

    \param node : Pointer to the node of the sample information.
  */
  void read_ecm_sample(const pugi::xml_node &node)
  {
    bool step_node = false;

    // current data values.
    double d_stp = m_ecm.getSampleStep();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case step:
            d_stp = dataNode.text().as_int();
            step_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    m_ecm.setSampleStep(d_stp);

    if (m_verbose) {
      if (!step_node)
        std::cout << "me : sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
      else
        std::cout << "me : sample : sample_step : " << m_ecm.getSampleStep() << std::endl;
    }
  }

  /*!
    Read face information.

    \param node : Pointer to the node of the camera information.
  */
  void read_face(const pugi::xml_node &node)
  {
    bool angle_appear_node = false;
    bool angle_disappear_node = false;
    bool near_clipping_node = false;
    bool far_clipping_node = false;
    bool fov_clipping_node = false;
    m_hasNearClipping = false;
    m_hasFarClipping = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case angle_appear:
            m_angleAppear = dataNode.text().as_double();
            angle_appear_node = true;
            break;

          case angle_disappear:
            m_angleDisappear = dataNode.text().as_double();
            angle_disappear_node = true;
            break;

          case near_clipping:
            m_nearClipping = dataNode.text().as_double();
            m_hasNearClipping = true;
            near_clipping_node = true;
            break;

          case far_clipping:
            m_farClipping = dataNode.text().as_double();
            m_hasFarClipping = true;
            far_clipping_node = true;
            break;

          case fov_clipping:
            if (dataNode.text().as_int())
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

    if (m_verbose) {
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
        if (m_fovClipping)
          std::cout << "face : Fov Clipping : True" << std::endl;
        else
          std::cout << "face : Fov Clipping : False" << std::endl;
      }
    }
  }

  /*!
    Read klt information.

    \param node : Pointer to the node of the camera information.
  */
  void read_klt(const pugi::xml_node &node)
  {
    bool mask_border_node = false;
    bool max_features_node = false;
    bool window_size_node = false;
    bool quality_node = false;
    bool min_distance_node = false;
    bool harris_node = false;
    bool size_block_node = false;
    bool pyramid_lvl_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case mask_border:
            m_kltMaskBorder = dataNode.text().as_uint();
            mask_border_node = true;
            break;

          case max_features:
            m_kltMaxFeatures = dataNode.text().as_uint();
            max_features_node = true;
            break;

          case window_size:
            m_kltWinSize = dataNode.text().as_uint();
            window_size_node = true;
            break;

          case quality:
            m_kltQualityValue = dataNode.text().as_double();
            quality_node = true;
            break;

          case min_distance:
            m_kltMinDist = dataNode.text().as_double();
            min_distance_node = true;
            break;

          case harris:
            m_kltHarrisParam = dataNode.text().as_double();
            harris_node = true;
            break;

          case size_block:
            m_kltBlockSize = dataNode.text().as_uint();
            size_block_node = true;
            break;

          case pyramid_lvl:
            m_kltPyramidLevels = dataNode.text().as_uint();
            pyramid_lvl_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
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
  }

  void read_lod(const pugi::xml_node &node)
  {
    bool use_lod_node = false;
    bool min_line_length_threshold_node = false;
    bool min_polygon_area_threshold_node = false;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case use_lod:
            m_useLod = (dataNode.text().as_int() != 0);
            use_lod_node = true;
            break;

          case min_line_length_threshold:
            m_minLineLengthThreshold = dataNode.text().as_double();
            min_line_length_threshold_node = true;
            break;

          case min_polygon_area_threshold:
            m_minPolygonAreaThreshold = dataNode.text().as_double();
            min_polygon_area_threshold_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    if (m_verbose) {
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
  }

  void read_projection_error(const pugi::xml_node &node)
  {
    bool step_node = false;
    bool kernel_size_node = false;

    // current data values.
    double d_stp = m_projectionErrorMe.getSampleStep();
    std::string kernel_size_str;

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case projection_error_sample_step:
            d_stp = dataNode.text().as_int();
            step_node = true;
            break;

          case projection_error_kernel_size:
            kernel_size_str = dataNode.text().as_string();
            kernel_size_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    m_projectionErrorMe.setSampleStep(d_stp);

    if (kernel_size_str == "3x3") {
      m_projectionErrorKernelSize = 1;
    }
    else if (kernel_size_str == "5x5") {
      m_projectionErrorKernelSize = 2;
    }
    else if (kernel_size_str == "7x7") {
      m_projectionErrorKernelSize = 3;
    }
    else if (kernel_size_str == "9x9") {
      m_projectionErrorKernelSize = 4;
    }
    else if (kernel_size_str == "11x11") {
      m_projectionErrorKernelSize = 5;
    }
    else if (kernel_size_str == "13x13") {
      m_projectionErrorKernelSize = 6;
    }
    else if (kernel_size_str == "15x15") {
      m_projectionErrorKernelSize = 7;
    }
    else {
      std::cerr << "Unsupported kernel size." << std::endl;
    }

    if (m_verbose) {
      if (!step_node)
        std::cout << "projection_error : sample_step : " << m_projectionErrorMe.getSampleStep() << " (default)"
        << std::endl;
      else
        std::cout << "projection_error : sample_step : " << m_projectionErrorMe.getSampleStep() << std::endl;

      if (!kernel_size_node)
        std::cout << "projection_error : kernel_size : " << m_projectionErrorKernelSize * 2 + 1 << "x"
        << m_projectionErrorKernelSize * 2 + 1 << " (default)" << std::endl;
      else
        std::cout << "projection_error : kernel_size : " << kernel_size_str << std::endl;
    }
  }

  /*!
    Read sample information.

    \param node : Pointer to the node of the sample information.
  */
  void read_sample_deprecated(const pugi::xml_node &node)
  {
    bool step_node = false;
    // bool nb_sample_node = false;

    // current data values.
    double d_stp = m_ecm.getSampleStep();

    for (pugi::xml_node dataNode = node.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::const_iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case step:
            d_stp = dataNode.text().as_int();
            step_node = true;
            break;

          default:
            break;
          }
        }
      }
    }

    m_ecm.setSampleStep(d_stp);

    if (m_verbose) {
      if (!step_node)
        std::cout << "[DEPRECATED] sample : sample_step : " << m_ecm.getSampleStep() << " (default)" << std::endl;
      else
        std::cout << "[DEPRECATED] sample : sample_step : " << m_ecm.getSampleStep() << std::endl;

      std::cout << "  WARNING : This node (sample) is deprecated." << std::endl;
      std::cout << "  It should be moved in the ecm node (me : sample)." << std::endl;
    }
  }

  double getAngleAppear() const { return m_angleAppear; }
  double getAngleDisappear() const { return m_angleDisappear; }

  void getCameraParameters(vpCameraParameters &cam) const { cam = m_cam; }

  void getEdgeMe(vpMe &moving_edge) const { moving_edge = m_ecm; }

  unsigned int getDepthDenseSamplingStepX() const { return m_depthDenseSamplingStepX; }
  unsigned int getDepthDenseSamplingStepY() const { return m_depthDenseSamplingStepY; }

  vpMbtFaceDepthNormal::vpFeatureEstimationType getDepthNormalFeatureEstimationMethod() const
  {
    return m_depthNormalFeatureEstimationMethod;
  }
  int getDepthNormalPclPlaneEstimationMethod() const { return m_depthNormalPclPlaneEstimationMethod; }
  int getDepthNormalPclPlaneEstimationRansacMaxIter() const { return m_depthNormalPclPlaneEstimationRansacMaxIter; }
  double getDepthNormalPclPlaneEstimationRansacThreshold() const
  {
    return m_depthNormalPclPlaneEstimationRansacThreshold;
  }
  unsigned int getDepthNormalSamplingStepX() const { return m_depthNormalSamplingStepX; }
  unsigned int getDepthNormalSamplingStepY() const { return m_depthNormalSamplingStepY; }

  double getFarClippingDistance() const { return m_farClipping; }
  bool getFovClipping() const { return m_fovClipping; }

  unsigned int getKltBlockSize() const { return m_kltBlockSize; }
  double getKltHarrisParam() const { return m_kltHarrisParam; }
  unsigned int getKltMaskBorder() const { return m_kltMaskBorder; }
  unsigned int getKltMaxFeatures() const { return m_kltMaxFeatures; }
  double getKltMinDistance() const { return m_kltMinDist; }
  unsigned int getKltPyramidLevels() const { return m_kltPyramidLevels; }
  double getKltQuality() const { return m_kltQualityValue; }
  unsigned int getKltWindowSize() const { return m_kltWinSize; }

  bool getLodState() const { return m_useLod; }
  double getLodMinLineLengthThreshold() const { return m_minLineLengthThreshold; }
  double getLodMinPolygonAreaThreshold() const { return m_minPolygonAreaThreshold; }

  double getNearClippingDistance() const { return m_nearClipping; }

  void getProjectionErrorMe(vpMe &me) const { me = m_projectionErrorMe; }
  unsigned int getProjectionErrorKernelSize() const { return m_projectionErrorKernelSize; }

  bool hasFarClippingDistance() const { return m_hasFarClipping; }
  bool hasNearClippingDistance() const { return m_hasNearClipping; }

  void setAngleAppear(const double &aappear) { m_angleAppear = aappear; }
  void setAngleDisappear(const double &adisappear) { m_angleDisappear = adisappear; }

  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }

  void setDepthDenseSamplingStepX(unsigned int stepX) { m_depthDenseSamplingStepX = stepX; }
  void setDepthDenseSamplingStepY(unsigned int stepY) { m_depthDenseSamplingStepY = stepY; }
  void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method)
  {
    m_depthNormalFeatureEstimationMethod = method;
  }
  void setDepthNormalPclPlaneEstimationMethod(int method) { m_depthNormalPclPlaneEstimationMethod = method; }
  void setDepthNormalPclPlaneEstimationRansacMaxIter(int maxIter)
  {
    m_depthNormalPclPlaneEstimationRansacMaxIter = maxIter;
  }
  void setDepthNormalPclPlaneEstimationRansacThreshold(double threshold)
  {
    m_depthNormalPclPlaneEstimationRansacThreshold = threshold;
  }
  void setDepthNormalSamplingStepX(unsigned int stepX) { m_depthNormalSamplingStepX = stepX; }
  void setDepthNormalSamplingStepY(unsigned int stepY) { m_depthNormalSamplingStepY = stepY; }

  void setEdgeMe(const vpMe &moving_edge) { m_ecm = moving_edge; }

  void setFarClippingDistance(const double &fclip) { m_farClipping = fclip; }

  void setKltBlockSize(const unsigned int &bs) { m_kltBlockSize = bs; }
  void setKltHarrisParam(const double &hp) { m_kltHarrisParam = hp; }
  void setKltMaskBorder(const unsigned int &mb) { m_kltMaskBorder = mb; }
  void setKltMaxFeatures(const unsigned int &mF) { m_kltMaxFeatures = mF; }
  void setKltMinDistance(const double &mD) { m_kltMinDist = mD; }
  void setKltPyramidLevels(const unsigned int &pL) { m_kltPyramidLevels = pL; }
  void setKltQuality(const double &q) { m_kltQualityValue = q; }
  void setKltWindowSize(const unsigned int &w) { m_kltWinSize = w; }

  void setNearClippingDistance(const double &nclip) { m_nearClipping = nclip; }

  void setProjectionErrorMe(const vpMe &me) { m_projectionErrorMe = me; }
  void setProjectionErrorKernelSize(const unsigned int &kernel_size) { m_projectionErrorKernelSize = kernel_size; }

  void setVerbose(bool verbose) { m_verbose = verbose; }

protected:
  //! Parser type
  int m_parserType;
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
  // LOD
  //! If true, the LOD is enabled, otherwise it is not
  bool m_useLod;
  //! Minimum line length to track a segment when LOD is enabled
  double m_minLineLengthThreshold;
  //! Minimum polygon area to track a face when LOD is enabled
  double m_minPolygonAreaThreshold;
  // Edge
  //! Moving edges parameters.
  vpMe m_ecm;
  // KLT
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
  // Depth normal
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
  // Depth dense
  //! Sampling step in X
  unsigned int m_depthDenseSamplingStepX;
  //! Sampling step in Y
  unsigned int m_depthDenseSamplingStepY;
  // Projection error
  //! ME parameters for projection error computation
  vpMe m_projectionErrorMe;
  //! Kernel size (actual_kernel_size = size*2 + 1) used for projection error computation
  unsigned int m_projectionErrorKernelSize;
  std::map<std::string, int> m_nodeMap;
  //! Verbose flag
  bool m_verbose;

  enum vpDataToParseMb
  {
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
    edge_threshold_type,
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
    depth_dense_sampling_step_Y,
    //<projection_error>
    projection_error,
    projection_error_sample_step,
    projection_error_kernel_size
  };

  /*!
    Initialise internal variables (including the map).
  */
  void init()
  {
    //<conf>
    m_nodeMap["conf"] = conf;
    //<face>
    m_nodeMap["face"] = face;
    m_nodeMap["angle_appear"] = angle_appear;
    m_nodeMap["angle_disappear"] = angle_disappear;
    m_nodeMap["near_clipping"] = near_clipping;
    m_nodeMap["far_clipping"] = far_clipping;
    m_nodeMap["fov_clipping"] = fov_clipping;
    //<camera>
    m_nodeMap["camera"] = camera;
    m_nodeMap["height"] = height;
    m_nodeMap["width"] = width;
    m_nodeMap["u0"] = u0;
    m_nodeMap["v0"] = v0;
    m_nodeMap["px"] = px;
    m_nodeMap["py"] = py;
    //<lod>
    m_nodeMap["lod"] = lod;
    m_nodeMap["use_lod"] = use_lod;
    m_nodeMap["min_line_length_threshold"] = min_line_length_threshold;
    m_nodeMap["min_polygon_area_threshold"] = min_polygon_area_threshold;
    //<ecm>
    m_nodeMap["ecm"] = ecm;
    m_nodeMap["mask"] = mask;
    m_nodeMap["size"] = size;
    m_nodeMap["nb_mask"] = nb_mask;
    m_nodeMap["range"] = range;
    m_nodeMap["tracking"] = tracking;
    m_nodeMap["contrast"] = contrast;
    m_nodeMap["edge_threshold_type"] = edge_threshold_type;
    m_nodeMap["edge_threshold"] = edge_threshold;
    m_nodeMap["mu1"] = mu1;
    m_nodeMap["mu2"] = mu2;
    m_nodeMap["sample"] = sample;
    m_nodeMap["step"] = step;
    //<klt>
    m_nodeMap["klt"] = klt;
    m_nodeMap["mask_border"] = mask_border;
    m_nodeMap["max_features"] = max_features;
    m_nodeMap["window_size"] = window_size;
    m_nodeMap["quality"] = quality;
    m_nodeMap["min_distance"] = min_distance;
    m_nodeMap["harris"] = harris;
    m_nodeMap["size_block"] = size_block;
    m_nodeMap["pyramid_lvl"] = pyramid_lvl;
    //<depth_normal>
    m_nodeMap["depth_normal"] = depth_normal;
    m_nodeMap["feature_estimation_method"] = feature_estimation_method;
    m_nodeMap["PCL_plane_estimation"] = PCL_plane_estimation;
    m_nodeMap["method"] = PCL_plane_estimation_method;
    m_nodeMap["ransac_max_iter"] = PCL_plane_estimation_ransac_max_iter;
    m_nodeMap["ransac_threshold"] = PCL_plane_estimation_ransac_threshold;
    m_nodeMap["sampling_step"] = depth_sampling_step;
    m_nodeMap["step_X"] = depth_sampling_step_X;
    m_nodeMap["step_Y"] = depth_sampling_step_Y;
    //<depth_dense>
    m_nodeMap["depth_dense"] = depth_dense;
    m_nodeMap["sampling_step"] = depth_dense_sampling_step;
    m_nodeMap["step_X"] = depth_dense_sampling_step_X;
    m_nodeMap["step_Y"] = depth_dense_sampling_step_Y;
    //<projection_error>
    m_nodeMap["projection_error"] = projection_error;
    m_nodeMap["sample_step"] = projection_error_sample_step;
    m_nodeMap["kernel_size"] = projection_error_kernel_size;
  }

private:
  static bool m_call_setlocale;
};

bool vpMbtXmlGenericParser::Impl::m_call_setlocale = true;

#endif // DOXYGEN_SHOULD_SKIP_THIS

vpMbtXmlGenericParser::vpMbtXmlGenericParser(int type) : m_impl(new Impl(type))
{ }

vpMbtXmlGenericParser::~vpMbtXmlGenericParser() { delete m_impl; }

/*!
  Parse an XML config file that contains parameters for the Generic Model-Based Tracker.

  \param filename : Document to parse.
*/
void vpMbtXmlGenericParser::parse(const std::string &filename) { m_impl->parse(filename); }

/*!
  Get the angle to determine if a face appeared.
*/
double vpMbtXmlGenericParser::getAngleAppear() const { return m_impl->getAngleAppear(); }

/*!
  Get the angle to determine if a face disappeared.
*/
double vpMbtXmlGenericParser::getAngleDisappear() const { return m_impl->getAngleDisappear(); }

void vpMbtXmlGenericParser::getCameraParameters(vpCameraParameters &cam) const { m_impl->getCameraParameters(cam); }

/*!
  Get moving edge parameters.
*/
void vpMbtXmlGenericParser::getEdgeMe(vpMe &ecm) const { m_impl->getEdgeMe(ecm); }

/*!
  Get depth dense sampling step in X.
*/
unsigned int vpMbtXmlGenericParser::getDepthDenseSamplingStepX() const { return m_impl->getDepthDenseSamplingStepX(); }

/*!
  Get depth dense sampling step in Y.
*/
unsigned int vpMbtXmlGenericParser::getDepthDenseSamplingStepY() const { return m_impl->getDepthDenseSamplingStepY(); }

/*!
  Get depth normal feature estimation method.
*/
vpMbtFaceDepthNormal::vpFeatureEstimationType vpMbtXmlGenericParser::getDepthNormalFeatureEstimationMethod() const
{
  return m_impl->getDepthNormalFeatureEstimationMethod();
}

/*!
  Get depth normal PCL plane estimation method.
*/
int vpMbtXmlGenericParser::getDepthNormalPclPlaneEstimationMethod() const
{
  return m_impl->getDepthNormalPclPlaneEstimationMethod();
}

/*!
  Get depth normal PCL maximum number of iterations.
*/
int vpMbtXmlGenericParser::getDepthNormalPclPlaneEstimationRansacMaxIter() const
{
  return m_impl->getDepthNormalPclPlaneEstimationRansacMaxIter();
}

/*!
  Get depth normal PCL RANSAC threshold.
*/
double vpMbtXmlGenericParser::getDepthNormalPclPlaneEstimationRansacThreshold() const
{
  return m_impl->getDepthNormalPclPlaneEstimationRansacThreshold();
}

/*!
  Get depth normal sampling step in X.
*/
unsigned int vpMbtXmlGenericParser::getDepthNormalSamplingStepX() const
{
  return m_impl->getDepthNormalSamplingStepX();
}

/*!
  Get depth normal sampling step in Y.
*/
unsigned int vpMbtXmlGenericParser::getDepthNormalSamplingStepY() const
{
  return m_impl->getDepthNormalSamplingStepY();
}

/*!
  Get the far clipping distance.
*/
double vpMbtXmlGenericParser::getFarClippingDistance() const { return m_impl->getFarClippingDistance(); }

/*!
  Get if FOV clipping should be used or not.
*/
bool vpMbtXmlGenericParser::getFovClipping() const { return m_impl->getFovClipping(); }

/*!
  Get the size of a block.
*/
unsigned int vpMbtXmlGenericParser::getKltBlockSize() const { return m_impl->getKltBlockSize(); }

/*!
  Get the Harris free parameter.
*/
double vpMbtXmlGenericParser::getKltHarrisParam() const { return m_impl->getKltHarrisParam(); }

/*!
  Get the Border of the mask.
*/
unsigned int vpMbtXmlGenericParser::getKltMaskBorder() const { return m_impl->getKltMaskBorder(); }

/*!
  Get the maximum number of features for the KLT.
*/
unsigned int vpMbtXmlGenericParser::getKltMaxFeatures() const { return m_impl->getKltMaxFeatures(); }

/*!
  Get the minimum distance between KLT points.
*/
double vpMbtXmlGenericParser::getKltMinDistance() const { return m_impl->getKltMinDistance(); }

/*!
  Get the number of pyramid levels
*/
unsigned int vpMbtXmlGenericParser::getKltPyramidLevels() const { return m_impl->getKltPyramidLevels(); }

/*!
  Get the quality of the KLT.
*/
double vpMbtXmlGenericParser::getKltQuality() const { return m_impl->getKltQuality(); }

/*!
  Get the size of the window used in the KLT tracker.
*/
unsigned int vpMbtXmlGenericParser::getKltWindowSize() const { return m_impl->getKltWindowSize(); }

/*!
  Get the state of LOD setting.
*/
bool vpMbtXmlGenericParser::getLodState() const { return m_impl->getLodState(); }

/*!
  Get the minimum line length to track a segment when LOD is enabled.
*/
double vpMbtXmlGenericParser::getLodMinLineLengthThreshold() const { return m_impl->getLodMinLineLengthThreshold(); }

/*!
  Get the minimum polygon area to track a face when LOD is enabled.
*/
double vpMbtXmlGenericParser::getLodMinPolygonAreaThreshold() const { return m_impl->getLodMinPolygonAreaThreshold(); }

/*!
  Get the near clipping distance.
*/
double vpMbtXmlGenericParser::getNearClippingDistance() const { return m_impl->getNearClippingDistance(); }

/*!
  Get ME parameters for projection error computation.
*/
void vpMbtXmlGenericParser::getProjectionErrorMe(vpMe &me) const { m_impl->getProjectionErrorMe(me); }

unsigned int vpMbtXmlGenericParser::getProjectionErrorKernelSize() const
{
  return m_impl->getProjectionErrorKernelSize();
}

/*!
  Has Far clipping been specified?

  \return True if yes, False otherwise.
*/
bool vpMbtXmlGenericParser::hasFarClippingDistance() const { return m_impl->hasFarClippingDistance(); }

/*!
  Has Near clipping been specified?

  \return True if yes, False otherwise.
*/
bool vpMbtXmlGenericParser::hasNearClippingDistance() const { return m_impl->hasNearClippingDistance(); }

/*!
  Set the angle to determine if a face appeared.

  \param aappear : New angleAppear
*/
void vpMbtXmlGenericParser::setAngleAppear(const double &aappear) { m_impl->setAngleAppear(aappear); }

/*!
  Set the angle to determine if a face disappeared.

  \param adisappear : New angleDisappear
*/
void vpMbtXmlGenericParser::setAngleDisappear(const double &adisappear) { m_impl->setAngleDisappear(adisappear); }

/*!
  Set camera parameters.

  \param cam : New camera parameters
*/
void vpMbtXmlGenericParser::setCameraParameters(const vpCameraParameters &cam) { m_impl->setCameraParameters(cam); }

/*!
  Set depth dense sampling step in X.

  \param stepX : New sampling step
*/
void vpMbtXmlGenericParser::setDepthDenseSamplingStepX(unsigned int stepX)
{
  m_impl->setDepthDenseSamplingStepX(stepX);
}

/*!
  Set depth dense sampling step in Y.

  \param stepY : New sampling step
*/
void vpMbtXmlGenericParser::setDepthDenseSamplingStepY(unsigned int stepY)
{
  m_impl->setDepthDenseSamplingStepY(stepY);
}

/*!
  Set depth normal feature estimation method.

  \param method : New feature estimation method
*/
void vpMbtXmlGenericParser::setDepthNormalFeatureEstimationMethod(
  const vpMbtFaceDepthNormal::vpFeatureEstimationType &method)
{
  m_impl->setDepthNormalFeatureEstimationMethod(method);
}

/*!
  Set depth normal PCL plane estimation method.

  \param method : New PCL plane estimation method
*/
void vpMbtXmlGenericParser::setDepthNormalPclPlaneEstimationMethod(int method)
{
  m_impl->setDepthNormalPclPlaneEstimationMethod(method);
}

/*!
  Set depth normal PCL RANSAC maximum number of iterations.

  \param maxIter : New maximum number of iterations
*/
void vpMbtXmlGenericParser::setDepthNormalPclPlaneEstimationRansacMaxIter(int maxIter)
{
  m_impl->setDepthNormalPclPlaneEstimationRansacMaxIter(maxIter);
}

/*!
  Set depth normal PCL RANSAC threshold.

  \param threshold : New RANSAC threshold
*/
void vpMbtXmlGenericParser::setDepthNormalPclPlaneEstimationRansacThreshold(double threshold)
{
  m_impl->setDepthNormalPclPlaneEstimationRansacThreshold(threshold);
}

/*!
  Set depth normal sampling step in X.

  \param stepX : New sampling step
*/
void vpMbtXmlGenericParser::setDepthNormalSamplingStepX(unsigned int stepX)
{
  m_impl->setDepthNormalSamplingStepX(stepX);
}

/*!
  Set depth normal sampling step in Y.

  \param stepY : New sampling step
*/
void vpMbtXmlGenericParser::setDepthNormalSamplingStepY(unsigned int stepY)
{
  m_impl->setDepthNormalSamplingStepY(stepY);
}

/*!
  Set moving edge parameters.

  \param moving_edge : New moving edge parameters
*/
void vpMbtXmlGenericParser::setEdgeMe(const vpMe &moving_edge) { m_impl->setEdgeMe(moving_edge); }

/*!
  Set the far clipping distance.

  \param fclip : New farClipping
*/
void vpMbtXmlGenericParser::setFarClippingDistance(const double &fclip) { m_impl->setFarClippingDistance(fclip); }

/*!
  Set the size of a block.

  \param bs : New blockSize
*/
void vpMbtXmlGenericParser::setKltBlockSize(const unsigned int &bs) { m_impl->setKltBlockSize(bs); }

/*!
  Set the Harris free parameter.

  \param hp : New harrisParam
*/
void vpMbtXmlGenericParser::setKltHarrisParam(const double &hp) { m_impl->setKltHarrisParam(hp); }

/*!
  Set the Border of the mask.

  \param mb = new maskBorder
*/
void vpMbtXmlGenericParser::setKltMaskBorder(const unsigned int &mb) { m_impl->setKltMaskBorder(mb); }

/*!
  Set the maximum number of features for the KLT.

  \param mF : New maxFeatures
*/
void vpMbtXmlGenericParser::setKltMaxFeatures(const unsigned int &mF) { m_impl->setKltMaxFeatures(mF); }

/*!
  Set the minimum distance between KLT points.

  \param mD : New minDist
*/
void vpMbtXmlGenericParser::setKltMinDistance(const double &mD) { m_impl->setKltMinDistance(mD); }

/*!
  Set the number of pyramid levels

  \param pL : New pyramidLevels
*/
void vpMbtXmlGenericParser::setKltPyramidLevels(const unsigned int &pL) { m_impl->setKltPyramidLevels(pL); }

/*!
  Set the quality of the KLT.

  \param q : New quality
*/
void vpMbtXmlGenericParser::setKltQuality(const double &q) { m_impl->setKltQuality(q); }

/*!
  Set the size of the window used in the KLT tracker.

  \param w : New winSize
*/
void vpMbtXmlGenericParser::setKltWindowSize(const unsigned int &w) { m_impl->setKltWindowSize(w); }

/*!
  Set the near clipping distance.

  \param nclip : New nearClipping
*/
void vpMbtXmlGenericParser::setNearClippingDistance(const double &nclip) { m_impl->setNearClippingDistance(nclip); }

/*!
  Set ME parameters for projection error computation.

  \param me : ME parameters
*/
void vpMbtXmlGenericParser::setProjectionErrorMe(const vpMe &me) { m_impl->setProjectionErrorMe(me); }

/*!
  Set kernel size used for projection error computation.

  \param size : Kernel size computed as kernel_size = size*2 + 1
*/
void vpMbtXmlGenericParser::setProjectionErrorKernelSize(const unsigned int &size)
{
  m_impl->setProjectionErrorKernelSize(size);
}

/*!
  Set verbose mode (print tracker configuration in the standard output if set).

  \param verbose : verbose flag
*/
void vpMbtXmlGenericParser::setVerbose(bool verbose) { m_impl->setVerbose(verbose); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpMbtXmlGenericParser.cpp.o) has no symbols
void dummy_vpMbtXmlGenericParser() { };

#endif
