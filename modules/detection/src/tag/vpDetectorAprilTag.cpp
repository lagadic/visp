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
 * Base class for April Tag detection.
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_APRILTAG
#include <map>

#include <apriltag.h>
#include <common/homography.h>
#include <tag16h5.h>
#include <tag25h7.h>
#include <tag25h9.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagStandard41h12.h>
#include <apriltag_pose.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
#endif

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpPose.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpDetectorAprilTag::Impl
{
public:
  Impl(const vpAprilTagFamily &tagFamily, const vpPoseEstimationMethod &method)
    : m_poseEstimationMethod(method), m_tagsId(), m_tagFamily(tagFamily),
      m_td(NULL), m_tf(NULL), m_detections(NULL), m_zAlignedWithCameraFrame(false)
  {
    switch (m_tagFamily) {
    case TAG_36h11:
      m_tf = tag36h11_create();
      break;

    case TAG_36h10:
      m_tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      break;

    case TAG_25h9:
      m_tf = tag25h9_create();
      break;

    case TAG_25h7:
      m_tf = tag25h7_create();
      break;

    case TAG_16h5:
      m_tf = tag16h5_create();
      break;

    case TAG_CIRCLE21h7:
      m_tf = tagCircle21h7_create();
      break;

    case TAG_CIRCLE49h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagCircle49h12_create();
#endif
      break;

    case TAG_CUSTOM48h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagCustom48h12_create();
#endif
      break;

    case TAG_STANDARD52h13:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagStandard52h13_create();
#endif
      break;

    case TAG_STANDARD41h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagStandard41h12_create();
#endif
      break;

    default:
      throw vpException(vpException::fatalError, "Unknow Tag family!");
    }

    if (m_tagFamily != TAG_36ARTOOLKIT && m_tf) {
      m_td = apriltag_detector_create();
      apriltag_detector_add_family(m_td, m_tf);
    }

    m_mapOfCorrespondingPoseMethods[DEMENTHON_VIRTUAL_VS] = vpPose::DEMENTHON;
    m_mapOfCorrespondingPoseMethods[LAGRANGE_VIRTUAL_VS] = vpPose::LAGRANGE;
  }

  Impl(const Impl &o)
    : m_poseEstimationMethod(o.m_poseEstimationMethod), m_tagsId(o.m_tagsId), m_tagFamily(o.m_tagFamily),
      m_td(NULL), m_tf(NULL), m_detections(NULL), m_zAlignedWithCameraFrame(o.m_zAlignedWithCameraFrame)
  {
    switch (m_tagFamily) {
    case TAG_36h11:
      m_tf = tag36h11_create();
      break;

    case TAG_36h10:
      m_tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      break;

    case TAG_25h9:
      m_tf = tag25h9_create();
      break;

    case TAG_25h7:
      m_tf = tag25h7_create();
      break;

    case TAG_16h5:
      m_tf = tag16h5_create();
      break;

    case TAG_CIRCLE21h7:
      m_tf = tagCircle21h7_create();
      break;

    case TAG_CIRCLE49h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagCircle49h12_create();
#endif
      break;

    case TAG_CUSTOM48h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagCustom48h12_create();
#endif
      break;

    case TAG_STANDARD52h13:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagStandard52h13_create();
#endif
      break;

    case TAG_STANDARD41h12:
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      m_tf = tagStandard41h12_create();
#endif
      break;

    default:
      throw vpException(vpException::fatalError, "Unknow Tag family!");
    }

    if (m_tagFamily != TAG_36ARTOOLKIT && m_tf) {
      m_td = apriltag_detector_create();
      apriltag_detector_add_family(m_td, m_tf);
    }

    m_mapOfCorrespondingPoseMethods[DEMENTHON_VIRTUAL_VS] = vpPose::DEMENTHON;
    m_mapOfCorrespondingPoseMethods[LAGRANGE_VIRTUAL_VS] = vpPose::LAGRANGE;

    if (o.m_detections != NULL) {
      m_detections = apriltag_detections_copy(o.m_detections);
    }
  }

  ~Impl()
  {
    if (m_td) {
      apriltag_detector_destroy(m_td);
    }

    if (m_tf) {
      switch (m_tagFamily) {
      case TAG_36h11:
        tag36h11_destroy(m_tf);
        break;

      case TAG_36h10:
        tag36h10_destroy(m_tf);
        break;

      case TAG_36ARTOOLKIT:
        break;

      case TAG_25h9:
        tag25h9_destroy(m_tf);
        break;

      case TAG_25h7:
        tag25h7_destroy(m_tf);
        break;

      case TAG_16h5:
        tag16h5_destroy(m_tf);
        break;

      case TAG_CIRCLE21h7:
        tagCircle21h7_destroy(m_tf);
        break;

      case TAG_CIRCLE49h12:
  #if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
        tagCustom48h12_destroy(m_tf);
  #endif
        break;

      case TAG_CUSTOM48h12:
  #if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
        tagCustom48h12_destroy(m_tf);
  #endif
        break;

      case TAG_STANDARD52h13:
  #if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
        tagStandard52h13_destroy(m_tf);
  #endif
        break;

      case TAG_STANDARD41h12:
  #if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
        tagStandard41h12_destroy(m_tf);
  #endif
        break;

      default:
        break;
      }
    }

    if (m_detections) {
      apriltag_detections_destroy(m_detections);
      m_detections = NULL;
    }
  }

  void convertHomogeneousMatrix(const apriltag_pose_t &pose, vpHomogeneousMatrix &cMo) {
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        cMo[i][j] = MATD_EL(pose.R, i, j);
      }
      cMo[i][3] = MATD_EL(pose.t, i, 0);
    }
  }

  bool detect(const vpImage<unsigned char> &I, double tagSize, const vpCameraParameters &cam,
              std::vector<std::vector<vpImagePoint> > &polygons,
              std::vector<std::string> &messages, bool displayTag, const vpColor color,
              unsigned int thickness, std::vector<vpHomogeneousMatrix> *cMo_vec,
              std::vector<vpHomogeneousMatrix> *cMo_vec2, std::vector<double> *projErrors,
              std::vector<double> *projErrors2)
  {
    if (m_tagFamily == TAG_36ARTOOLKIT) {
      //TAG_36ARTOOLKIT is not available anymore
      std::cerr << "TAG_36ARTOOLKIT detector is not available anymore." << std::endl;
      return false;
    }
#if !defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
    if (m_tagFamily == TAG_CIRCLE49h12 || m_tagFamily == TAG_CUSTOM48h12 || m_tagFamily == TAG_STANDARD41h12 || m_tagFamily == TAG_STANDARD52h13) {
      std::cerr << "TAG_CIRCLE49h12, TAG_CUSTOM48h12, TAG_STANDARD41h12 and TAG_STANDARD52h13 are disabled." << std::endl;
      return false;
    }
#endif

    const bool computePose = (cMo_vec != NULL);

    image_u8_t im = {/*.width =*/(int32_t)I.getWidth(),
                     /*.height =*/(int32_t)I.getHeight(),
                     /*.stride =*/(int32_t)I.getWidth(),
                     /*.buf =*/I.bitmap};

    if (m_detections) {
      apriltag_detections_destroy(m_detections);
      m_detections = NULL;
    }

    m_detections = apriltag_detector_detect(m_td, &im);
    int nb_detections = zarray_size(m_detections);
    bool detected = nb_detections > 0;

    polygons.resize(static_cast<size_t>(nb_detections));
    messages.resize(static_cast<size_t>(nb_detections));
    m_tagsId.resize(static_cast<size_t>(nb_detections));

    for (int i = 0; i < zarray_size(m_detections); i++) {
      apriltag_detection_t *det;
      zarray_get(m_detections, i, &det);

      std::vector<vpImagePoint> polygon;
      for (int j = 0; j < 4; j++) {
        polygon.push_back(vpImagePoint(det->p[j][1], det->p[j][0]));
      }
      polygons[static_cast<size_t>(i)] = polygon;
      std::stringstream ss;
      ss << m_tagFamily << " id: " << det->id;
      messages[static_cast<size_t>(i)] = ss.str();
      m_tagsId[static_cast<size_t>(i)] = det->id;

      if (displayTag) {
        vpColor Ox = (color == vpColor::none) ? vpColor::red : color;
        vpColor Oy = (color == vpColor::none) ? vpColor::green : color;
        vpColor Ox2 = (color == vpColor::none) ? vpColor::yellow : color;
        vpColor Oy2 = (color == vpColor::none) ? vpColor::blue : color;

        vpDisplay::displayLine(I, (int)det->p[0][1], (int)det->p[0][0], (int)det->p[1][1], (int)det->p[1][0],
                               Ox, thickness);
        vpDisplay::displayLine(I, (int)det->p[0][1], (int)det->p[0][0], (int)det->p[3][1], (int)det->p[3][0],
                               Oy, thickness);
        vpDisplay::displayLine(I, (int)det->p[1][1], (int)det->p[1][0], (int)det->p[2][1], (int)det->p[2][0],
                               Ox2, thickness);
        vpDisplay::displayLine(I, (int)det->p[2][1], (int)det->p[2][0], (int)det->p[3][1], (int)det->p[3][0],
                               Oy2, thickness);
      }

      if (computePose) {
        vpHomogeneousMatrix cMo, cMo2;
        double err1, err2;
        if (getPose(static_cast<size_t>(i), tagSize, cam, cMo, cMo_vec2 ? &cMo2 : NULL,
                    projErrors ? &err1 : NULL, projErrors2 ? &err2 : NULL)) {
          cMo_vec->push_back(cMo);
          if (cMo_vec2) {
            cMo_vec2->push_back(cMo2);
          }
          if (projErrors) {
            projErrors->push_back(err1);
          }
          if (projErrors2) {
            projErrors2->push_back(err2);
          }
        }
        // else case should never happen
      }
    }

    return detected;
  }

  bool getPose(size_t tagIndex, double tagSize, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, vpHomogeneousMatrix *cMo2,
               double *projErrors, double *projErrors2) {
    if (m_detections == NULL) {
      throw(vpException(vpException::fatalError, "Cannot get tag index=%d pose: detection empty", tagIndex));
    }
    if (m_tagFamily == TAG_36ARTOOLKIT) {
      //TAG_36ARTOOLKIT is not available anymore
      std::cerr << "TAG_36ARTOOLKIT detector is not available anymore." << std::endl;
      return  false;
    }
#if !defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
    if (m_tagFamily == TAG_CIRCLE49h12 || m_tagFamily == TAG_CUSTOM48h12 || m_tagFamily == TAG_STANDARD41h12 || m_tagFamily == TAG_STANDARD52h13) {
      std::cerr << "TAG_CIRCLE49h12, TAG_CUSTOM48h12, TAG_STANDARD41h12 and TAG_STANDARD52h13 are disabled." << std::endl;
      return false;
    }
#endif

    apriltag_detection_t *det;
    zarray_get(m_detections, static_cast<int>(tagIndex), &det);

    int nb_detections = zarray_size(m_detections);
    if (tagIndex >= (size_t)nb_detections) {
      return false;
    }

    //In AprilTag3, estimate_pose_for_tag_homography() and estimate_tag_pose() have been added.
    //They use a tag frame aligned with the camera frame
    //Before the release of AprilTag3, convention used was to define the z-axis of the tag going upward.
    //To keep compatibility, we maintain the same convention than before and there is setZAlignedWithCameraAxis().
    //Under the hood, we use aligned frames everywhere and transform the pose according to the option.

    vpHomogeneousMatrix cMo_homography_ortho_iter;
    if (m_poseEstimationMethod == HOMOGRAPHY_ORTHOGONAL_ITERATION ||
        m_poseEstimationMethod == BEST_RESIDUAL_VIRTUAL_VS) {
      double fx = cam.get_px(), fy = cam.get_py();
      double cx = cam.get_u0(), cy = cam.get_v0();

      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = tagSize;
      info.fx = fx;
      info.fy = fy;
      info.cx = cx;
      info.cy = cy;

      //projErrors and projErrors2 will be override later
      getPoseWithOrthogonalMethod(info, cMo, cMo2, projErrors, projErrors2);
      cMo_homography_ortho_iter = cMo;
    }

    vpHomogeneousMatrix cMo_homography;
    if (m_poseEstimationMethod == HOMOGRAPHY || m_poseEstimationMethod == HOMOGRAPHY_VIRTUAL_VS ||
        m_poseEstimationMethod == BEST_RESIDUAL_VIRTUAL_VS) {
      double fx = cam.get_px(), fy = cam.get_py();
      double cx = cam.get_u0(), cy = cam.get_v0();

      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = tagSize;
      info.fx = fx;
      info.fy = fy;
      info.cx = cx;
      info.cy = cy;

      apriltag_pose_t pose;
      estimate_pose_for_tag_homography(&info, &pose);
      convertHomogeneousMatrix(pose, cMo);

      matd_destroy(pose.R);
      matd_destroy(pose.t);

      cMo_homography = cMo;
    }

    // Add marker object points
    vpPose pose;
    vpPoint pt;

    vpImagePoint imPt;
    double x = 0.0, y = 0.0;
    std::vector<vpPoint> pts(4);
    pt.setWorldCoordinates(-tagSize / 2.0, tagSize / 2.0, 0.0);
    imPt.set_uv(det->p[0][0], det->p[0][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[0] = pt;

    pt.setWorldCoordinates(tagSize / 2.0, tagSize / 2.0, 0.0);
    imPt.set_uv(det->p[1][0], det->p[1][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[1] = pt;

    pt.setWorldCoordinates(tagSize / 2.0, -tagSize / 2.0, 0.0);
    imPt.set_uv(det->p[2][0], det->p[2][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[2] = pt;

    pt.setWorldCoordinates(-tagSize / 2.0, -tagSize / 2.0, 0.0);
    imPt.set_uv(det->p[3][0], det->p[3][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[3] = pt;

    pose.addPoints(pts);

    if (m_poseEstimationMethod != HOMOGRAPHY && m_poseEstimationMethod != HOMOGRAPHY_VIRTUAL_VS &&
        m_poseEstimationMethod != HOMOGRAPHY_ORTHOGONAL_ITERATION) {
      if (m_poseEstimationMethod == BEST_RESIDUAL_VIRTUAL_VS) {
        vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;

        double residual_dementhon = std::numeric_limits<double>::max(),
               residual_lagrange = std::numeric_limits<double>::max();
        double residual_homography = pose.computeResidual(cMo_homography);
        double residual_homography_ortho_iter = pose.computeResidual(cMo_homography_ortho_iter);

        if (pose.computePose(vpPose::DEMENTHON, cMo_dementhon)) {
          residual_dementhon = pose.computeResidual(cMo_dementhon);
        }

        if (pose.computePose(vpPose::LAGRANGE, cMo_lagrange)) {
          residual_lagrange = pose.computeResidual(cMo_lagrange);
        }

        std::vector<double> residuals;
        residuals.push_back(residual_dementhon);
        residuals.push_back(residual_lagrange);
        residuals.push_back(residual_homography);
        residuals.push_back(residual_homography_ortho_iter);
        std::vector<vpHomogeneousMatrix> poses;
        poses.push_back(cMo_dementhon);
        poses.push_back(cMo_lagrange);
        poses.push_back(cMo_homography);
        poses.push_back(cMo_homography_ortho_iter);

        std::ptrdiff_t minIndex = std::min_element(residuals.begin(), residuals.end()) - residuals.begin();
        cMo = *(poses.begin() + minIndex);
      } else {
        pose.computePose(m_mapOfCorrespondingPoseMethods[m_poseEstimationMethod], cMo);
      }
    }

    if (m_poseEstimationMethod != HOMOGRAPHY &&
        m_poseEstimationMethod != HOMOGRAPHY_ORTHOGONAL_ITERATION) {
      // Compute final pose using VVS
      pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }

    //Only with HOMOGRAPHY_ORTHOGONAL_ITERATION we can directly get two solutions
    if (m_poseEstimationMethod != HOMOGRAPHY_ORTHOGONAL_ITERATION) {
      if (cMo2) {
        double scale = tagSize/2.0;
        double data_p0[] = {-scale, scale, 0};
        double data_p1[] = {scale, scale, 0};
        double data_p2[] = {scale, -scale, 0};
        double data_p3[] = {-scale, -scale, 0};
        matd_t* p[4] = {matd_create_data(3, 1, data_p0),
                        matd_create_data(3, 1, data_p1),
                        matd_create_data(3, 1, data_p2),
                        matd_create_data(3, 1, data_p3)};
        matd_t* v[4];
        for (int i = 0; i < 4; i++) {
          double data_v[] = {(det->p[i][0] - cam.get_u0())/cam.get_px(), (det->p[i][1] - cam.get_v0())/cam.get_py(), 1};
          v[i] = matd_create_data(3, 1, data_v);
        }

        apriltag_pose_t solution1, solution2;
        const int nIters = 50;
        solution1.R = matd_create_data(3, 3, cMo.getRotationMatrix().data);
        solution1.t = matd_create_data(3, 1, cMo.getTranslationVector().data);

        double err2;
        get_second_solution(v, p, &solution1, &solution2, nIters, &err2);

        for (int i = 0; i < 4; i++) {
          matd_destroy(p[i]);
          matd_destroy(v[i]);
        }

        if (solution2.R) {
          convertHomogeneousMatrix(solution2, *cMo2);

          matd_destroy(solution2.R);
          matd_destroy(solution2.t);
        }

        matd_destroy(solution1.R);
        matd_destroy(solution1.t);
      }
    }

    //Compute projection error with vpPose::computeResidual() for consistency
    if (projErrors) {
      *projErrors = pose.computeResidual(cMo);
    }
    if (projErrors2 && cMo2) {
      *projErrors2 = pose.computeResidual(*cMo2);
    }

    if (!m_zAlignedWithCameraFrame) {
      vpHomogeneousMatrix oMo;
      // Apply a rotation of 180deg around x axis
      oMo[0][0] = 1; oMo[0][1] =  0; oMo[0][2] = 0;
      oMo[1][0] = 0; oMo[1][1] = -1; oMo[1][2] = 0;
      oMo[2][0] = 0; oMo[2][1] =  0; oMo[2][2] = -1;
      cMo = cMo*oMo;
      if (cMo2) {
        *cMo2 = *cMo2*oMo;
      }
    }

    return true;
  }

  void getPoseWithOrthogonalMethod(apriltag_detection_info_t &info, vpHomogeneousMatrix &cMo1, vpHomogeneousMatrix *cMo2,
                                   double *err1, double *err2) {
    apriltag_pose_t pose1, pose2;
    double err_1, err_2;
    estimate_tag_pose_orthogonal_iteration(&info, &err_1, &pose1, &err_2, &pose2, 50);
    if (err_1 <= err_2) {
      convertHomogeneousMatrix(pose1, cMo1);
      if (cMo2) {
        if (pose2.R) {
          convertHomogeneousMatrix(pose2, *cMo2);
        } else {
          *cMo2 = cMo1;
        }
      }
    } else {
      convertHomogeneousMatrix(pose2, cMo1);
      if (cMo2) {
        convertHomogeneousMatrix(pose1, *cMo2);
      }
    }

    matd_destroy(pose1.R);
    matd_destroy(pose1.t);
    if (pose2.R) {
      matd_destroy(pose2.t);
    }
    matd_destroy(pose2.R);

    if (err1)
      *err1 = err_1;
    if (err2)
      *err2 = err_2;
  }

  bool getZAlignedWithCameraAxis() { return m_zAlignedWithCameraFrame; }

  bool getAprilTagDecodeSharpening(double &decodeSharpening) const {
    if (m_td) {
      decodeSharpening = m_td->decode_sharpening;
      return true;
    }
    return false;
  }

  bool getNbThreads(int &nThreads) const {
    if (m_td) {
      nThreads = m_td->nthreads;
      return true;
    }
    return false;
  }

  bool getQuadDecimate(float &quadDecimate) const {
    if (m_td) {
      quadDecimate = m_td->quad_decimate;
      return true;
    }
    return false;
  }
  
  bool getQuadSigma(float &quadSigma) const {
    if (m_td) {
      quadSigma = m_td->quad_sigma;
      return true;
    }
    return false;
  }

  bool getRefineEdges(bool &refineEdges) const {
    if (m_td) {
      refineEdges = m_td->refine_edges;
      return true;
    }
    return false;
  }

  bool getZAlignedWithCameraAxis() const {
    return m_zAlignedWithCameraFrame;
  }

  std::vector<int> getTagsId() const { return m_tagsId; }

  void setAprilTagDecodeSharpening(double decodeSharpening) {
    if (m_td) {
      m_td->decode_sharpening = decodeSharpening;
    }
  }

  void setNbThreads(int nThreads) {
    if (m_td) {
      m_td->nthreads = nThreads;
    }
  }

  void setQuadDecimate(float quadDecimate) {
    if (m_td) {
      m_td->quad_decimate = quadDecimate;
    }
  }

  void setQuadSigma(float quadSigma) {
    if (m_td) {
      m_td->quad_sigma = quadSigma;
    }
  }

  void setRefineDecode(bool) { }

  void setRefineEdges(bool refineEdges) {
    if (m_td) {
      m_td->refine_edges = refineEdges ? 1 : 0;
    }
  }

  void setRefinePose(bool) { }

  void setPoseEstimationMethod(const vpPoseEstimationMethod &method) { m_poseEstimationMethod = method; }

  void setZAlignedWithCameraAxis(bool zAlignedWithCameraFrame) { m_zAlignedWithCameraFrame = zAlignedWithCameraFrame; }

protected:
  std::map<vpPoseEstimationMethod, vpPose::vpPoseMethodType> m_mapOfCorrespondingPoseMethods;
  vpPoseEstimationMethod m_poseEstimationMethod;
  std::vector<int> m_tagsId;
  vpAprilTagFamily m_tagFamily;
  apriltag_detector_t *m_td;
  apriltag_family_t *m_tf;
  zarray_t *m_detections;
  bool m_zAlignedWithCameraFrame;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpDetectorAprilTag::vpDetectorAprilTag(const vpAprilTagFamily &tagFamily,
                                       const vpPoseEstimationMethod &poseEstimationMethod)
  : m_displayTag(false), m_displayTagColor(vpColor::none), m_displayTagThickness(2),
    m_poseEstimationMethod(poseEstimationMethod), m_tagFamily(tagFamily), m_defaultCam(),
    m_impl(new Impl(tagFamily, poseEstimationMethod))
{
}

vpDetectorAprilTag::vpDetectorAprilTag(const vpDetectorAprilTag &o)
  : vpDetectorBase (o),
    m_displayTag(false), m_displayTagColor(vpColor::none), m_displayTagThickness(2),
    m_poseEstimationMethod(o.m_poseEstimationMethod), m_tagFamily(o.m_tagFamily), m_defaultCam(),
    m_impl(new Impl(*o.m_impl))
{
}

vpDetectorAprilTag &vpDetectorAprilTag::operator=(vpDetectorAprilTag o)
{
  swap(*this, o);
  return *this;
}

vpDetectorAprilTag::~vpDetectorAprilTag() { delete m_impl; }

/*!
  Detect AprilTag tags in the image. Return true if at least one tag is
  detected, false otherwise.

  \param I : Input image.
  \return true if at least one tag is detected.
*/
bool vpDetectorAprilTag::detect(const vpImage<unsigned char> &I)
{
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  std::vector<vpHomogeneousMatrix> cMo_vec;
  const double tagSize = 1.0;
  bool detected = m_impl->detect(I, tagSize, m_defaultCam, m_polygon, m_message, m_displayTag,
                                 m_displayTagColor, m_displayTagThickness,
                                 NULL, NULL, NULL, NULL);
  m_nb_objects = m_message.size();

  return detected;
}

/*!
  Detect AprilTag tags in the image and compute the corresponding tag poses considering that all the tags have
  the same size.

  If tags with different sizes have to be considered, you may use getPose().

  \param[in] I : Input image.
  \param[in] tagSize : Tag size in meter corresponding to the external width of the pattern.
  \param[in] cam : Camera intrinsic parameters.
  \param[out] cMo_vec : List of tag poses.
  \param[out] cMo_vec2 : Optional second list of tag poses, since there are 2 solutions for planar pose estimation.
  \param[out] projErrors : Optional (sum of squared) projection errors in the normalized camera frame.
  \param[out] projErrors2 : Optional (sum of squared) projection errors for the 2nd solution in the normalized camera frame.
  \return true if at least one tag is detected.

  \sa getPose()
*/
bool vpDetectorAprilTag::detect(const vpImage<unsigned char> &I, double tagSize, const vpCameraParameters &cam,
                                std::vector<vpHomogeneousMatrix> &cMo_vec, std::vector<vpHomogeneousMatrix> *cMo_vec2,
                                std::vector<double> *projErrors, std::vector<double> *projErrors2)
{
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  bool detected = m_impl->detect(I, tagSize, cam, m_polygon, m_message, m_displayTag,
                                 m_displayTagColor, m_displayTagThickness,
                                 &cMo_vec, cMo_vec2, projErrors, projErrors2);
  m_nb_objects = m_message.size();

  return detected;
}

/*!
  Get the pose of a tag depending on its size and camera parameters.
  This function is useful to get the pose of tags with different sizes, while
  detect(const vpImage<unsigned char> &, const double, const vpCameraParameters &, std::vector<vpHomogeneousMatrix> &)
  considers that all the tags have the same size.

  \param[in] tagIndex : Index of the tag. Value should be in range [0, nb tags-1] with nb_tags = getNbObjects().
  Note that this index doesn't correspond to the tag id.
  \param[in] tagSize : Tag size in meter corresponding to the external width of the pattern.
  \param[in] cam : Camera intrinsic parameters.
  \param[out] cMo : Pose of the tag.
  \param[out] cMo2 : Optional second pose of the tag.
  \param[out] projError : Optional (sum of squared) projection errors in the normalized camera frame.
  \param[out] projError2 : Optional (sum of squared) projection errors for the 2nd solution in the normalized camera frame.
  \return true if success, false otherwise.

  The following code shows how to use this function:
  \code
  vpCameraParameters cam;
  vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
  detector.detect(I);
  for (size_t i = 0; i < detector.getNbObjects(); i++) {
    vpHomogeneousMatrix cMo;
    double tagSize = 0.1;
    detector.getPose(i, tagSize, cam, cMo);
  }
  \endcode

  \sa detect(const vpImage<unsigned char> &, double, const vpCameraParameters &, std::vector<vpHomogeneousMatrix> *,
             std::vector<vpHomogeneousMatrix> *, std::vector<double> *, std::vector<double> *)
 */
bool vpDetectorAprilTag::getPose(size_t tagIndex, double tagSize, const vpCameraParameters &cam,
                                 vpHomogeneousMatrix &cMo, vpHomogeneousMatrix *cMo2,
                                 double *projError, double *projError2)
{
  return m_impl->getPose(tagIndex, tagSize, cam, cMo, cMo2, projError, projError2);
}

/*!
  Return a vector that contains for each tag id the corresponding tag 3D corners coordinates in the tag frame.

  \param tagsId : A vector containing the id of each tag that is detected. It's size corresponds
  to the number of tags that are detected. This vector is returned by getTagsId().

  \param tagsSize : a map that contains as first element a tag id and as second elements its 3D size in meter.
  When first element of this map is -1, the second element corresponds to the default tag size.
  \code
  std::map<int, double> tagsSize;
  tagsSize[-1] = 0.05; // Default tag size in meter, used when detected tag id is not in this map
  tagsSize[10] = 0.1;  // All tags with id 10 are 0.1 meter large
  tagsSize[20] = 0.2;  // All tags with id 20 are 0.2 meter large
  \endcode

  \sa getTagsCorners(), getTagsId()
 */
std::vector<std::vector<vpPoint> > vpDetectorAprilTag::getTagsPoints3D(const std::vector<int>& tagsId,
                                                                       const std::map<int, double>& tagsSize) const
{
  std::vector<std::vector<vpPoint> > tagsPoints3D;

  double default_size = -1;
  {
    std::map<int, double>::const_iterator it = tagsSize.find(-1);
    if (it != tagsSize.end()) {
      default_size = it->second; // Default size
    }
  }
  for (size_t i = 0; i < tagsId.size(); i++) {
    std::map<int, double>::const_iterator it = tagsSize.find(tagsId[i]);
    double tagSize = default_size; // Default size
    if (it == tagsSize.end()) {
      if (default_size < 0) { // no default size found
        throw(vpException(vpException::fatalError, "Tag with id %d has no 3D size or there is no default 3D size defined", tagsId[i]));
      }
    } else {
      tagSize = it->second;
    }
    std::vector<vpPoint> points3D(4);
    if (m_impl->getZAlignedWithCameraAxis()) {
      points3D[0] = vpPoint(-tagSize/2,  tagSize/2, 0);
      points3D[1] = vpPoint( tagSize/2,  tagSize/2, 0);
      points3D[2] = vpPoint( tagSize/2, -tagSize/2, 0);
      points3D[3] = vpPoint(-tagSize/2, -tagSize/2, 0);
    } else {
      points3D[0] = vpPoint(-tagSize/2, -tagSize/2, 0);
      points3D[1] = vpPoint( tagSize/2, -tagSize/2, 0);
      points3D[2] = vpPoint( tagSize/2,  tagSize/2, 0);
      points3D[3] = vpPoint(-tagSize/2,  tagSize/2, 0);
    }
    tagsPoints3D.push_back(points3D);
  }

  return tagsPoints3D;
}

/*!
  Return the corners coordinates for the detected tags.

  \sa getTagsId(), getTagsPoints3D()
*/
std::vector<std::vector<vpImagePoint> > vpDetectorAprilTag::getTagsCorners() const
{
  return m_polygon;
}

/*!
  Return the decoded Apriltag id for each detection.

  \sa getTagsCorners(), getTagsPoints3D()
*/
std::vector<int> vpDetectorAprilTag::getTagsId() const
{
  return m_impl->getTagsId();
}

void vpDetectorAprilTag::setAprilTagDecodeSharpening(double decodeSharpening)
{
  return m_impl->setAprilTagDecodeSharpening(decodeSharpening);
}

void vpDetectorAprilTag::setAprilTagFamily(const vpAprilTagFamily &tagFamily)
{
  //back-up settings
  double decodeSharpening = 0.25;
  m_impl->getAprilTagDecodeSharpening(decodeSharpening);
  int nThreads = 1;
  m_impl->getNbThreads(nThreads);
  float quadDecimate = 1;
  m_impl->getQuadDecimate(quadDecimate);
  float quadSigma = 0;
  m_impl->getQuadSigma(quadSigma);
  bool refineEdges = true;
  m_impl->getRefineEdges(refineEdges);
  bool zAxis = m_impl->getZAlignedWithCameraAxis();

  delete m_impl;
  m_impl = new Impl(tagFamily, m_poseEstimationMethod);
  m_impl->setAprilTagDecodeSharpening(decodeSharpening);
  m_impl->setNbThreads(nThreads);
  m_impl->setQuadDecimate(quadDecimate);
  m_impl->setQuadSigma(quadSigma);
  m_impl->setRefineEdges(refineEdges);
  m_impl->setZAlignedWithCameraAxis(zAxis);
}

/*!
  Set the number of threads for April Tag detection (default is 1).

  \param nThreads : Number of thread.
*/
void vpDetectorAprilTag::setAprilTagNbThreads(int nThreads)
{
  if (nThreads > 0) {
    m_impl->setNbThreads(nThreads);
  }
}

/*!
  Set the method to use to compute the pose, \see vpPoseEstimationMethod

  \param poseEstimationMethod : The method to used.
*/
void vpDetectorAprilTag::setAprilTagPoseEstimationMethod(const vpPoseEstimationMethod &poseEstimationMethod)
{
  m_poseEstimationMethod = poseEstimationMethod;
  m_impl->setPoseEstimationMethod(poseEstimationMethod);
}

/*!
  From the AprilTag code:
  <blockquote>
  detection of quads can be done on a lower-resolution image,
  improving speed at a cost of pose accuracy and a slight
  decrease in detection rate. Decoding the binary payload is
  still done at full resolution.
  </blockquote>
  Default is 1.0, increase this value to reduce the computation time.

  \param quadDecimate : Value for quad_decimate.
*/
void vpDetectorAprilTag::setAprilTagQuadDecimate(float quadDecimate)
{
  m_impl->setQuadDecimate(quadDecimate);
}

/*!
  From the AprilTag code:
  <blockquote>
  What Gaussian blur should be applied to the segmented image
  (used for quad detection?)  Parameter is the standard deviation
  in pixels.  Very noisy images benefit from non-zero values
  (e.g. 0.8).
  </blockquote>
  Default is 0.0.

  \param quadSigma : Value for quad_sigma.
*/
void vpDetectorAprilTag::setAprilTagQuadSigma(float quadSigma)
{
  m_impl->setQuadSigma(quadSigma);
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  Deprecated parameter from AprilTag 2 version.
*/
vp_deprecated void vpDetectorAprilTag::setAprilTagRefineDecode(bool refineDecode) {
  m_impl->setRefineDecode(refineDecode);
}
#endif

/*!
  From the AprilTag code:
  <blockquote>
  When non-zero, the edges of the each quad are adjusted to "snap
  to" strong gradients nearby. This is useful when decimation is
  employed, as it can increase the quality of the initial quad
  estimate substantially. Generally recommended to be on (1).
  Very computationally inexpensive. Option is ignored if
  quad_decimate = 1.
  </blockquote>
  Default is 1.

  \param refineEdges : If true, set refine_edges to 1.
*/
void vpDetectorAprilTag::setAprilTagRefineEdges(bool refineEdges)
{
  m_impl->setRefineEdges(refineEdges);
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  Deprecated parameter from AprilTag 2 version.
*/
vp_deprecated void vpDetectorAprilTag::setAprilTagRefinePose(bool refinePose)
{
  m_impl->setRefinePose(refinePose);
}
#endif

void swap(vpDetectorAprilTag &o1, vpDetectorAprilTag &o2)
{
  using std::swap;

  swap(o1.m_impl, o2.m_impl);
}

/*!
 * Modify the resulting tag pose returned by getPose() in order to get
 * a pose where z-axis is aligned when the camera plane is parallel to the tag.
 * \param zAlignedWithCameraFrame : Flag to get a pose where z-axis is aligned with the camera frame.
 */
void vpDetectorAprilTag::setZAlignedWithCameraAxis(bool zAlignedWithCameraFrame)
{
  m_impl->setZAlignedWithCameraAxis(zAlignedWithCameraFrame);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorAprilTag.cpp.o) has
// no symbols
void dummy_vpDetectorAprilTag() {}
#endif
