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
#include <tag36artoolkit.h>
#include <tag36h10.h>
#include <tag36h11.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/vision/vpPose.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpDetectorAprilTag::Impl
{
public:
  Impl(const vpAprilTagFamily &tagFamily, const vpPoseEstimationMethod &method)
    : m_cam(), m_poseEstimationMethod(method), m_tagFamily(tagFamily), m_tagPoses(), m_tagSize(1.0), m_td(NULL),
      m_tf(NULL), m_detections(NULL), m_zAlignedWithCameraFrame(false)
  {
    switch (m_tagFamily) {
    case TAG_36h11:
      m_tf = tag36h11_create();
      break;

    case TAG_36h10:
      m_tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      m_tf = tag36artoolkit_create();
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

    default:
      throw vpException(vpException::fatalError, "Unknow Tag family!");
      break;
    }

    m_td = apriltag_detector_create();
    apriltag_detector_add_family(m_td, m_tf);

    m_mapOfCorrespondingPoseMethods[DEMENTHON_VIRTUAL_VS] = vpPose::DEMENTHON;
    m_mapOfCorrespondingPoseMethods[LAGRANGE_VIRTUAL_VS] = vpPose::LAGRANGE;
  }

  ~Impl()
  {
    apriltag_detector_destroy(m_td);

    switch (m_tagFamily) {
    case TAG_36h11:
      tag36h11_destroy(m_tf);
      break;

    case TAG_36h10:
      tag36h10_destroy(m_tf);
      break;

    case TAG_36ARTOOLKIT:
      tag36artoolkit_destroy(m_tf);
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

    default:
      break;
    }

    if (m_detections) {
      apriltag_detections_destroy(m_detections);
      m_detections = NULL;
    }
  }

  bool detect(const vpImage<unsigned char> &I, std::vector<std::vector<vpImagePoint> > &polygons,
              std::vector<std::string> &messages, const bool computePose, const bool displayTag,
              const vpColor color, const unsigned int thickness)
  {
    std::vector<vpHomogeneousMatrix> tagPosesPrev = m_tagPoses;
    m_tagPoses.clear();

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

    polygons.resize((size_t)nb_detections);
    messages.resize((size_t)nb_detections);

    for (int i = 0; i < zarray_size(m_detections); i++) {
      apriltag_detection_t *det;
      zarray_get(m_detections, i, &det);

      std::vector<vpImagePoint> polygon;
      for (int j = 0; j < 4; j++) {
        polygon.push_back(vpImagePoint(det->p[j][1], det->p[j][0]));
      }
      polygons[i] = polygon;
      std::stringstream ss;
      ss << m_tagFamily << " id: " << det->id;
      messages[i] = ss.str();

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
        vpHomogeneousMatrix cMo;
        {
          if ((int)tagPosesPrev.size() > i) {
            cMo = tagPosesPrev[i];
          }
        }
        if (getPose(i, m_tagSize, m_cam, cMo)) {
          m_tagPoses.push_back(cMo);
        }
        // else case should never happen
      }
    }

    return detected;
  }

  bool getPose(size_t tagIndex, const double tagSize, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo) {
    if (m_detections == NULL) {
      throw(vpException(vpException::fatalError, "Cannot get tag index=%d pose: detection empty", tagIndex));
    }
    apriltag_detection_t *det;
    zarray_get(m_detections, static_cast<int>(tagIndex), &det);

    int nb_detections = zarray_size(m_detections);
    if (tagIndex >= (size_t)nb_detections) {
      return false;
    }

    if (m_poseEstimationMethod == HOMOGRAPHY || m_poseEstimationMethod == HOMOGRAPHY_VIRTUAL_VS
        || m_poseEstimationMethod == BEST_RESIDUAL_VIRTUAL_VS) {
      double fx = cam.get_px(), fy = cam.get_py();
      double cx = cam.get_u0(), cy = cam.get_v0();

      matd_t *M = homography_to_pose(det->H, fx, fy, cx, cy, tagSize / 2);

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          cMo[i][j] = MATD_EL(M, i, j);
        }
        cMo[i][3] = MATD_EL(M, i, 3);
      }

      matd_destroy(M);

      if (m_zAlignedWithCameraFrame) {
        vpHomogeneousMatrix oMo;
        // Apply a rotation of 180deg around x axis
        oMo[0][0] = 1; oMo[0][1] =  0; oMo[0][2] = 0;
        oMo[1][0] = 0; oMo[1][1] = -1; oMo[1][2] = 0;
        oMo[2][0] = 0; oMo[2][1] =  0; oMo[2][2] = -1;
        cMo = cMo*oMo;
      }
    }

    // Add marker object points
    vpPose pose;
    if (m_poseEstimationMethod != HOMOGRAPHY) {
      vpPoint pt;

      vpImagePoint imPt;
      double x = 0.0, y = 0.0;
      std::vector<vpPoint> pts(4);
      if (m_zAlignedWithCameraFrame) {
        pt.setWorldCoordinates(-tagSize / 2.0, tagSize / 2.0, 0.0);
      }
      else {
        pt.setWorldCoordinates(-tagSize / 2.0, -tagSize / 2.0, 0.0);
      }
      imPt.set_uv(det->p[0][0], det->p[0][1]);
      vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
      pt.set_x(x);
      pt.set_y(y);
      pts[0] = pt;

      if (m_zAlignedWithCameraFrame) {
        pt.setWorldCoordinates(tagSize / 2.0, tagSize / 2.0, 0.0);
      }
      else {
        pt.setWorldCoordinates(tagSize / 2.0, -tagSize / 2.0, 0.0);
      }
      imPt.set_uv(det->p[1][0], det->p[1][1]);
      vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
      pt.set_x(x);
      pt.set_y(y);
      pts[1] = pt;

      if (m_zAlignedWithCameraFrame) {
        pt.setWorldCoordinates(tagSize / 2.0, -tagSize / 2.0, 0.0);
      }
      else {
        pt.setWorldCoordinates(tagSize / 2.0, tagSize / 2.0, 0.0);
      }
      imPt.set_uv(det->p[2][0], det->p[2][1]);
      vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
      pt.set_x(x);
      pt.set_y(y);
      pts[2] = pt;

      if (m_zAlignedWithCameraFrame) {
        pt.setWorldCoordinates(-tagSize / 2.0, -tagSize / 2.0, 0.0);
      }
      else {
        pt.setWorldCoordinates(-tagSize / 2.0, tagSize / 2.0, 0.0);
      }
      imPt.set_uv(det->p[3][0], det->p[3][1]);
      vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
      pt.set_x(x);
      pt.set_y(y);
      pts[3] = pt;

      pose.addPoints(pts);
    }

    if (m_poseEstimationMethod != HOMOGRAPHY && m_poseEstimationMethod != HOMOGRAPHY_VIRTUAL_VS) {
      if (m_poseEstimationMethod == BEST_RESIDUAL_VIRTUAL_VS) {
        vpHomogeneousMatrix cMo_dementhon, cMo_lagrange, cMo_homography = cMo;

        double residual_dementhon = std::numeric_limits<double>::max(),
               residual_lagrange = std::numeric_limits<double>::max();
        double residual_homography = pose.computeResidual(cMo_homography);

        if (pose.computePose(vpPose::DEMENTHON, cMo_dementhon)) {
          residual_dementhon = pose.computeResidual(cMo_dementhon);
        }

        if (pose.computePose(vpPose::LAGRANGE, cMo_lagrange)) {
          residual_lagrange = pose.computeResidual(cMo_lagrange);
        }

        if (residual_dementhon < residual_lagrange) {
          if (residual_dementhon < residual_homography) {
            cMo = cMo_dementhon;
          } else {
            cMo = cMo_homography;
          }
        } else if (residual_lagrange < residual_homography) {
          cMo = cMo_lagrange;
        } else {
          //              cMo = cMo_homography; //already the case
        }
      } else {
        pose.computePose(m_mapOfCorrespondingPoseMethods[m_poseEstimationMethod], cMo);
      }
    }

    if (m_poseEstimationMethod != HOMOGRAPHY) {
      // Compute final pose using VVS
      pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }

    return true;
  }

  void getTagPoses(std::vector<vpHomogeneousMatrix> &tagPoses) const { tagPoses = m_tagPoses; }

  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }

  void setNbThreads(const int nThreads) { m_td->nthreads = nThreads; }

  void setQuadDecimate(const float quadDecimate) { m_td->quad_decimate = quadDecimate; }

  void setQuadSigma(const float quadSigma) { m_td->quad_sigma = quadSigma; }

  void setRefineDecode(const bool refineDecode) { m_td->refine_decode = refineDecode ? 1 : 0; }

  void setRefineEdges(const bool refineEdges) { m_td->refine_edges = refineEdges ? 1 : 0; }

  void setRefinePose(const bool refinePose) { m_td->refine_pose = refinePose ? 1 : 0; }

  void setTagSize(const double tagSize) { m_tagSize = tagSize; }

  void setPoseEstimationMethod(const vpPoseEstimationMethod &method) { m_poseEstimationMethod = method; }

  void setZAlignedWithCameraAxis(bool zAlignedWithCameraFrame) { m_zAlignedWithCameraFrame = zAlignedWithCameraFrame; }

protected:
  vpCameraParameters m_cam;
  std::map<vpPoseEstimationMethod, vpPose::vpPoseMethodType> m_mapOfCorrespondingPoseMethods;
  vpPoseEstimationMethod m_poseEstimationMethod;
  vpAprilTagFamily m_tagFamily;
  std::vector<vpHomogeneousMatrix> m_tagPoses;
  double m_tagSize;
  apriltag_detector_t *m_td;
  apriltag_family_t *m_tf;
  zarray_t *m_detections;
  bool m_zAlignedWithCameraFrame;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

/*!
   Default constructor.
*/
vpDetectorAprilTag::vpDetectorAprilTag(const vpAprilTagFamily &tagFamily,
                                       const vpPoseEstimationMethod &poseEstimationMethod)
  : m_displayTag(false), m_displayTagColor(vpColor::none), m_displayTagThickness(2),
    m_poseEstimationMethod(poseEstimationMethod), m_tagFamily(tagFamily), m_zAlignedWithCameraFrame(false),
    m_impl(new Impl(tagFamily, poseEstimationMethod))
{
}

/*!
   Destructor that desallocate memory.
*/
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

  bool detected = m_impl->detect(I, m_polygon, m_message, false, m_displayTag,
                                 m_displayTagColor, m_displayTagThickness);
  m_nb_objects = m_message.size();

  return detected;
}

/*!
  Detect AprilTag tags in the image and compute the corresponding tag poses considering that all the tags have
  the same size.

  If tags with different sizes have to be considered, you may use getPose().

  \param I : Input image.
  \param tagSize : Tag size in meter corresponding to the external width of the pattern.
  \param cam : Camera intrinsic parameters.
  \param cMo_vec : List of tag poses.
  \return true if at least one tag is detected.

  \sa getPose()
*/
bool vpDetectorAprilTag::detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam,
                                std::vector<vpHomogeneousMatrix> &cMo_vec)
{
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  m_impl->setTagSize(tagSize);
  m_impl->setCameraParameters(cam);
  bool detected = m_impl->detect(I, m_polygon, m_message, true, m_displayTag,
                                 m_displayTagColor, m_displayTagThickness);
  m_nb_objects = m_message.size();
  m_impl->getTagPoses(cMo_vec);

  return detected;
}

/*!
  Get the pose of a tag depending on its size and camera parameters.
  This function is useful to get the pose of tags with different sizes, while
  detect(const vpImage<unsigned char> &, const double, const vpCameraParameters &, std::vector<vpHomogeneousMatrix> &)
  considers that all the tags have the same size.

  \param[in] tagIndex : Index of the tag. Value should be in range [0, nb tags-1] with nb_tags = getNbObjects().
  \param[in] tagSize : Tag size in meter corresponding to the external width of the pattern.
  \param[in] cam : Camera intrinsic parameters.
  \param[out] cMo : Pose of the tag.
  \return true if success, false otherwise.

  The following code shows how to use this function:
  \code
    vpCameraParameters cam;
  vpDetectorAprilTag detector(tagFamily);
  detector.detect(I);
  for (size_t i = 0; i < detector.getNbObjects(); i++) {
    vpHomogeneousMatrix cMo;
    double tagSize;
    detector.getPose(i, tagSize, cam, cMo);
  }
  \endcode

  \sa detect(const vpImage<unsigned char> &, const double, const vpCameraParameters &, std::vector<vpHomogeneousMatrix> &)
 */
bool vpDetectorAprilTag::getPose(size_t tagIndex, const double tagSize, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo)
{
  return (m_impl->getPose(tagIndex, tagSize, cam, cMo));
}

/*!
  Set the number of threads for April Tag detection (default is 1).

  \param nThreads : Number of thread.
*/
void vpDetectorAprilTag::setAprilTagNbThreads(const int nThreads)
{
  if (nThreads > 0)
    m_impl->setNbThreads(nThreads);
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
void vpDetectorAprilTag::setAprilTagQuadDecimate(const float quadDecimate) { m_impl->setQuadDecimate(quadDecimate); }

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
void vpDetectorAprilTag::setAprilTagQuadSigma(const float quadSigma) { m_impl->setQuadSigma(quadSigma); }

/*!
  From the AprilTag code:
  <blockquote>
  when non-zero, detections are refined in a way intended to
  increase the number of detected tags. Especially effective for
  very small tags near the resolution threshold (e.g. 10px on a
  side).
  </blockquote>
  Default is 0.

  \param refineDecode : If true, set refine_decode to 1.
*/
void vpDetectorAprilTag::setAprilTagRefineDecode(const bool refineDecode) { m_impl->setRefineDecode(refineDecode); }

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
void vpDetectorAprilTag::setAprilTagRefineEdges(const bool refineEdges) { m_impl->setRefineEdges(refineEdges); }

/*!
  From the AprilTag code:
  <blockquote>
  when non-zero, detections are refined in a way intended to
  increase the accuracy of the extracted pose. This is done by
  maximizing the contrast around the black and white border of
  the tag. This generally increases the number of successfully
  detected tags, though not as effectively (or quickly) as
  refine_decode.
  This option must be enabled in order for "goodness" to be
  computed.
  </blockquote>
  Default is 0.

  \param refinePose : If true, set refine_pose to 1.
*/
void vpDetectorAprilTag::setAprilTagRefinePose(const bool refinePose) { m_impl->setRefinePose(refinePose); }

/*!
 * Modify the resulting tag pose returned by getPose() in order to get
 * a pose where z-axis is aligned when the camera plane is parallel to the tag.
 * \param zAlignedWithCameraFrame : Flag to get a pose where z-axis is aligned with the camera frame.
 */
void vpDetectorAprilTag::setZAlignedWithCameraAxis(bool zAlignedWithCameraFrame)
{
  m_zAlignedWithCameraFrame = zAlignedWithCameraFrame;
  m_impl->setZAlignedWithCameraAxis(zAlignedWithCameraFrame);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorAprilTag.cpp.o) has
// no symbols
void dummy_vpDetectorAprilTag() {}
#endif
