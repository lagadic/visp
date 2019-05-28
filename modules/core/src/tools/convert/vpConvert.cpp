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
 * Directory management.
 *
 * Authors:
 * Fabien Spindler
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpConvert.cpp
  \brief Tools for type or general conversion.
*/

#include <algorithm> // std::transform
#include <vector>    // std::vector

#include <visp3/core/vpConvert.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
/**!
   Unary function used to transform a cv::KeyPoint to a vpImagePoint.
   \param keypoint : KeyPoint to convert.

   \return A vpImagePoint with the 2D coordinates corresponding to the
   location of the KeyPoint.
 */
vpImagePoint vpConvert::keyPointToVpImagePoint(const cv::KeyPoint &keypoint)
{
  return vpImagePoint(keypoint.pt.y, keypoint.pt.x);
}

/*!
   Unary function to convert a cv::Point2f to a vpImagePoint.
   \param point : Point to convert.

   \return A vpImagePoint with the 2D coordinates stored in cv::Point2f.
 */
vpImagePoint vpConvert::point2fToVpImagePoint(const cv::Point2f &point) { return vpImagePoint(point.y, point.x); }

/*!
   Unary function to convert a cv::Point2d to a vpImagePoint.
   \param point : Point to convert.

   \return A vpImagePoint with the 2D coordinates stored in cv::Point2d.
 */
vpImagePoint vpConvert::point2dToVpImagePoint(const cv::Point2d &point) { return vpImagePoint(point.y, point.x); }

/*!
   Unary function to convert a cv::Point3f to a vpPoint (object frame).
   \param point3f : Point to convert.
   \return A vpPoint with 3D coordinates in the object frame from from a
   cv::Point3f.
 */
vpPoint vpConvert::point3fToVpObjectPoint(const cv::Point3f &point3f)
{
  vpPoint pt;
  pt.set_oX(point3f.x);
  pt.set_oY(point3f.y);
  pt.set_oZ(point3f.z);
  pt.set_oW(1.0);
  return pt;
}

/*!
   Unary function to convert a cv::Point3f to a vpPoint (camera frame).
   \param point3f : Point to convert.
   \return A vpPoint with 3D coordinates in the camera frame from from a
   cv::Point3f.
 */
vpPoint vpConvert::point3fToVpCamPoint(const cv::Point3f &point3f)
{
  vpPoint pt;
  pt.set_X(point3f.x);
  pt.set_Y(point3f.y);
  pt.set_Z(point3f.z);
  pt.set_W(1.0);
  return pt;
}

/*!
   Unary function to convert a cv::Point3d to a vpPoint (object frame).
   \param point3d : Point to convert.
   \return A vpPoint with 3D coordinates in the object frame from from a
   cv::Point3d.
 */
vpPoint vpConvert::point3dToVpObjectPoint(const cv::Point3d &point3d)
{
  vpPoint pt;
  pt.set_oX(point3d.x);
  pt.set_oY(point3d.y);
  pt.set_oZ(point3d.z);
  pt.set_oW(1.0);
  return pt;
}

/*!
   Unary function to convert a cv::Point3d to a vpPoint (camera frame).
   \param point3d : Point to convert.
   \return A vpPoint with 3D coordinates in the camera frame from from a
   cv::Point3d.
 */
vpPoint vpConvert::point3dToVpCamPoint(const cv::Point3d &point3d)
{
  vpPoint pt;
  pt.set_X(point3d.x);
  pt.set_Y(point3d.y);
  pt.set_Z(point3d.z);
  pt.set_W(1.0);
  return pt;
}

/*!
   Unary function to convert a vpImagePoint to a cv::Point2f.
   \param point : Image point to convert.

   \return A cv::Point2f with the 2D coordinates stored in vpImagePoint.
 */
cv::Point2f vpConvert::vpImagePointToPoint2f(const vpImagePoint &point)
{
  return cv::Point2f((float)point.get_u(), (float)point.get_v());
}

/*!
   Unary function to convert a vpImagePoint to a cv::Point2d.
   \param point : Image point to convert.

   \return A cv::Point2d with the 2D coordinates stored in vpImagePoint.
 */
cv::Point2d vpConvert::vpImagePointToPoint2d(const vpImagePoint &point)
{
  return cv::Point2d(point.get_u(), point.get_v());
}

/*!
   Unary function to convert the 3D coordinates in the camera frame to a
   cv::Point3f. \param point : Point to convert.

   \return A cv::Point3f with the 3D coordinates stored in vpPoint in the
   camera frame.
 */
cv::Point3f vpConvert::vpCamPointToPoint3f(const vpPoint &point)
{
  return cv::Point3f((float)point.get_X(), (float)point.get_Y(), (float)point.get_Z());
}

/*!
   Unary function to convert the 3D coordinates in the camera frame to a
   cv::Point3d. \param point : Point to convert.

   \return A cv::Point3d with the 3D coordinates stored in vpPoint in the
   camera frame.
 */
cv::Point3d vpConvert::vpCamPointToPoint3d(const vpPoint &point)
{
  return cv::Point3d(point.get_X(), point.get_Y(), point.get_Z());
}

/*!
   Unary function to convert the 3D coordinates in the object frame to a
   cv::Point3f. \param point : Point to convert.

   \return A cv::Point3f with the 3D coordinates stored in vpPoint in the
   object frame.
 */
cv::Point3f vpConvert::vpObjectPointToPoint3f(const vpPoint &point)
{
  return cv::Point3f((float)point.get_oX(), (float)point.get_oY(), (float)point.get_oZ());
}

/*!
   Unary function to convert the 3D coordinates in the object frame to a
   cv::Point3d. \param point : Point to convert.

   \return A cv::Point3d with the 3D coordinates stored in vpPoint in the
   object frame.
 */
cv::Point3d vpConvert::vpObjectPointToPoint3d(const vpPoint &point)
{
  return cv::Point3d(point.get_oX(), point.get_oY(), point.get_oZ());
}

/*!
   Unary function to return the train index stored in a cv::DMatch.
   \param match : The cv::DMatch we want to get the train index.

   \return The train index stored in a cv::DMatch.
 */
int vpConvert::dMatchToTrainIndex(const cv::DMatch &match) { return match.trainIdx; }

/*!
   Convert a cv::KeyPoint to a vpImagePoint.
   \param from : cv::KeyPoint to convert.
   \param to : vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const cv::KeyPoint &from, vpImagePoint &to) { to = keyPointToVpImagePoint(from); }

/*!
   Convert a cv::Point2f to a vpImagePoint.
   \param from : cv::Point2f to convert.
   \param to : vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const cv::Point2f &from, vpImagePoint &to) { to = point2fToVpImagePoint(from); }

/*!
   Convert a cv::Point2d to a vpImagePoint.
   \param from : cv::Point2d to convert.
   \param to : vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const cv::Point2d &from, vpImagePoint &to) { to = point2dToVpImagePoint(from); }

/*!
   Convert a cv::Point3f to a vpPoint.
   \param from : cv::Point3f to convert.
   \param to : vpPoint converted.
   \param cameraFrame : If true, convert into the camera frame, otherwise in
   the object frame.
 */
void vpConvert::convertFromOpenCV(const cv::Point3f &from, vpPoint &to, const bool cameraFrame)
{
  if (cameraFrame) {
    to = point3fToVpCamPoint(from);
  } else {
    to = point3fToVpObjectPoint(from);
  }
}

/*!
   Convert a cv::Point3d to a vpPoint.
   \param from : cv::Point3d to convert.
   \param to : vpPoint converted.
   \param cameraFrame : If true, convert into the camera frame, otherwise in
   the object frame.
 */
void vpConvert::convertFromOpenCV(const cv::Point3d &from, vpPoint &to, const bool cameraFrame)
{
  if (cameraFrame) {
    to = point3dToVpCamPoint(from);
  } else {
    to = point3dToVpObjectPoint(from);
  }
}

/*!
   Convert a vector of cv::KeyPoint to a vector of vpImagePoint.
   \param from : Vector of cv::KeyPoint to convert.
   \param to : Vector of vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::KeyPoint> &from, std::vector<vpImagePoint> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), keyPointToVpImagePoint);
}

/*!
   Convert a vector of cv::Point2f to a vector of vpImagePoint.
   \param from : Vector of cv::Point2f to convert.
   \param to : Vector of vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::Point2f> &from, std::vector<vpImagePoint> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), point2fToVpImagePoint);
}

/*!
   Convert a vector of cv::Point2d to a vector of vpImagePoint.
   \param from : Vector of cv::Point2d to convert.
   \param to : Vector of vpImagePoint converted.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::Point2d> &from, std::vector<vpImagePoint> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), point2dToVpImagePoint);
}

/*!
   Convert a vector of cv::Point3f to a vector of vpPoint.
   \param from : Vector of cv::Point3f to convert.
   \param to : Vector of vpPoint converted.
   \param cameraFrame : If true, convert into the camera frame, otherwise in
   the object frame.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::Point3f> &from, std::vector<vpPoint> &to,
                                  const bool cameraFrame)
{
  to.resize(from.size());
  if (cameraFrame) {
    std::transform(from.begin(), from.end(), to.begin(), point3fToVpCamPoint);
  } else {
    std::transform(from.begin(), from.end(), to.begin(), point3fToVpObjectPoint);
  }
}

/*!
   Convert a vector of cv::Point3d to a vector of vpPoint.
   \param from : Vector of cv::Point3d to convert.
   \param to : Vector of vpPoint converted.
   \param cameraFrame : If true, convert into the camera frame, otherwise in
   the object frame.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::Point3d> &from, std::vector<vpPoint> &to,
                                  const bool cameraFrame)
{
  to.resize(from.size());
  if (cameraFrame) {
    std::transform(from.begin(), from.end(), to.begin(), point3dToVpCamPoint);
  } else {
    std::transform(from.begin(), from.end(), to.begin(), point3dToVpObjectPoint);
  }
}

/*!
   Convert a vector of cv::DMatch to a vector of unsigned int (for a query
   index 0, to[0] ==> train index).

   \warning The list of query indexes in DMatch must be ordered in a way that
   from[i].queryIdx == i.

   \param from : Vector of cv::DMatch to convert.
   \param to : Vector of unsigned int converted.
 */
void vpConvert::convertFromOpenCV(const std::vector<cv::DMatch> &from, std::vector<unsigned int> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), dMatchToTrainIndex);
}

/*!
   Convert a vpImagePoint to a cv::Point2f.
   \param from : vpImagePoint to convert.
   \param to : cv::Point2f converted.
 */
void vpConvert::convertToOpenCV(const vpImagePoint &from, cv::Point2f &to) { to = vpImagePointToPoint2f(from); }

/*!
   Convert a vpImagePoint to a cv::Point2d.
   \param from : vpImagePoint to convert.
   \param to : cv::Point2d converted.
 */
void vpConvert::convertToOpenCV(const vpImagePoint &from, cv::Point2d &to) { to = vpImagePointToPoint2d(from); }

/*!
   Convert a vpPoint to a cv::Point3f.
   \param from : vpPoint to convert.
   \param to : cv::Point3f converted.
   \param cameraFrame : If true, convert from coordinates in the camera frame,
   otherwise in the object frame.
 */
void vpConvert::convertToOpenCV(const vpPoint &from, cv::Point3f &to, const bool cameraFrame)
{
  if (cameraFrame) {
    to = vpCamPointToPoint3f(from);
  } else {
    to = vpObjectPointToPoint3f(from);
  }
}

/*!
   Convert a vpPoint to a cv::Point3d.
   \param from : vpPoint to convert.
   \param to : cv::Point3d converted.
   \param cameraFrame : If true, convert from coordinates in the camera frame,
   otherwise in the object frame.
 */
void vpConvert::convertToOpenCV(const vpPoint &from, cv::Point3d &to, const bool cameraFrame)
{
  if (cameraFrame) {
    to = vpCamPointToPoint3d(from);
  } else {
    to = vpObjectPointToPoint3d(from);
  }
}

/*!
   Convert a vector of vpImagePoint to a vector of cv::Point2f.
   \param from : Vector of vpImagePoint to convert.
   \param to : Vector of cv::Point2f converted.
 */
void vpConvert::convertToOpenCV(const std::vector<vpImagePoint> &from, std::vector<cv::Point2f> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), vpImagePointToPoint2f);
}

/*!
   Convert a vector of vpImagePoint to a vector of cv::Point2d.
   \param from : Vector of vpImagePoint to convert.
   \param to : Vector of cv::Point2d converted.
 */
void vpConvert::convertToOpenCV(const std::vector<vpImagePoint> &from, std::vector<cv::Point2d> &to)
{
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), vpImagePointToPoint2d);
}

/*!
   Convert a vector of vpPoint to a vector of cv::Point3f.
   \param from : Vector of vpPoint to convert.
   \param to : Vector of cv::Point3f converted.
   \param cameraFrame : If true, the camera frame is considered, otherwise the
   object frame.
 */
void vpConvert::convertToOpenCV(const std::vector<vpPoint> &from, std::vector<cv::Point3f> &to, const bool cameraFrame)
{
  to.resize(from.size());
  if (cameraFrame) {
    std::transform(from.begin(), from.end(), to.begin(), vpCamPointToPoint3f);
  } else {
    std::transform(from.begin(), from.end(), to.begin(), vpObjectPointToPoint3f);
  }
}

/*!
   Convert a vector of vpPoint to a vector of cv::Point3d.
   \param from : Vector of vpPoint to convert.
   \param to : Vector of cv::Point3d converted.
   \param cameraFrame : If true, the camera frame is considered, otherwise the
   object frame.
 */
void vpConvert::convertToOpenCV(const std::vector<vpPoint> &from, std::vector<cv::Point3d> &to, const bool cameraFrame)
{
  to.resize(from.size());
  if (cameraFrame) {
    std::transform(from.begin(), from.end(), to.begin(), vpCamPointToPoint3d);
  } else {
    std::transform(from.begin(), from.end(), to.begin(), vpObjectPointToPoint3d);
  }
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpConvert.cpp.o) has no
// symbols
void dummy_vpConvert(){};
#endif
