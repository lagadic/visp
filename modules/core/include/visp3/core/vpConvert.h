/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef __vpConvert_h__
#define __vpConvert_h__

/*!
  \file vpConvert.h
  \brief Tools for type or general conversion.
 */

#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#endif

#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPoint.h>

/*!
  \class vpConvert
  \ingroup group_core_bridges
  Bridges over other frameworks like OpenCV.
 */
class VISP_EXPORT vpConvert
{

public:
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  static void convertFromOpenCV(const cv::KeyPoint &from, vpImagePoint &to);
  static void convertFromOpenCV(const cv::Point2f &from, vpImagePoint &to);
  static void convertFromOpenCV(const cv::Point2d &from, vpImagePoint &to);
  static void convertFromOpenCV(const cv::Point3f &from, vpPoint &to, const bool cameraFrame = false);
  static void convertFromOpenCV(const cv::Point3d &from, vpPoint &to, const bool cameraFrame = false);

  static void convertFromOpenCV(const std::vector<cv::KeyPoint> &from, std::vector<vpImagePoint> &to);
  static void convertFromOpenCV(const std::vector<cv::Point2f> &from, std::vector<vpImagePoint> &to);
  static void convertFromOpenCV(const std::vector<cv::Point2d> &from, std::vector<vpImagePoint> &to);
  static void convertFromOpenCV(const std::vector<cv::Point3f> &from, std::vector<vpPoint> &to,
                                const bool cameraFrame = false);
  static void convertFromOpenCV(const std::vector<cv::Point3d> &from, std::vector<vpPoint> &to,
                                const bool cameraFrame = false);
  static void convertFromOpenCV(const std::vector<cv::DMatch> &from, std::vector<unsigned int> &to);

  static void convertToOpenCV(const vpImagePoint &from, cv::Point2f &to);
  static void convertToOpenCV(const vpImagePoint &from, cv::Point2d &to);
  static void convertToOpenCV(const vpPoint &from, cv::Point3f &to, const bool cameraFrame = false);
  static void convertToOpenCV(const vpPoint &from, cv::Point3d &to, const bool cameraFrame = false);

  static void convertToOpenCV(const std::vector<vpImagePoint> &from, std::vector<cv::Point2f> &to);
  static void convertToOpenCV(const std::vector<vpImagePoint> &from, std::vector<cv::Point2d> &to);
  static void convertToOpenCV(const std::vector<vpPoint> &from, std::vector<cv::Point3f> &to,
                              const bool cameraFrame = false);
  static void convertToOpenCV(const std::vector<vpPoint> &from, std::vector<cv::Point3d> &to,
                              const bool cameraFrame = false);

private:
  static vpImagePoint keyPointToVpImagePoint(const cv::KeyPoint &keypoint);
  static vpImagePoint point2fToVpImagePoint(const cv::Point2f &point);
  static vpImagePoint point2dToVpImagePoint(const cv::Point2d &point);
  static vpPoint point3fToVpObjectPoint(const cv::Point3f &point3f);
  static vpPoint point3dToVpObjectPoint(const cv::Point3d &point3d);
  static vpPoint point3fToVpCamPoint(const cv::Point3f &point3f);
  static vpPoint point3dToVpCamPoint(const cv::Point3d &point3d);
  static int dMatchToTrainIndex(const cv::DMatch &match);

  static cv::Point2f vpImagePointToPoint2f(const vpImagePoint &point);
  static cv::Point2d vpImagePointToPoint2d(const vpImagePoint &point);
  static cv::Point3f vpCamPointToPoint3f(const vpPoint &point);
  static cv::Point3d vpCamPointToPoint3d(const vpPoint &point);
  static cv::Point3f vpObjectPointToPoint3f(const vpPoint &point);
  static cv::Point3d vpObjectPointToPoint3d(const vpPoint &point);

#endif
};

#endif
