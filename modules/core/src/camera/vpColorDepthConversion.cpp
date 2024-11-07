/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Color to depth conversion.
 */

/*!
 * \file vpColorDepthConversion.cpp
 * \brief color to depth conversion
 */

#include <visp3/core/vpColorDepthConversion.h>

// System
#include <algorithm>

// Core
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{

/*!
 * Adjust 2D image point to the image boundary.
 *
 * \param[in] ip : 2D point to adjust.
 * \param[in] width : Image width.
 * \param[in] height : Image height.
 * \return Adjusted 2D point (<=> [i,j] in [0,height][0,width] ).
 */
vpImagePoint adjust2DPointToBoundary(const vpImagePoint &ip, double width, double height)
{
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  return { vpMath::clamp(ip.get_i(), 0., height), vpMath::clamp(ip.get_j(), 0., width) };
#else
  return vpImagePoint(vpMath::clamp(ip.get_i(), 0., height), vpMath::clamp(ip.get_j(), 0., width));
#endif
}

/*!
 * Change the frame of a 3D point.
 *
 * \param[in] extrinsics_params : bMa homogeneous matrix.
 * \param[in] from_point : <X,Y,Z> expressed into a.
 * \return <X,Y,Z> expressed into b.
 */
vpColVector transform(const vpHomogeneousMatrix &extrinsics_params, vpColVector from_point)
{
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  from_point = { from_point, 0, 3 };
  from_point.stack(1.);
  return { extrinsics_params * from_point, 0, 3 };
#else
  from_point = vpColVector(from_point, 0, 3);
  from_point.stack(1.);
  return vpColVector(extrinsics_params * from_point, 0, 3);
#endif
}

/*!
 * Project 3D point to pixel.
 *
 * \param[in] intrinsic_cam_params : Intrinsic camera parameters.
 * \param[in] point : <X,Y,Z> expressed into the camera frame.
 * \return Image point expressed into image frame.
 */
vpImagePoint project(const vpCameraParameters &intrinsic_cam_params, const vpColVector &point)
{
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  vpImagePoint iP {};
#else
  vpImagePoint iP;
#endif
  vpMeterPixelConversion::convertPoint(intrinsic_cam_params, point[0] / point[2], point[1] / point[2], iP);

  return iP;
}

/*!
 * Deproject pixel to 3D point.
 *
 * \param[in] intrinsic_cam_params : Intrinsic camera parameters.
 * \param[in] pixel : Image point expressed into image frame.
 * \param[in] depth : Depth of the image point [m].
 * \return <X,Y,Z> expressed into the camera frame.
 */
vpColVector deproject(const vpCameraParameters &intrinsic_cam_params, const vpImagePoint &pixel, double depth)
{
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  double x { 0. }, y { 0. };
  vpPixelMeterConversion::convertPoint(intrinsic_cam_params, pixel, x, y);
  return { x * depth, y * depth, depth };
#else
  double x = 0., y = 0.;
  vpPixelMeterConversion::convertPoint(intrinsic_cam_params, pixel, x, y);

  vpColVector p(3);
  p[0] = x * depth;
  p[1] = y * depth;
  p[2] = depth;
  return p;
#endif
}

} // namespace

#endif // DOXYGEN_SHOULD_SKIP_THIS

/*!
 * Project color image point to depth frame.
 *
 * \param[in] I_depth : Depth raw image.
 * \param[in] depth_scale : Depth scale to convert depth raw values in [m]. If depth raw values in `I_depth` are in
 * [mm], depth scale should be 0.001.
 * \param[in] depth_min : Minimal depth value for correspondence [m].
 * \param[in] depth_max : Maximal depth value for correspondence [m].
 * \param[in] depth_intrinsics : Intrinsic depth camera parameters.
 * \param[in] color_intrinsics : Intrinsic color camera parameters.
 * \param[in] color_M_depth : Relationship between color and depth cameras (ie, extrinsic rgb-d camera parameters).
 * \param[in] depth_M_color : Relationship between depth and color cameras (ie, extrinsic rgb-d camera parameters).
 * \param[in] from_pixel : Image point expressed into the color camera frame.
 * \return Image point expressed into the depth camera frame.
 */
vpImagePoint vpColorDepthConversion::projectColorToDepth(
  const vpImage<uint16_t> &I_depth, double depth_scale, double depth_min, double depth_max,
  const vpCameraParameters &depth_intrinsics, const vpCameraParameters &color_intrinsics,
  const vpHomogeneousMatrix &color_M_depth, const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel)
{
  return projectColorToDepth(I_depth.bitmap, depth_scale, depth_min, depth_max, I_depth.getWidth(), I_depth.getHeight(),
                             depth_intrinsics, color_intrinsics, color_M_depth, depth_M_color, from_pixel);
}

/*!
 * Project color image point to depth frame.
 *
 * \param[in] data : Depth raw values.
 * \param[in] depth_scale : Depth scale to convert depth raw values in [m]. If depth raw values in `data` are in [mm],
 * depth scale should be 0.001.
 * \param[in] depth_min : Minimal depth value for correspondence [m].
 * \param[in] depth_max : Maximal depth value for correspondence [m].
 * \param[in] depth_width : Depth image width [pixel].
 * \param[in] depth_height : Depth image height [pixel].
 * \param[in] depth_intrinsics : Intrinsic depth camera parameters.
 * \param[in] color_intrinsics : Intrinsic color camera parameters.
 * \param[in] color_M_depth : Relationship between color and depth cameras (ie, extrinsic rgb-d camera parameters).
 * \param[in] depth_M_color : Relationship between depth and color cameras (ie, extrinsic rgb-d camera parameters).
 * \param[in] from_pixel : Image point expressed into the color camera frame.
 * \return Image point expressed into the depth camera frame.
 */
vpImagePoint vpColorDepthConversion::projectColorToDepth(
  const uint16_t *data, double depth_scale, double depth_min, double depth_max, double depth_width,
  double depth_height, const vpCameraParameters &depth_intrinsics, const vpCameraParameters &color_intrinsics,
  const vpHomogeneousMatrix &color_M_depth, const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel)
{
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  vpImagePoint depth_pixel {};

  // Find line start pixel
  const auto min_point = deproject(color_intrinsics, from_pixel, depth_min);
  const auto min_transformed_point = transform(depth_M_color, min_point);
  auto start_pixel = project(depth_intrinsics, min_transformed_point);
  start_pixel = adjust2DPointToBoundary(start_pixel, depth_width, depth_height);

  // Find line end depth pixel
  const auto max_point = deproject(color_intrinsics, from_pixel, depth_max);
  const auto max_transformed_point = transform(depth_M_color, max_point);
  auto end_pixel = project(depth_intrinsics, max_transformed_point);
  end_pixel = adjust2DPointToBoundary(end_pixel, depth_width, depth_height);

  // search along line for the depth pixel that it's projected pixel is the closest to the input pixel
  auto min_dist = -1.;
  for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel) && (curr_pixel != end_pixel);
       curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel)) {
    const auto depth = depth_scale * data[static_cast<int>((curr_pixel.get_v() * depth_width) + curr_pixel.get_u())];
    bool stop_for_loop = false;
    if (std::fabs(depth) <= std::numeric_limits<double>::epsilon()) {
      stop_for_loop = true;
    }
    if (!stop_for_loop) {
      const auto point = deproject(depth_intrinsics, curr_pixel, depth);
      const auto transformed_point = transform(color_M_depth, point);
      const auto projected_pixel = project(color_intrinsics, transformed_point);

      const auto new_dist = vpMath::sqr(projected_pixel.get_v() - from_pixel.get_v()) +
        vpMath::sqr(projected_pixel.get_u() - from_pixel.get_u());
      if ((new_dist < min_dist) || (min_dist < 0)) {
        min_dist = new_dist;
        depth_pixel = curr_pixel;
      }
    }
  }

#else
  vpImagePoint depth_pixel;

  // Find line start pixel
  const vpColVector min_point = deproject(color_intrinsics, from_pixel, depth_min);
  const vpColVector min_transformed_point = transform(depth_M_color, min_point);
  vpImagePoint start_pixel = project(depth_intrinsics, min_transformed_point);
  start_pixel = adjust2DPointToBoundary(start_pixel, depth_width, depth_height);

  // Find line end depth pixel
  const vpColVector max_point = deproject(color_intrinsics, from_pixel, depth_max);
  const vpColVector max_transformed_point = transform(depth_M_color, max_point);
  vpImagePoint end_pixel = project(depth_intrinsics, max_transformed_point);
  end_pixel = adjust2DPointToBoundary(end_pixel, depth_width, depth_height);

  // search along line for the depth pixel that it's projected pixel is the closest to the input pixel
  double min_dist = -1.;
  for (vpImagePoint curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel) && curr_pixel != end_pixel;
       curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel)) {
    const double depth = depth_scale * data[static_cast<int>(curr_pixel.get_v() * depth_width + curr_pixel.get_u())];

    bool stop_for_loop = false;
    if (std::fabs(depth) <= std::numeric_limits<double>::epsilon()) {
      stop_for_loop = true;
    }
    if (!stop_for_loop) {
      const vpColVector point = deproject(depth_intrinsics, curr_pixel, depth);
      const vpColVector transformed_point = transform(color_M_depth, point);
      const vpImagePoint projected_pixel = project(color_intrinsics, transformed_point);

      const double new_dist = vpMath::sqr(projected_pixel.get_v() - from_pixel.get_v()) +
        vpMath::sqr(projected_pixel.get_u() - from_pixel.get_u());
      if (new_dist < min_dist || min_dist < 0) {
        min_dist = new_dist;
        depth_pixel = curr_pixel;
      }
    }
  }
#endif
  return depth_pixel;
}
END_VISP_NAMESPACE
