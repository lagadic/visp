/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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
 * RealSense SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <librealsense/rs.hpp>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

template <class Type>
void vp_rs_get_frame_data_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, const rs::stream &stream, vpImage<Type> &data)
{
  // Retrieve data from stream
  if (m_device->is_stream_enabled(stream)) {
    int width = m_intrinsics[(rs_stream)stream].width;
    int height = m_intrinsics[(rs_stream)stream].height;
    data.resize(height, width);

    memcpy((unsigned char *)data.bitmap, (unsigned char *)m_device->get_frame_data(stream), width*height*sizeof(Type));
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - stream %d not enabled!", (rs_stream)stream);
  }
}

// Retrieve color image
void vp_rs_get_color_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, vpImage<vpRGBa> &color)
{
  if (m_device->is_stream_enabled(rs::stream::color)) {
    int width = m_intrinsics[RS_STREAM_COLOR].width;
    int height = m_intrinsics[RS_STREAM_COLOR].height;
    color.resize(height, width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8)
      vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)color.bitmap, width, height);
    else
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }
}

// Retrieve grey image
void vp_rs_get_grey_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, vpImage<unsigned char> &grey)
{
  if (m_device->is_stream_enabled(rs::stream::color)) {
    int width = m_intrinsics[RS_STREAM_COLOR].width;
    int height = m_intrinsics[RS_STREAM_COLOR].height;
    grey.resize(height, width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8)
      vpImageConvert::RGBToGrey((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)grey.bitmap, width, height);
    else
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }
}

// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, float max_Z, std::vector<vpColVector> &pointcloud)
{
  if (m_device->is_stream_enabled(rs::stream::depth)) {
    const float depth_scale = m_device->get_depth_scale();

    vpColVector p3d(4); // X,Y,Z coordinates
    rs::float3 depth_point;
    uint16_t * depth = (uint16_t *)m_device->get_frame_data(rs::stream::depth);
    int width = m_intrinsics[RS_STREAM_DEPTH].width;
    int height = m_intrinsics[RS_STREAM_DEPTH].height;
    pointcloud.resize(width*height);

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        float scaled_depth = depth[i*width + j] * depth_scale;

        rs::float2 depth_pixel = { (float) j, (float) i};
        depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

        if (depth_point.z <= 0 || depth_point.z > max_Z) {
          depth_point.x = depth_point.y = depth_point.z = 0;
        }
        p3d[0] = depth_point.x;
        p3d[1] = depth_point.y;
        p3d[2] = depth_point.z;
        p3d[3] = 1;

        pointcloud[i*width + j] = p3d;
      }
    }
  }
  else {
    pointcloud.clear();
  }
}

#ifdef VISP_HAVE_PCL
// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, float max_Z, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device->is_stream_enabled(rs::stream::depth)) {
    int width = m_intrinsics[RS_STREAM_DEPTH].width;
    int height = m_intrinsics[RS_STREAM_DEPTH].height;
    pointcloud->width = width;
    pointcloud->height = height;
    pointcloud->resize(width * height);

    const float depth_scale = m_device->get_depth_scale();

    // Fill the PointCloud2 fields.
    rs::float3 depth_point;
    uint16_t * depth = (uint16_t *)m_device->get_frame_data(rs::stream::depth);

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        float scaled_depth = depth[i*width + j] * depth_scale;

        rs::float2 depth_pixel = { (float) j, (float) i};
        depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

        if (depth_point.z <= 0 || depth_point.z > max_Z) {
          depth_point.x = depth_point.y = depth_point.z = 0;
        }
        pointcloud->points[i*width + j].x = depth_point.x;
        pointcloud->points[i*width + j].y = depth_point.y;
        pointcloud->points[i*width + j].z = depth_point.z;
      }
    }
  }
  else {
    pointcloud->clear();
  }
}

// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, float max_Z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device->is_stream_enabled(rs::stream::depth) && m_device->is_stream_enabled(rs::stream::color)) {
    int depth_width = m_intrinsics[RS_STREAM_DEPTH].width;
    int depth_height = m_intrinsics[RS_STREAM_DEPTH].height;
    pointcloud->width = depth_width;
    pointcloud->height = depth_height;
    pointcloud->resize(depth_width * depth_height);

    const float depth_scale = m_device->get_depth_scale();

    rs::extrinsics depth_2_color_extrinsic = m_device->get_extrinsics(rs::stream::depth, rs::stream::color);

    // Fill the PointCloud2 fields.
    rs::float3 depth_point, color_point;
    rs::float2 color_pixel;
    uint16_t * depth = (uint16_t *)m_device->get_frame_data(rs::stream::depth);
    unsigned char * color = (unsigned char *)m_device->get_frame_data(rs::stream::color);
    int color_width = m_intrinsics[RS_STREAM_COLOR].width;
    int color_height = m_intrinsics[RS_STREAM_COLOR].height;

    for (int i = 0; i < depth_height; i++) {
      for (int j = 0; j < depth_width; j++) {
        float scaled_depth = depth[i*depth_width + j] * depth_scale;

        rs::float2 depth_pixel = { (float) j, (float) i};
        depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

        if (depth_point.z <= 0 || depth_point.z > max_Z) {
          depth_point.x = depth_point.y = depth_point.z = 0;
        }
        pointcloud->points[i*depth_width + j].x = depth_point.x;
        pointcloud->points[i*depth_width + j].y = depth_point.y;
        pointcloud->points[i*depth_width + j].z = depth_point.z;

        color_point = depth_2_color_extrinsic.transform(depth_point);
        color_pixel = m_intrinsics[RS_STREAM_COLOR].project(color_point);

        if (color_pixel.y < 0 || color_pixel.y >= color_height
            || color_pixel.x < 0 || color_pixel.x >= color_width)
        {
          // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
          // This color value is same as the librealsense out of bounds color value.
          unsigned int r = 96, g = 157, b = 198;
          uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                          static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

          pointcloud->points[i*depth_width + j].rgb = *reinterpret_cast<float*>(&rgb);
        }
        else
        {
          unsigned int i_ = (unsigned int) color_pixel.y;
          unsigned int j_ = (unsigned int) color_pixel.x;

          uint32_t rgb = (static_cast<uint32_t>(color[(i_*color_width + j_)*3]) << 16 |
                          static_cast<uint32_t>(color[(i_*color_width + j_)*3] + 1) << 8 |
                          static_cast<uint32_t>(color[(i_*color_width + j_)*3] + 2));
          pointcloud->points[i*depth_width + j].rgb = *reinterpret_cast<float*>(&rgb);
        }
      }
    }
  }
  else {
    pointcloud->clear();
  }
}
#endif
