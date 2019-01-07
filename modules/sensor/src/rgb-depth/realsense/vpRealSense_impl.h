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
 * RealSense SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE)
#include <librealsense/rs.hpp>
#include <thread>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_PCL
#include <pcl/pcl_config.h>
#endif

template <class Type>
void vp_rs_get_frame_data_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                               const rs::stream &stream, vpImage<Type> &data)
{
  // Retrieve data from stream
  if (m_device->is_stream_enabled(stream)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(stream);
    if (it_intrinsics == m_intrinsics.end()) {
      std::stringstream ss;
      ss << "Cannot find intrinsics for " << stream << "  stream!";
      throw vpException(vpException::fatalError, ss.str());
    }

    unsigned int width = (unsigned int)it_intrinsics->second.width;
    unsigned int height = (unsigned int)it_intrinsics->second.height;
    data.resize(height, width);

    memcpy((unsigned char *)data.bitmap, (unsigned char *)m_device->get_frame_data(stream),
           width * height * sizeof(Type));
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - stream %d not enabled!", (rs_stream)stream);
  }
}

// Retrieve data from native stream
void vp_rs_get_native_frame_data_impl(const rs::device *m_device,
                                      const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                                      const rs::stream &native_stream, unsigned char *const data,
                                      const rs::stream &stream)
{
  if (m_device->is_stream_enabled(native_stream)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(native_stream);
    if (it_intrinsics == m_intrinsics.end()) {
      std::stringstream ss;
      ss << "Cannot find intrinsics for " << native_stream << "  stream!";
      throw vpException(vpException::fatalError, ss.str());
    }

    size_t size = (size_t)(it_intrinsics->second.width * it_intrinsics->second.height);

    switch (m_device->get_stream_format(native_stream)) {
    // 8 bits
    case rs::format::raw8:
      std::cout << "Stream: raw8 not tested!" << std::endl;
      /* FALLTHRU */
    case rs::format::y8:
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size);
      break;

    // Four 10-bit luminance values encoded into a 5-byte macropixel
    case rs::format::raw10:
      std::cout << "Stream: raw10 not tested!" << std::endl;
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), (5 * (size / 4)));
      break;

    // 16 bits
    case rs::format::raw16:
      std::cout << "Stream: raw16 not tested!" << std::endl;
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 2);
      break;

    case rs::format::disparity16:
      std::cout << "Stream: disparity16 not tested!" << std::endl;
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 2);
      break;
    case rs::format::y16:
    case rs::format::z16:
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 2);
      break;

    // 24 bits
    case rs::format::bgr8:
    case rs::format::rgb8:
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 3);
      break;

    // 32 bits
    case rs::format::yuyv:
      std::cout << "Stream: yuyv not tested!" << std::endl;
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 4);
      break;
    case rs::format::bgra8:
    case rs::format::rgba8:
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 4);
      break;

    // 96 bits
    case rs::format::xyz32f:
      std::cout << "Stream: xyz32f not tested!" << std::endl;
      memcpy(data, (unsigned char *)m_device->get_frame_data(stream), size * 3 * 4);
      break;

    case rs::format::any:
    default:
      break;
    }
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - stream %d not enabled!", (rs_stream)native_stream);
  }
}

// Retrieve color image
void vp_rs_get_color_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                          vpImage<vpRGBa> &color)
{
  if (m_device->is_stream_enabled(rs::stream::color)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(rs::stream::color);
    if (it_intrinsics == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for color stream!");
    }

    unsigned int width = (unsigned int)it_intrinsics->second.width;
    unsigned int height = (unsigned int)it_intrinsics->second.height;
    color.resize(height, width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8) {
      vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color),
                                (unsigned char *)color.bitmap, width, height);
    } else if (m_device->get_stream_format(rs::stream::color) == rs::format::rgba8) {
      memcpy((unsigned char *)color.bitmap, (unsigned char *)m_device->get_frame_data(rs::stream::color),
             width * height * sizeof(vpRGBa));
    } else if (m_device->get_stream_format(rs::stream::color) == rs::format::bgr8) {
      vpImageConvert::BGRToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color),
                                (unsigned char *)color.bitmap, width, height);
    } else {
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
    }
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }
}

// Retrieve grey image
void vp_rs_get_grey_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                         vpImage<unsigned char> &grey)
{
  if (m_device->is_stream_enabled(rs::stream::color)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(rs::stream::color);
    if (it_intrinsics == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for color stream!");
    }

    unsigned int width = (unsigned int)it_intrinsics->second.width;
    unsigned int height = (unsigned int)it_intrinsics->second.height;
    grey.resize(height, width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8) {
      vpImageConvert::RGBToGrey((unsigned char *)m_device->get_frame_data(rs::stream::color),
                                (unsigned char *)grey.bitmap, width, height);
    } else if (m_device->get_stream_format(rs::stream::color) == rs::format::rgba8) {
      vpImageConvert::RGBaToGrey((unsigned char *)m_device->get_frame_data(rs::stream::color),
                                 (unsigned char *)grey.bitmap, width * height);
    } else if (m_device->get_stream_format(rs::stream::color) == rs::format::bgr8) {
      vpImageConvert::BGRToGrey((unsigned char *)m_device->get_frame_data(rs::stream::color),
                                (unsigned char *)grey.bitmap, width, height);
    } else {
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
    }
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }
}

// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                               float max_Z, std::vector<vpColVector> &pointcloud, const float invalidDepthValue = 0.0f,
                               const rs::stream &stream_depth = rs::stream::depth)
{
  if (m_device->is_stream_enabled(rs::stream::depth)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(stream_depth);
    if (it_intrinsics == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for depth stream!");
    }

    const float depth_scale = m_device->get_depth_scale();

    vpColVector p3d(4); // X,Y,Z coordinates
    rs::float3 depth_point;
    uint16_t *depth = (uint16_t *)m_device->get_frame_data(stream_depth);
    int width = it_intrinsics->second.width;
    int height = it_intrinsics->second.height;
    pointcloud.resize((size_t)(width * height));

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        float scaled_depth = depth[i * width + j] * depth_scale;

        rs::float2 depth_pixel = {(float)j, (float)i};
        depth_point = it_intrinsics->second.deproject(depth_pixel, scaled_depth);

        if (depth_point.z <= 0 || depth_point.z > max_Z) {
          depth_point.x = depth_point.y = depth_point.z = invalidDepthValue;
        }
        p3d[0] = depth_point.x;
        p3d[1] = depth_point.y;
        p3d[2] = depth_point.z;
        p3d[3] = 1;

        pointcloud[(size_t)(i * width + j)] = p3d;
      }
    }
  } else {
    pointcloud.clear();
  }
}

#ifdef VISP_HAVE_PCL
// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                               float max_Z, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
                               const float invalidDepthValue = 0.0f, const rs::stream &stream_depth = rs::stream::depth)
{
  if (m_device->is_stream_enabled(rs::stream::depth)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics = m_intrinsics.find(stream_depth);
    if (it_intrinsics == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for depth stream!");
    }

    int width = it_intrinsics->second.width;
    int height = it_intrinsics->second.height;
    pointcloud->width = (uint32_t)width;
    pointcloud->height = (uint32_t)height;
    pointcloud->resize((size_t)(width * height));

    const float depth_scale = m_device->get_depth_scale();

    // Fill the PointCloud2 fields.
    rs::float3 depth_point;
    uint16_t *depth = (uint16_t *)m_device->get_frame_data(stream_depth);

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        float scaled_depth = depth[i * width + j] * depth_scale;

        rs::float2 depth_pixel = {(float)j, (float)i};
        depth_point = it_intrinsics->second.deproject(depth_pixel, scaled_depth);

        if (depth_point.z <= 0 || depth_point.z > max_Z) {
          depth_point.x = depth_point.y = depth_point.z = invalidDepthValue;
        }
        pointcloud->points[(size_t)(i * width + j)].x = depth_point.x;
        pointcloud->points[(size_t)(i * width + j)].y = depth_point.y;
        pointcloud->points[(size_t)(i * width + j)].z = depth_point.z;
      }
    }
  } else {
    pointcloud->clear();
  }
}

// Retrieve point cloud
void vp_rs_get_pointcloud_impl(const rs::device *m_device, const std::map<rs::stream, rs::intrinsics> &m_intrinsics,
                               float max_Z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
                               const float invalidDepthValue = 0.0f, const rs::stream &stream_color = rs::stream::color,
                               const rs::stream &stream_depth = rs::stream::depth)
{
  if (m_device->is_stream_enabled(rs::stream::depth) && m_device->is_stream_enabled(rs::stream::color)) {
    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics_depth = m_intrinsics.find(stream_depth);
    if (it_intrinsics_depth == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for depth stream!");
    }

    std::map<rs::stream, rs::intrinsics>::const_iterator it_intrinsics_color = m_intrinsics.find(stream_color);
    if (it_intrinsics_color == m_intrinsics.end()) {
      throw vpException(vpException::fatalError, "Cannot find intrinsics for color stream!");
    }

    if (m_device->get_stream_format(rs::stream::color) != rs::format::rgb8 &&
        m_device->get_stream_format(rs::stream::color) != rs::format::rgba8 &&
        m_device->get_stream_format(rs::stream::color) != rs::format::bgr8 &&
        m_device->get_stream_format(rs::stream::color) != rs::format::bgra8) {
      throw vpException(vpException::fatalError, "Color stream type must be rgb8, rgba8, bgr8 or bgra8!");
    }

    int depth_width = it_intrinsics_depth->second.width;
    int depth_height = it_intrinsics_depth->second.height;
    pointcloud->width = (uint32_t)depth_width;
    pointcloud->height = (uint32_t)depth_height;
    pointcloud->resize((size_t)(depth_width * depth_height));

    const float depth_scale = m_device->get_depth_scale();
    rs::extrinsics depth_2_color_extrinsic = m_device->get_extrinsics(stream_depth, stream_color);

    const uint16_t *depth = (uint16_t *)m_device->get_frame_data(stream_depth);
    const unsigned char *color = (unsigned char *)m_device->get_frame_data(stream_color);
    int color_width = it_intrinsics_color->second.width;
    int color_height = it_intrinsics_color->second.height;

    bool swap_rgb = m_device->get_stream_format(rs::stream::color) == rs::format::bgr8 ||
                    m_device->get_stream_format(rs::stream::color) == rs::format::bgra8;
    unsigned int nb_color_pixel = (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8 ||
                                   m_device->get_stream_format(rs::stream::color) == rs::format::bgr8)
                                      ? 3
                                      : 4;

    int nb_threads = (int)std::thread::hardware_concurrency();
    int step = depth_height / nb_threads;

    std::vector<std::thread> workers;
    for (int index = 0; index < nb_threads; index++) {
      int start_index = index * step;
      int end_index = index == nb_threads - 1 ? depth_height : (index + 1) * step;

      workers.push_back(std::thread([&, start_index, end_index]() {
        for (int i = start_index; i < end_index; i++) {
          for (int j = 0; j < depth_width; j++) {
            float scaled_depth = depth[i * depth_width + j] * depth_scale;

            rs::float2 depth_pixel = {(float)j, (float)i};
            rs::float3 depth_point = it_intrinsics_depth->second.deproject(depth_pixel, scaled_depth);

            if (depth_point.z <= 0 || depth_point.z > max_Z) {
              depth_point.x = depth_point.y = depth_point.z = invalidDepthValue;
            }
            pointcloud->points[(size_t)(i * depth_width + j)].x = depth_point.x;
            pointcloud->points[(size_t)(i * depth_width + j)].y = depth_point.y;
            pointcloud->points[(size_t)(i * depth_width + j)].z = depth_point.z;

            rs::float3 color_point = depth_2_color_extrinsic.transform(depth_point);
            rs::float2 color_pixel = it_intrinsics_color->second.project(color_point);

            if (color_pixel.y < 0 || color_pixel.y >= color_height || color_pixel.x < 0 ||
                color_pixel.x >= color_width) {
// For out of bounds color data, default to a shade of blue in order to
// visually distinguish holes. This color value is same as the librealsense
// out of bounds color value.
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
              unsigned int r = 96, g = 157, b = 198;
              uint32_t rgb =
                  (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

              pointcloud->points[(size_t)(i * depth_width + j)].rgb = *reinterpret_cast<float *>(&rgb);
#else
              pointcloud->points[(size_t)(i * depth_width + j)].r = (uint8_t)96;
              pointcloud->points[(size_t)(i * depth_width + j)].g = (uint8_t)157;
              pointcloud->points[(size_t)(i * depth_width + j)].b = (uint8_t)198;
#endif
            } else {
              unsigned int i_ = (unsigned int)color_pixel.y;
              unsigned int j_ = (unsigned int)color_pixel.x;

#if PCL_VERSION_COMPARE(<, 1, 1, 0)
              uint32_t rgb = 0;
              if (swap_rgb) {
                rgb = (static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) |
                       static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1]) << 8 |
                       static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2]) << 16);
              } else {
                rgb = (static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) << 16 |
                       static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1]) << 8 |
                       static_cast<uint32_t>(color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2]));
              }

              pointcloud->points[(size_t)(i * depth_width + j)].rgb = *reinterpret_cast<float *>(&rgb);
#else
              if (swap_rgb) {
                pointcloud->points[(size_t)(i * depth_width + j)].b =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel];
                pointcloud->points[(size_t)(i * depth_width + j)].g =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1];
                pointcloud->points[(size_t)(i * depth_width + j)].r =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2];
              } else {
                pointcloud->points[(size_t)(i * depth_width + j)].r =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel];
                pointcloud->points[(size_t)(i * depth_width + j)].g =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1];
                pointcloud->points[(size_t)(i * depth_width + j)].b =
                    (uint32_t)color[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2];
              }
#endif
            }
          }
        }
      }));
    }

    std::for_each(workers.begin(), workers.end(), [](std::thread &t) { t.join(); });
  } else {
    pointcloud->clear();
  }
}
#endif
#endif
