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

#include <iostream>
#include <iomanip>

#include <visp3/core/vpImageConvert.h>
#include <visp3/sensor/vpRealSense.h>

#ifdef VISP_HAVE_REALSENSE

#include "vpRealSense_impl.h"

vpRealSense::vpRealSense()
  : m_context(), m_device(NULL), m_num_devices(0), m_serial_no(), m_intrinsics(), m_max_Z(8), m_enable_color(true), m_enable_depth(true), m_enable_point_cloud(true)
{

}

vpRealSense::~vpRealSense()
{
  close();
}

/*!
  Open access to the RealSense device and start the streaming.
 */
void vpRealSense::open()
{
  m_num_devices = m_context.get_device_count();

  if(m_num_devices == 0)
    throw vpException(vpException::fatalError, "RealSense Camera - No device detected.");

  std::vector<rs::device *> detected_devices;
  rs::device *device;
  for (int i = 0; i < m_num_devices; ++i) {
    device = m_context.get_device(i);
    std::string serial_no = device->get_serial();
    if (serial_no.compare(m_serial_no) == 0) {
      m_device = device;
    }
    detected_devices.push_back(device);
  }

  // Exit with error if no serial number is specified and multiple cameras are detected.
  if ((m_serial_no.empty()) && (m_num_devices > 1)) {
    throw vpException(vpException::fatalError, "RealSense Camera - Multiple cameras detected (%d) but no input serial number specified. Exiting!", m_num_devices);
  }
  // Exit with error if no camera is detected that matches the input serial number.
  if ((! m_serial_no.empty()) && (m_device == NULL)) {
    throw vpException(vpException::fatalError, "RealSense Camera - No camera detected with input serial_no \"%s\" Exiting!", m_serial_no.c_str());
  }

  // At this point, m_device will be null if no input serial number was specified and only one camera is connected.
  // This is a valid use case and the code will proceed.
  m_device = m_context.get_device(0);

  std::cout << "RealSense Camera - Connecting to camera with Serial No: " << m_device->get_serial() << std::endl;

  if (m_enable_color)
    m_device->enable_stream(rs::stream::color, rs::preset::best_quality);

  if (m_enable_depth) {
    m_device->enable_stream(rs::stream::depth, rs::preset::best_quality);
    m_device->enable_stream(rs::stream::infrared, rs::preset::best_quality);
    try { m_device->enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0); } catch(...) {}
  }

  // Compute field of view for each enabled stream
  m_intrinsics.clear();
  std::streamsize ss = std::cout.precision();
  for(int i = 0; i < 4; ++i)
  {
    auto stream = rs::stream(i);
    if(!m_device->is_stream_enabled(stream)) continue;
    auto intrin = m_device->get_stream_intrinsics(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;

    m_intrinsics.push_back(intrin);
  }
  std::cout.precision(ss);

  // Start device
  m_device->start();
}

/*!
  Stop device streaming.
 */
void vpRealSense::close()
{
  if (m_device) {
    if(m_device->is_streaming()) {
      m_device->stop();
    }
    m_device = NULL;
  }
}

/*!
  Set active RealSence device using its serial number.
  \param serial_no : Device serial number.

  \code
#include <visp3/sensor/vpRealSense.h>

int main()
{
  vpRealSense rs;
  rs.setDeviceBySerialNumber("541142003219");
  rs.open();
}
  \endcode
 */
void vpRealSense::setDeviceBySerialNumber(const std::string &serial_no)
{
  if (m_serial_no.empty()) {
    throw vpException(vpException::fatalError, "RealSense Camera - Multiple cameras detected (%d) but no input serial number specified. Exiting!", m_num_devices);
  }

  m_serial_no = serial_no;
}

/*!
   Return camera parameters corresponding to a specific stream.
   \param stream : color, depth, infrared or infrared2 stream for which camera parameters are returned.
   \param type : Indicate if the model should include distorsion paramater or not.
 */

vpCameraParameters vpRealSense::getCameraParameters(const rs::stream &stream, vpCameraParameters::vpCameraParametersProjType type) const
{
  auto intrinsics = m_device->get_stream_intrinsics(stream);

  vpCameraParameters cam;
  double px = intrinsics.ppx;
  double py = intrinsics.ppy;
  double u0 = intrinsics.fx;
  double v0 = intrinsics.fy;
  if (type == vpCameraParameters::perspectiveProjWithDistortion) {
    double kdu = intrinsics.coeffs[0];
    cam.initPersProjWithDistortion(px, py, u0, v0, -kdu, kdu);
  }
  else {
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
  }
  return cam;
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, std::vector<vpPoint3dTextured> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  if (m_device->is_stream_enabled(rs::stream::color)) {
    int color_width = m_intrinsics[RS_STREAM_COLOR].width;
    int color_height = m_intrinsics[RS_STREAM_COLOR].height;
    color.resize(color_height, color_width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8)
      vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)color.bitmap, color_width, color_height);
    else
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Compute point cloud
  if (m_enable_point_cloud) {

    pointcloud.clear();

    if (m_enable_depth) {
      const float depth_scale = m_device->get_depth_scale();

      rs::extrinsics depth_2_color_extrinsic = m_device->get_extrinsics(rs::stream::depth, rs::stream::color);

      // Fill the PointCloud2 fields.
      vpPoint3dTextured p3d;
      rs::float3 depth_point, color_point;
      rs::float2 color_pixel;

      for (int i = 0; i < m_intrinsics[RS_STREAM_DEPTH].height; i++) {
        for (int j = 0; j < m_intrinsics[RS_STREAM_DEPTH].width; j++) {
          float scaled_depth = depth[i][j] * depth_scale;

          rs::float2 depth_pixel = { (float) j, (float) i};
          depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

          if (depth_point.z <= 0 || depth_point.z > m_max_Z) {
            depth_point.x = depth_point.y = depth_point.z = 0;
          }
          p3d.setXYZ(depth_point.x, depth_point.y, depth_point.z);

          if (m_enable_color) {
            color_point = depth_2_color_extrinsic.transform(depth_point);
            color_pixel = m_intrinsics[RS_STREAM_COLOR].project(color_point);

            if (color_pixel.y < 0 || color_pixel.y >= color.getHeight()
                || color_pixel.x < 0 || color_pixel.x >= color.getWidth())
            {
              // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
              // This color value is same as the librealsense out of bounds color value.
              p3d.setRGB(96, 157, 198);
            }
            else
            {
              unsigned int i_ = (unsigned int) color_pixel.y;
              unsigned int j_ = (unsigned int) color_pixel.x;
              vpRGBa rgba = color[i_][j_];

              p3d.setRGB(rgba.R, rgba.G, rgba.B);
            }
          }
          pointcloud.push_back(p3d);
        }
      }
    }
  }

}

#ifdef VISP_HAVE_PCL
/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  if (m_device->is_stream_enabled(rs::stream::color)) {
    int color_width = m_intrinsics[RS_STREAM_COLOR].width;
    int color_height = m_intrinsics[RS_STREAM_COLOR].height;
    color.resize(color_height, color_width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8)
      vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)color.bitmap, color_width, color_height);
    else
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Compute point cloud
  if (m_enable_point_cloud) {
    int width = m_intrinsics[RS_STREAM_DEPTH].width;
    int height = m_intrinsics[RS_STREAM_DEPTH].height;
    pointcloud->width = width;
    pointcloud->height = height;
    pointcloud->resize(pointcloud->width * pointcloud->height);

    if (m_enable_depth) {
      const float depth_scale = m_device->get_depth_scale();

      // Fill the PointCloud2 fields.
      rs::float3 depth_point;

      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          float scaled_depth = depth[i][j] * depth_scale;

          rs::float2 depth_pixel = { (float) j, (float) i};
          depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

          if (depth_point.z <= 0 || depth_point.z > m_max_Z) {
            depth_point.x = depth_point.y = depth_point.z = 0;
          }
          pointcloud->points[i*width + j].x = depth_point.x;
          pointcloud->points[i*width + j].y = depth_point.y;
          pointcloud->points[i*width + j].z = depth_point.z;
        }
      }
    }
  }
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  if (m_device->is_stream_enabled(rs::stream::color)) {
    int color_width = m_intrinsics[RS_STREAM_COLOR].width;
    int color_height = m_intrinsics[RS_STREAM_COLOR].height;
    color.resize(color_height, color_width);

    if (m_device->get_stream_format(rs::stream::color) == rs::format::rgb8)
      vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)color.bitmap, color_width, color_height);
    else
      throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Compute point cloud
  if (m_enable_point_cloud) {
    int width = m_intrinsics[RS_STREAM_DEPTH].width;
    int height = m_intrinsics[RS_STREAM_DEPTH].height;
    pointcloud->width = width;
    pointcloud->height = height;
    pointcloud->resize(pointcloud->width * pointcloud->height);

    if (m_enable_depth) {
      const float depth_scale = m_device->get_depth_scale();

      rs::extrinsics depth_2_color_extrinsic = m_device->get_extrinsics(rs::stream::depth, rs::stream::color);

      // Fill the PointCloud2 fields.
      rs::float3 depth_point, color_point;
      rs::float2 color_pixel;

      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          float scaled_depth = depth[i][j] * depth_scale;

          rs::float2 depth_pixel = { (float) j, (float) i};
          depth_point = m_intrinsics[RS_STREAM_DEPTH].deproject(depth_pixel, scaled_depth);

          if (depth_point.z <= 0 || depth_point.z > m_max_Z) {
            depth_point.x = depth_point.y = depth_point.z = 0;
          }
          pointcloud->points[i*width + j].x = depth_point.x;
          pointcloud->points[i*width + j].y = depth_point.y;
          pointcloud->points[i*width + j].z = depth_point.z;

          if (m_enable_color) {
            color_point = depth_2_color_extrinsic.transform(depth_point);
            color_pixel = m_intrinsics[RS_STREAM_COLOR].project(color_point);


            if (color_pixel.y < 0 || color_pixel.y >= color.getHeight()
                || color_pixel.x < 0 || color_pixel.x >= color.getWidth())
            {
              // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
              // This color value is same as the librealsense out of bounds color value.
              unsigned int r = 96, g = 157, b = 198;
              uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

              pointcloud->points[i*width + j].rgb = *reinterpret_cast<float*>(&rgb);
            }
            else
            {
              unsigned int i_ = (unsigned int) color_pixel.y;
              unsigned int j_ = (unsigned int) color_pixel.x;

              uint32_t rgb = (static_cast<uint32_t>(color[i_][j_].R) << 16 |
                            static_cast<uint32_t>(color[i_][j_].G) << 8 | static_cast<uint32_t>(color[i_][j_].B));
              pointcloud->points[i*width + j].rgb = *reinterpret_cast<float*>(&rgb);
            }
          }

        }
      }
    }
  }
}

#endif

/*!
  \relates vpRealSense

  Return information from sensor.
  \param os : Input stream.
  \param rs : RealSense interface.

  The following example shows how to use this method.
  \code
#include <visp3/sensor/vpRealSense.h>

int main()
{
  vpRealSense rs;
  rs.open();
  std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;
}
  \endcode
 */
std::ostream & operator<<(std::ostream &os, const vpRealSense &rs)
{
  os << "Device name: " << rs.getHandler()->get_name() << std::endl;
  return os;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpRealSense.cpp.o) has no symbols
void dummy_vpRealSense() {};
#endif
