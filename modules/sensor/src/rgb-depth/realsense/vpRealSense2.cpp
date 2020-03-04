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
 * librealSense2 interface.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <iomanip>
#include <map>
#include <set>
#include <cstring>
#include <visp3/core/vpImageConvert.h>
#include <visp3/sensor/vpRealSense2.h>

#define MANUAL_POINTCLOUD 1

namespace {
bool operator==(const rs2_extrinsics &lhs, const rs2_extrinsics &rhs)
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (std::fabs(lhs.rotation[i*3 + j] - rhs.rotation[i*3 + j]) >
          std::numeric_limits<float>::epsilon()) {
        return false;
      }
    }

    if (std::fabs(lhs.translation[i] - rhs.translation[i]) >
        std::numeric_limits<float>::epsilon()) {
      return false;
    }
  }

  return true;
}
}

/*!
 * Default constructor.
 */
vpRealSense2::vpRealSense2()
  : m_depthScale(0.0f), m_invalidDepthValue(0.0f),
    m_max_Z(8.0f), m_pipe(), m_pipelineProfile(), m_pointcloud(), m_points()
{
}

/*!
 * Default destructor that stops the streaming.
 * \sa stop()
 */
vpRealSense2::~vpRealSense2() { close(); }

/*!
  Acquire greyscale image from RealSense device.
  \param grey : Greyscale image.
 */
void vpRealSense2::acquire(vpImage<unsigned char> &grey)
{
  auto data = m_pipe.wait_for_frames();
  auto color_frame = data.get_color_frame();
  getGreyFrame(color_frame, grey);
}

/*!
  Acquire color image from RealSense device.
  \param color : Color image.
 */
void vpRealSense2::acquire(vpImage<vpRGBa> &color)
{
  auto data = m_pipe.wait_for_frames();
  auto color_frame = data.get_color_frame();
  getColorFrame(color_frame, color);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param data_infrared : Infrared image buffer or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared,
                           rs2::align *const align_to)
{
  acquire(data_image, data_depth, data_pointCloud, data_infrared, NULL, align_to);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param data_infrared1 : First infrared image buffer or NULL if not wanted.
  \param data_infrared2 : Second infrared image buffer (if supported by the device)
  or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.

  The following code shows how to use this function to get color, infrared 1 and infrared 2 frames
  acquired by a D435 device:
  \code
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

int main() {
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
  rs.open(config);
  vpImage<vpRGBa> Ic(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);
  vpImage<unsigned char> Ii1(rs.getIntrinsics(RS2_STREAM_INFRARED).height,
                             rs.getIntrinsics(RS2_STREAM_INFRARED).width);
  vpImage<unsigned char> Ii2(rs.getIntrinsics(RS2_STREAM_INFRARED).height,
                             rs.getIntrinsics(RS2_STREAM_INFRARED).width);

#ifdef VISP_HAVE_X11
  vpDisplayX dc(Ic, 0, 0, "Color");
  vpDisplayX di1(Ii1, 100, 100, "Infrared 1");
  vpDisplayX di2(Ii2, 200, 200, "Infrared 2");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc(Ic, 0, 0, "Color");
  vpDisplayGDI di1(Ii1, 100, 100, "Infrared 1");
  vpDisplayGDI di2(Ii2, 100, 100, "Infrared 2");
#endif

  while (true) {
    rs.acquire((unsigned char *) Ic.bitmap, NULL, NULL, Ii1.bitmap, Ii2.bitmap, NULL);
    vpDisplay::display(Ic);
    vpDisplay::display(Ii1);
    vpDisplay::display(Ii2);
    vpDisplay::flush(Ic);
    vpDisplay::flush(Ii1);
    vpDisplay::flush(Ii2);
    if (vpDisplay::getClick(Ic, false) || vpDisplay::getClick(Ii1, false) || vpDisplay::getClick(Ii2, false))
      break;
  }
  return 0;
}
  \endcode
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared1,
                           unsigned char *const data_infrared2, rs2::align *const align_to)
{
  auto data = m_pipe.wait_for_frames();
  if (align_to != NULL) {
    // Infrared stream is not aligned
    // see https://github.com/IntelRealSense/librealsense/issues/1556#issuecomment-384919994
#if (RS2_API_VERSION > ((2 * 10000) + (9 * 100) + 0))
    data = align_to->process(data);
#else
    data = align_to->proccess(data);
#endif
  }

  if (data_image != NULL) {
    auto color_frame = data.get_color_frame();
    getNativeFrameData(color_frame, data_image);
  }

  if (data_depth != NULL || data_pointCloud != NULL) {
    auto depth_frame = data.get_depth_frame();
    if (data_depth != NULL) {
      getNativeFrameData(depth_frame, data_depth);
    }

    if (data_pointCloud != NULL) {
      getPointcloud(depth_frame, *data_pointCloud);
    }
  }

  if (data_infrared1 != NULL) {
    auto infrared_frame = data.first(RS2_STREAM_INFRARED);
    getNativeFrameData(infrared_frame, data_infrared1);
  }

  if (data_infrared2 != NULL) {
    auto infrared_frame = data.get_infrared_frame(2);
    getNativeFrameData(infrared_frame, data_infrared2);
  }
}

#ifdef VISP_HAVE_PCL
/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and without texture
  information) pointer or NULL if not wanted.
  \param data_infrared : Infrared image buffer or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, unsigned char *const data_infrared,
                           rs2::align *const align_to)
{
  acquire(data_image, data_depth, data_pointCloud, pointcloud, data_infrared, NULL, align_to);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and without texture
  information) pointer or NULL if not wanted.
  \param data_infrared1 : First infrared image buffer or NULL if not wanted.
  \param data_infrared2 : Second infrared image (if supported by the device)
  buffer or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, unsigned char *const data_infrared1,
                           unsigned char *const data_infrared2, rs2::align *const align_to)
{
  auto data = m_pipe.wait_for_frames();
  if (align_to != NULL) {
    // Infrared stream is not aligned
    // see https://github.com/IntelRealSense/librealsense/issues/1556#issuecomment-384919994
#if (RS2_API_VERSION > ((2 * 10000) + (9 * 100) + 0))
    data = align_to->process(data);
#else
    data = align_to->proccess(data);
#endif
  }

  if (data_image != NULL) {
    auto color_frame = data.get_color_frame();
    getNativeFrameData(color_frame, data_image);
  }

  if (data_depth != NULL || data_pointCloud != NULL || pointcloud != NULL) {
    auto depth_frame = data.get_depth_frame();
    if (data_depth != NULL) {
      getNativeFrameData(depth_frame, data_depth);
    }

    if (data_pointCloud != NULL) {
      getPointcloud(depth_frame, *data_pointCloud);
    }

    if (pointcloud != NULL) {
      getPointcloud(depth_frame, pointcloud);
    }
  }

  if (data_infrared1 != NULL) {
    auto infrared_frame = data.first(RS2_STREAM_INFRARED);
    getNativeFrameData(infrared_frame, data_infrared1);
  }

  if (data_infrared2 != NULL) {
    auto infrared_frame = data.get_infrared_frame(2);
    getNativeFrameData(infrared_frame, data_infrared2);
  }
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and with texture information)
  pointer or NULL if not wanted.
  \param data_infrared : Infrared image buffer or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud, unsigned char *const data_infrared,
                           rs2::align *const align_to)
{
  acquire(data_image, data_depth, data_pointCloud, pointcloud, data_infrared, NULL, align_to);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and with texture information)
  pointer or NULL if not wanted.
  \param data_infrared1 : First infrared image buffer or NULL if not wanted.
  \param data_infrared2 : Second infrared image (if supported by the device)
  buffer or NULL if not wanted.
  \param align_to : Align to a reference stream or NULL if not wanted.
  Only depth and color streams can be aligned.
 */
void vpRealSense2::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                           std::vector<vpColVector> *const data_pointCloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud, unsigned char *const data_infrared1,
                           unsigned char *const data_infrared2, rs2::align *const align_to)
{
  auto data = m_pipe.wait_for_frames();
  if (align_to != NULL) {
    // Infrared stream is not aligned
    // see https://github.com/IntelRealSense/librealsense/issues/1556#issuecomment-384919994
#if (RS2_API_VERSION > ((2 * 10000) + (9 * 100) + 0))
    data = align_to->process(data);
#else
    data = align_to->proccess(data);
#endif
  }

  auto color_frame = data.get_color_frame();
  if (data_image != NULL) {
    getNativeFrameData(color_frame, data_image);
  }

  if (data_depth != NULL || data_pointCloud != NULL || pointcloud != NULL) {
    auto depth_frame = data.get_depth_frame();
    if (data_depth != NULL) {
      getNativeFrameData(depth_frame, data_depth);
    }

    if (data_pointCloud != NULL) {
      getPointcloud(depth_frame, *data_pointCloud);
    }

    if (pointcloud != NULL) {
      getPointcloud(depth_frame, color_frame, pointcloud);
    }
  }

  if (data_infrared1 != NULL) {
    auto infrared_frame = data.first(RS2_STREAM_INFRARED);
    getNativeFrameData(infrared_frame, data_infrared1);
  }

  if (data_infrared2 != NULL) {
    auto infrared_frame = data.get_infrared_frame(2);
    getNativeFrameData(infrared_frame, data_infrared2);
  }
}
#endif

/*!
  librealsense documentation:
  <blockquote>
  Stop the pipeline streaming.
  The pipeline stops delivering samples to the attached computer vision
  modules and processing blocks, stops the device streaming and releases the
  device resources used by the pipeline. It is the application's
  responsibility to release any frame reference it owns. The method takes
  effect only after \c start() was called, otherwise an exception is raised.
  </blockquote>
 */
void vpRealSense2::close() { m_pipe.stop(); }

/*!
   Return the camera parameters corresponding to a specific stream. This
   function has to be called after open().
   \param stream : stream for which camera intrinsic parameters are returned.
   \param type : Indicate if the model should include distorsion parameters or not.

   \sa getIntrinsics()
 */
vpCameraParameters vpRealSense2::getCameraParameters(const rs2_stream &stream,
                                                     vpCameraParameters::vpCameraParametersProjType type) const
{
  auto rs_stream = m_pipelineProfile.get_stream(stream).as<rs2::video_stream_profile>();
  auto intrinsics = rs_stream.get_intrinsics();

  vpCameraParameters cam;
  double u0 = intrinsics.ppx;
  double v0 = intrinsics.ppy;
  double px = intrinsics.fx;
  double py = intrinsics.fy;

  if (type == vpCameraParameters::perspectiveProjWithDistortion) {
    double kdu = intrinsics.coeffs[0];
    cam.initPersProjWithDistortion(px, py, u0, v0, -kdu, kdu);
  } else {
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
  }

  return cam;
}

/*!
   Get intrinsic parameters corresponding to the stream. This function has to
   be called after open().
   \param stream : stream for which the camera intrinsic parameters are returned.

   \sa getCameraParameters()
  */
rs2_intrinsics vpRealSense2::getIntrinsics(const rs2_stream &stream) const
{
  auto vsp = m_pipelineProfile.get_stream(stream).as<rs2::video_stream_profile>();
  return vsp.get_intrinsics();
}

void vpRealSense2::getColorFrame(const rs2::frame &frame, vpImage<vpRGBa> &color)
{
  auto vf = frame.as<rs2::video_frame>();
  unsigned int width = (unsigned int)vf.get_width();
  unsigned int height = (unsigned int)vf.get_height();
  color.resize(height, width);

  if (frame.get_profile().format() == RS2_FORMAT_RGB8) {
    vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
                              reinterpret_cast<unsigned char *>(color.bitmap), width, height);
  } else if (frame.get_profile().format() == RS2_FORMAT_RGBA8) {
    memcpy(reinterpret_cast<unsigned char *>(color.bitmap),
           const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
           width * height * sizeof(vpRGBa));
  } else if (frame.get_profile().format() == RS2_FORMAT_BGR8) {
    vpImageConvert::BGRToRGBa(const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
                              reinterpret_cast<unsigned char *>(color.bitmap), width, height);
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
}

/*!
   Get depth scale value used to convert all the uint16_t values contained in a depth
   frame into a distance in meter.
  */
float vpRealSense2::getDepthScale()
{
  return m_depthScale;
}

void vpRealSense2::getGreyFrame(const rs2::frame &frame, vpImage<unsigned char> &grey)
{
  auto vf = frame.as<rs2::video_frame>();
  unsigned int width = (unsigned int)vf.get_width();
  unsigned int height = (unsigned int)vf.get_height();
  grey.resize(height, width);

  if (frame.get_profile().format() == RS2_FORMAT_RGB8) {
    vpImageConvert::RGBToGrey(const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
                              grey.bitmap, width, height);
  } else if (frame.get_profile().format() == RS2_FORMAT_RGBA8) {
    vpImageConvert::RGBaToGrey(const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
                               grey.bitmap, width * height);
  } else if (frame.get_profile().format() == RS2_FORMAT_BGR8) {
    vpImageConvert::BGRToGrey(const_cast<unsigned char *>(static_cast<const unsigned char *>(frame.get_data())),
                              grey.bitmap, width, height);
  } else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not supported!");
  }
}

void vpRealSense2::getNativeFrameData(const rs2::frame &frame, unsigned char *const data)
{
  auto vf = frame.as<rs2::video_frame>();
  int size = vf.get_width() * vf.get_height();

  switch (frame.get_profile().format()) {
  case RS2_FORMAT_RGB8:
  case RS2_FORMAT_BGR8:
    memcpy(data, frame.get_data(), size * 3);
    break;

  case RS2_FORMAT_RGBA8:
  case RS2_FORMAT_BGRA8:
    memcpy(data, frame.get_data(), size * 4);
    break;

  case RS2_FORMAT_Y16:
  case RS2_FORMAT_Z16:
    memcpy(data, frame.get_data(), size * 2);
    break;

  case RS2_FORMAT_Y8:
    memcpy(data, frame.get_data(), size);
    break;

  default:
    break;
  }
}

void vpRealSense2::getPointcloud(const rs2::depth_frame &depth_frame, std::vector<vpColVector> &pointcloud)
{
  if (m_depthScale <= std::numeric_limits<float>::epsilon()) {
    std::stringstream ss;
    ss << "Error, depth scale <= 0: " << m_depthScale;
    throw vpException(vpException::fatalError, ss.str());
  }

  auto vf = depth_frame.as<rs2::video_frame>();
  const int width = vf.get_width();
  const int height = vf.get_height();
  pointcloud.resize((size_t)(width * height));

  const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
  const rs2_intrinsics depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  // Multi-threading if OpenMP
  // Concurrent writes at different locations are safe
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < height; i++) {
    auto depth_pixel_index = i * width;

    for (int j = 0; j < width; j++, depth_pixel_index++) {
      if (p_depth_frame[depth_pixel_index] == 0) {
        pointcloud[(size_t)depth_pixel_index].resize(4, false);
        pointcloud[(size_t)depth_pixel_index][0] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][1] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][2] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][3] = 1.0;
        continue;
      }

      // Get the depth value of the current pixel
      auto pixels_distance = m_depthScale * p_depth_frame[depth_pixel_index];

      float points[3];
      const float pixel[] = {(float)j, (float)i};
      rs2_deproject_pixel_to_point(points, &depth_intrinsics, pixel, pixels_distance);

      if (pixels_distance > m_max_Z)
        points[0] = points[1] = points[2] = m_invalidDepthValue;

      pointcloud[(size_t)depth_pixel_index].resize(4, false);
      pointcloud[(size_t)depth_pixel_index][0] = points[0];
      pointcloud[(size_t)depth_pixel_index][1] = points[1];
      pointcloud[(size_t)depth_pixel_index][2] = points[2];
      pointcloud[(size_t)depth_pixel_index][3] = 1.0;
    }
  }
}


#ifdef VISP_HAVE_PCL
void vpRealSense2::getPointcloud(const rs2::depth_frame &depth_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_depthScale <= std::numeric_limits<float>::epsilon()) {
    std::stringstream ss;
    ss << "Error, depth scale <= 0: " << m_depthScale;
    throw vpException(vpException::fatalError, ss.str());
  }

  auto vf = depth_frame.as<rs2::video_frame>();
  const int width = vf.get_width();
  const int height = vf.get_height();
  pointcloud->width = (uint32_t)width;
  pointcloud->height = (uint32_t)height;
  pointcloud->resize((size_t)(width * height));

#if MANUAL_POINTCLOUD // faster to compute manually when tested
  const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
  const rs2_intrinsics depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  // Multi-threading if OpenMP
  // Concurrent writes at different locations are safe
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < height; i++) {
    auto depth_pixel_index = i * width;

    for (int j = 0; j < width; j++, depth_pixel_index++) {
      if (p_depth_frame[depth_pixel_index] == 0) {
        pointcloud->points[(size_t)(depth_pixel_index)].x = m_invalidDepthValue;
        pointcloud->points[(size_t)(depth_pixel_index)].y = m_invalidDepthValue;
        pointcloud->points[(size_t)(depth_pixel_index)].z = m_invalidDepthValue;
        continue;
      }

      // Get the depth value of the current pixel
      auto pixels_distance = m_depthScale * p_depth_frame[depth_pixel_index];

      float points[3];
      const float pixel[] = {(float)j, (float)i};
      rs2_deproject_pixel_to_point(points, &depth_intrinsics, pixel, pixels_distance);

      if (pixels_distance > m_max_Z)
        points[0] = points[1] = points[2] = m_invalidDepthValue;

      pointcloud->points[(size_t)(depth_pixel_index)].x = points[0];
      pointcloud->points[(size_t)(depth_pixel_index)].y = points[1];
      pointcloud->points[(size_t)(depth_pixel_index)].z = points[2];
    }
  }
#else
  m_points = m_pointcloud.calculate(depth_frame);
  auto vertices = m_points.get_vertices();

  for (size_t i = 0; i < m_points.size(); i++) {
    if (vertices[i].z <= 0.0f || vertices[i].z > m_max_Z) {
      pointcloud->points[i].x = m_invalidDepthValue;
      pointcloud->points[i].y = m_invalidDepthValue;
      pointcloud->points[i].z = m_invalidDepthValue;
    } else {
      pointcloud->points[i].x = vertices[i].x;
      pointcloud->points[i].y = vertices[i].y;
      pointcloud->points[i].z = vertices[i].z;
    }
  }
#endif
}

void vpRealSense2::getPointcloud(const rs2::depth_frame &depth_frame, const rs2::frame &color_frame,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_depthScale <= std::numeric_limits<float>::epsilon()) {
    throw vpException(vpException::fatalError, "Error, depth scale <= 0: %f", m_depthScale);
  }

  auto vf = depth_frame.as<rs2::video_frame>();
  const int depth_width = vf.get_width();
  const int depth_height = vf.get_height();
  pointcloud->width = static_cast<uint32_t>(depth_width);
  pointcloud->height = static_cast<uint32_t>(depth_height);
  pointcloud->resize(static_cast<uint32_t>(depth_width * depth_height));

  vf = color_frame.as<rs2::video_frame>();
  const int color_width = vf.get_width();
  const int color_height = vf.get_height();

  const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
  const rs2_extrinsics depth2ColorExtrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().
      get_extrinsics_to(color_frame.get_profile().as<rs2::video_stream_profile>());
  const rs2_intrinsics depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
  const rs2_intrinsics color_intrinsics = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  auto color_format = color_frame.as<rs2::video_frame>().get_profile().format();
  const bool swap_rb = color_format == RS2_FORMAT_BGR8 || color_format == RS2_FORMAT_BGRA8;
  unsigned int nb_color_pixel = (color_format == RS2_FORMAT_RGB8 || color_format == RS2_FORMAT_BGR8) ? 3 : 4;
  const unsigned char *p_color_frame = static_cast<const unsigned char *>(color_frame.get_data());
  rs2_extrinsics identity;
  memset(identity.rotation, 0, sizeof(identity.rotation));
  memset(identity.translation, 0, sizeof(identity.translation));
  for (int i = 0; i < 3; i++) {
    identity.rotation[i*3 + i] = 1;
  }
  const bool registered_streams = (depth2ColorExtrinsics == identity) &&
                                  (color_width == depth_width) && (color_height == depth_height);

  // Multi-threading if OpenMP
  // Concurrent writes at different locations are safe
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < depth_height; i++) {
    auto depth_pixel_index = i * depth_width;

    for (int j = 0; j < depth_width; j++, depth_pixel_index++) {
      if (p_depth_frame[depth_pixel_index] == 0) {
        pointcloud->points[(size_t)depth_pixel_index].x = m_invalidDepthValue;
        pointcloud->points[(size_t)depth_pixel_index].y = m_invalidDepthValue;
        pointcloud->points[(size_t)depth_pixel_index].z = m_invalidDepthValue;

        // For out of bounds color data, default to a shade of blue in order to
        // visually distinguish holes. This color value is same as the librealsense
        // out of bounds color value.
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
        unsigned int r = 96, g = 157, b = 198;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        pointcloud->points[(size_t)depth_pixel_index].rgb = *reinterpret_cast<float *>(&rgb);
#else
        pointcloud->points[(size_t)depth_pixel_index].r = (uint8_t)96;
        pointcloud->points[(size_t)depth_pixel_index].g = (uint8_t)157;
        pointcloud->points[(size_t)depth_pixel_index].b = (uint8_t)198;
#endif
        continue;
      }

      // Get the depth value of the current pixel
      auto pixels_distance = m_depthScale * p_depth_frame[depth_pixel_index];

      float depth_point[3];
      const float pixel[] = {(float)j, (float)i};
      rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, pixel, pixels_distance);

      if (pixels_distance > m_max_Z) {
        depth_point[0] = depth_point[1] = depth_point[2] = m_invalidDepthValue;
      }

      pointcloud->points[(size_t)depth_pixel_index].x = depth_point[0];
      pointcloud->points[(size_t)depth_pixel_index].y = depth_point[1];
      pointcloud->points[(size_t)depth_pixel_index].z = depth_point[2];

      if (!registered_streams) {
        float color_point[3];
        rs2_transform_point_to_point(color_point, &depth2ColorExtrinsics, depth_point);
        float color_pixel[2];
        rs2_project_point_to_pixel(color_pixel, &color_intrinsics, color_point);

        if (color_pixel[1] < 0 || color_pixel[1] >= color_height || color_pixel[0] < 0 || color_pixel[0] >= color_width) {
          // For out of bounds color data, default to a shade of blue in order to
          // visually distinguish holes. This color value is same as the librealsense
          // out of bounds color value.
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
          unsigned int r = 96, g = 157, b = 198;
          uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

          pointcloud->points[(size_t)depth_pixel_index].rgb = *reinterpret_cast<float *>(&rgb);
#else
          pointcloud->points[(size_t)depth_pixel_index].r = (uint8_t)96;
          pointcloud->points[(size_t)depth_pixel_index].g = (uint8_t)157;
          pointcloud->points[(size_t)depth_pixel_index].b = (uint8_t)198;
#endif
        } else {
          unsigned int i_ = (unsigned int)color_pixel[1];
          unsigned int j_ = (unsigned int)color_pixel[0];

#if PCL_VERSION_COMPARE(<, 1, 1, 0)
          uint32_t rgb = 0;
          if (swap_rb) {
            rgb =
                (static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) |
                 static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1]) << 8 |
                 static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2]) << 16);
          } else {
            rgb = (static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) << 16 |
                   static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1]) << 8 |
                   static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2]));
          }

          pointcloud->points[(size_t)(i * depth_width + j)].rgb = *reinterpret_cast<float *>(&rgb);
#else
          if (swap_rb) {
            pointcloud->points[(size_t)depth_pixel_index].b =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel];
            pointcloud->points[(size_t)depth_pixel_index].g =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1];
            pointcloud->points[(size_t)depth_pixel_index].r =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2];
          } else {
            pointcloud->points[(size_t)depth_pixel_index].r =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel];
            pointcloud->points[(size_t)depth_pixel_index].g =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1];
            pointcloud->points[(size_t)depth_pixel_index].b =
                p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2];
          }
#endif
        }
      } else {
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
          uint32_t rgb = 0;
          if (swap_rb) {
            rgb =
                (static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel]) |
                 static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 1]) << 8 |
                 static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 2]) << 16);
          } else {
            rgb = (static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel]) << 16 |
                   static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 1]) << 8 |
                   static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 2]));
          }

          pointcloud->points[(size_t)(i * depth_width + j)].rgb = *reinterpret_cast<float *>(&rgb);
#else
          if (swap_rb) {
            pointcloud->points[(size_t)depth_pixel_index].b =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel];
            pointcloud->points[(size_t)depth_pixel_index].g =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 1];
            pointcloud->points[(size_t)depth_pixel_index].r =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 2];
          } else {
            pointcloud->points[(size_t)depth_pixel_index].r =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel];
            pointcloud->points[(size_t)depth_pixel_index].g =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 1];
            pointcloud->points[(size_t)depth_pixel_index].b =
                p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 2];
          }
#endif
      }
    }
  }
}

#endif

/*!
   Get the extrinsic transformation from one stream to another. This function
   has to be called after open().
   \param from, to : streams for which the camera extrinsic parameters are returned.
  */
vpHomogeneousMatrix vpRealSense2::getTransformation(const rs2_stream &from, const rs2_stream &to) const
{
  auto from_stream = m_pipelineProfile.get_stream(from);
  auto to_stream = m_pipelineProfile.get_stream(to);
  rs2_extrinsics extrinsics = from_stream.get_extrinsics_to(to_stream);

  vpTranslationVector t;
  vpRotationMatrix R;
  for (unsigned int i = 0; i < 3; i++) {
    t[i] = extrinsics.translation[i];
    for (unsigned int j = 0; j < 3; j++)
      R[i][j] = extrinsics.rotation[j * 3 + i]; //rotation is column-major order
  }

  vpHomogeneousMatrix to_M_from(t, R);
  return to_M_from;
}

/*!
  Open access to the RealSense device and start the streaming.
 */
void vpRealSense2::open(const rs2::config &cfg)
{
  m_pipelineProfile = m_pipe.start(cfg);

  rs2::device dev = m_pipelineProfile.get_device();

  // Go over the device's sensors
  for (rs2::sensor &sensor : dev.query_sensors()) {
    // Check if the sensor is a depth sensor
    if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
      m_depthScale = dpt.get_depth_scale();
    }
  }
}

namespace
{
// Helper functions to print information about the RealSense device
void print(const rs2_extrinsics &extrinsics, std::ostream &os)
{
  std::stringstream ss;
  ss << "Rotation Matrix:\n";

  for (auto i = 0; i < 3; ++i) {
    for (auto j = 0; j < 3; ++j) {
      ss << std::left << std::setw(15) << std::setprecision(5) << extrinsics.rotation[j * 3 + i];
    }
    ss << std::endl;
  }

  ss << "\nTranslation Vector: ";
  for (size_t i = 0; i < sizeof(extrinsics.translation) / sizeof(extrinsics.translation[0]); ++i)
    ss << std::setprecision(15) << extrinsics.translation[i] << "  ";

  os << ss.str() << "\n\n";
}

void print(const rs2_intrinsics &intrinsics, std::ostream &os)
{
  std::stringstream ss;
  ss << std::left << std::setw(14) << "Width: "
     << "\t" << intrinsics.width << "\n"
     << std::left << std::setw(14) << "Height: "
     << "\t" << intrinsics.height << "\n"
     << std::left << std::setw(14) << "PPX: "
     << "\t" << std::setprecision(15) << intrinsics.ppx << "\n"
     << std::left << std::setw(14) << "PPY: "
     << "\t" << std::setprecision(15) << intrinsics.ppy << "\n"
     << std::left << std::setw(14) << "Fx: "
     << "\t" << std::setprecision(15) << intrinsics.fx << "\n"
     << std::left << std::setw(14) << "Fy: "
     << "\t" << std::setprecision(15) << intrinsics.fy << "\n"
     << std::left << std::setw(14) << "Distortion: "
     << "\t" << rs2_distortion_to_string(intrinsics.model) << "\n"
     << std::left << std::setw(14) << "Coeffs: ";

  for (size_t i = 0; i < sizeof(intrinsics.coeffs) / sizeof(intrinsics.coeffs[0]); ++i)
    ss << "\t" << std::setprecision(15) << intrinsics.coeffs[i] << "  ";

  os << ss.str() << "\n\n";
}

void safe_get_intrinsics(const rs2::video_stream_profile &profile, rs2_intrinsics &intrinsics)
{
  try {
    intrinsics = profile.get_intrinsics();
  } catch (...) {
  }
}

bool operator==(const rs2_intrinsics &lhs, const rs2_intrinsics &rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.ppx == rhs.ppx && lhs.ppy == rhs.ppy &&
         lhs.fx == rhs.fx && lhs.fy == rhs.fy && lhs.model == rhs.model &&
         !std::memcmp(lhs.coeffs, rhs.coeffs, sizeof(rhs.coeffs));
}

std::string get_str_formats(const std::set<rs2_format> &formats)
{
  std::stringstream ss;
  for (auto format = formats.begin(); format != formats.end(); ++format) {
    ss << *format << ((format != formats.end()) && (next(format) == formats.end()) ? "" : "/");
  }
  return ss.str();
}

struct stream_and_resolution {
  rs2_stream stream;
  int stream_index;
  int width;
  int height;
  std::string stream_name;

  bool operator<(const stream_and_resolution &obj) const
  {
    return (std::make_tuple(stream, stream_index, width, height) <
            std::make_tuple(obj.stream, obj.stream_index, obj.width, obj.height));
  }
};

struct stream_and_index {
  rs2_stream stream;
  int stream_index;

  bool operator<(const stream_and_index &obj) const
  {
    return (std::make_tuple(stream, stream_index) < std::make_tuple(obj.stream, obj.stream_index));
  }
};
} // anonymous namespace

/*!
  \relates vpRealSense2

  Return information from the sensor.
  \param os : Input stream.
  \param rs : RealSense object.

  The following example shows how to use this method.
  \code
#include <visp3/sensor/vpRealSense2.h>

int main()
{
  vpRealSense2 rs;
  rs.open();
  std::cout << "RealSense sensor information:\n" << rs << std::endl;
  return 0;
}
  \endcode
 */
std::ostream &operator<<(std::ostream &os, const vpRealSense2 &rs)
{
  os << std::left << std::setw(30) << "Device Name" << std::setw(20) << "Serial Number" << std::setw(20)
     << "Firmware Version" << std::endl;

  rs2::device dev = rs.m_pipelineProfile.get_device();
  os << std::left << std::setw(30) << dev.get_info(RS2_CAMERA_INFO_NAME) << std::setw(20)
     << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::setw(20) << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
     << std::endl;

  // Show which options are supported by this device
  os << " Device info: \n";
  for (auto j = 0; j < RS2_CAMERA_INFO_COUNT; ++j) {
    auto param = static_cast<rs2_camera_info>(j);
    if (dev.supports(param))
      os << "    " << std::left << std::setw(30) << rs2_camera_info_to_string(rs2_camera_info(param)) << ": \t"
         << dev.get_info(param) << "\n";
  }

  os << "\n";

  for (auto &&sensor : dev.query_sensors()) {
    os << "Options for " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

    os << std::setw(55) << " Supported options:" << std::setw(10) << "min" << std::setw(10) << " max" << std::setw(6)
       << " step" << std::setw(10) << " default" << std::endl;
    for (auto j = 0; j < RS2_OPTION_COUNT; ++j) {
      auto opt = static_cast<rs2_option>(j);
      if (sensor.supports(opt)) {
        auto range = sensor.get_option_range(opt);
        os << "    " << std::left << std::setw(50) << opt << " : " << std::setw(5) << range.min << "... "
           << std::setw(12) << range.max << std::setw(6) << range.step << std::setw(10) << range.def << "\n";
      }
    }

    os << "\n";
  }

  for (auto &&sensor : dev.query_sensors()) {
    os << "Stream Profiles supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << "\n";

    os << std::setw(55) << " Supported modes:" << std::setw(10) << "stream" << std::setw(10) << " resolution"
       << std::setw(6) << " fps" << std::setw(10) << " format"
       << "\n";
    // Show which streams are supported by this device
    for (auto &&profile : sensor.get_stream_profiles()) {
      if (auto video = profile.as<rs2::video_stream_profile>()) {
        os << "    " << profile.stream_name() << "\t  " << video.width() << "x" << video.height() << "\t@ "
           << profile.fps() << "Hz\t" << profile.format() << "\n";
      } else {
        os << "    " << profile.stream_name() << "\t@ " << profile.fps() << "Hz\t" << profile.format() << "\n";
      }
    }

    os << "\n";
  }

  std::map<stream_and_index, rs2::stream_profile> streams;
  std::map<stream_and_resolution, std::vector<std::pair<std::set<rs2_format>, rs2_intrinsics> > > intrinsics_map;
  for (auto &&sensor : dev.query_sensors()) {
    // Intrinsics
    for (auto &&profile : sensor.get_stream_profiles()) {
      if (auto video = profile.as<rs2::video_stream_profile>()) {
        if (streams.find(stream_and_index{profile.stream_type(), profile.stream_index()}) == streams.end()) {
          streams[stream_and_index{profile.stream_type(), profile.stream_index()}] = profile;
        }

        rs2_intrinsics intrinsics{};
        stream_and_resolution stream_res{profile.stream_type(), profile.stream_index(), video.width(), video.height(),
                                         profile.stream_name()};
        safe_get_intrinsics(video, intrinsics);
        auto it = std::find_if(
            (intrinsics_map[stream_res]).begin(), (intrinsics_map[stream_res]).end(),
            [&](const std::pair<std::set<rs2_format>, rs2_intrinsics> &kvp) { return intrinsics == kvp.second; });
        if (it == (intrinsics_map[stream_res]).end()) {
          (intrinsics_map[stream_res]).push_back({{profile.format()}, intrinsics});
        } else {
          it->first.insert(profile.format()); // If the intrinsics are equals,
                                              // add the profile format to
                                              // format set
        }
      }
    }
  }

  os << "Provided Intrinsic:\n";
  for (auto &kvp : intrinsics_map) {
    auto stream_res = kvp.first;
    for (auto &intrinsics : kvp.second) {
      auto formats = get_str_formats(intrinsics.first);
      os << "Intrinsic of \"" << stream_res.stream_name << "\"\t  " << stream_res.width << "x" << stream_res.height
         << "\t  " << formats << "\n";
      if (intrinsics.second == rs2_intrinsics{}) {
        os << "Intrinsic NOT available!\n\n";
      } else {
        print(intrinsics.second, os);
      }
    }
  }

  // Print Extrinsics
  os << "\nProvided Extrinsic:\n";
  for (auto kvp1 = streams.begin(); kvp1 != streams.end(); ++kvp1) {
    for (auto kvp2 = streams.begin(); kvp2 != streams.end(); ++kvp2) {
      os << "Extrinsic from \"" << kvp1->second.stream_name() << "\"\t  "
         << "To"
         << "\t  \"" << kvp2->second.stream_name() << "\"\n";
      auto extrinsics = kvp1->second.get_extrinsics_to(kvp2->second);
      print(extrinsics, os);
    }
  }

  return os;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpRealSense2.cpp.o) has no
// symbols
void dummy_vpRealSense2(){};
#endif
