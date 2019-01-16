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

#include <iomanip>
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/sensor/vpRealSense.h>

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

#include "vpRealSense_impl.h"

/*!
 * Default constructor.
 */
vpRealSense::vpRealSense()
  : m_context(), m_device(NULL), m_num_devices(0), m_serial_no(), m_intrinsics(), m_max_Z(8), m_enableStreams(),
    m_useStreamPresets(), m_streamPresets(), m_streamParams(), m_invalidDepthValue(0.0f)
{
  initStream();
}

/*!
 * Default destructor that stops the streaming.
 * \sa stop()
 */
vpRealSense::~vpRealSense() { close(); }

void vpRealSense::initStream()
{
  // General default presets
  // Color stream
  m_useStreamPresets[rs::stream::color] = true;
  m_streamPresets[rs::stream::color] = rs::preset::best_quality;
  m_streamParams[rs::stream::color] = vpRsStreamParams(640, 480, rs::format::rgba8, 60);

  // Depth stream
  m_useStreamPresets[rs::stream::depth] = true;
  m_streamPresets[rs::stream::depth] = rs::preset::best_quality;
  m_streamParams[rs::stream::depth] = vpRsStreamParams(640, 480, rs::format::z16, 60);

  // Infrared stream
  m_useStreamPresets[rs::stream::infrared] = true;
  m_streamPresets[rs::stream::infrared] = rs::preset::best_quality;
  m_streamParams[rs::stream::infrared] = vpRsStreamParams(640, 480, rs::format::y16, 60);

  // Infrared stream 2
  m_useStreamPresets[rs::stream::infrared2] = true;
  m_streamPresets[rs::stream::infrared2] = rs::preset::best_quality;
  m_streamParams[rs::stream::infrared2] = vpRsStreamParams(640, 480, rs::format::y16, 60);

  // Enable all streams
  m_enableStreams[rs::stream::color] = true;
  m_enableStreams[rs::stream::depth] = true;
  m_enableStreams[rs::stream::infrared] = true;
  m_enableStreams[rs::stream::infrared2] = true;
}

/*!
  Open access to the RealSense device and start the streaming.
 */
void vpRealSense::open()
{
  m_num_devices = m_context.get_device_count();

  if (m_num_devices == 0)
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

  // Exit with error if no serial number is specified and multiple cameras are
  // detected.
  if ((m_serial_no.empty()) && (m_num_devices > 1)) {
    throw vpException(vpException::fatalError,
                      "RealSense Camera - Multiple cameras detected (%d) but "
                      "no input serial number specified. Exiting!",
                      m_num_devices);
  }
  // Exit with error if no camera is detected that matches the input serial
  // number.
  if ((!m_serial_no.empty()) && (m_device == NULL)) {
    throw vpException(vpException::fatalError,
                      "RealSense Camera - No camera detected with input "
                      "serial_no \"%s\" Exiting!",
                      m_serial_no.c_str());
  }

  // At this point, m_device will be null if no input serial number was
  // specified and only one camera is connected. This is a valid use case and
  // the code will proceed.
  m_device = m_context.get_device(0);
  m_serial_no = m_device->get_serial();

  std::cout << "RealSense Camera - Connecting to camera with Serial No: " << m_serial_no << std::endl;

  // Enable only infrared2 stream if supported
  m_enableStreams[rs::stream::infrared2] = m_enableStreams[rs::stream::infrared2]
                                               ? m_device->supports(rs::capabilities::infrared2)
                                               : m_enableStreams[rs::stream::infrared2];

  if (m_device->is_streaming()) {
    m_device->stop();
  }

  for (int j = 0; j < 4; j++) {
    auto s = (rs::stream)j;
    auto capabilities = (rs::capabilities)j;
    if (m_device->supports(capabilities) && m_device->is_stream_enabled(s)) {
      m_device->disable_stream(s);
    }
  }

  if (m_enableStreams[rs::stream::color]) {
    if (m_useStreamPresets[rs::stream::color]) {
      m_device->enable_stream(rs::stream::color, m_streamPresets[rs::stream::color]);
    } else {
      m_device->enable_stream(rs::stream::color, m_streamParams[rs::stream::color].m_streamWidth,
                              m_streamParams[rs::stream::color].m_streamHeight,
                              m_streamParams[rs::stream::color].m_streamFormat,
                              m_streamParams[rs::stream::color].m_streamFramerate);
    }
  }

  if (m_enableStreams[rs::stream::depth]) {
    if (m_useStreamPresets[rs::stream::depth]) {
      m_device->enable_stream(rs::stream::depth, m_streamPresets[rs::stream::depth]);
    } else {
      m_device->enable_stream(rs::stream::depth, m_streamParams[rs::stream::depth].m_streamWidth,
                              m_streamParams[rs::stream::depth].m_streamHeight,
                              m_streamParams[rs::stream::depth].m_streamFormat,
                              m_streamParams[rs::stream::depth].m_streamFramerate);
    }
  }

  if (m_enableStreams[rs::stream::infrared]) {
    if (m_useStreamPresets[rs::stream::infrared]) {
      m_device->enable_stream(rs::stream::infrared, m_streamPresets[rs::stream::infrared]);
    } else {
      m_device->enable_stream(rs::stream::infrared, m_streamParams[rs::stream::infrared].m_streamWidth,
                              m_streamParams[rs::stream::infrared].m_streamHeight,
                              m_streamParams[rs::stream::infrared].m_streamFormat,
                              m_streamParams[rs::stream::infrared].m_streamFramerate);
    }
  }

  if (m_enableStreams[rs::stream::infrared2]) {
    if (m_useStreamPresets[rs::stream::infrared2]) {
      m_device->enable_stream(rs::stream::infrared2, m_streamPresets[rs::stream::infrared2]);
    } else {
      m_device->enable_stream(rs::stream::infrared2, m_streamParams[rs::stream::infrared2].m_streamWidth,
                              m_streamParams[rs::stream::infrared2].m_streamHeight,
                              m_streamParams[rs::stream::infrared2].m_streamFormat,
                              m_streamParams[rs::stream::infrared2].m_streamFramerate);
    }
  }

  // Compute field of view for each enabled stream
  m_intrinsics.clear();
  for (int i = 0; i < 4; ++i) {
    auto stream = rs::stream(i);
    if (!m_device->is_stream_enabled(stream))
      continue;
    auto intrin = m_device->get_stream_intrinsics(stream);

    m_intrinsics[stream] = intrin;
  }

  // Add synthetic stream intrinsics
  if (m_enableStreams[rs::stream::color]) {
    m_intrinsics[rs::stream::rectified_color] = m_device->get_stream_intrinsics(rs::stream::rectified_color);

    if (m_enableStreams[rs::stream::depth]) {
      m_intrinsics[rs::stream::color_aligned_to_depth] =
          m_device->get_stream_intrinsics(rs::stream::color_aligned_to_depth);
      m_intrinsics[rs::stream::depth_aligned_to_color] =
          m_device->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
      m_intrinsics[rs::stream::depth_aligned_to_rectified_color] =
          m_device->get_stream_intrinsics(rs::stream::depth_aligned_to_rectified_color);
    }
  }

  if (m_enableStreams[rs::stream::depth] && m_enableStreams[rs::stream::infrared2]) {
    m_intrinsics[rs::stream::depth_aligned_to_infrared2] =
        m_device->get_stream_intrinsics(rs::stream::depth_aligned_to_infrared2);
    m_intrinsics[rs::stream::infrared2_aligned_to_depth] =
        m_device->get_stream_intrinsics(rs::stream::infrared2_aligned_to_depth);
  }

  // Start device
  m_device->start();
}

/*!
  Stop device streaming.
 */
void vpRealSense::close()
{
  if (m_device) {
    if (m_device->is_streaming()) {
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
    throw vpException(vpException::fatalError,
                      "RealSense Camera - Multiple cameras detected (%d) but "
                      "no input serial number specified. Exiting!",
                      m_num_devices);
  }

  m_serial_no = serial_no;
}

/*!
   Return camera parameters corresponding to a specific stream. This function
   has to be called after open(). \param stream : color, depth, infrared or
   infrared2 stream for which camera intrinsic parameters are returned. \param
   type : Indicate if the model should include distorsion paramater or not.

   \sa getIntrinsics()
 */

vpCameraParameters vpRealSense::getCameraParameters(const rs::stream &stream,
                                                    vpCameraParameters::vpCameraParametersProjType type) const
{
  auto intrinsics = this->getIntrinsics(stream);

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
   be called after open(). \param stream : color, depth, infrared or infrared2
   stream for which camera intrinsic parameters are returned. \sa
   getCameraParameters()
  */
rs::intrinsics vpRealSense::getIntrinsics(const rs::stream &stream) const
{
  if (!m_device) {
    throw vpException(vpException::fatalError, "RealSense Camera - device handler is null. Cannot "
                                               "retrieve intrinsics. Exiting!");
  }

  return m_device->get_stream_intrinsics(stream);
}

/*!
   Get extrinsic transformation from one stream to an other. This function has
   to be called after open(). \param from, to : color, depth, infrared or
   infrared2 stream between which camera extrinsic parameters are returned.

   \sa getTransformation()
  */
rs::extrinsics vpRealSense::getExtrinsics(const rs::stream &from, const rs::stream &to) const
{
  if (!m_device) {
    throw vpException(vpException::fatalError, "RealSense Camera - device handler is null. Cannot "
                                               "retrieve extrinsics. Exiting!");
  }
  if (!m_device->is_stream_enabled(from)) {
    throw vpException(vpException::fatalError,
                      "RealSense Camera - stream (%d) is not enabled to "
                      "retrieve extrinsics. Exiting!",
                      (int)from);
  }
  if (!m_device->is_stream_enabled(to)) {
    throw vpException(vpException::fatalError,
                      "RealSense Camera - stream (%d) is not enabled to "
                      "retrieve extrinsics. Exiting!",
                      (int)to);
  }
  return m_device->get_extrinsics(from, to);
}

/*!
   Get extrinsic transformation from one stream to an other. This function has
   to be called after open(). \param from, to : color, depth, infrared or
   infrared2 stream between which camera extrinsic parameters are returned.

   \sa getExtrinsics()
  */
vpHomogeneousMatrix vpRealSense::getTransformation(const rs::stream &from, const rs::stream &to) const
{
  rs::extrinsics extrinsics = this->getExtrinsics(from, to);
  vpTranslationVector t;
  vpRotationMatrix R;
  for (unsigned int i = 0; i < 3; i++) {
    t[i] = extrinsics.translation[i];
    for (unsigned int j = 0; j < 3; j++)
      R[i][j] = extrinsics.rotation[j * 3 + i]; //rotation is column-major order
  }
  vpHomogeneousMatrix fMt(t, R);
  return fMt;
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each
  column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of
  a point.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param pointcloud : Point cloud data as a vector of column vectors. Each
  column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of
  a point.
 */
void vpRealSense::acquire(std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each
  column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of
  a point.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each
  column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of
  a point.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each
  column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of
  a point.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param data_infrared : Infrared image buffer or NULL if not wanted.
  \param data_infrared2 : Infrared (for the second IR camera if available)
  image buffer or NULL if not wanted. \param stream_color : Type of color
  stream (e.g. rs::stream::color, rs::stream::rectified_color,
  rs::stream::color_aligned_to_depth). \param stream_depth : Type of depth
  stream (e.g. rs::stream::depth, rs::stream::depth_aligned_to_color,
  rs::stream::depth_aligned_to_rectified_color,
  rs::stream::depth_aligned_to_infrared2). \param stream_infrared : Type of
  infrared stream (only rs::stream::infrared should be possible). \param
  stream_infrared2 : Type of infrared2 stream (e.g. rs::stream::infrared2,
  rs::stream::infrared2_aligned_to_depth) if supported by the camera.
 */
void vpRealSense::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                          std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared,
                          unsigned char *const data_infrared2, const rs::stream &stream_color,
                          const rs::stream &stream_depth, const rs::stream &stream_infrared,
                          const rs::stream &stream_infrared2)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }

  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  if (data_image != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::color, data_image, stream_color);
  }

  if (data_depth != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, data_depth, stream_depth);

    if (data_pointCloud != NULL) {
      vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, *data_pointCloud, m_invalidDepthValue, stream_depth);
    }
  }

  if (data_infrared != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, data_infrared, stream_infrared);
  }

  if (data_infrared2 != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared2, data_infrared2, stream_infrared2);
  }
}

/*!
  Set stream settings.
  \param stream : Stream type (color / depth / infrared).
  \param preset : Preset to use.
 */
void vpRealSense::setStreamSettings(const rs::stream &stream, const rs::preset &preset)
{
  m_useStreamPresets[stream] = true;
  m_streamPresets[stream] = preset;
}

/*!
  Set stream settings.
  \param stream : Stream type (color / depth / infrared).
  \param params : Stream parameters to use.

  \note You can find the stream settings of the different Intel RealSense
cameras at this <a
href=https://github.com/IntelRealSense/librealsense/blob/v1.11.0/doc/supported_video_formats.pdf>url</a>.

  \code
#include <visp3/sensor/vpRealSense.h>

int main()
{
  vpRealSense rs;

  rs.setEnableStream(rs::stream::color, true);
  rs.setEnableStream(rs::stream::depth, false);
  rs.setEnableStream(rs::stream::infrared, false);
  rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(1920, 1080, rs::format::rgba8, 30));

  rs.open();

  //[...]
}
  \endcode
 */
void vpRealSense::setStreamSettings(const rs::stream &stream, const vpRsStreamParams &params)
{
  m_useStreamPresets[stream] = false;
  m_streamParams[stream] = params;
}

/*!
  Enable or disable a stream.
  \param stream : Stream type (color / depth / infrared).
  \param status : Stream status.

  \code
#include <visp3/sensor/vpRealSense.h>

int main()
{
  vpRealSense rs;

  rs.setEnableStream(rs::stream::color, true);
  rs.setEnableStream(rs::stream::depth, false);
  rs.setEnableStream(rs::stream::infrared, false);
  rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(1920, 1080, rs::format::rgba8, 30));

  rs.open();

  //[...]
}
  \endcode
 */
void vpRealSense::setEnableStream(const rs::stream &stream, const bool status) { m_enableStreams[stream] = status; }

#ifdef VISP_HAVE_PCL
/*!
  Acquire data from RealSense device.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve infrared image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, infrared);

  // Retrieve depth image
  vp_rs_get_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, depth);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue);
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and without texture
  information) pointer or NULL if not wanted. \param data_infrared : Infrared
  image buffer or NULL if not wanted. \param data_infrared2 : Infrared (for
  the second IR camera if available) image buffer or NULL if not wanted.
  \param stream_color : Type of color stream (e.g. rs::stream::color,
  rs::stream::rectified_color, rs::stream::color_aligned_to_depth). \param
  stream_depth : Type of depth stream (e.g. rs::stream::depth,
  rs::stream::depth_aligned_to_color,
  rs::stream::depth_aligned_to_rectified_color,
  rs::stream::depth_aligned_to_infrared2). \param stream_infrared : Type of
  infrared stream (only rs::stream::infrared should be possible). \param
  stream_infrared2 : Type of infrared2 stream (e.g. rs::stream::infrared2,
  rs::stream::infrared2_aligned_to_depth) if supported by the camera.
 */
void vpRealSense::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                          std::vector<vpColVector> *const data_pointCloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, unsigned char *const data_infrared,
                          unsigned char *const data_infrared2, const rs::stream &stream_color,
                          const rs::stream &stream_depth, const rs::stream &stream_infrared,
                          const rs::stream &stream_infrared2)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }

  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  if (data_image != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::color, data_image, stream_color);
  }

  if (data_depth != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, data_depth, stream_depth);
  }

  if (data_pointCloud != NULL) {
    vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, *data_pointCloud, m_invalidDepthValue, stream_depth);
  }

  if (pointcloud != NULL) {
    vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue, stream_depth);
  }

  if (data_infrared != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, data_infrared, stream_infrared);
  }

  if (data_infrared2 != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared2, data_infrared2, stream_infrared2);
  }
}

/*!
  Acquire data from RealSense device.
  \param data_image : Color image buffer or NULL if not wanted.
  \param data_depth : Depth image buffer or NULL if not wanted.
  \param data_pointCloud : Point cloud vector pointer or NULL if not wanted.
  \param pointcloud : Point cloud (in PCL format and with texture information)
  pointer or NULL if not wanted. \param data_infrared : Infrared image buffer
  or NULL if not wanted. \param data_infrared2 : Infrared (for the second IR
  camera if available) image buffer or NULL if not wanted. \param stream_color
  : Type of color stream (e.g. rs::stream::color, rs::stream::rectified_color,
  rs::stream::color_aligned_to_depth). \param stream_depth : Type of depth
  stream (e.g. rs::stream::depth, rs::stream::depth_aligned_to_color,
  rs::stream::depth_aligned_to_rectified_color,
  rs::stream::depth_aligned_to_infrared2). \param stream_infrared : Type of
  infrared stream (only rs::stream::infrared should be possible). \param
  stream_infrared2 : Type of infrared2 stream (e.g. rs::stream::infrared2,
  rs::stream::infrared2_aligned_to_depth) if supported by the camera.
 */
void vpRealSense::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                          std::vector<vpColVector> *const data_pointCloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud, unsigned char *const data_infrared,
                          unsigned char *const data_infrared2, const rs::stream &stream_color,
                          const rs::stream &stream_depth, const rs::stream &stream_infrared,
                          const rs::stream &stream_infrared2)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }

  if (!m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  if (data_image != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::color, data_image, stream_color);
  }

  if (data_depth != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::depth, data_depth, stream_depth);
  }

  if (data_pointCloud != NULL) {
    vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, *data_pointCloud, m_invalidDepthValue, stream_depth);
  }

  if (pointcloud != NULL) {
    vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud, m_invalidDepthValue, stream_color,
                              stream_depth);
  }

  if (data_infrared != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared, data_infrared, stream_infrared);
  }

  if (data_infrared2 != NULL) {
    vp_rs_get_native_frame_data_impl(m_device, m_intrinsics, rs::stream::infrared2, data_infrared2, stream_infrared2);
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
  return 0;
}
  \endcode
 */
std::ostream &operator<<(std::ostream &os, const vpRealSense &rs)
{
  os << "Device name: " << rs.getHandler()->get_name() << std::endl;
  std::streamsize ss = std::cout.precision();
  for (int i = 0; i < 4; ++i) {
    auto stream = rs::stream(i);
    if (!rs.getHandler()->is_stream_enabled(stream))
      continue;
    auto intrin = rs.getHandler()->get_stream_intrinsics(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov()
              << ", distortion = " << intrin.model() << std::endl;
  }
  std::cout.precision(ss);

  return os;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpRealSense.cpp.o) has no
// symbols
void dummy_vpRealSense(){};
#endif
