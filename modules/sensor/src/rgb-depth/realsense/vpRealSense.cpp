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

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

#include "vpRealSense_impl.h"

/*!
 * Default constructor.
 */
vpRealSense::vpRealSense()
  : m_context(), m_device(NULL), m_num_devices(0), m_serial_no(), m_intrinsics(), m_max_Z(8), m_enable_color(true), m_enable_depth(true)
{

}

/*!
 * Default destructor that stops the streaming.
 * \sa stop()
 */
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
  for(int i = 0; i < 4; ++i) {
    auto stream = rs::stream(i);
    if(!m_device->is_stream_enabled(stream)) continue;
    auto intrin = m_device->get_stream_intrinsics(stream);

    m_intrinsics.push_back(intrin);
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
   Return camera parameters corresponding to a specific stream. This function has to be called after open().
   \param stream : color, depth, infrared or infrared2 stream for which camera intrinsic parameters are returned.
   \param type : Indicate if the model should include distorsion paramater or not.

   \sa getIntrinsics()
 */

vpCameraParameters vpRealSense::getCameraParameters(const rs::stream &stream, vpCameraParameters::vpCameraParametersProjType type) const
{
  auto intrinsics = this->getIntrinsics(stream);

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
   Get intrinsic parameters corresponding to the stream. This function has to be called after open().
   \param stream : color, depth, infrared or infrared2 stream for which camera intrinsic parameters are returned.
   \sa getCameraParameters()
  */
rs::intrinsics vpRealSense::getIntrinsics(const rs::stream &stream) const
{
  if (! m_device) {
    throw vpException(vpException::fatalError, "RealSense Camera - device handler is null. Cannot retrieve intrinsics. Exiting!");
  }
  if(!m_device->is_stream_enabled(stream)) {
    throw vpException(vpException::fatalError, "RealSense Camera - stream (%d) is not enabled to retrieve intrinsics. Exiting!", (int)stream);
  }
  return m_intrinsics[(int)stream];
}

/*!
   Get extrinsic transformation from one stream to an other. This function has to be called after open().
   \param from, to : color, depth, infrared or infrared2 stream between which camera extrinsic parameters are returned.

   \sa getTransformation()
  */
rs::extrinsics vpRealSense::getExtrinsics(const rs::stream &from, const rs::stream &to) const
{
  if (! m_device) {
    throw vpException(vpException::fatalError, "RealSense Camera - device handler is null. Cannot retrieve extrinsics. Exiting!");
  }
  if(!m_device->is_stream_enabled(from)) {
    throw vpException(vpException::fatalError, "RealSense Camera - stream (%d) is not enabled to retrieve extrinsics. Exiting!", (int)from);
  }
  if(!m_device->is_stream_enabled(to)) {
    throw vpException(vpException::fatalError, "RealSense Camera - stream (%d) is not enabled to retrieve extrinsics. Exiting!", (int)to);
  }
  return m_device->get_extrinsics(from, to);
}

/*!
   Get extrinsic transformation from one stream to an other. This function has to be called after open().
   \param from, to : color, depth, infrared or infrared2 stream between which camera extrinsic parameters are returned.

   \sa getExtrinsics()
  */
vpHomogeneousMatrix vpRealSense::getTransformation(const rs::stream &from, const rs::stream &to) const
{
  rs::extrinsics extrinsics = this->getExtrinsics(from, to);
  vpTranslationVector t;
  vpRotationMatrix R;
  for(unsigned int i=0; i<3; i++) {
    t[i] = extrinsics.translation[i];
    for(unsigned int j=0; j<3; j++)
      R[i][j] = extrinsics.rotation[i*3+j];
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
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of a point.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param pointcloud : Point cloud data as a vector of column vectors. Each column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of a point.
 */
void vpRealSense::acquire(std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
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
  if (! m_device->is_streaming()) {
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
  \param pointcloud : Point cloud data as a vector of column vectors. Each column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of a point.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of a point.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param pointcloud : Point cloud data as a vector of column vectors. Each column vector is 4-dimension and contains X,Y,Z,1 normalized coordinates of a point.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, std::vector<vpColVector> &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

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
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
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
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
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
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve grey image
  vp_rs_get_grey_impl(m_device, m_intrinsics, grey);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
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
  if (! m_device->is_streaming()) {
    open();
  }

  m_device->wait_for_frames();

  // Retrieve color image
  vp_rs_get_color_impl(m_device, m_intrinsics, color);

  // Retrieve point cloud
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data information.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param color : Color image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
}

/*!
  Acquire data from RealSense device.
  \param grey : Grey level image.
  \param infrared : Infrared image.
  \param depth : Depth image.
  \param pointcloud : Point cloud data with texture information.
 */
void vpRealSense::acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  if (m_device == NULL) {
    throw vpException(vpException::fatalError, "RealSense Camera - Device not opened!");
  }
  if (! m_device->is_streaming()) {
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
  vp_rs_get_pointcloud_impl(m_device, m_intrinsics, m_max_Z, pointcloud);
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
  std::streamsize ss = std::cout.precision();
  for(int i = 0; i < 4; ++i)
  {
    auto stream = rs::stream(i);
    if(!rs.getHandler()->is_stream_enabled(stream)) continue;
    auto intrin = rs.getHandler()->get_stream_intrinsics(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
  }
  std::cout.precision(ss);

  return os;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpRealSense.cpp.o) has no symbols
void dummy_vpRealSense() {};
#endif
