/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * libStructure interface.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_THREADS)
#include <cstring>
#include <functional>
#include <iomanip>
#include <map>
#include <set>
#include <thread>

#include <ST/Utilities.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/sensor/vpOccipitalStructure.h>

#define MANUAL_POINTCLOUD 1

BEGIN_VISP_NAMESPACE
/*!
 * Default constructor.
 */
vpOccipitalStructure::vpOccipitalStructure() : m_invalidDepthValue(0.0f), m_maxZ(15000.0f) { }

/*!
 * Default destructor that stops the streaming.
 * \sa stop()
 */
vpOccipitalStructure::~vpOccipitalStructure() { close(); }

/*!
  Acquire greyscale image from Structure Core device.
  \param gray        : Greyscale image.
  \param undistorted : Set to true to get undistorted grayscale image.
  \param ts          : Image timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(vpImage<unsigned char> &gray, bool undistorted, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_visibleFrame.isValid()) {
    if (!undistorted)
      memcpy(gray.bitmap, m_delegate.m_visibleFrame.yData(), m_delegate.m_visibleFrame.ySize());
    else
      memcpy(gray.bitmap, m_delegate.m_visibleFrame.undistorted().yData(), m_delegate.m_visibleFrame.ySize());

    if (ts != nullptr)
      *ts = m_delegate.m_visibleFrame.arrivalTimestamp();
  }

  u.unlock();
}

/*!
  Acquire color image from Structure Core device.
  \param rgb         : RGB image.
  \param undistorted : Set to true to get undistorted image.
  \param ts          : Image timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(vpImage<vpRGBa> &rgb, bool undistorted, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_visibleFrame.isValid()) {
    // Detecting if it's a Color Structure Core device.
    if (m_delegate.m_cameraType == ST::StructureCoreCameraType::Color)
      vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.rgbData()),
                                reinterpret_cast<unsigned char *>(rgb.bitmap),
                                m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());

    else // If it's a monochrome Structure Core device.
    {
      if (!undistorted)
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.yData()),
                                   reinterpret_cast<unsigned char *>(rgb.bitmap),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
      else
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.undistorted().yData()),
                                   reinterpret_cast<unsigned char *>(rgb.bitmap),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
    }

    if (ts != nullptr)
      *ts = m_delegate.m_visibleFrame.arrivalTimestamp();
  }

  u.unlock();
}

/*!
 Acquire rgb image and IMU data from Structure Core device.
 \param rgb               : RGB image or nullptr if not wanted.
 \param depth             : Depth image or nullptr if not wanted.
 \param acceleration_data : Acceleration data or nullptr if not wanted.
 \param gyroscope_data    : Gyroscope data or nullptr if not wanted.
 \param undistorted       : Set to true to get undistorted image.
 \param ts                : Image timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(vpImage<vpRGBa> *rgb, vpImage<vpRGBa> *depth, vpColVector *acceleration_data,
                                   vpColVector *gyroscope_data, bool undistorted, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (rgb != nullptr && m_delegate.m_visibleFrame.isValid()) {
    // Detecting if it's a Color Structure Core device.
    if (m_delegate.m_cameraType == ST::StructureCoreCameraType::Color)
      vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.rgbData()),
                                reinterpret_cast<unsigned char *>(rgb->bitmap),
                                m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());

    else // If it's a monochrome Structure Core device.
    {
      if (!undistorted)
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.yData()),
                                   reinterpret_cast<unsigned char *>(rgb->bitmap),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
      else
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.undistorted().yData()),
                                   reinterpret_cast<unsigned char *>(rgb->bitmap),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
    }

    if (ts != nullptr)
      *ts = m_delegate.m_visibleFrame.arrivalTimestamp();
  }

  if (depth != nullptr && m_delegate.m_depthFrame.isValid())
    memcpy((unsigned char *)depth->bitmap, m_delegate.m_depthFrame.convertDepthToRgba(),
           m_delegate.m_depthFrame.width() * m_delegate.m_depthFrame.height() * 4);

  if (acceleration_data != nullptr) {
    ST::Acceleration accel = m_delegate.m_accelerometerEvent.acceleration();
    acceleration_data[0] = accel.x;
    acceleration_data[1] = accel.y;
    acceleration_data[2] = accel.z;
  }

  if (gyroscope_data != nullptr) {
    ST::RotationRate gyro_data = m_delegate.m_gyroscopeEvent.rotationRate();
    gyroscope_data[0] = gyro_data.x;
    gyroscope_data[1] = gyro_data.y;
    gyroscope_data[2] = gyro_data.z;
  }

  if (ts != nullptr) // By default, frames are synchronized.
    *ts = m_delegate.m_visibleFrame.arrivalTimestamp();

  u.unlock();
}

/*!
 Acquire grayscale image, depth and IMU data from Structure Core device.
 \param gray              : Gray image or nullptr if not wanted.
 \param depth             : Depth image or nullptr if not wanted.
 \param acceleration_data : Acceleration data or nullptr if not wanted.
 \param gyroscope_data    : Gyroscope data or nullptr if not wanted.
 \param undistorted       : Set to true to get undistorted image.
 \param ts                : Image timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(vpImage<unsigned char> *gray, vpImage<vpRGBa> *depth, vpColVector *acceleration_data,
                                   vpColVector *gyroscope_data, bool undistorted, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (gray != nullptr && m_delegate.m_visibleFrame.isValid()) {
    if (!undistorted)
      memcpy(gray->bitmap, m_delegate.m_visibleFrame.yData(), m_delegate.m_visibleFrame.ySize());
    else
      memcpy(gray->bitmap, m_delegate.m_visibleFrame.undistorted().yData(), m_delegate.m_visibleFrame.ySize());

    if (ts != nullptr)
      *ts = m_delegate.m_visibleFrame.arrivalTimestamp();
  }

  if (depth != nullptr && m_delegate.m_depthFrame.isValid())
    memcpy((unsigned char *)depth->bitmap, m_delegate.m_depthFrame.convertDepthToRgba(),
           m_delegate.m_depthFrame.width() * m_delegate.m_depthFrame.height() * 4);

  if (acceleration_data != nullptr) {
    ST::Acceleration accel = m_delegate.m_accelerometerEvent.acceleration();
    acceleration_data[0] = accel.x;
    acceleration_data[1] = accel.y;
    acceleration_data[2] = accel.z;
  }

  if (gyroscope_data != nullptr) {
    ST::RotationRate gyro_data = m_delegate.m_gyroscopeEvent.rotationRate();
    gyroscope_data[0] = gyro_data.x;
    gyroscope_data[1] = gyro_data.y;
    gyroscope_data[2] = gyro_data.z;
  }

  if (ts != nullptr) // By default, frames are synchronized.
    *ts = m_delegate.m_visibleFrame.arrivalTimestamp();

  u.unlock();
}

/*!
  Acquire data from Structure Core device.
  \param data_image        : Visible image buffer or nullptr if not wanted.
  \param data_depth        : Depth image buffer in millimeters or nullptr if not wanted.
  \param data_pointCloud   : Point cloud vector pointer or nullptr if not wanted.
  \param data_infrared     : Infrared image buffer or nullptr if not wanted.
  \param acceleration_data : Acceleration data or nullptr if not wanted.
  \param gyroscope_data    : Gyroscope data or nullptr if not wanted.
  \param undistorted       : Set to true if you want undistorted monochrome device image.
  \param ts                : Data timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                                   std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared,
                                   vpColVector *acceleration_data, vpColVector *gyroscope_data, bool undistorted,
                                   double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_depthFrame.isValid() && data_depth != nullptr)
    memcpy(data_depth, m_delegate.m_depthFrame.depthInMillimeters(),
           m_delegate.m_depthFrame.width() * m_delegate.m_depthFrame.height() * sizeof(float));

  if (m_delegate.m_visibleFrame.isValid() && data_image != nullptr) {
    if (m_delegate.m_cameraType == ST::StructureCoreCameraType::Color)
      vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.rgbData()),
                                reinterpret_cast<unsigned char *>(data_image),
                                m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());

    else // If it's a monochrome Structure Core device.
    {
      if (!undistorted)
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
      else
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.undistorted().yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
    }
  }

  if (m_delegate.m_infraredFrame.isValid() && data_infrared != nullptr)
    memcpy(data_infrared, m_delegate.m_infraredFrame.data(),
           m_delegate.m_infraredFrame.width() * m_delegate.m_infraredFrame.height() * sizeof(uint16_t));

  if (data_pointCloud != nullptr)
    getPointcloud(*data_pointCloud);

  if (acceleration_data != nullptr) {
    ST::Acceleration accel = m_delegate.m_accelerometerEvent.acceleration();
    acceleration_data[0] = accel.x;
    acceleration_data[1] = accel.y;
    acceleration_data[2] = accel.z;
  }

  if (gyroscope_data != nullptr) {
    ST::RotationRate gyro_data = m_delegate.m_gyroscopeEvent.rotationRate();
    gyroscope_data[0] = gyro_data.x;
    gyroscope_data[1] = gyro_data.y;
    gyroscope_data[2] = gyro_data.z;
  }

  if (ts != nullptr) // By default, frames are synchronized.
    *ts = m_delegate.m_visibleFrame.arrivalTimestamp();

  u.unlock();
}

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
/*!
  Acquire data from Structure Core device.
  \param data_image        : Color image buffer or nullptr if not wanted.
  \param data_depth        : Depth image buffer or nullptr if not wanted.
  \param data_pointCloud   : Point cloud vector pointer or nullptr if not wanted.
  \param pointcloud        : Point cloud (in PCL format and without texture information) pointer or nullptr if not wanted.
  \param data_infrared     : Infrared image buffer or nullptr if not wanted.
  \param acceleration_data : Acceleration data or nullptr if not wanted.
  \param gyroscope_data    : Gyroscope data or nullptr if not wanted.
  \param undistorted       : Set to true if you want undistorted monochrome device image.
  \param ts                : Data timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                                   std::vector<vpColVector> *const data_pointCloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, unsigned char *const data_infrared,
                                   vpColVector *acceleration_data, vpColVector *gyroscope_data, bool undistorted,
                                   double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_visibleFrame.isValid()) {
    if (m_delegate.m_cameraType == ST::StructureCoreCameraType::Color)
      vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.rgbData()),
                                reinterpret_cast<unsigned char *>(data_image),
                                m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());

    else // If it's a monochrome Structure Core device.
    {
      if (!undistorted)
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
      else
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.undistorted().yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
    }
  }

  if (m_delegate.m_depthFrame.isValid() && data_depth != nullptr)
    memcpy(data_depth, m_delegate.m_depthFrame.depthInMillimeters(),
           m_delegate.m_depthFrame.width() * m_delegate.m_depthFrame.height() * sizeof(float));

  if (m_delegate.m_infraredFrame.isValid() && data_infrared != nullptr)
    memcpy(data_infrared, m_delegate.m_infraredFrame.data(),
           m_delegate.m_infraredFrame.width() * m_delegate.m_infraredFrame.height() * sizeof(uint16_t));

  if (data_pointCloud != nullptr)
    getPointcloud(*data_pointCloud);

  if (m_delegate.m_depthFrame.isValid() && pointcloud != nullptr)
    getPointcloud(pointcloud);

  if (acceleration_data != nullptr) {
    ST::Acceleration accel = m_delegate.m_accelerometerEvent.acceleration();
    acceleration_data[0] = accel.x;
    acceleration_data[1] = accel.y;
    acceleration_data[2] = accel.z;
  }

  if (gyroscope_data != nullptr) {
    ST::RotationRate gyro_data = m_delegate.m_gyroscopeEvent.rotationRate();
    gyroscope_data[0] = gyro_data.x;
    gyroscope_data[1] = gyro_data.y;
    gyroscope_data[2] = gyro_data.z;
  }

  if (ts != nullptr) // By default, frames are synchronized.
    *ts = m_delegate.m_visibleFrame.arrivalTimestamp();

  u.unlock();
}

/*!
  Acquire data from Structure Core device.
  \param data_image        : Color image buffer or nullptr if not wanted.
  \param data_depth        : Depth image buffer or nullptr if not wanted.
  \param data_pointCloud   : Point cloud vector pointer or nullptr if not wanted.
  \param pointcloud        : Point cloud (in PCL format) pointer or nullptr if not wanted.
  \param data_infrared     : Infrared image buffer or nullptr if not wanted.
  \param acceleration_data : Acceleration data or nullptr if not wanted.
  \param gyroscope_data    : Gyroscope data or nullptr if not wanted.
  \param undistorted       : Set to true if you want undistorted monochrome device image.
  \param ts                : Data timestamp or nullptr if not wanted.
 */
void vpOccipitalStructure::acquire(unsigned char *const data_image, unsigned char *const data_depth,
                                   std::vector<vpColVector> *const data_pointCloud,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
                                   unsigned char *const data_infrared, vpColVector *acceleration_data,
                                   vpColVector *gyroscope_data, bool undistorted, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_depthFrame.isValid() && data_depth != nullptr)
    memcpy(data_depth, m_delegate.m_depthFrame.depthInMillimeters(),
           m_delegate.m_depthFrame.width() * m_delegate.m_depthFrame.height() * sizeof(float));

  if (m_delegate.m_visibleFrame.isValid() && data_image != nullptr) {
    if (m_delegate.m_cameraType == ST::StructureCoreCameraType::Color)
      vpImageConvert::RGBToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.rgbData()),
                                reinterpret_cast<unsigned char *>(data_image),
                                m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());

    else // If it's a monochrome Structure Core device.
    {
      if (!undistorted)
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
      else
        vpImageConvert::GreyToRGBa(const_cast<unsigned char *>(m_delegate.m_visibleFrame.undistorted().yData()),
                                   reinterpret_cast<unsigned char *>(data_image),
                                   m_delegate.m_visibleFrame.width() * m_delegate.m_visibleFrame.height());
    }
  }

  if (m_delegate.m_infraredFrame.isValid() && data_infrared != nullptr)
    memcpy(data_infrared, m_delegate.m_infraredFrame.data(),
           m_delegate.m_infraredFrame.width() * m_delegate.m_infraredFrame.height() * sizeof(uint16_t));

  if (m_delegate.m_depthFrame.isValid() && data_pointCloud != nullptr)
    getPointcloud(*data_pointCloud);

  if (m_delegate.m_depthFrame.isValid() && m_delegate.m_visibleFrame.isValid() && pointcloud != nullptr)
    getColoredPointcloud(pointcloud);

  if (acceleration_data != nullptr) {
    ST::Acceleration accel = m_delegate.m_accelerometerEvent.acceleration();
    acceleration_data[0] = accel.x;
    acceleration_data[1] = accel.y;
    acceleration_data[2] = accel.z;
  }

  if (gyroscope_data != nullptr) {
    ST::RotationRate gyro_data = m_delegate.m_gyroscopeEvent.rotationRate();
    gyroscope_data[0] = gyro_data.x;
    gyroscope_data[1] = gyro_data.y;
    gyroscope_data[2] = gyro_data.z;
  }

  if (ts != nullptr) // By default, frames are synchronized.
    *ts = m_delegate.m_visibleFrame.arrivalTimestamp();

  u.unlock();
}

#endif

/*!
  Get angular velocities from IMU's gyroscope.
  \param imu_vel : IMU 3-dim angular velocity vector from gyro.
  \param ts      : Timestamp.

  \note Default IMU streaming rate is 400Hz.

  \code
    vpOccipitalStructure st;
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = false;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    st.open(settings);

    vpColVector imu_vel;
    double timestamp;
    st.getIMUVelocity(&imu_vel, &timestamp);
  \endcode
 */
void vpOccipitalStructure::getIMUVelocity(vpColVector *imu_vel, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (imu_vel != nullptr) {
    imu_vel->resize(3, false);
    ST::RotationRate imu_rotationRate = m_delegate.m_gyroscopeEvent.rotationRate();
    (*imu_vel)[0] = imu_rotationRate.x;
    (*imu_vel)[1] = imu_rotationRate.y;
    (*imu_vel)[2] = imu_rotationRate.z;
  }

  if (ts != nullptr)
    *ts = m_delegate.m_gyroscopeEvent.arrivalTimestamp();

  u.unlock();
}

/*!
  Get linear acceleration from IMU's accelerometer.
  \param imu_acc : IMU 3-dim angular velocity vector from gyro.
  \param ts      : Timestamp.

  \note Default IMU streaming rate is 400Hz.

  \code
    vpOccipitalStructure st;
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = false;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    st.open(settings);

    vpColVector imu_acc;
    double timestamp;
    st.getIMUAcceleration(&imu_acc, &timestamp);
  \endcode
 */
void vpOccipitalStructure::getIMUAcceleration(vpColVector *imu_acc, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (imu_acc != nullptr) {
    imu_acc->resize(3, false);
    ST::Acceleration imu_acceleration = m_delegate.m_accelerometerEvent.acceleration();
    (*imu_acc)[0] = imu_acceleration.x;
    (*imu_acc)[1] = imu_acceleration.y;
    (*imu_acc)[2] = imu_acceleration.z;
  }

  if (ts != nullptr)
    *ts = m_delegate.m_accelerometerEvent.arrivalTimestamp();

  u.unlock();
}

/*!
  Get IMU data (angular velocities and linear acceleration) from Structure Core device.
  \param imu_vel : IMU 3-dim angular velocity vector from gyro.
  \param imu_acc : IMU 3-dim linear acceleration vector.
  \param ts      : Timestamp.

  \note Default IMU streaming rate is 400Hz.
  \note Be aware that IMU frame's X and Z axes are opposite to X and Z axes of pose frame.

  \code
    vpOccipitalStructure st;
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = false;
    settings.structureCore.accelerometerEnabled = true;
    settings.structureCore.gyroscopeEnabled = true;
    st.open(settings);

    vpColVector imu_acc, imu_vel;
    double timestamp;
    st.getIMUData(&imu_acc, &imu_vel, &timestamp);
  \endcode
 */
void vpOccipitalStructure::getIMUData(vpColVector *imu_vel, vpColVector *imu_acc, double *ts)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);
  double imu_vel_timestamp, imu_acc_timestamp;

  if (imu_vel != nullptr) {
    imu_vel->resize(3, false);
    ST::RotationRate imu_rotationRate = m_delegate.m_gyroscopeEvent.rotationRate();
    (*imu_vel)[0] = imu_rotationRate.x;
    (*imu_vel)[1] = imu_rotationRate.y;
    (*imu_vel)[2] = imu_rotationRate.z;
    imu_vel_timestamp =
      m_delegate.m_gyroscopeEvent.arrivalTimestamp(); // Relative to an unspecified epoch. (see documentation).
  }

  if (imu_acc != nullptr) {
    imu_acc->resize(3, false);
    ST::Acceleration imu_acceleration = m_delegate.m_accelerometerEvent.acceleration();
    (*imu_acc)[0] = imu_acceleration.x;
    (*imu_acc)[1] = imu_acceleration.y;
    (*imu_acc)[2] = imu_acceleration.z;
    imu_acc_timestamp = m_delegate.m_accelerometerEvent.arrivalTimestamp();
  }

  if (ts != nullptr)
    *ts = std::max<double>(imu_vel_timestamp, imu_acc_timestamp);

  u.unlock();
}

/*!
  Open access to the Structure SDK CaptureSession and starts streaming.
  \param settings : CaptureSession settings.
 */
bool vpOccipitalStructure::open(const ST::CaptureSessionSettings &settings)
{
  if (m_init) {
    close();
  }

  m_captureSession.setDelegate(&m_delegate);

  if (!m_captureSession.startMonitoring(settings)) {
    std::cout << "Failed to initialize capture session!" << std::endl;
    return false;
  }

  // This will lock the open() function until the CaptureSession
  // recieves the first frame or a timeout.
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  std::cv_status var =
    m_delegate.cv_sampleLock.wait_for(u, std::chrono::seconds(20)); // Make sure a device is connected.

// In case the device is not connected, open() should return false.
  if (var == std::cv_status::timeout) {
    m_init = false;
    return m_init;
  }

  m_init = true;
  m_captureSessionSettings = settings;
  return m_init;
}

/*!
  libStructure documentation:
  <blockquote>
  Unlike the start functions, this function runs synchronously and will block
  until the device has successfully stopped streaming. Once successful, the
  captureSessionEventDidOccur delegate will receive CaptureSessionEventId::Ready.
  If an error occurs, the captureSessionEventDidOccur delegate will receive
  CaptureSessionEventId::Error or the specific error case enum.
  </blockquote>
 */
void vpOccipitalStructure::close()
{
  if (m_init) {
    m_captureSession.stopStreaming();
    m_init = false;
  }
}

/*!
  Returns width of given stream image.
  \param stream_type : Type of stream (visible, depth, infrared).
 */
unsigned int vpOccipitalStructure::getWidth(vpOccipitalStructureStream stream_type)
{
  switch (stream_type) {
  case vpOccipitalStructureStream::visible:
    if (m_delegate.m_visibleFrame.isValid())
      return m_delegate.m_visibleFrame.width();
    break;

  case vpOccipitalStructureStream::depth:
    if (m_delegate.m_depthFrame.isValid())
      return m_delegate.m_depthFrame.width();
    break;

  case vpOccipitalStructureStream::infrared:
    if (m_delegate.m_infraredFrame.isValid())
      return m_delegate.m_infraredFrame.width();
    break;

  default:
    break;
  }

  return 0;
}

/*!
  Returns height of given stream image.
  \param stream_type : Type of stream (visible, depth, infrared).
 */
unsigned int vpOccipitalStructure::getHeight(vpOccipitalStructureStream stream_type)
{
  switch (stream_type) {
  case vpOccipitalStructureStream::visible:
    if (m_delegate.m_visibleFrame.isValid())
      return m_delegate.m_visibleFrame.height();
    break;

  case vpOccipitalStructureStream::depth:
    if (m_delegate.m_depthFrame.isValid())
      return m_delegate.m_depthFrame.height();
    break;

  case vpOccipitalStructureStream::infrared:
    if (m_delegate.m_infraredFrame.isValid())
      return m_delegate.m_infraredFrame.height();
    break;

  default:
    break;
  }

  return 0;
}

/*!
  Returns depth in millimeters at (x,y) if it exists, NAN otherwise.
  \param x : Pixel x-location.
  \param y : Pixel y-location
 */
float vpOccipitalStructure::getDepth(int x, int y)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_depthFrame.isValid())
    return m_delegate.m_depthFrame(x, y);

  else
    return 0.;

  u.unlock();
}

/*!
  Returns 3D coordinates of point whose projection is at (row, col).
  \param row : Pixel row.
  \param col : Pixel column.
 */
// Return vpColVector
vpPoint vpOccipitalStructure::unprojectPoint(int row, int col)
{
  std::unique_lock<std::mutex> u(m_delegate.m_sampleLock);
  m_delegate.cv_sampleLock.wait(u);

  if (m_delegate.m_depthFrame.isValid()) {
    ST::Vector3f point_3D = m_delegate.m_depthFrame.unprojectPoint(row, col);
    return vpPoint(point_3D.x, point_3D.y, point_3D.z);
  }

  else // Should be returning invalid vpPoint.
    return vpPoint(0., 0., 0.);

  u.unlock();
}

/*!
  Get the extrinsic transformation from one stream to another. This function
  has to be called after open().
  \param from, to   : Streams for which the camera extrinsic parameters are returned.
 */
vpHomogeneousMatrix vpOccipitalStructure::getTransform(const vpOccipitalStructureStream from,
                                                       const vpOccipitalStructureStream to)
{
  vpHomogeneousMatrix result;

  switch (from) {
  case vpOccipitalStructure::depth:
    if (to == vpOccipitalStructure::visible) {
      ST::Matrix4 v_M_d = m_delegate.m_depthFrame.visibleCameraPoseInDepthCoordinateFrame();

      result[0][0] = v_M_d.m00;
      result[0][1] = v_M_d.m10;
      result[0][2] = v_M_d.m20;
      result[0][3] = v_M_d.m30;
      result[1][0] = v_M_d.m01;
      result[1][1] = v_M_d.m11;
      result[1][2] = v_M_d.m21;
      result[1][3] = v_M_d.m31;
      result[2][0] = v_M_d.m02;
      result[2][1] = v_M_d.m12;
      result[2][2] = v_M_d.m22;
      result[2][3] = v_M_d.m32;
    }

    if (to == vpOccipitalStructure::imu) {
      ST::Matrix4 imu_M_d = m_captureSession.getImuFromDepthExtrinsics().inversed();

      result[0][0] = imu_M_d.m00;
      result[0][1] = imu_M_d.m10;
      result[0][2] = imu_M_d.m20;
      result[0][3] = imu_M_d.m30;
      result[1][0] = imu_M_d.m01;
      result[1][1] = imu_M_d.m11;
      result[1][2] = imu_M_d.m21;
      result[1][3] = imu_M_d.m31;
      result[2][0] = imu_M_d.m02;
      result[2][1] = imu_M_d.m12;
      result[2][2] = imu_M_d.m22;
      result[2][3] = imu_M_d.m32;
    }
    break;

  case vpOccipitalStructure::visible:
    if (to == vpOccipitalStructure::depth) {
      ST::Matrix4 d_M_v = m_delegate.m_depthFrame.visibleCameraPoseInDepthCoordinateFrame().inversed();

      result[0][0] = d_M_v.m00;
      result[0][1] = d_M_v.m10;
      result[0][2] = d_M_v.m20;
      result[0][3] = d_M_v.m30;
      result[1][0] = d_M_v.m01;
      result[1][1] = d_M_v.m11;
      result[1][2] = d_M_v.m21;
      result[1][3] = d_M_v.m31;
      result[2][0] = d_M_v.m02;
      result[2][1] = d_M_v.m12;
      result[2][2] = d_M_v.m22;
      result[2][3] = d_M_v.m32;
    }

    if (to == vpOccipitalStructure::imu) {
      ST::Matrix4 imu_M_v = m_captureSession.getImuFromVisibleExtrinsics().inversed();

      result[0][0] = imu_M_v.m00;
      result[0][1] = imu_M_v.m10;
      result[0][2] = imu_M_v.m20;
      result[0][3] = imu_M_v.m30;
      result[1][0] = imu_M_v.m01;
      result[1][1] = imu_M_v.m11;
      result[1][2] = imu_M_v.m21;
      result[1][3] = imu_M_v.m31;
      result[2][0] = imu_M_v.m02;
      result[2][1] = imu_M_v.m12;
      result[2][2] = imu_M_v.m22;
      result[2][3] = imu_M_v.m32;
    }
    break;

  case vpOccipitalStructure::infrared:
    break;

  case vpOccipitalStructure::imu:
    if (to == vpOccipitalStructure::depth) {
      ST::Matrix4 d_M_imu = m_captureSession.getImuFromDepthExtrinsics();

      result[0][0] = d_M_imu.m00;
      result[0][1] = d_M_imu.m10;
      result[0][2] = d_M_imu.m20;
      result[0][3] = d_M_imu.m30;
      result[1][0] = d_M_imu.m01;
      result[1][1] = d_M_imu.m11;
      result[1][2] = d_M_imu.m21;
      result[1][3] = d_M_imu.m31;
      result[2][0] = d_M_imu.m02;
      result[2][1] = d_M_imu.m12;
      result[2][2] = d_M_imu.m22;
      result[2][3] = d_M_imu.m32;
    }

    if (to == vpOccipitalStructure::visible) {
      ST::Matrix4 v_M_imu = m_captureSession.getImuFromVisibleExtrinsics();

      result[0][0] = v_M_imu.m00;
      result[0][1] = v_M_imu.m10;
      result[0][2] = v_M_imu.m20;
      result[0][3] = v_M_imu.m30;
      result[1][0] = v_M_imu.m01;
      result[1][1] = v_M_imu.m11;
      result[1][2] = v_M_imu.m21;
      result[1][3] = v_M_imu.m31;
      result[2][0] = v_M_imu.m02;
      result[2][1] = v_M_imu.m12;
      result[2][2] = v_M_imu.m22;
      result[2][3] = v_M_imu.m32;
    }
    break;

  default:
    break;
  }

  return result;
}

/*!
  Get intrinsic parameters corresponding to the stream. This function has to be called after open().
  \param stream_type : Stream for which the camera intrinsic parameters are returned.
 */
ST::Intrinsics vpOccipitalStructure::getIntrinsics(const vpOccipitalStructureStream stream_type) const
{
  ST::Intrinsics result;

  switch (stream_type) {
  case vpOccipitalStructure::visible:
    result = m_delegate.m_visibleFrame.intrinsics();
    break;

  case vpOccipitalStructure::depth:
    result = m_delegate.m_depthFrame.intrinsics();
    break;

  case vpOccipitalStructure::infrared:
    result = m_delegate.m_infraredFrame.intrinsics();
    break;

  default:
    // Deal with exceptions.
    break;
  }

  return result;
}

/*!
  Converts the depth frame into a 3D point cloud using intrinsic calibration
  then writes out the result as PLY mesh at the provided path.
  \param filename : PLY file name.
 */
void vpOccipitalStructure::saveDepthImageAsPointCloudMesh(std::string &filename)
{
  m_delegate.m_depthFrame.saveImageAsPointCloudMesh(filename.c_str());
}

/*!
  Get intrinsic parameters of input stream type.
  \param stream_type : Type of stream (visible, depth).
  \param proj_type : Perspective projection model type; with or without distortion.
 */
vpCameraParameters vpOccipitalStructure::getCameraParameters(const vpOccipitalStructureStream stream_type,
                                                             vpCameraParameters::vpCameraParametersProjType proj_type)
{
  ST::Intrinsics cam_intrinsics;
  switch (stream_type) {
  case vpOccipitalStructureStream::visible:
    cam_intrinsics = m_delegate.m_visibleFrame.intrinsics();
    if (proj_type == vpCameraParameters::perspectiveProjWithDistortion)
      m_visible_camera_parameters = vpCameraParameters(cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.cx,
                                                       cam_intrinsics.cy, cam_intrinsics.k1, -cam_intrinsics.k1);

    if (proj_type == vpCameraParameters::perspectiveProjWithoutDistortion)
      m_visible_camera_parameters =
      vpCameraParameters(cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.cx, cam_intrinsics.cy);

    return m_visible_camera_parameters;
    break;

  case vpOccipitalStructureStream::depth:
    cam_intrinsics = m_delegate.m_depthFrame.intrinsics();
    m_depth_camera_parameters =
      vpCameraParameters(cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.cx, cam_intrinsics.cy);

    return m_depth_camera_parameters;
    break;

  default: // Should throw exception for not having this type of extrinsic transforms.
    return vpCameraParameters();
    break;
  }
}

// Protected functions.
void vpOccipitalStructure::getPointcloud(std::vector<vpColVector> &pointcloud)
{
  ST::DepthFrame last_df = m_delegate.m_depthFrame;
  const int width = last_df.width();
  const int height = last_df.height();
  pointcloud.resize((size_t)(width * height));

  const float *p_depth_frame = reinterpret_cast<const float *>(last_df.depthInMillimeters());

// Multi-threading if OpenMP
// Concurrent writes at different locations are safe
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < height; i++) {
    auto depth_pixel_index = i * width;

    for (int j = 0; j < width; j++, depth_pixel_index++) {
      if (std::isnan(p_depth_frame[depth_pixel_index])) {
        pointcloud[(size_t)depth_pixel_index].resize(4, false);
        pointcloud[(size_t)depth_pixel_index][0] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][1] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][2] = m_invalidDepthValue;
        pointcloud[(size_t)depth_pixel_index][3] = 1.0;
        continue;
      }

      // Get the depth value of the current pixel
      auto pixels_distance = p_depth_frame[depth_pixel_index];

      ST::Vector3f point_3D = last_df.unprojectPoint(i, j);

      if (pixels_distance > m_maxZ)
        point_3D.x = point_3D.y = point_3D.z = m_invalidDepthValue;

      pointcloud[(size_t)depth_pixel_index].resize(4, false);
      pointcloud[(size_t)depth_pixel_index][0] = point_3D.x * 0.001;
      pointcloud[(size_t)depth_pixel_index][1] = point_3D.y * 0.001;
      pointcloud[(size_t)depth_pixel_index][2] = point_3D.z * 0.001;
      pointcloud[(size_t)depth_pixel_index][3] = 1.0;
    }
  }
}

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
void vpOccipitalStructure::getPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
  ST::DepthFrame last_df = m_delegate.m_depthFrame;
  const int width = last_df.width();
  const int height = last_df.height();
  pointcloud->width = (uint32_t)width;
  pointcloud->height = (uint32_t)height;
  pointcloud->resize((size_t)(width * height));

#if MANUAL_POINTCLOUD // faster to compute manually when tested
  const float *p_depth_frame = reinterpret_cast<const float *>(last_df.depthInMillimeters());

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
      auto pixels_distance = p_depth_frame[depth_pixel_index];

      // Get 3D coordinates in depth frame. (in mm)
      ST::Vector3f point_3D = last_df.unprojectPoint(i, j);

      if (pixels_distance > m_maxZ)
        point_3D.x = point_3D.y = point_3D.z = m_invalidDepthValue;

      pointcloud->points[(size_t)(depth_pixel_index)].x = point_3D.x * 0.001;
      pointcloud->points[(size_t)(depth_pixel_index)].y = point_3D.y * 0.001;
      pointcloud->points[(size_t)(depth_pixel_index)].z = point_3D.z * 0.001;
    }
  }
// #else
//   m_points = m_pointcloud.calculate(depth_frame);
//   auto vertices = m_points.get_vertices();

//   for (size_t i = 0; i < m_points.size(); i++) {
//     if (vertices[i].z <= 0.0f || vertices[i].z > m_maxZ) {
//       pointcloud->points[i].x = m_invalidDepthValue;
//       pointcloud->points[i].y = m_invalidDepthValue;
//       pointcloud->points[i].z = m_invalidDepthValue;
//     } else {
//       pointcloud->points[i].x = vertices[i].x;
//       pointcloud->points[i].y = vertices[i].y;
//       pointcloud->points[i].z = vertices[i].z;
//     }
//   }
#endif
}

void vpOccipitalStructure::getColoredPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud)
{
  ST::DepthFrame last_df = m_delegate.m_depthFrame;
  ST::ColorFrame last_cf = m_delegate.m_visibleFrame;

  const int depth_width = last_df.width();
  const int depth_height = last_df.height();
  pointcloud->width = static_cast<uint32_t>(depth_width);
  pointcloud->height = static_cast<uint32_t>(depth_height);
  pointcloud->resize(static_cast<uint32_t>(depth_width * depth_height));

  const int color_width = last_cf.width();
  const int color_height = last_cf.height();

  const float *p_depth_frame = reinterpret_cast<const float *>(last_df.depthInMillimeters());
  const ST::Matrix4 depth2ColorExtrinsics = last_df.visibleCameraPoseInDepthCoordinateFrame(); // c_M_d

  const bool swap_rb = false;
  unsigned int nb_color_pixel = 3; // RGB image.
  const uint8_t *p_color_frame = static_cast<const uint8_t *>(last_cf.rgbData());

  const bool registered_streams = last_df.isRegisteredTo(last_cf);

  // Multi-threading if OpenMP
  // Concurrent writes at different locations are safe
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < depth_height; i++) {
    auto depth_pixel_index = i * depth_width;

    for (int j = 0; j < depth_width; j++, depth_pixel_index++) {
      if (std::isnan(p_depth_frame[depth_pixel_index])) {
        pointcloud->points[(size_t)depth_pixel_index].x = m_invalidDepthValue;
        pointcloud->points[(size_t)depth_pixel_index].y = m_invalidDepthValue;
        pointcloud->points[(size_t)depth_pixel_index].z = m_invalidDepthValue;

        // For out of bounds color data, default to black.
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
        unsigned int r = 0, g = 0, b = 0;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        pointcloud->points[(size_t)depth_pixel_index].rgb = *reinterpret_cast<float *>(&rgb);
#else
        pointcloud->points[(size_t)depth_pixel_index].r = (uint8_t)0;
        pointcloud->points[(size_t)depth_pixel_index].g = (uint8_t)0;
        pointcloud->points[(size_t)depth_pixel_index].b = (uint8_t)0;
#endif
        continue;
      }

      // Get the depth value of the current pixel
      auto pixels_distance = p_depth_frame[depth_pixel_index];

      ST::Vector3f depth_point_3D = last_df.unprojectPoint(i, j);

      if (pixels_distance > m_maxZ)
        depth_point_3D.x = depth_point_3D.y = depth_point_3D.z = m_invalidDepthValue;

      pointcloud->points[(size_t)depth_pixel_index].x = depth_point_3D.x * 0.001;
      pointcloud->points[(size_t)depth_pixel_index].y = depth_point_3D.y * 0.001;
      pointcloud->points[(size_t)depth_pixel_index].z = depth_point_3D.z * 0.001;

      if (!registered_streams) {

        ST::Vector3f color_point_3D = depth2ColorExtrinsics * depth_point_3D;

        ST::Vector2f color_pixel;
        if (color_point_3D.z != 0) {
          double x, y, pixel_x, pixel_y;
          x = static_cast<double>(color_point_3D.y / color_point_3D.z);
          y = static_cast<double>(color_point_3D.x / color_point_3D.z);
          vpMeterPixelConversion::convertPoint(m_visible_camera_parameters, y, x, pixel_y, pixel_x);
          color_pixel.x = pixel_x;
          color_pixel.y = pixel_y;
        }

        if (color_pixel.y < 0 || color_pixel.y >= color_height || color_pixel.x < 0 || color_pixel.x >= color_width) {
          // For out of bounds color data, default to a shade of blue in order to
          // visually distinguish holes.
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
          unsigned int r = 0, g = 0, b = 0;
          uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

          pointcloud->points[(size_t)depth_pixel_index].rgb = *reinterpret_cast<float *>(&rgb);
#else
          pointcloud->points[(size_t)depth_pixel_index].r = (uint8_t)0;
          pointcloud->points[(size_t)depth_pixel_index].g = (uint8_t)0;
          pointcloud->points[(size_t)depth_pixel_index].b = (uint8_t)0;
#endif
        }
        else {
          unsigned int i_ = (unsigned int)color_pixel.x;
          unsigned int j_ = (unsigned int)color_pixel.y;

#if PCL_VERSION_COMPARE(<, 1, 1, 0)
          uint32_t rgb = 0;
          if (swap_rb) {
            rgb =
              (static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) |
               static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1]) << 8 |
               static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2])
                   << 16);
          }
          else {
            rgb =
              (static_cast<uint32_t>(p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel]) << 16 |
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
          }
          else {
            pointcloud->points[(size_t)depth_pixel_index].r =
              p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel];
            pointcloud->points[(size_t)depth_pixel_index].g =
              p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 1];
            pointcloud->points[(size_t)depth_pixel_index].b =
              p_color_frame[(i_ * (unsigned int)color_width + j_) * nb_color_pixel + 2];
          }
#endif
        }
      }
      else {
#if PCL_VERSION_COMPARE(<, 1, 1, 0)
        uint32_t rgb = 0;
        if (swap_rb) {
          rgb = (static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel]) |
                 static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 1]) << 8 |
                 static_cast<uint32_t>(p_color_frame[(i * (unsigned int)color_width + j) * nb_color_pixel + 2]) << 16);
        }
        else {
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
        }
        else {
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
END_VISP_NAMESPACE
#endif
