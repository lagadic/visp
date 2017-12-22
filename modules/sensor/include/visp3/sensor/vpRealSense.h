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
 * RealSense SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpRealSense_h_
#define __vpRealSense_h_

#include <map>
#include <stdint.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>

#if defined(VISP_HAVE_REALSENSE) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

#include <librealsense/rs.hpp>

#ifdef VISP_HAVE_PCL
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#endif

/*!
  \class vpRealSense

  \ingroup group_sensor_rgbd

  This class is a wrapper over the Intel librealsense library
https://github.com/IntelRealSense/librealsense. It allows to capture data from
the Intel RealSense cameras.

  The usage of vpRealSense class is enabled when librealsense 3rd party is
successfully installed. Installation instructions are provided following
https://github.com/IntelRealSense/librealsense#installation-guide.

  Moreover, if Point Cloud Library (PCL) 3rd party is installed we also
propose interfaces to retrieve point cloud as pcl::PointCloud<pcl::PointXYZ>
or pcl::PointCloud<pcl::PointXYZRGB> data structures.

  \warning Notice that the usage of this class requires compiler and library
support for the ISO C++ 2011 standard. This support must be enabled with the
-std=c++11 compiler option. Hereafter we give an example of a CMakeLists.txt
file that allows to build sample-realsense.cpp that uses vpRealSense class.
  \code
project(sample)
cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
include_directories(${VISP_INCLUDE_DIRS})

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_executable(sample-realsense sample-realsense.cpp)
target_link_libraries(sample-realsense ${VISP_LIBRARIES})
  \endcode

  To acquire images from the RealSense color camera and convert them into grey
level images, a good starting is to use the following code that corresponds to
the content of sample-realsense.cpp:
\code
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>

int main()
{
  vpRealSense rs;
  rs.open();

  vpImage<unsigned char> I(rs.getIntrinsics(rs::stream::color).height, rs.getIntrinsics(rs::stream::color).width);
#ifdef VISP_HAVE_X11
  vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I);
#endif

  while (1) {
    rs.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false))
      break;
  }
  return 0;
}
  \endcode

  If you want to acquire color images, in the previous sample replace:
  \code
  vpImage<unsigned char> I(rs.getIntrinsics(rs::stream::color).height, rs.getIntrinsics(rs::stream::color).width);
  \endcode
  by
  \code
  vpImage<vpRGBa> I(rs.getIntrinsics(rs::stream::color).height, rs.getIntrinsics(rs::stream::color).width);
  \endcode

  If you are interested in the point cloud and if ViSP is build with PCL
support, you can start from the following example where we use PCL library to
visualize the point cloud:

\code
#include <visp3/sensor/vpRealSense.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  vpRealSense rs;
  rs.open();
  std::cout << rs << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  rs.acquire(pointcloud);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0,0,-0.5, 0,-1,0);

  while (1) {
    rs.acquire(pointcloud);

    static bool update = false;
    if (! update) {
      viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      update = true;
    }
    else {
      viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
    }

    viewer->spinOnce (100);
  }
  return 0;
}
  \endcode

  If you want to change the default stream parameters, you can use \p
setEnableStream() to enable only the desired stream and \p setStreamSettings()
to set the stream settings. The following code allows to capture the color
stream in 1920x1080 also with the infrared stream:

\code
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>

int main() {
  vpRealSense rs;
  rs.setEnableStream(rs::stream::color, true);
  rs.setEnableStream(rs::stream::depth, false);
  rs.setEnableStream(rs::stream::infrared, true);
  rs.setEnableStream(rs::stream::infrared2, false);
  rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(1920, 1080, rs::format::rgba8, 30));
  rs.setStreamSettings(rs::stream::infrared, vpRealSense::vpRsStreamParams(640, 480, rs::format::y8, 30));
  rs.open();

  vpImage<vpRGBa> Ic(rs.getIntrinsics(rs::stream::color).height, rs.getIntrinsics(rs::stream::color).width);
  vpImage<unsigned char> Ii(rs.getIntrinsics(rs::stream::infrared).height, rs.getIntrinsics(rs::stream::infrared).width);

#ifdef VISP_HAVE_X11
  vpDisplayX dc(Ic, 0, 0, "Color");
  vpDisplayX di(Ii, 100, 100, "Infrared");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc(Ic, 0, 0, "Color");
  vpDisplayGDI di(Ii, 100, 100, "Infrared");
#endif

  while (1) {
    rs.acquire((unsigned char *) Ic.bitmap, NULL, NULL, Ii.bitmap);
    vpDisplay::display(Ic);
    vpDisplay::display(Ii);
    vpDisplay::flush(Ic);
    vpDisplay::flush(Ii);
    if (vpDisplay::getClick(Ic, false) || vpDisplay::getClick(Ii, false))
      break;
  }
  return 0;
}
  \endcode

  This example shows how to get depth stream aligned on color stream:
  \code
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>

int main() {
  vpRealSense rs;
  rs.setEnableStream(rs::stream::color, true);
  rs.setEnableStream(rs::stream::depth, true);
  rs.setEnableStream(rs::stream::infrared, false);
  rs.setEnableStream(rs::stream::infrared2, false);
  rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30));
  rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 30));
  rs.open();

  vpImage<vpRGBa> Ic(rs.getIntrinsics(rs::stream::color).height, rs.getIntrinsics(rs::stream::color).width);
  vpImage<uint16_t> Id_raw(rs.getIntrinsics(rs::stream::depth).height, rs.getIntrinsics(rs::stream::depth).width);
  vpImage<vpRGBa> Id(rs.getIntrinsics(rs::stream::depth).height, rs.getIntrinsics(rs::stream::depth).width);

#ifdef VISP_HAVE_X11
  vpDisplayX dc(Ic, 0, 0, "Color");
  vpDisplayX dd(Id, 100, 100, "Depth aligned to color");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc(Ic, 0, 0, "Color");
  vpDisplayGDI dd(Id, 100, 100, "Depth aligned to color");
#endif

  while (1) {
    rs.acquire((unsigned char *) Ic.bitmap, (unsigned char *) Id_raw.bitmap, NULL, NULL, NULL,
               rs::stream::color, rs::stream::depth_aligned_to_color);
    vpImageConvert::createDepthHistogram(Id_raw, Id);
    vpDisplay::display(Ic);
    vpDisplay::display(Id);
    vpDisplay::flush(Ic);
    vpDisplay::flush(Id);
    if (vpDisplay::getClick(Ic, false) || vpDisplay::getClick(Id, false))
      break;
  }
  return 0;
}
  \endcode

  This is how you get intrinsics for non native stream (the native stream has
  to be enabled!):

\code
#include <visp3/sensor/vpRealSense.h>

int main() {
  vpRealSense rs;
  rs.setEnableStream(rs::stream::color, true);
  rs.setEnableStream(rs::stream::depth, true);
  rs.setEnableStream(rs::stream::infrared, false);
  rs.setEnableStream(rs::stream::infrared2, false);
  rs.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30));
  rs.setStreamSettings(rs::stream::depth, vpRealSense::vpRsStreamParams(640, 480, rs::format::z16, 30));
  rs.open();

  rs::device * dev = rs.getHandler();
  rs::intrinsics depth_aligned_intrinsic = dev->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
  std::cout << "Intrinsics [fx, fy, ppx, ppy]: " << depth_aligned_intrinsic.fx << " ; " << depth_aligned_intrinsic.fy
            << " ; " << depth_aligned_intrinsic.ppx << " ; " << depth_aligned_intrinsic.ppy << std::endl; return 0;
}
  \endcode

  Useful information can be retrieved using \p getHandler():
  \code
#include <librealsense/rs.hpp>
#include <visp3/sensor/vpRealSense.h>

int main() {
  vpRealSense rs;
  rs.open();

  rs::device *device = rs.getHandler();
  std::cout << "Stream width: " << device->get_stream_width(rs::stream::color) << std::endl;
  std::cout << "Stream height: " << device->get_stream_height(rs::stream::color) << std::endl;
  std::cout << "Stream format: " << device->get_stream_format(rs::stream::color) << std::endl;
  std::cout << "Stream framerate: " << device->get_stream_framerate(rs::stream::color) << std::endl;

  std::cout << "API version: " << rs_get_api_version(nullptr) << std::endl;
  std::cout << "Firmware: " << rs_get_device_firmware_version((const rs_device *) device, nullptr) << std::endl;
  std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

  return 0;
}
  \endcode

  Camera parameters can be set in the following manner:
  \code
  rs::device * dev = rs.getHandler();
  dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1); //enable lr auto exposure for the R200
  \endcode

  \note This class has been tested with the Intel RealSense SR300
(Firmware: 3.15.0.0) and R200 (Firmware: 1.0.71.06) using librealsense (API
version: 1.12.01). The following streams are enabled by default:
  - Color stream with preset: best quality
  - Depth stream with preset: best quality
  - Infrared stream with preset: best quality
  - Infrared2 if supported with preset: best quality
*/
class VISP_EXPORT vpRealSense
{
public:
  vpRealSense();
  virtual ~vpRealSense();

  void acquire(std::vector<vpColVector> &pointcloud);
#ifdef VISP_HAVE_PCL
  void acquire(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif
  void acquire(vpImage<unsigned char> &grey); // tested
  void acquire(vpImage<unsigned char> &grey, std::vector<vpColVector> &pointcloud);
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               std::vector<vpColVector> &pointcloud);
#ifdef VISP_HAVE_PCL
  void acquire(vpImage<unsigned char> &grey, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif

  void acquire(vpImage<vpRGBa> &color); // tested
  void acquire(vpImage<vpRGBa> &color, std::vector<vpColVector> &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               std::vector<vpColVector> &pointcloud);

  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared,
               unsigned char *const data_infrared2 = NULL, const rs::stream &stream_color = rs::stream::color,
               const rs::stream &stream_depth = rs::stream::depth,
               const rs::stream &stream_infrared = rs::stream::infrared,
               const rs::stream &stream_infrared2 = rs::stream::infrared2);

#ifdef VISP_HAVE_PCL
  void acquire(vpImage<vpRGBa> &color, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);

  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
               unsigned char *const data_infrared, unsigned char *const data_infrared2 = NULL,
               const rs::stream &stream_color = rs::stream::color, const rs::stream &stream_depth = rs::stream::depth,
               const rs::stream &stream_infrared = rs::stream::infrared,
               const rs::stream &stream_infrared2 = rs::stream::infrared2);
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
               unsigned char *const data_infrared, unsigned char *const data_infrared2 = NULL,
               const rs::stream &stream_color = rs::stream::color, const rs::stream &stream_depth = rs::stream::depth,
               const rs::stream &stream_infrared = rs::stream::infrared,
               const rs::stream &stream_infrared2 = rs::stream::infrared2);
#endif

  void close();

  vpCameraParameters getCameraParameters(
      const rs::stream &stream,
      vpCameraParameters::vpCameraParametersProjType type = vpCameraParameters::perspectiveProjWithDistortion) const;
  //! Get access to device handler
  rs::device *getHandler() const { return m_device; }

  rs::extrinsics getExtrinsics(const rs::stream &from, const rs::stream &to) const;
  rs::intrinsics getIntrinsics(const rs::stream &stream) const;

  //! Get the value used when the pixel value (u, v) in the depth map is
  //! invalid for the point cloud. For instance, the Point Cloud Library (PCL)
  //! uses NAN values for points where the depth is invalid.
  inline float getInvalidDepthValue() const { return m_invalidDepthValue; }

  //! Get number of devices that are detected
  int getNumDevices() const { return m_context.get_device_count(); }
  //! Get device serial number.
  //! \sa setDeviceBySerialNumber()
  std::string getSerialNumber() const { return m_serial_no; }
  vpHomogeneousMatrix getTransformation(const rs::stream &from, const rs::stream &to) const;

  void open();

  void setDeviceBySerialNumber(const std::string &serial_no);

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRealSense &rs);

  struct vpRsStreamParams {
    int m_streamWidth;
    int m_streamHeight;
    rs::format m_streamFormat;
    int m_streamFramerate;

    vpRsStreamParams()
      : m_streamWidth(640), m_streamHeight(480), m_streamFormat(rs::format::rgba8), m_streamFramerate(30)
    {
    }

    vpRsStreamParams(const int streamWidth, const int streamHeight, const rs::format &streamFormat,
                     const int streamFramerate)
      : m_streamWidth(streamWidth), m_streamHeight(streamHeight), m_streamFormat(streamFormat),
        m_streamFramerate(streamFramerate)
    {
    }
  };

  void setEnableStream(const rs::stream &stream, const bool status);

  //! Set the value used when the pixel value (u, v) in the depth map is
  //! invalid for the point cloud. For instance, the Point Cloud Library (PCL)
  //! uses NAN values for points where the depth is invalid.
  inline void setInvalidDepthValue(const float value) { m_invalidDepthValue = value; }

  void setStreamSettings(const rs::stream &stream, const rs::preset &preset);
  void setStreamSettings(const rs::stream &stream, const vpRsStreamParams &params);

protected:
  rs::context m_context;
  rs::device *m_device;
  int m_num_devices;
  std::string m_serial_no;
  std::map<rs::stream, rs::intrinsics> m_intrinsics;
  float m_max_Z; //!< Maximal Z depth in meter
  std::map<rs::stream, bool> m_enableStreams;
  std::map<rs::stream, bool> m_useStreamPresets;
  std::map<rs::stream, rs::preset> m_streamPresets;
  std::map<rs::stream, vpRsStreamParams> m_streamParams;
  float m_invalidDepthValue;

  void initStream();
};

#endif
#endif
