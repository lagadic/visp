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
 * librealSense2 interface.
 *
 *****************************************************************************/

#ifndef __vpRealSense2_h_
#define __vpRealSense2_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#ifdef VISP_HAVE_PCL
#include <pcl/common/common_headers.h>
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

/*!
  \class vpRealSense2

  \ingroup group_sensor_rgbd

  This class provides a lightweight wrapper over the Intel librealsense2
library https://github.com/IntelRealSense/librealsense. It allows to capture
data from the Intel RealSense cameras.

  \note Supported devices for Intel® RealSense™ SDK 2.0 (build 2.8.3):
    - Intel® RealSense™ Camera D400-Series (not tested)
    - Intel® RealSense™ Developer Kit SR300 (vpRealSense2 is ok)

  The usage of vpRealSense2 class is enabled when librealsense2 3rd party is
successfully installed.

  Moreover, if Point Cloud Library (PCL) 3rd party is installed, we also
propose interfaces to retrieve point cloud as pcl::PointCloud<pcl::PointXYZ>
or pcl::PointCloud<pcl::PointXYZRGB> data structures.

  \warning Notice that the usage of this class requires compiler and library
support for the ISO C++ 2011 standard. This support must be enabled with the
-std=c++11 compiler option. Hereafter we give an example of a CMakeLists.txt
file that allows to build sample-realsense.cpp that uses vpRealSense2 class.
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
the content of sample-realsense.cpp: \code #include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

int main()
{
  vpRealSense2 rs;
  rs.open();

  vpImage<unsigned char> I(rs.getIntrinsics(RS2_STREAM_COLOR).height,
rs.getIntrinsics(RS2_STREAM_COLOR).width); #ifdef VISP_HAVE_X11 vpDisplayX
d(I); #elif defined(VISP_HAVE_GDI) vpDisplayGDI d(I); #endif

  while (true) {
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
  vpImage<unsigned char> I(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);
  \endcode
  by
  \code
  vpImage<vpRGBa> I(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);
  \endcode

  If you are interested in the point cloud and if ViSP is build with PCL
  support, you can start from the following example where we use PCL library to
  visualize the point cloud:
  \code
#include <visp3/sensor/vpRealSense2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  vpRealSense2 rs;
  rs.open();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  rs.acquire(NULL, NULL, NULL, pointcloud);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, -0.5, 0, -1, 0);

  while (true) {
    rs.acquire(NULL, NULL, NULL, pointcloud);

    static bool update = false;
    if (!update) {
      viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      update = true;
    } else {
      viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
    }

    viewer->spinOnce(30);
  }
  return 0;
}
  \endcode

  If you want to change the default stream parameters, refer to the
librealsense2 `rs2::config` documentation. The following code allows to
capture the color stream in 1920x1080:
\code
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>

int main() {
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
  rs.open(config);

  vpImage<vpRGBa> Ic(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);
  vpImage<unsigned char> Ii(rs.getIntrinsics(RS2_STREAM_INFRARED).height,
                            rs.getIntrinsics(RS2_STREAM_INFRARED).width);

#ifdef VISP_HAVE_X11
  vpDisplayX dc(Ic, 0, 0, "Color");
  vpDisplayX di(Ii, 100, 100, "Infrared");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc(Ic, 0, 0, "Color");
  vpDisplayGDI di(Ii, 100, 100, "Infrared");
#endif

  while (true) {
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
#include <visp3/sensor/vpRealSense2.h>

int main() {
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
  rs.open(config);

  vpImage<vpRGBa> Ic(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);
  vpImage<uint16_t> Id_raw(rs.getIntrinsics(RS2_STREAM_DEPTH).height, rs.getIntrinsics(RS2_STREAM_DEPTH).width);
  vpImage<vpRGBa> Id(rs.getIntrinsics(RS2_STREAM_DEPTH).height, rs.getIntrinsics(RS2_STREAM_DEPTH).width);

#ifdef VISP_HAVE_X11
  vpDisplayX dc(Ic, 0, 0, "Color");
  vpDisplayX dd(Id, 100, 100, "Depth aligned to color");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI dc(Ic, 0, 0, "Color");
  vpDisplayGDI dd(Id, 100, 100, "Depth aligned to color");
#endif

  rs2::align align_to(RS2_STREAM_COLOR);
  while (true) {
    rs.acquire((unsigned char *) Ic.bitmap, (unsigned char *) Id_raw.bitmap, NULL, NULL, &align_to);
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

  References to `rs2::pipeline_profile` and `rs2::pipeline` can be retrieved
  with (`rs.open() must be called before`):
  \code
rs2::pipeline_profile& profile = rs.getPipelineProfile(); rs2::pipeline& pipeline = rs.getPipeline();
  \endcode

  Information about the sensor can be printed with:
  \code
#include <visp3/sensor/vpRealSense2.h>

int main() {
  vpRealSense2 rs;
  rs.open();
    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

  return 0;
}
  \endcode

  \note This class has been tested with the Intel RealSense SR300
(Firmware: 3.21.0.0) using librealsense (API version: 2.8.3). Refer to the
librealsense2 documentation or [API how
to](https://github.com/IntelRealSense/librealsense/wiki/API-How-To) for
additional information.
*/
class VISP_EXPORT vpRealSense2
{
public:
  vpRealSense2();
  virtual ~vpRealSense2();

  void acquire(vpImage<unsigned char> &grey);
  void acquire(vpImage<vpRGBa> &color);
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, unsigned char *const data_infrared,
               rs2::align *const align_to = NULL);

#ifdef VISP_HAVE_PCL
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
               unsigned char *const data_infrared = NULL, rs2::align *const align_to = NULL);
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
               unsigned char *const data_infrared = NULL, rs2::align *const align_to = NULL);
#endif

  void close();

  vpCameraParameters getCameraParameters(
      const rs2_stream &stream,
      vpCameraParameters::vpCameraParametersProjType type = vpCameraParameters::perspectiveProjWithDistortion) const;

  rs2_intrinsics getIntrinsics(const rs2_stream &stream) const;

  //! Get the value used when the pixel value (u, v) in the depth map is
  //! invalid for the point cloud. For instance, the Point Cloud Library (PCL)
  //! uses NAN values for points where the depth is invalid.
  inline float getInvalidDepthValue() const { return m_invalidDepthValue; }

  //! Get the maximum Z value (used to discard bad reconstructed depth for
  //! pointcloud).
  inline float getMaxZ() const { return m_max_Z; }

  //! Get a reference to `rs2::pipeline`.
  rs2::pipeline &getPipeline() { return m_pipe; }

  //! Get a reference to `rs2::pipeline_profile`.
  rs2::pipeline_profile &getPipelineProfile() { return m_pipelineProfile; }

  vpHomogeneousMatrix getTransformation(const rs2_stream &from, const rs2_stream &to) const;

  void open(const rs2::config &cfg = rs2::config());

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRealSense2 &rs);

  //! Set the value used when the pixel value (u, v) in the depth map is
  //! invalid for the point cloud. For instance, the Point Cloud Library (PCL)
  //! uses NAN values for points where the depth is invalid.
  inline void setInvalidDepthValue(const float value) { m_invalidDepthValue = value; }

  //! Set the maximum Z value (used to discard bad reconstructed depth for
  //! pointcloud).
  inline void setMaxZ(const float maxZ) { m_max_Z = maxZ; }

protected:
  rs2_intrinsics m_colorIntrinsics;
  rs2_extrinsics m_depth2ColorExtrinsics;
  rs2_intrinsics m_depthIntrinsics;
  float m_depthScale;
  float m_invalidDepthValue;
  float m_max_Z;
  rs2::pipeline m_pipe;
  rs2::pipeline_profile m_pipelineProfile;
  rs2::pointcloud m_pointcloud;
  rs2::points m_points;

  void getColorFrame(const rs2::frame &frame, vpImage<vpRGBa> &color);
  void getGreyFrame(const rs2::frame &frame, vpImage<unsigned char> &grey);
  void getNativeFrameData(const rs2::frame &frame, unsigned char *const data);
  void getPointcloud(const rs2::depth_frame &depth_frame, std::vector<vpColVector> &pointcloud);
#ifdef VISP_HAVE_PCL
  void getPointcloud(const rs2::depth_frame &depth_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void getPointcloud(const rs2::depth_frame &depth_frame, const rs2::frame &color_frame,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif
};

#endif
#endif
