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

#ifndef __vpRealSense_h_
#define __vpRealSense_h_

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
#  include <pcl/point_types.h>
#  include <pcl/common/projection_matrix.h>
#endif

/*!
  \class vpRealSense

  \ingroup group_sensor_rgbd

  This class is a wrapper over the Intel librealsense library https://github.com/IntelRealSense/librealsense.
  It allows to capture data from the Intel RealSense F200, SR300 and R200 cameras.

  The usage of vpRealSense class is enabled when librealsense 3rd party is successfully installed. Installation
  instructions are provided following https://github.com/IntelRealSense/librealsense#installation-guide.

  Moreover, if Point Cloud Library (PCL) 3rd party is installed we also propose interfaces to retrieve point cloud as
  pcl::PointCloud<pcl::PointXYZ> or pcl::PointCloud<pcl::PointXYZRGB> data structures.

  \warning Notice that the usage of this class requires compiler and library support for the ISO C++ 2011 standard.
  This support must be enabled with the -std=c++11 compiler option. Hereafter we give an example of
  a CMakeLists.txt file that allows to build sample-realsense.cpp that uses vpRealSense class.
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

  To acquire images from the RealSense color camera and convert them into grey level images, a good starting is to use
  the following:
  \code
#include <visp3/sensor/vpRealSense.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

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

  If you are interested in the point cloud and if ViSP is build with PCL support, you can start from the
  following example where we use PCL library to visualize the point cloud:
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
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      update = true;
    }
    else {
      viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
    }

    viewer->spinOnce (100);
  }
}
  \endcode
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
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, std::vector<vpColVector> &pointcloud);
#ifdef VISP_HAVE_PCL
  void acquire(vpImage<unsigned char> &grey, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<unsigned char> &grey, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif

  void acquire(vpImage<vpRGBa> &color);  // tested
  void acquire(vpImage<vpRGBa> &color, std::vector<vpColVector> &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, std::vector<vpColVector> &pointcloud);

#ifdef VISP_HAVE_PCL
  void acquire(vpImage<vpRGBa> &color, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<uint16_t> &infrared, vpImage<uint16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif

  void close();

  vpCameraParameters getCameraParameters(const rs::stream &stream, vpCameraParameters::vpCameraParametersProjType type=vpCameraParameters::perspectiveProjWithDistortion) const;
  //! Get access to device handler
  rs::device *getHandler() const {
    return m_device;
  }

  rs::extrinsics getExtrinsics(const rs::stream &from, const rs::stream &to) const;
  rs::intrinsics getIntrinsics(const rs::stream &stream) const;

  //! Get number of devices that are detected
  int getNumDevices() const {
    return m_context.get_device_count();
  }
  //! Get device serial number.
  //! \sa setDeviceBySerialNumber()
  std::string getSerialNumber() const {
    return m_serial_no;
  }
  vpHomogeneousMatrix getTransformation(const rs::stream &from, const rs::stream &to) const;

  void open();

  void setDeviceBySerialNumber(const std::string &serial_no);

  friend VISP_EXPORT std::ostream & operator<< (std::ostream &os, const vpRealSense &rs);

protected:
  rs::context m_context;
  rs::device *m_device;
  int m_num_devices;
  std::string m_serial_no;
  std::vector <rs::intrinsics> m_intrinsics;
  float m_max_Z; //!< Maximal Z depth in meter
  bool m_enable_color;
  bool m_enable_depth;
};

#endif
#endif

