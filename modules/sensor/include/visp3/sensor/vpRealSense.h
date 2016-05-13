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

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_REALSENSE

#include <librealsense/rs.hpp>

#ifdef VISP_HAVE_PCL
#  include <pcl/point_types.h>
#  include <pcl/common/projection_matrix.h>
#endif

class VISP_EXPORT vpPoint3dTextured
{
public:
  vpPoint3dTextured() {
    for(unsigned int i=0; i<3; i++) {
      m_xyz[i] = 0;
      m_rgb[i] = 255; // Default to white
    }
  };
  virtual ~vpPoint3dTextured() {};

  //! Get X coordinate of the point in meter.
  float get_X() const {
    return m_xyz[0];
  }
  //! Get Y coordinate of the point in meter.
  float get_Y() const {
    return m_xyz[1];
  }
  //! Get Z coordinate of the point in meter.
  float get_Z() const {
    return m_xyz[2];
  }
  //! Set 3D coordinates to a same value (in meter).
  void setXYZ(float v) {
    m_xyz[0] = m_xyz[1] = m_xyz[2] = v;
  }
  //! Set 3D coordinates from a vector of float. Units are meters.
  void setXYZ(float x, float y, float z) {
    m_xyz[0] = x;
    m_xyz[1] = y;
    m_xyz[2] = z;
  }
  //! Set RGB texture from a vector of unsigned char.
  void setRGB(unsigned char r, unsigned char g, unsigned char b) {
    m_rgb[0] = r;
    m_rgb[1] = g;
    m_rgb[2] = b;
  }

  float m_xyz[3]; //!< 3D coordinates of the point (X Y Z)
  unsigned char m_rgb[3]; //!< Texture in RGB
};

/*!
  \class vpRealSense

  \ingroup group_sensor_rgbd
*/
class VISP_EXPORT vpRealSense
{
public:
  vpRealSense();
  virtual ~vpRealSense();

  void acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, std::vector<vpPoint3dTextured> &pointcloud);
#ifdef VISP_HAVE_PCL
  void acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif

  void close();

  //! Get access to device handler
  rs::device *getHandler() const {
    return m_device;
  }
  //! Get intrinsic parameters.
  std::vector <rs::intrinsics> getIntrinsics() const {
    return m_intrinsics;
  }
  //! Get number of devices that are detected
  int getNumDevices() const {
    return m_context.get_device_count();
  }
  //! Get device serial number.
  //! \sa setDeviceBySerialNumber()
  std::string getSerialNumber() const {
    return m_serial_no;
  }

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
  bool m_enable_point_cloud;

};

#endif
#endif

