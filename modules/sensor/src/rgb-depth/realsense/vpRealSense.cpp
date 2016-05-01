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

vpRealSense::vpRealSense()
  : m_context(), m_device(NULL), m_num_devices(0), m_serial_no(), m_intrinsics()
{

}

vpRealSense::~vpRealSense()
{

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

  m_device->enable_stream(rs::stream::depth, rs::preset::best_quality);
  m_device->enable_stream(rs::stream::color, rs::preset::best_quality);
  m_device->enable_stream(rs::stream::infrared, rs::preset::best_quality);
  try { m_device->enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0); } catch(...) {}

  // Compute field of view for each enabled stream
  m_intrinsics.clear();
  for(int i = 0; i < 4; ++i)
  {
    auto stream = rs::stream(i);
    if(!m_device->is_stream_enabled(stream)) continue;
    auto intrin = m_device->get_stream_intrinsics(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;

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
      std::cout << "DBG: stop streaming" << std::endl;
      m_device->stop();
    }
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
 */
void vpRealSense::setDeviceBySerialNumber(const std::string &serial_no)
{
  m_serial_no = serial_no;
}

/*!
  Acquire data from RealSense device.
   \param color : Color image
 */
void vpRealSense::acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth)
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

    std::cout << "DBG: color format: " << m_device->get_stream_format(rs::stream::color) << std::endl;
    std::cout << "DBG: stream color conversion" << std::endl;
    vpImageConvert::RGBToRGBa((unsigned char *)m_device->get_frame_data(rs::stream::color), (unsigned char *)color.bitmap, color_width, color_height);
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - color stream not enabled!");
  }

  // Retrieve infrared image
  if (m_device->is_stream_enabled(rs::stream::infrared)) {
    int infrared_width = m_intrinsics[RS_STREAM_INFRARED].width;
    int infrared_height = m_intrinsics[RS_STREAM_INFRARED].height;
    infrared.resize(infrared_height, infrared_width);

    std::cout << "DBG: infrared format: " << m_device->get_stream_format(rs::stream::infrared) << std::endl;

    memcpy((unsigned char *)infrared.bitmap, (unsigned char *)m_device->get_frame_data(rs::stream::infrared), infrared_width*infrared_height*sizeof(u_int16_t));
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - infrared stream not enabled!");
  }

  // Retrieve depth image
  if (m_device->is_stream_enabled(rs::stream::depth)) {
    int depth_width = m_intrinsics[RS_STREAM_DEPTH].width;
    int depth_height = m_intrinsics[RS_STREAM_DEPTH].height;
    depth.resize(depth_height, depth_width);

    std::cout << "DBG: depth format: " << m_device->get_stream_format(rs::stream::depth) << std::endl;

    memcpy((unsigned char *)depth.bitmap, (unsigned char *)m_device->get_frame_data(rs::stream::depth), depth_width*depth_height*sizeof(u_int16_t));
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - depth stream not enabled!");
  }

}

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
  os << "Device name: " << rs.getDevice()->get_name() << std::endl;
  return os;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpRealSense.cpp.o) has no symbols
void dummy_vpRealSense() {};
#endif
