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

/*!
  \class vpRealSense

  \ingroup group_sensor_rgbd
*/
class VISP_EXPORT vpRealSense
{
public:
  vpRealSense();
  virtual ~vpRealSense();

  void acquire(vpImage<vpRGBa> &color, vpImage<u_int16_t> &infrared, vpImage<u_int16_t> &depth);

  void close();

  rs::device *getDevice() const {
    return m_device;
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
};

#endif
#endif

