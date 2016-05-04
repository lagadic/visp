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

#include <librealsense/rs.hpp>
#include <visp3/core/vpImage.h>

template <class Type>
void vp_rs_get_frame_data_impl(const rs::device *m_device, const std::vector <rs::intrinsics> &m_intrinsics, const rs::stream &stream, vpImage<Type> &data)
{
  // Retrieve color image
  if (m_device->is_stream_enabled(stream)) {
    int width = m_intrinsics[(rs_stream)stream].width;
    int height = m_intrinsics[(rs_stream)stream].height;
    data.resize(height, width);

    memcpy((unsigned char *)data.bitmap, (unsigned char *)m_device->get_frame_data(stream), width*height*sizeof(Type));
  }
  else {
    throw vpException(vpException::fatalError, "RealSense Camera - stream %d not enabled!", (rs_stream)stream);
  }
}
