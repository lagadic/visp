/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Color to Depth conversion.
 */

#ifndef VP_COLOR_DEPTH_CONVERSION_H
#define VP_COLOR_DEPTH_CONVERSION_H

#include <visp3/core/vpConfig.h>

// Internal
#include "vpCameraParameters.h"
#include "vpImage.h"

BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpColorDepthConversion
{
public:
  static vpImagePoint projectColorToDepth(const vpImage<uint16_t> &I_depth, double depth_scale, double depth_min,
                                          double depth_max, const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);
  static vpImagePoint projectColorToDepth(const uint16_t *data, double depth_scale, double depth_min, double depth_max,
                                          double depth_width, double depth_height,
                                          const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);
};
END_VISP_NAMESPACE

#endif
