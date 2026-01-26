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
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
/**
 * \brief Class that permits to project a color image into a depth frame.
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you are interested in learning how to use this class, you may have a look at:</span><br>
 *
 * - \ref tutorial-planar-object-pose
*/
class VISP_EXPORT vpColorDepthConversion
{
public:
  static vpImagePoint projectColorToDepth(const vpImage<uint16_t> &I_depth, const double &depth_scale, const double &depth_min,
                                          const double &depth_max, const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);
  static vpImagePoint projectColorToDepth(const uint16_t *data, const double &depth_scale, const double &depth_min, const double &depth_max,
                                          const double &depth_width, const double &depth_height,
                                          const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);

  static vpImagePoint projectColorToDepth(const vpImage<float> &I_depth, const double &depth_min,
                                          const double &depth_max, const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);

  static vpImagePoint projectColorToDepth(const float *data, const double &depth_min, const double &depth_max,
                                          const double &depth_width, const double &depth_height,
                                          const vpCameraParameters &depth_intrinsics,
                                          const vpCameraParameters &color_intrinsics,
                                          const vpHomogeneousMatrix &color_M_depth,
                                          const vpHomogeneousMatrix &depth_M_color, const vpImagePoint &from_pixel);
};
END_VISP_NAMESPACE

#endif
