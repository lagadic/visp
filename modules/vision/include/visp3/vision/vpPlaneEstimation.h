/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Plane estimation.
 *
 * Authors:
 *
 *****************************************************************************/

/*!
  \file vpPlaneEstimation.h
  \brief Tools for plane estimation.
*/

#pragma once

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                                                     \
    (!defined(_MSC_VER) || ((VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && (_MSC_VER >= 1910)))

// Visual Studio: Optionals are available from Visual Studio 2017 RTW (15.0)	[1910]

// System
#include <optional>

// Internal
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPolygon.h>

/*!
  \class vpPlaneEstimation
  \ingroup group_vision

  \note This class is only available with c++17 enabled.
 */
class VISP_EXPORT vpPlaneEstimation
{
public:
  static std::optional<vpPlane> estimatePlane(const vpImage<uint16_t> &I_depth_raw, double depth_scale,
                                              const vpCameraParameters &depth_intrinsics, const vpPolygon &roi,
                                              const unsigned int avg_nb_of_pts_to_estimate = 500,
                                              std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map = {});

private:
  static constexpr auto MinPointNbToEstimatePlane{20u};
  static constexpr auto MaxSubSampFactorToEstimatePlane{20u};
};

#endif
