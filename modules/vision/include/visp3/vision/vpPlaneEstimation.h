/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Plane estimation.
 */

/*!
 * \file vpPlaneEstimation.h
 * \brief Tools for plane estimation.
 */

#pragma once

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher.
// Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

// System
#include <functional>
#include <optional>

// Core
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPolygon.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpPlaneEstimation
 * \ingroup group_vision_plane
 *
 * \note This class is only available with c++17 enabled.
*/
class VISP_EXPORT vpPlaneEstimation
{
public:
  /*!
   * Based on depth, estimate the plane equation of the roi.
   *
   * \param[in] I_depth_raw : Depth raw value.
   * \param[in] depth_scale : Depth scale (used to convert depth value into meters).
   * \param[in] depth_intrinsics : Depth camera parameters.
   * \param[in] roi : Region of interest.
   * \param[in] avg_nb_of_pts_to_estimate : Average number of points to use to estimate the plane (default: 500).
   * \param[out] heat_map : Plane estimation heat map (optional).
   * \return Plane equation.
   */
  static std::optional<vpPlane> estimatePlane(const vpImage<uint16_t> &I_depth_raw, double depth_scale,
                                              const vpCameraParameters &depth_intrinsics, const vpPolygon &roi,
                                              const unsigned int avg_nb_of_pts_to_estimate = 500,
                                              std::optional<std::reference_wrapper<vpImage<vpRGBa> > > heat_map = {});

private:
  //! Minimal number of points required to estimate a plane
  static constexpr auto MinPointNbToEstimatePlane { 20u };
  //! Maximal subsampling factor applied to the point cloud to estimate a plane
  static constexpr auto MaxSubSampFactorToEstimatePlane { 20u };
};
END_VISP_NAMESPACE
#endif
