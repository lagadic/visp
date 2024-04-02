/****************************************************************************
 *
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
 * Convert image types.
 *
*****************************************************************************/

/*!
  \file vpImageConvert_pcl.cpp
  \brief Depth image to point cloud conversion.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_THREADS)

#include <visp3/core/vpImageConvert.h>

#if defined(_OPENMP)
#include <omp.h>
#endif

/*!
 * \param[in] depth_raw : Depth raw image.
 * \param[in] depth_scale : Depth scale to apply to data in `depth_raw`.
 * \param[in] cam_depth : Depth camera intrinsics.
 * \param[out] pointcloud : Computed point cloud.
 * \param[in] mask : Optional mask. When set to nullptr, all the pixels in `depth_raw` are considered. Otherwise,
 * we consider only pixels that have a mask value that differ from 0. You should ensure that mask size and `depth_raw`
 * size are the same.
 * \param[in] Z_min : Min Z value to retain the 3D point in the point cloud.
 * \param[in] Z_max : Max Z value to retain the 3D point in the point cloud.
 */
void vpImageConvert::depthToPointCloud(const vpImage<uint16_t> &depth_raw, float depth_scale,
                                       const vpCameraParameters &cam_depth,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
                                       const vpImage<unsigned char> *mask, float Z_min, float Z_max)
{
  pointcloud->clear();
  int size = static_cast<int>(depth_raw.getSize());
  unsigned int width = depth_raw.getWidth();
  unsigned int height = depth_raw.getHeight();

  if (mask) {
    if ((width != mask->getWidth()) || (height != mask->getHeight())) {
      throw(vpImageException(vpImageException::notInitializedError, "Depth image and mask size differ"));
    }
#if defined(_OPENMP)
    std::mutex mutex;
#pragma omp parallel for
#endif
    for (int p = 0; p < size; ++p) {
      if (mask->bitmap[p]) {
        if (static_cast<int>(depth_raw.bitmap[p])) {
          float Z = static_cast<float>(depth_raw.bitmap[p]) * depth_scale;
          if (Z < Z_max) {
            double x = 0;
            double y = 0;
            unsigned int j = p % width;
            unsigned int i = (p - j) / width;
            vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
            vpColVector point_3D({ x * Z, y * Z, Z });
            if (point_3D[2] > Z_min) {
#if defined(_OPENMP)
              std::lock_guard<std::mutex> lock(mutex);
#endif
              pointcloud->push_back(pcl::PointXYZ(point_3D[0], point_3D[1], point_3D[2]));
            }
          }
        }
      }
    }
  }
  else {
#if defined(_OPENMP)
    std::mutex mutex;
#pragma omp parallel for
#endif
    for (int p = 0; p < size; ++p) {
      if (static_cast<int>(depth_raw.bitmap[p])) {
        float Z = static_cast<float>(depth_raw.bitmap[p]) * depth_scale;
        if (Z < 2.5) {
          double x = 0;
          double y = 0;
          unsigned int j = p % width;
          unsigned int i = (p - j) / width;
          vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
          vpColVector point_3D({ x * Z, y * Z, Z, 1 });
          if (point_3D[2] >= 0.1) {
#if defined(_OPENMP)
            std::lock_guard<std::mutex> lock(mutex);
#endif
            pointcloud->push_back(pcl::PointXYZ(point_3D[0], point_3D[1], point_3D[2]));
          }
        }
      }
    }
  }
}
#endif
