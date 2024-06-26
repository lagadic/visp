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
 * Convert image types.
 *
*****************************************************************************/

/*!
  \file vpImageConvert_pcl.cpp
  \brief Depth image to point cloud conversion.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)

#include <visp3/core/vpImageConvert.h>

#if defined(_OPENMP)
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * Create a point cloud from a depth image.
 *
 * \param[in] depth_raw : Depth raw image.
 * \param[in] depth_scale : Depth scale to apply to data in `depth_raw`.
 * \param[in] cam_depth : Depth camera intrinsics.
 * \param[out] pointcloud : Computed 3D point cloud.
 * The 3D points reconstructed from the raw depth image are those
 * that have their corresponding 2D projection in the depth mask and have a Z value within ]Z_min, Z_max[ range.
 * When the depth mask is set to nullptr, we reconstruct all 3D points from the complete depth raw image and
 * retain only those whose Z value lies between ]Z_min, Z_max[ range.
 * You must also ensure that the size of the depth mask and the size of the depth raw image are the same.
 * \param[inout] pointcloud_mutex : Optional mutex to protect from concurrent access to `pointcloud`. When set to
 * nullptr, you should ensure that there is no thread that wants to access to `pointcloud`, like for example
 * the one used in vpDisplayPCL.
 * \param[in] depth_mask : Optional depth_mask. When set to nullptr, all the pixels in `depth_raw` are considered. Otherwise,
 * we consider only pixels that have a mask value that differ from 0.
 * \param[in] Z_min : Min Z value to retain the 3D point in the point cloud.
 * \param[in] Z_max : Max Z value to retain the 3D point in the point cloud.
 *
 * \return The size of the point cloud.
 */
  int vpImageConvert::depthToPointCloud(const vpImage<uint16_t> &depth_raw, float depth_scale,
                                         const vpCameraParameters &cam_depth,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::mutex *pointcloud_mutex,
                                         const vpImage<unsigned char> *depth_mask, float Z_min, float Z_max)
{

  int size = static_cast<int>(depth_raw.getSize());
  unsigned int width = depth_raw.getWidth();
  unsigned int height = depth_raw.getHeight();
  int pcl_size = 0;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  if (depth_mask) {
    if ((width != depth_mask->getWidth()) || (height != depth_mask->getHeight())) {
      throw(vpImageException(vpImageException::notInitializedError, "Depth image and mask size differ"));
    }
    if (pointcloud_mutex) {
      pointcloud_mutex->lock();
    }
    pointcloud->clear();
#if defined(_OPENMP)
    std::mutex mutex;
#pragma omp parallel for
#endif
    for (int p = 0; p < size; ++p) {
      if (depth_mask->bitmap[p]) {
        if (static_cast<int>(depth_raw.bitmap[p])) {
          float Z = static_cast<float>(depth_raw.bitmap[p]) * depth_scale;
          if (Z < Z_max) {
            double x = 0;
            double y = 0;
            unsigned int j = p % width;
            unsigned int i = (p - j) / width;
            vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
            vpColVector point_3D({ x * Z, y * Z, Z });
            if (point_3D[index_2] > Z_min) {
#if defined(_OPENMP)
              std::lock_guard<std::mutex> lock(mutex);
#endif
              pointcloud->push_back(pcl::PointXYZ(point_3D[index_0], point_3D[index_1], point_3D[index_2]));
            }
          }
        }
      }
    }
    pcl_size = pointcloud->size();
    if (pointcloud_mutex) {
      pointcloud_mutex->unlock();
    }
  }
  else {
    if (pointcloud_mutex) {
      pointcloud_mutex->lock();
    }
    pointcloud->clear();
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
          if (point_3D[index_2] >= 0.1) {
#if defined(_OPENMP)
            std::lock_guard<std::mutex> lock(mutex);
#endif
            pointcloud->push_back(pcl::PointXYZ(point_3D[index_0], point_3D[index_1], point_3D[index_2]));
          }
        }
      }
    }
    pcl_size = pointcloud->size();
    if (pointcloud_mutex) {
      pointcloud_mutex->unlock();
    }
  }

  return pcl_size;
}

/*!
 * Create a textured point cloud from an aligned color and depth image.
 *
 * \param[in] color : Color image.
 * \param[in] depth_raw : Depth raw image.
 * \param[in] depth_scale : Depth scale to apply to data in `depth_raw`.
 * \param[in] cam_depth : Depth camera intrinsics.
 * \param[out] pointcloud : Computed 3D point cloud with RGB information.
 * The 3D points reconstructed from the raw depth image are those
 * that have their corresponding 2D projection in the depth mask and have a Z value within ]Z_min, Z_max[ range.
 * When the depth mask is set to nullptr, we reconstruct all 3D points from the complete depth raw image and
 * retain only those whose Z value lies between ]Z_min, Z_max[ range.
 * \param[inout] pointcloud_mutex : Optional mutex to protect from concurrent access to `pointcloud`. When set to
 * nullptr, you should ensure that there is no thread that wants to access to `pointcloud`, like for example
 * the one used in vpDisplayPCL.
 * \param[in] depth_mask : Optional depth_mask. When set to nullptr, all the pixels in `depth_raw` are considered. Otherwise,
 * we consider only pixels that have a mask value that differ from 0 and that a Z value in ]Z_min, Z_max[] range.
 * You should also ensure that mask size and `depth_raw` size are the same.
 * \param[in] Z_min : Min Z value to retain the 3D point in the point cloud.
 * \param[in] Z_max : Max Z value to retain the 3D point in the point cloud.
 *
 * \return The size of the point cloud.
 */
int vpImageConvert::depthToPointCloud(const vpImage<vpRGBa> &color, const vpImage<uint16_t> &depth_raw,
                                       float depth_scale, const vpCameraParameters &cam_depth,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::mutex *pointcloud_mutex,
                                       const vpImage<unsigned char> *depth_mask, float Z_min, float Z_max)
{
  int size = static_cast<int>(depth_raw.getSize());
  unsigned int width = depth_raw.getWidth();
  unsigned int height = depth_raw.getHeight();
  int pcl_size = 0;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  if (depth_mask) {
    if ((width != depth_mask->getWidth()) || (height != depth_mask->getHeight())) {
      throw(vpImageException(vpImageException::notInitializedError, "Depth image and mask size differ"));
    }
    if (pointcloud_mutex) {
      pointcloud_mutex->lock();
    }
    pointcloud->clear();
#if defined(_OPENMP)
    std::mutex mutex;
#pragma omp parallel for
#endif
    for (int p = 0; p < size; ++p) {
      if (depth_mask->bitmap[p]) {
        if (static_cast<int>(depth_raw.bitmap[p])) {
          float Z = static_cast<float>(depth_raw.bitmap[p]) * depth_scale;
          if (Z < Z_max) {
            double x = 0;
            double y = 0;
            unsigned int j = p % width;
            unsigned int i = (p - j) / width;
            vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
            vpColVector point_3D({ x * Z, y * Z, Z });
            if (point_3D[index_2] > Z_min) {
#if defined(_OPENMP)
              std::lock_guard<std::mutex> lock(mutex);
#endif
#if PCL_VERSION_COMPARE(>=,1,14,1)
              pointcloud->push_back(pcl::PointXYZRGB(point_3D[index_0], point_3D[index_1], point_3D[index_2],
                                                     color.bitmap[p].R, color.bitmap[p].G, color.bitmap[p].B));
#else
              pcl::PointXYZRGB pt(color.bitmap[p].R, color.bitmap[p].G, color.bitmap[p].B);
              pt.x = point_3D[index_0];
              pt.y = point_3D[index_1];
              pt.z = point_3D[index_2];
              pointcloud->push_back(pcl::PointXYZRGB(pt));
#endif
            }
          }
        }
      }
    }
    pcl_size = pointcloud->size();
    if (pointcloud_mutex) {
      pointcloud_mutex->unlock();
    }
  }
  else {
    if (pointcloud_mutex) {
      pointcloud_mutex->lock();
    }
    pointcloud->clear();
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
          if (point_3D[index_2] >= 0.1) {
#if defined(_OPENMP)
            std::lock_guard<std::mutex> lock(mutex);
#endif
#if PCL_VERSION_COMPARE(>=,1,14,1)
            pointcloud->push_back(pcl::PointXYZRGB(point_3D[index_0], point_3D[index_1], point_3D[index_2],
                                                   color.bitmap[p].R, color.bitmap[p].G, color.bitmap[p].B));
#else
            pcl::PointXYZRGB pt(color.bitmap[p].R, color.bitmap[p].G, color.bitmap[p].B);
            pt.x = point_3D[index_0];
            pt.y = point_3D[index_1];
            pt.z = point_3D[index_2];
            pointcloud->push_back(pcl::PointXYZRGB(pt));
#endif
          }
        }
      }
    }
    pcl_size = pointcloud->size();
    if (pointcloud_mutex) {
      pointcloud_mutex->unlock();
    }
  }

  return pcl_size;
}
END_VISP_NAMESPACE
#endif
