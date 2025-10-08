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
 */

/*!
  \file vpImageConvert.h
  \brief Convert image types
*/

#ifndef VP_IMAGE_CONVERT_H
#define VP_IMAGE_CONVERT_H

#include <stdint.h>

// image
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
// color
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpRGBa.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#if (VISP_HAVE_OPENCV_VERSION < 0x050000)
#include <opencv2/imgproc/types_c.h>
#endif
#endif

#ifdef VISP_HAVE_YARP
#include <yarp/sig/Image.h>
#endif

#if defined(_WIN32)

// Mute warning with clang-cl
// warning : non-portable path to file '<WinSock2.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
// warning : non-portable path to file '<Windows.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wnonportable-system-include-path"
#endif

// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif
#endif

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)
#include <mutex>
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <type_traits>
#endif
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpImageConvert

  \ingroup group_core_image

  Convert image types.

  The following example available in tutorial-image-converter.cpp shows how to
  convert an OpenCV cv::Mat image into a vpImage:

  \include tutorial-image-converter.cpp

*/
class VISP_EXPORT vpImageConvert
{

public:
  static void createDepthHistogram(const vpImage<uint16_t> &src_depth, vpImage<vpRGBa> &dest_rgba);
  static void createDepthHistogram(const vpImage<uint16_t> &src_depth, vpImage<unsigned char> &dest_depth);

  static void createDepthHistogram(const vpImage<float> &src_depth, vpImage<vpRGBa> &dest_depth);
  static void createDepthHistogram(const vpImage<float> &src_depth, vpImage<unsigned char> &dest_depth);

  static void convert(const vpImage<unsigned char> &src, vpImage<vpRGBa> &dest);
  static void convert(const vpImage<vpRGBa> &src, vpImage<unsigned char> &dest, unsigned int nThreads = 0);

  static void convert(const vpImage<float> &src, vpImage<unsigned char> &dest);
  static void convert(const vpImage<vpRGBf> &src, vpImage<vpRGBa> &dest);
  static void convert(const vpImage<unsigned char> &src, vpImage<float> &dest);

  static void convert(const vpImage<double> &src, vpImage<unsigned char> &dest);
  static void convert(const vpImage<unsigned char> &src, vpImage<double> &dest);

  static void convert(const vpImage<uint16_t> &src, vpImage<unsigned char> &dest, unsigned char bitshift = 8);
  static void convert(const vpImage<unsigned char> &src, vpImage<uint16_t> &dest, unsigned char bitshift = 8);

  /*!
    Make a copy of an image.
    \param src : source image.
    \param dest : destination image.
  */
  template <typename Type> static void convert(const vpImage<Type> &src, vpImage<Type> &dest) { dest = src; }

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
  static void convert(const cv::Mat &src, vpImage<vpRGBa> &dest, bool flip = false);
  static void convert(const cv::Mat &src, vpImage<unsigned char> &dest, bool flip = false, unsigned int nThreads = 0);
  static void convert(const cv::Mat &src, vpImage<float> &dest, bool flip = false);
  static void convert(const cv::Mat &src, vpImage<double> &dest, bool flip = false);
  static void convert(const cv::Mat &src, vpImage<vpRGBf> &dest, bool flip = false);
  static void convert(const cv::Mat &src, vpImage<uint16_t> &dest, bool flip = false);
  static void convert(const vpImage<vpRGBa> &src, cv::Mat &dest);
  static void convert(const vpImage<unsigned char> &src, cv::Mat &dest, bool copyData = true);
  static void convert(const vpImage<float> &src, cv::Mat &dest, bool copyData = true);
  static void convert(const vpImage<double> &src, cv::Mat &dest, bool copyData = true);
  static void convert(const vpImage<vpRGBf> &src, cv::Mat &dest);
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename T, bool useFullScale>
  static void convert(const vpImage<vpRGBa> &src, vpImage<vpHSV<T, useFullScale>> &dest);

  template <typename T, bool useFullScale>
  static void convert(const vpImage<vpHSV<T, useFullScale>> &src, vpImage<vpRGBa> &dest);

  template <typename T, typename U, bool useFullScale1, bool useFullScale2>
  static typename std::enable_if<!std::is_same<T, U>::value, void>::type convert(const vpImage<vpHSV<T, useFullScale1>> &src, vpImage<vpHSV<U, useFullScale2>> &dest)
  {
    const int height = src.getHeight(), width = src.getWidth();
    const int size = height * width;
    dest.resize(height, width);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < size; ++i) {
      dest.bitmap[i].buildFrom(src.bitmap[i]);
    }
  }
#endif

#ifdef VISP_HAVE_YARP
  static void convert(const vpImage<unsigned char> &src, yarp::sig::ImageOf<yarp::sig::PixelMono> *dest,
                      bool copyData = true);
  static void convert(const yarp::sig::ImageOf<yarp::sig::PixelMono> *src, vpImage<unsigned char> &dest,
                      bool copyData = true);

  static void convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgba> *dest, bool copyData = true);
  static void convert(const yarp::sig::ImageOf<yarp::sig::PixelRgba> *src, vpImage<vpRGBa> &dest, bool copyData = true);

  static void convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgb> *dest);
  static void convert(const yarp::sig::ImageOf<yarp::sig::PixelRgb> *src, vpImage<vpRGBa> &dest);
#endif

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename MaskType>
  static typename std::enable_if< std::is_same<MaskType, unsigned char>::value || std::is_same<MaskType, bool>::value, int>::type
#else
  template <typename MaskType>
  static int
#endif
    /**
     * \brief Convert a raw depth image into a pcl::PointCloud that has no texture.
     *
     * \param[in] depth_raw Raw depth image.
     * \param[in] depth_scale Depth scale to convert the raw depth image into meters.
     * \param[in] cam_depth The depth camera parameters.
     * \param[out] pointcloud A pointer towards the pcl::PointCloud that has no texture.
     * \param[in] pointcloud_mutex Optional, if set a pointer towards the mutex that protects the point cloud.
     * \param[in] depth_mask Optional, if set a pointer towards a binary image that indicates if the point must be
     * considered or not. (Either true or a value different from 0 to keep the point, false or 0 to discard it).
     * \param[in] Z_min The minimum depth to keep the point.
     * \param[in] Z_max The maximum depth to keep the point.
     *
     * \sa To see how to use it in the context of color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
     */
    depthToPointCloud(const vpImage<uint16_t> &depth_raw,
                                 float depth_scale, const vpCameraParameters &cam_depth,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
                                 std::mutex *pointcloud_mutex = nullptr,
                                 const vpImage<MaskType> *depth_mask = nullptr, float Z_min = 0.2, float Z_max = 2.5)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename MaskType>
  static typename std::enable_if< std::is_same<MaskType, unsigned char>::value || std::is_same<MaskType, bool>::value, int>::type
#else
  template <typename MaskType>
  static int
#endif
    /**
     * \brief Convert a raw depth image in a textured pcl::PointCloud using a vpImage<RGBa> that is aligned with the
     * raw depth image to get the texture.
     *
     * \param[in] color The color image that gives the texture of the points.
     * \param[in] depth_raw Raw depth image.
     * \param[in] depth_scale Depth scale to convert the raw depth image into meters.
     * \param[in] cam_depth The depth camera parameters.
     * \param[out] pointcloud A pointer towards the pcl::PointCloud that has no texture.
     * \param[in] pointcloud_mutex Optional, if set a pointer towards the mutex that protects the point cloud.
     * \param[in] depth_mask Optional, if set a pointer towards a binary image that indicates if the point must be
     * considered or not. (Either true or a value different from 0 to keep the point, false or 0 to discard it).
     * \param[in] Z_min The minimum depth to keep the point.
     * \param[in] Z_max The maximum depth to keep the point.
     *
     * \sa To see how to use it in the context of color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
     */
    depthToPointCloud(const vpImage<vpRGBa> &color, const vpImage<uint16_t> &depth_raw,
                                 float depth_scale, const vpCameraParameters &cam_depth,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,
                                 std::mutex *pointcloud_mutex = nullptr,
                                 const vpImage<MaskType> *depth_mask = nullptr, float Z_min = 0.2, float Z_max = 2.5)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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
#if defined(VISP_HAVE_OPENMP)
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
#endif

  static void split(const vpImage<vpRGBa> &src, vpImage<unsigned char> *pR, vpImage<unsigned char> *pG,
                      vpImage<unsigned char> *pB, vpImage<unsigned char> *pa = nullptr);

  static void merge(const vpImage<unsigned char> *R, const vpImage<unsigned char> *G, const vpImage<unsigned char> *B,
                    const vpImage<unsigned char> *a, vpImage<vpRGBa> &RGBa);

  /*!
    Converts a yuv pixel value in rgb format.

    \param y Y component of a pixel.
    \param u U component of a pixel.
    \param v V component of a pixel.
    \param r Red component from the YUV coding format. This value is computed
    using:
    \f[ r = 0.9999695*y - 0.0009508*(u-128) + 1.1359061*(v-128) \f]
    \param g Green component from the YUV coding format. This value is
    computed using: \f[g = 0.9999695*y - 0.3959609*(u-128) - 0.5782955*(v-128) \f]
    \param b Blue component from the YUV coding format. This value is
    computed using: \f[b = 0.9999695*y + 2.04112*(u-128) - 0.0016314*(v-128) \f]
  */
  static inline void YUVToRGB(unsigned char y, unsigned char u, unsigned char v, unsigned char &r, unsigned char &g,
                              unsigned char &b)
  {
    double dr, dg, db;
    dr = floor(((0.9999695 * y) - (0.0009508 * (u - 128))) + (1.1359061 * (v - 128)));
    dg = floor(((0.9999695 * y) - (0.3959609 * (u - 128))) - (0.5782955 * (v - 128)));
    db = floor(((0.9999695 * y) + (2.04112 * (u - 128))) - (0.0016314 * (v - 128)));

    dr = dr < 0. ? 0. : dr;
    dg = dg < 0. ? 0. : dg;
    db = db < 0. ? 0. : db;
    dr = dr > 255. ? 255. : dr;
    dg = dg > 255. ? 255. : dg;
    db = db > 255. ? 255. : db;

    r = static_cast<unsigned char>(dr);
    g = static_cast<unsigned char>(dg);
    b = static_cast<unsigned char>(db);
  }
  static void YUYVToRGBa(unsigned char *yuyv, unsigned char *rgba, unsigned int width, unsigned int height);
  static void YUYVToRGB(unsigned char *yuyv, unsigned char *rgb, unsigned int width, unsigned int height);
  static void YUYVToGrey(unsigned char *yuyv, unsigned char *grey, unsigned int size);
  static void YUV411ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size);
  static void YUV411ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size);
  static void YUV411ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size);
  static void YUV422ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size);
  static void YUV422ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size);
  static void YUV422ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size);
  static void YUV420ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height);
  static void YUV420ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
  static void YUV420ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size);

  static void YUV444ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size);
  static void YUV444ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size);
  static void YUV444ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size);

  static void YV12ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height);
  static void YV12ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
  static void YVU9ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height);
  static void YVU9ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
  static void RGBToRGBa(unsigned char *rgb, unsigned char *rgba, unsigned int size);
  static void RGBaToRGB(unsigned char *rgba, unsigned char *rgb, unsigned int size);

  static void RGBToGrey(unsigned char *rgb, unsigned char *grey, unsigned int width, unsigned int height,
                        bool flip = false);
  static void RGBToGrey(unsigned char *rgb, unsigned char *grey, unsigned int size);
  static void RGBaToGrey(unsigned char *rgba, unsigned char *grey, unsigned int width, unsigned int height,
                         unsigned int nThreads = 0);
  static void RGBaToGrey(unsigned char *rgba, unsigned char *grey, unsigned int size);

  static void RGBToRGBa(unsigned char *rgb, unsigned char *rgba, unsigned int width, unsigned int height,
                        bool flip = false);

  static void GreyToRGBa(unsigned char *grey, unsigned char *rgba, unsigned int width, unsigned int height);
  static void GreyToRGBa(unsigned char *grey, unsigned char *rgba, unsigned int size);
  static void GreyToRGB(unsigned char *grey, unsigned char *rgb, unsigned int size);

  static void BGRToRGBa(unsigned char *bgr, unsigned char *rgba, unsigned int width, unsigned int height,
                        bool flip = false);

  static void BGRToGrey(unsigned char *bgr, unsigned char *grey, unsigned int width, unsigned int height,
                        bool flip = false, unsigned int nThreads = 0);

  static void BGRaToGrey(unsigned char *bgra, unsigned char *grey, unsigned int width, unsigned int height,
                         bool flip = false, unsigned int nThreads = 0);
  static void BGRaToRGBa(unsigned char *bgra, unsigned char *rgba, unsigned int width, unsigned int height,
                         bool flip = false);

  static void YCbCrToRGB(unsigned char *ycbcr, unsigned char *rgb, unsigned int size);
  static void YCbCrToRGBa(unsigned char *ycbcr, unsigned char *rgb, unsigned int size);
  static void YCbCrToGrey(unsigned char *ycbcr, unsigned char *grey, unsigned int size);
  static void YCrCbToRGB(unsigned char *ycrcb, unsigned char *rgb, unsigned int size);
  static void YCrCbToRGBa(unsigned char *ycrcb, unsigned char *rgb, unsigned int size);
  static void MONO16ToGrey(unsigned char *grey16, unsigned char *grey, unsigned int size);
  static void MONO16ToRGBa(unsigned char *grey16, unsigned char *rgba, unsigned int size);

  static void HSVToRGBa(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
                        unsigned int size);
  static void HSVToRGBa(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                        unsigned char *rgba, unsigned int size, bool h_full = true);
  static void RGBaToHSV(const unsigned char *rgba, double *hue, double *saturation, double *value, unsigned int size);
  static void RGBaToHSV(const unsigned char *rgba, unsigned char *hue, unsigned char *saturation, unsigned char *value,
                        unsigned int size, bool h_full = true);

  static void HSVToRGB(const double *hue, const double *saturation, const double *value, unsigned char *rgb,
                       unsigned int size);
  static void HSVToRGB(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                       unsigned char *rgb, unsigned int size, bool h_full = true);
  static void RGBToHSV(const unsigned char *rgb, double *hue, double *saturation, double *value, unsigned int size);
  static void RGBToHSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation, unsigned char *value,
                       unsigned int size, bool h_full = true);

#ifndef VISP_SKIP_BAYER_CONVERSION
  static void demosaicBGGRToRGBaBilinear(const uint8_t *bggr, uint8_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);
  static void demosaicBGGRToRGBaBilinear(const uint16_t *bggr, uint16_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);

  static void demosaicGBRGToRGBaBilinear(const uint8_t *gbrg, uint8_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);
  static void demosaicGBRGToRGBaBilinear(const uint16_t *gbrg, uint16_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);

  static void demosaicGRBGToRGBaBilinear(const uint8_t *grbg, uint8_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);
  static void demosaicGRBGToRGBaBilinear(const uint16_t *grbg, uint16_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);

  static void demosaicRGGBToRGBaBilinear(const uint8_t *rggb, uint8_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);
  static void demosaicRGGBToRGBaBilinear(const uint16_t *rggb, uint16_t *rgba, unsigned int width, unsigned int height,
                                         unsigned int nThreads = 0);

  static void demosaicBGGRToRGBaMalvar(const uint8_t *bggr, uint8_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);
  static void demosaicBGGRToRGBaMalvar(const uint16_t *bggr, uint16_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);

  static void demosaicGBRGToRGBaMalvar(const uint8_t *gbrg, uint8_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);
  static void demosaicGBRGToRGBaMalvar(const uint16_t *gbrg, uint16_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);

  static void demosaicGRBGToRGBaMalvar(const uint8_t *grbg, uint8_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);
  static void demosaicGRBGToRGBaMalvar(const uint16_t *grbg, uint16_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);

  static void demosaicRGGBToRGBaMalvar(const uint8_t *rggb, uint8_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);
  static void demosaicRGGBToRGBaMalvar(const uint16_t *rggb, uint16_t *rgba, unsigned int width, unsigned int height,
                                       unsigned int nThreads = 0);
#endif

private:
  static void computeYCbCrLUT();

  static void HSV2RGB(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
                      unsigned int size, unsigned int step);
  static void HSV2RGB(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value, unsigned char *rgba,
                      unsigned int size, unsigned int step, bool h_full);
  static void RGB2HSV(const unsigned char *rgb, double *hue, double *saturation, double *value, unsigned int size,
                      unsigned int step);
  static void RGB2HSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation, unsigned char *value,
                      unsigned int size, unsigned int step, bool h_full);

private:
  static bool YCbCrLUTcomputed;
  static int vpCrr[256];
  static int vpCgb[256];
  static int vpCgr[256];
  static int vpCbb[256];
};

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/**
 * \brief Convert a RGBa image into a HSV image.
 *
 * \tparam T The type of the channels of the vpHSV pixels.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; 180].
 * \param[in] src The RGBa image.
 * \param[in] dest The HSV image.
 */
template <typename T, bool useFullScale>
void vpImageConvert::convert(const vpImage<vpRGBa> &src, vpImage<vpHSV<T, useFullScale>> &dest)
{
  const int height = src.getHeight(), width = src.getWidth();
  const int size = height * width;
  dest.resize(height, width);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < size; ++i) {
    dest.bitmap[i].buildFrom(src.bitmap[i]);
  }
}

/**
 * \brief Convert an HSV image into a RGBa image.
 *
 * \tparam T The type of the channels of the vpHSV pixels.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; 180].
 * \param[in] src The HSV image.
 * \param[in] dest The RGBa image.
 */
template <typename T, bool useFullScale>
void vpImageConvert::convert(const vpImage<vpHSV<T, useFullScale>> &src, vpImage<vpRGBa> &dest)
{
  const int height = src.getHeight(), width = src.getWidth();
  const int size = height * width;
  dest.resize(height, width);
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < size; ++i) {
    dest.bitmap[i].buildFrom(src.bitmap[i]);
  }
}
#endif
END_VISP_NAMESPACE
#endif
