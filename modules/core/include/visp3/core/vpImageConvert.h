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
#include <visp3/core/vpRGBa.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#endif

#ifdef VISP_HAVE_YARP
#include <yarp/sig/Image.h>
#endif

#if defined(_WIN32)
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>
#endif

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_THREADS)
#include <mutex>
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
  static int depthToPointCloud(const vpImage<uint16_t> &depth_raw,
                               float depth_scale, const vpCameraParameters &cam_depth,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
                               std::mutex *pointcloud_mutex = nullptr,
                               const vpImage<unsigned char> *mask = nullptr, float Z_min = 0.2, float Z_max = 2.5);
  static int depthToPointCloud(const vpImage<vpRGBa> &color, const vpImage<uint16_t> &depth_raw,
                               float depth_scale, const vpCameraParameters &cam_depth,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,
                               std::mutex *pointcloud_mutex = nullptr,
                               const vpImage<unsigned char> *mask = nullptr, float Z_min = 0.2, float Z_max = 2.5);
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
END_VISP_NAMESPACE
#endif
