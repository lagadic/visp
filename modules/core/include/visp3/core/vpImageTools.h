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
 * Image tools.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpImageTools_H
#define vpImageTools_H

/*!
  \file vpImageTools.h

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...
*/

#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_PTHREAD
#include <pthread.h>
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpRectOriented.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#if defined _OPENMP
#include <omp.h>
#endif

/*!
  \class vpImageTools

  \ingroup group_core_image

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...

*/
class VISP_EXPORT vpImageTools
{
public:
  enum vpImageInterpolationType {
    INTERPOLATION_NEAREST, /*!< Nearest neighbor interpolation (fastest). */
    INTERPOLATION_LINEAR,  /*!< Bi-linear interpolation. */
    INTERPOLATION_CUBIC    /*!< Bi-cubic interpolation. */
  };

  template <class Type>
  static inline void binarise(vpImage<Type> &I, Type threshold1, Type threshold2, Type value1, Type value2, Type value3,
                              const bool useLUT = true);
  static void changeLUT(vpImage<unsigned char> &I, unsigned char A, unsigned char newA, unsigned char B,
                        unsigned char newB);

  template <class Type>
  static void crop(const vpImage<Type> &I, double roi_top, double roi_left, unsigned int roi_height,
                   unsigned int roi_width, vpImage<Type> &crop, unsigned int v_scale = 1, unsigned int h_scale = 1);

  static void columnMean(const vpImage<double> &I, vpRowVector &result);

  template <class Type>
  static void crop(const vpImage<Type> &I, const vpImagePoint &topLeft, unsigned int roi_height, unsigned int roi_width,
                   vpImage<Type> &crop, unsigned int v_scale = 1, unsigned int h_scale = 1);
  template <class Type>
  static void crop(const vpImage<Type> &I, const vpRect &roi, vpImage<Type> &crop, unsigned int v_scale = 1,
                   unsigned int h_scale = 1);
  template <class Type>
  static void crop(const unsigned char *bitmap, unsigned int width, unsigned int height, const vpRect &roi,
                   vpImage<Type> &crop, unsigned int v_scale = 1, unsigned int h_scale = 1);

  static void extract(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, const vpRectOriented &r);
  static void extract(const vpImage<unsigned char> &Src, vpImage<double> &Dst, const vpRectOriented &r);

  template <class Type> static void flip(const vpImage<Type> &I, vpImage<Type> &newI);

  template <class Type> static void flip(vpImage<Type> &I);

  static void imageDifference(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                              vpImage<unsigned char> &Idiff);
  static void imageDifference(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, vpImage<vpRGBa> &Idiff);

  static void imageDifferenceAbsolute(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                      vpImage<unsigned char> &Idiff);
  static void imageDifferenceAbsolute(const vpImage<double> &I1, const vpImage<double> &I2, vpImage<double> &Idiff);
  static void imageDifferenceAbsolute(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, vpImage<vpRGBa> &Idiff);

  static void imageAdd(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, vpImage<unsigned char> &Ires,
                       const bool saturate = false);

  static void imageSubtract(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            vpImage<unsigned char> &Ires, const bool saturate = false);

  static void initUndistortMap(const vpCameraParameters &cam, unsigned int width, unsigned int height,
                               vpArray2D<int> &mapU, vpArray2D<int> &mapV,
                               vpArray2D<float> &mapDu, vpArray2D<float> &mapDv);

  static double interpolate(const vpImage<unsigned char> &I, const vpImagePoint &point,
                            const vpImageInterpolationType &method = INTERPOLATION_NEAREST);

  static void integralImage(const vpImage<unsigned char> &I, vpImage<double> &II, vpImage<double> &IIsq);

  static double normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2,
                                      const bool useOptimized = true);

  static void normalize(vpImage<double> &I);

  static void remap(const vpImage<unsigned char> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                    const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<unsigned char> &Iundist);
  static void remap(const vpImage<vpRGBa> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                    const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<vpRGBa> &Iundist);

  template <class Type>
  static void resize(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int width, const unsigned int height,
                     const vpImageInterpolationType &method = INTERPOLATION_NEAREST, unsigned int nThreads=0);

  template <class Type>
  static void resize(const vpImage<Type> &I, vpImage<Type> &Ires,
                     const vpImageInterpolationType &method = INTERPOLATION_NEAREST, unsigned int nThreads=0);

  static void templateMatching(const vpImage<unsigned char> &I, const vpImage<unsigned char> &I_tpl,
                               vpImage<double> &I_score, const unsigned int step_u, const unsigned int step_v,
                               const bool useOptimized = true);

  template <class Type>
  static void undistort(const vpImage<Type> &I, const vpCameraParameters &cam, vpImage<Type> &newI,
                        unsigned int nThreads=2);

  template <class Type>
  static void warpImage(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst,
                        const vpImageInterpolationType &interpolation=INTERPOLATION_NEAREST,
                        bool fixedPointArithmetic=true, bool pixelCenter=false);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  template <class Type>
  vp_deprecated static void createSubImage(const vpImage<Type> &I, unsigned int i_sub, unsigned int j_sub,
                                           unsigned int nrow_sub, unsigned int ncol_sub, vpImage<Type> &S);

  template <class Type>
  vp_deprecated static void createSubImage(const vpImage<Type> &I, const vpRect &rect, vpImage<Type> &S);
//@}
#endif

private:
  // Cubic interpolation
  static float cubicHermite(const float A, const float B, const float C, const float D, const float t);

  template <class Type> static Type getPixelClamped(const vpImage<Type> &I, const float u, const float v);

  static int coordCast(double x);

  // Linear interpolation
  static double lerp(double A, double B, double t);
  static float lerp(float A, float B, float t);
  static int64_t lerp2(int64_t A, int64_t B, int64_t t, int64_t t_1);

  static double normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2, const vpImage<double> &II,
                                      const vpImage<double> &IIsq, const vpImage<double> &II_tpl,
                                      const vpImage<double> &IIsq_tpl, const unsigned int i0, const unsigned int j0);

  template <class Type>
  static void resizeBicubic(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i, const unsigned int j,
                            const float u, const float v, const float xFrac, const float yFrac);

  template <class Type>
  static void resizeBilinear(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i, const unsigned int j,
                             const float u, const float v, const float xFrac, const float yFrac);

  template <class Type>
  static void resizeNearest(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i, const unsigned int j,
                            const float u, const float v);

  template <class Type>
  static void warpNN(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine, bool centerCorner, bool fixedPoint);

  template <class Type>
  static void warpLinear(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine, bool centerCorner, bool fixedPoint);

  static bool checkFixedPoint(unsigned int x, unsigned int y, const vpMatrix &T, bool affine);
};

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  Crop a region of interest (ROI) in an image.

  \deprecated This fonction is deprecated. You should rather use
  crop(const vpImage<Type> &, unsigned int, unsigned int, unsigned int,
  unsigned int, vpImage<Type> &).

  \param I : Input image from which a sub image will be extracted.
  \param roi_top : ROI vertical position of the upper/left corner in the input
  image.
  \param roi_left : ROI  horizontal position of the upper/left corner
  in the input image.
  \param roi_height : Cropped image height corresponding to the ROI height.
  \param roi_width : Cropped image width corresponding to the ROI height.
  \param crop : Cropped image.

  \sa crop(const vpImage<Type> &, unsigned int, unsigned int, unsigned int,
  unsigned int, vpImage<Type> &)
*/
template <class Type>
void vpImageTools::createSubImage(const vpImage<Type> &I, unsigned int roi_top, unsigned int roi_left,
                                  unsigned int roi_height, unsigned int roi_width, vpImage<Type> &crop)
{
  vpImageTools::crop(I, roi_top, roi_left, roi_height, roi_width, crop);
}

/*!
  Crop an image region of interest.

  \deprecated This fonction is deprecated. You should rather use
  crop(const vpImage<Type> &, const vpRect &, vpImage<Type> &).

  \param I : Input image from which a sub image will be extracted.

  \param roi : Region of interest in image \e I corresponding to the
  cropped part of the image.

  \param crop : Cropped image.

  \sa crop(const vpImage<Type> &, const vpRect &, vpImage<Type> &)
*/
template <class Type> void vpImageTools::createSubImage(const vpImage<Type> &I, const vpRect &roi, vpImage<Type> &crop)
{
  vpImageTools::crop(I, roi, crop);
}

#endif // #if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  Crop a region of interest (ROI) in an image. The ROI coordinates and
  dimension are defined in the original image.

  Setting \e v_scale and \e h_scale to values different from 1 allows also to
  subsample the cropped image.

  \param I : Input image from which a sub image will be extracted.
  \param roi_top : ROI vertical position of the upper/left corner in the input
  image.
  \param roi_left : ROI  horizontal position of the upper/left corner
  in the input image.
  \param roi_height : Cropped image height corresponding
  to the ROI height.
  \param roi_width : Cropped image width corresponding to
  the ROI height.
  \param crop : Cropped image.
  \param v_scale [in] : Vertical subsampling factor applied to the ROI.
  \param h_scale [in] : Horizontal subsampling factor applied to the ROI.

  \sa crop(const vpImage<Type> &, const vpRect &, vpImage<Type> &)
*/
template <class Type>
void vpImageTools::crop(const vpImage<Type> &I, double roi_top, double roi_left, unsigned int roi_height,
                        unsigned int roi_width, vpImage<Type> &crop, unsigned int v_scale, unsigned int h_scale)
{
  int i_min = (std::max)((int)(ceil(roi_top / v_scale)), 0);
  int j_min = (std::max)((int)(ceil(roi_left / h_scale)), 0);
  int i_max = (std::min)((int)(ceil((roi_top + roi_height)) / v_scale), (int)(I.getHeight() / v_scale));
  int j_max = (std::min)((int)(ceil((roi_left + roi_width) / h_scale)), (int)(I.getWidth() / h_scale));

  unsigned int i_min_u = (unsigned int)i_min;
  unsigned int j_min_u = (unsigned int)j_min;

  unsigned int r_width = (unsigned int)(j_max - j_min);
  unsigned int r_height = (unsigned int)(i_max - i_min);

  crop.resize(r_height, r_width);

  if (v_scale == 1 && h_scale == 1) {
    for (unsigned int i = 0; i < r_height; i++) {
      void *src = (void *)(I[i + i_min_u] + j_min_u);
      void *dst = (void *)crop[i];
      memcpy(dst, src, r_width * sizeof(Type));
    }
  } else if (h_scale == 1) {
    for (unsigned int i = 0; i < r_height; i++) {
      void *src = (void *)(I[(i + i_min_u) * v_scale] + j_min_u);
      void *dst = (void *)crop[i];
      memcpy(dst, src, r_width * sizeof(Type));
    }
  } else {
    for (unsigned int i = 0; i < r_height; i++) {
      for (unsigned int j = 0; j < r_width; j++) {
        crop[i][j] = I[(i + i_min_u) * v_scale][(j + j_min_u) * h_scale];
      }
    }
  }
}

/*!
  Crop a region of interest (ROI) in an image. The ROI coordinates and
  dimension are defined in the original image.

  Setting \e v_scale and \e h_scale to values different from 1 allows also to
  subsample the cropped image.

  \param I : Input image from which a sub image will be extracted.
  \param topLeft : ROI position of the upper/left corner in the input image.
  \param roi_height : Cropped image height corresponding to the ROI height.
  \param roi_width : Cropped image width corresponding to the ROI height.
  \param crop : Cropped image.
  \param v_scale [in] : Vertical subsampling factor applied to the ROI.
  \param h_scale [in] : Horizontal subsampling factor applied to the ROI.

  \sa crop(const vpImage<Type> &, const vpRect &, vpImage<Type> &)

*/
template <class Type>
void vpImageTools::crop(const vpImage<Type> &I, const vpImagePoint &topLeft, unsigned int roi_height,
                        unsigned int roi_width, vpImage<Type> &crop, unsigned int v_scale, unsigned int h_scale)
{
  vpImageTools::crop(I, topLeft.get_i(), topLeft.get_j(), roi_height, roi_width, crop, v_scale, h_scale);
}

/*!
  Crop a region of interest (ROI) in an image. The ROI coordinates and
  dimension are defined in the original image.

  Setting \e v_scale and \e h_scale to values different from 1 allows also to
  subsample the cropped image.

  \param I : Input image from which a sub image will be extracted.

  \param roi : Region of interest in image \e I corresponding to the
  cropped part of the image.

  \param crop : Cropped image.
  \param v_scale [in] : Vertical subsampling factor applied to the ROI.
  \param h_scale [in] : Horizontal subsampling factor applied to the ROI.

*/
template <class Type>
void vpImageTools::crop(const vpImage<Type> &I, const vpRect &roi, vpImage<Type> &crop, unsigned int v_scale,
                        unsigned int h_scale)
{
  vpImageTools::crop(I, roi.getTop(), roi.getLeft(), (unsigned int)roi.getHeight(), (unsigned int)roi.getWidth(), crop,
                     v_scale, h_scale);
}

/*!
  Crop a region of interest (ROI) in an image. The ROI coordinates and
  dimension are defined in the original image.

  Setting \e v_scale and \e h_scale to values different from 1 allows also to
  subsample the cropped image.

  \param bitmap : Pointer to the input image from which a sub image will be
  extracted. \param width, height : Size of the input image.

  \param roi : Region of interest corresponding to the cropped part of the
  image.

  \param crop : Cropped image.
  \param v_scale [in] : Vertical subsampling factor applied to the ROI.
  \param h_scale [in] : Horizontal subsampling factor applied to the ROI.
*/
template <class Type>
void vpImageTools::crop(const unsigned char *bitmap, unsigned int width, unsigned int height, const vpRect &roi,
                        vpImage<Type> &crop, unsigned int v_scale, unsigned int h_scale)
{
  int i_min = (std::max)((int)(ceil(roi.getTop() / v_scale)), 0);
  int j_min = (std::max)((int)(ceil(roi.getLeft() / h_scale)), 0);
  int i_max = (std::min)((int)(ceil((roi.getTop() + roi.getHeight()) / v_scale)), (int)(height / v_scale));
  int j_max = (std::min)((int)(ceil((roi.getLeft() + roi.getWidth()) / h_scale)), (int)(width / h_scale));

  unsigned int i_min_u = (unsigned int)i_min;
  unsigned int j_min_u = (unsigned int)j_min;

  unsigned int r_width = (unsigned int)(j_max - j_min);
  unsigned int r_height = (unsigned int)(i_max - i_min);

  crop.resize(r_height, r_width);

  if (v_scale == 1 && h_scale == 1) {
    for (unsigned int i = 0; i < r_height; i++) {
      void *src = (void *)(bitmap + ((i + i_min_u) * width + j_min_u) * sizeof(Type));
      void *dst = (void *)crop[i];
      memcpy(dst, src, r_width * sizeof(Type));
    }
  } else if (h_scale == 1) {
    for (unsigned int i = 0; i < r_height; i++) {
      void *src = (void *)(bitmap + ((i + i_min_u) * width * v_scale + j_min_u) * sizeof(Type));
      void *dst = (void *)crop[i];
      memcpy(dst, src, r_width * sizeof(Type));
    }
  } else {
    for (unsigned int i = 0; i < r_height; i++) {
      unsigned int i_src = (i + i_min_u) * width * v_scale + j_min_u * h_scale;
      for (unsigned int j = 0; j < r_width; j++) {
        void *src = (void *)(bitmap + (i_src + j * h_scale) * sizeof(Type));
        void *dst = (void *)&crop[i][j];
        memcpy(dst, src, sizeof(Type));
      }
    }
  }
}

/*!
  Binarise an image.

  - Pixels whose values are less than \e threshold1 are set to \e value1

  - Pixels whose values are greater then or equal to \e threshold1 and
    less then or equal to \e threshold2 are set to \e value2

  - Pixels whose values are greater than \e threshold2 are set to \e value3
*/
template <class Type>
inline void vpImageTools::binarise(vpImage<Type> &I, Type threshold1, Type threshold2, Type value1, Type value2,
                                   Type value3, const bool useLUT)
{
  if (useLUT) {
    std::cerr << "LUT not available for this type ! Will use the iteration method." << std::endl;
  }

  Type v;
  Type *p = I.bitmap;
  Type *pend = I.bitmap + I.getWidth() * I.getHeight();
  for (; p < pend; p++) {
    v = *p;
    if (v < threshold1)
      *p = value1;
    else if (v > threshold2)
      *p = value3;
    else
      *p = value2;
  }
}

/*!
  Binarise an image.

  - Pixels whose values are less than \e threshold1 are set to \e value1

  - Pixels whose values are greater then or equal to \e threshold1 and
    less then or equal to \e threshold2 are set to \e value2

  - Pixels whose values are greater than \e threshold2 are set to \e value3
*/
template <>
inline void vpImageTools::binarise(vpImage<unsigned char> &I, unsigned char threshold1, unsigned char threshold2,
                                   unsigned char value1, unsigned char value2, unsigned char value3, const bool useLUT)
{
  if (useLUT) {
    // Construct the LUT
    unsigned char lut[256];
    for (unsigned int i = 0; i < 256; i++) {
      lut[i] = i < threshold1 ? value1 : (i > threshold2 ? value3 : value2);
    }

    I.performLut(lut);
  } else {
    unsigned char *p = I.bitmap;
    unsigned char *pend = I.bitmap + I.getWidth() * I.getHeight();
    for (; p < pend; p++) {
      unsigned char v = *p;
      if (v < threshold1)
        *p = value1;
      else if (v > threshold2)
        *p = value3;
      else
        *p = value2;
    }
  }
}

#ifdef VISP_HAVE_PTHREAD

#ifndef DOXYGEN_SHOULD_SKIP_THIS
template <class Type> class vpUndistortInternalType
{
public:
  Type *src;
  Type *dst;
  unsigned int width;
  unsigned int height;
  vpCameraParameters cam;
  unsigned int nthreads;
  unsigned int threadid;

public:
  vpUndistortInternalType() : src(NULL), dst(NULL), width(0), height(0), cam(), nthreads(0), threadid(0) {}

  vpUndistortInternalType(const vpUndistortInternalType<Type> &u) { *this = u; }
  vpUndistortInternalType &operator=(const vpUndistortInternalType<Type> &u)
  {
    src = u.src;
    dst = u.dst;
    width = u.width;
    height = u.height;
    cam = u.cam;
    nthreads = u.nthreads;
    threadid = u.threadid;

    return *this;
  }

  static void *vpUndistort_threaded(void *arg);
};

template <class Type> void *vpUndistortInternalType<Type>::vpUndistort_threaded(void *arg)
{
  vpUndistortInternalType<Type> *undistortSharedData = static_cast<vpUndistortInternalType<Type> *>(arg);
  int offset = (int)undistortSharedData->threadid;
  int width = (int)undistortSharedData->width;
  int height = (int)undistortSharedData->height;
  int nthreads = (int)undistortSharedData->nthreads;

  double u0 = undistortSharedData->cam.get_u0();
  double v0 = undistortSharedData->cam.get_v0();
  double px = undistortSharedData->cam.get_px();
  double py = undistortSharedData->cam.get_py();
  double kud = undistortSharedData->cam.get_kud();

  double invpx = 1.0 / px;
  double invpy = 1.0 / py;

  double kud_px2 = kud * invpx * invpx;
  double kud_py2 = kud * invpy * invpy;

  Type *dst = undistortSharedData->dst + (height / nthreads * offset) * width;
  Type *src = undistortSharedData->src;

  for (double v = height / nthreads * offset; v < height / nthreads * (offset + 1); v++) {
    double deltav = v - v0;
    // double fr1 = 1.0 + kd * (vpMath::sqr(deltav * invpy));
    double fr1 = 1.0 + kud_py2 * deltav * deltav;

    for (double u = 0; u < width; u++) {
      // computation of u,v : corresponding pixel coordinates in I.
      double deltau = u - u0;
      // double fr2 = fr1 + kd * (vpMath::sqr(deltau * invpx));
      double fr2 = fr1 + kud_px2 * deltau * deltau;

      double u_double = deltau * fr2 + u0;
      double v_double = deltav * fr2 + v0;

      // computation of the bilinear interpolation

      // declarations
      int u_round = (int)(u_double);
      int v_round = (int)(v_double);
      if (u_round < 0.f)
        u_round = -1;
      if (v_round < 0.f)
        v_round = -1;
      double du_double = (u_double) - (double)u_round;
      double dv_double = (v_double) - (double)v_round;
      Type v01;
      Type v23;
      if ((0 <= u_round) && (0 <= v_round) && (u_round < ((width)-1)) && (v_round < ((height)-1))) {
        // process interpolation
        const Type *_mp = &src[v_round * width + u_round];
        v01 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        _mp += width;
        v23 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        *dst = (Type)(v01 + ((v23 - v01) * dv_double));
      } else {
        *dst = 0;
      }
      dst++;
    }
  }

  pthread_exit((void *)0);
  return NULL;
}
#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // VISP_HAVE_PTHREAD

/*!
  Undistort an image

  \param I : Input image to undistort.

  \param cam : Parameters of the camera causing distortion.

  \param undistI : Undistorted output image. The size of this image
  will be the same than the input image \e I. If the distortion
  parameter \f$K_d\f$ is null (see cam.get_kd_mp()), \e undistI is
  just a copy of \e I.

  \param nThreads : Number of threads to use if pthreads library is available.

  \warning This function works only with Types authorizing "+,-,
  multiplication by a scalar" operators.

  \warning This function is time consuming :
    - On "Rhea"(Intel Core 2 Extreme X6800 2.93GHz, 2Go RAM)
      or "Charon"(Intel Xeon 3 GHz, 2Go RAM) : ~8 ms for a 640x480 image.

  \note If you want to undistort multiple images, you should call `vpImageTools::initUndistortMap()`
  once and then `vpImageTools::remap()` to undistort the images. This will be less time consuming.

  \sa initUndistortMap, remap
*/
template <class Type>
void vpImageTools::undistort(const vpImage<Type> &I, const vpCameraParameters &cam, vpImage<Type> &undistI,
                             unsigned int nThreads)
{
#ifdef VISP_HAVE_PTHREAD
  //
  // Optimized version using pthreads
  //
  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  undistI.resize(height, width);

  double kud = cam.get_kud();

  // if (kud == 0) {
  if (std::fabs(kud) <= std::numeric_limits<double>::epsilon()) {
    // There is no need to undistort the image
    undistI = I;
    return;
  }

  unsigned int nthreads = nThreads;
  pthread_attr_t attr;
  pthread_t *callThd = new pthread_t[nthreads];
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  vpUndistortInternalType<Type> *undistortSharedData;
  undistortSharedData = new vpUndistortInternalType<Type>[nthreads];

  for (unsigned int i = 0; i < nthreads; i++) {
    // Each thread works on a different set of data.
    //    vpTRACE("create thread %d", i);
    undistortSharedData[i].src = I.bitmap;
    undistortSharedData[i].dst = undistI.bitmap;
    undistortSharedData[i].width = I.getWidth();
    undistortSharedData[i].height = I.getHeight();
    undistortSharedData[i].cam = cam;
    undistortSharedData[i].nthreads = nthreads;
    undistortSharedData[i].threadid = i;
    pthread_create(&callThd[i], &attr, &vpUndistortInternalType<Type>::vpUndistort_threaded, &undistortSharedData[i]);
  }
  pthread_attr_destroy(&attr);
  /* Wait on the other threads */

  for (unsigned int i = 0; i < nthreads; i++) {
    //  vpTRACE("join thread %d", i);
    pthread_join(callThd[i], NULL);
  }

  delete[] callThd;
  delete[] undistortSharedData;
#else  // VISP_HAVE_PTHREAD
  (void)nThreads;
  //
  // optimized version without pthreads
  //
  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  undistI.resize(height, width);

  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double px = cam.get_px();
  double py = cam.get_py();
  double kud = cam.get_kud();

  // if (kud == 0) {
  if (std::fabs(kud) <= std::numeric_limits<double>::epsilon()) {
    // There is no need to undistort the image
    undistI = I;
    return;
  }

  double invpx = 1.0 / px;
  double invpy = 1.0 / py;

  double kud_px2 = kud * invpx * invpx;
  double kud_py2 = kud * invpy * invpy;

  Type *dst = undistI.bitmap;
  for (double v = 0; v < height; v++) {
    double deltav = v - v0;
    // double fr1 = 1.0 + kd * (vpMath::sqr(deltav * invpy));
    double fr1 = 1.0 + kud_py2 * deltav * deltav;

    for (double u = 0; u < width; u++) {
      // computation of u,v : corresponding pixel coordinates in I.
      double deltau = u - u0;
      // double fr2 = fr1 + kd * (vpMath::sqr(deltau * invpx));
      double fr2 = fr1 + kud_px2 * deltau * deltau;

      double u_double = deltau * fr2 + u0;
      double v_double = deltav * fr2 + v0;

      // printf("[%g][%g] %g %g : ", u, v, u_double, v_double );

      // computation of the bilinear interpolation

      // declarations
      int u_round = (int)(u_double);
      int v_round = (int)(v_double);
      if (u_round < 0.f)
        u_round = -1;
      if (v_round < 0.f)
        v_round = -1;
      double du_double = (u_double) - (double)u_round;
      double dv_double = (v_double) - (double)v_round;
      Type v01;
      Type v23;
      if ((0 <= u_round) && (0 <= v_round) && (u_round < (((int)width) - 1)) && (v_round < (((int)height) - 1))) {
        // process interpolation
        const Type *_mp = &I[(unsigned int)v_round][(unsigned int)u_round];
        v01 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        _mp += width;
        v23 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        *dst = (Type)(v01 + ((v23 - v01) * dv_double));
        // printf("R %d G %d B %d\n", dst->R, dst->G, dst->B);
      } else {
        *dst = 0;
      }
      dst++;
    }
  }
#endif // VISP_HAVE_PTHREAD

#if 0
  // non optimized version
  int width = I.getWidth();
  int height = I.getHeight();

  undistI.resize(height,width);

  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double px = cam.get_px();
  double py = cam.get_py();
  double kd = cam.get_kud();

  if (kd == 0) {
    // There is no need to undistort the image
    undistI = I;
    return;
  }

  for(int v = 0 ; v < height; v++){
    for(int u = 0; u < height; u++){
      double r2 = vpMath::sqr(((double)u - u0)/px) +
                  vpMath::sqr(((double)v-v0)/py);
      double u_double = ((double)u - u0)*(1.0+kd*r2) + u0;
      double v_double = ((double)v - v0)*(1.0+kd*r2) + v0;
      undistI[v][u] = I.getPixelBI((float)u_double,(float)v_double);
    }
  }
#endif
}

/*!
  Flip vertically the input image and give the result in the output image.

  \param I : Input image to flip.
  \param newI : Output image which is the flipped input image.
*/

template <class Type> void vpImageTools::flip(const vpImage<Type> &I, vpImage<Type> &newI)
{
  unsigned int height = 0, width = 0;

  height = I.getHeight();
  width = I.getWidth();
  newI.resize(height, width);

  for (unsigned int i = 0; i < height; i++) {
    memcpy(newI.bitmap + i * width, I.bitmap + (height - 1 - i) * width, width * sizeof(Type));
  }
}

/*!
  Flip vertically the input image.

  \param I : Input image which is flipped and modified in output.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  vpImage<vpRGBa> I;
#ifdef _WIN32
  std::string filename("C:/temp/ViSP-images/Klimt/Klimt.ppm");
#else
  std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.ppm");
#endif

  // Read an image from the disk
  vpImageIo::read(I, filename);

  // Flip the image
  vpImageTools::flip(I);

  // Write the image in a PGM P5 image file format
  vpImageIo::write(I, "Klimt-flip.ppm");
}
  \endcode
*/
template <class Type> void vpImageTools::flip(vpImage<Type> &I)
{
  unsigned int height = 0, width = 0;
  unsigned int i = 0;
  vpImage<Type> Ibuf;

  height = I.getHeight();
  width = I.getWidth();
  Ibuf.resize(1, width);

  for (i = 0; i < height / 2; i++) {
    memcpy(Ibuf.bitmap, I.bitmap + i * width, width * sizeof(Type));

    memcpy(I.bitmap + i * width, I.bitmap + (height - 1 - i) * width, width * sizeof(Type));
    memcpy(I.bitmap + (height - 1 - i) * width, Ibuf.bitmap, width * sizeof(Type));
  }
}

template <class Type> Type vpImageTools::getPixelClamped(const vpImage<Type> &I, const float u, const float v)
{
  unsigned int i, j;
  if (u < 0.f)
    j = 0;
  else if (u > static_cast<float>(I.getWidth()) - 1.f)
    j = I.getWidth() - 1;
  else
    j = static_cast<unsigned int>(u);

  if (v < 0.f)
    i = 0;
  else if (v > static_cast<float>(I.getHeight()) - 1.f)
    i = I.getHeight() - 1;
  else
    i = static_cast<unsigned int>(v);

  return I[i][j];
}

// Reference:
// http://blog.demofox.org/2015/08/15/resizing-images-with-bicubic-interpolation/
template <class Type>
void vpImageTools::resizeBicubic(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i,
                                 const unsigned int j, const float u, const float v, const float xFrac,
                                 const float yFrac)
{
  // 1st row
  Type p00 = getPixelClamped(I, u - 1, v - 1);
  Type p01 = getPixelClamped(I, u + 0, v - 1);
  Type p02 = getPixelClamped(I, u + 1, v - 1);
  Type p03 = getPixelClamped(I, u + 2, v - 1);

  // 2nd row
  Type p10 = getPixelClamped(I, u - 1, v + 0);
  Type p11 = getPixelClamped(I, u + 0, v + 0);
  Type p12 = getPixelClamped(I, u + 1, v + 0);
  Type p13 = getPixelClamped(I, u + 2, v + 0);

  // 3rd row
  Type p20 = getPixelClamped(I, u - 1, v + 1);
  Type p21 = getPixelClamped(I, u + 0, v + 1);
  Type p22 = getPixelClamped(I, u + 1, v + 1);
  Type p23 = getPixelClamped(I, u + 2, v + 1);

  // 4th row
  Type p30 = getPixelClamped(I, u - 1, v + 2);
  Type p31 = getPixelClamped(I, u + 0, v + 2);
  Type p32 = getPixelClamped(I, u + 1, v + 2);
  Type p33 = getPixelClamped(I, u + 2, v + 2);

  float col0 = cubicHermite(p00, p01, p02, p03, xFrac);
  float col1 = cubicHermite(p10, p11, p12, p13, xFrac);
  float col2 = cubicHermite(p20, p21, p22, p23, xFrac);
  float col3 = cubicHermite(p30, p31, p32, p33, xFrac);
  float value = cubicHermite(col0, col1, col2, col3, yFrac);
  Ires[i][j] = vpMath::saturate<Type>(value);
}

template <>
inline void vpImageTools::resizeBicubic(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, const unsigned int i,
                                        const unsigned int j, const float u, const float v, const float xFrac,
                                        const float yFrac)
{
  // 1st row
  vpRGBa p00 = getPixelClamped(I, u - 1, v - 1);
  vpRGBa p01 = getPixelClamped(I, u + 0, v - 1);
  vpRGBa p02 = getPixelClamped(I, u + 1, v - 1);
  vpRGBa p03 = getPixelClamped(I, u + 2, v - 1);

  // 2nd row
  vpRGBa p10 = getPixelClamped(I, u - 1, v + 0);
  vpRGBa p11 = getPixelClamped(I, u + 0, v + 0);
  vpRGBa p12 = getPixelClamped(I, u + 1, v + 0);
  vpRGBa p13 = getPixelClamped(I, u + 2, v + 0);

  // 3rd row
  vpRGBa p20 = getPixelClamped(I, u - 1, v + 1);
  vpRGBa p21 = getPixelClamped(I, u + 0, v + 1);
  vpRGBa p22 = getPixelClamped(I, u + 1, v + 1);
  vpRGBa p23 = getPixelClamped(I, u + 2, v + 1);

  // 4th row
  vpRGBa p30 = getPixelClamped(I, u - 1, v + 2);
  vpRGBa p31 = getPixelClamped(I, u + 0, v + 2);
  vpRGBa p32 = getPixelClamped(I, u + 1, v + 2);
  vpRGBa p33 = getPixelClamped(I, u + 2, v + 2);

  for (int c = 0; c < 3; c++) {
    float col0 = cubicHermite(static_cast<float>(reinterpret_cast<unsigned char *>(&p00)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p01)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p02)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p03)[c]), xFrac);
    float col1 = cubicHermite(static_cast<float>(reinterpret_cast<unsigned char *>(&p10)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p11)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p12)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p13)[c]), xFrac);
    float col2 = cubicHermite(static_cast<float>(reinterpret_cast<unsigned char *>(&p20)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p21)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p22)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p23)[c]), xFrac);
    float col3 = cubicHermite(static_cast<float>(reinterpret_cast<unsigned char *>(&p30)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p31)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p32)[c]),
                              static_cast<float>(reinterpret_cast<unsigned char *>(&p33)[c]), xFrac);
    float value = cubicHermite(col0, col1, col2, col3, yFrac);

    reinterpret_cast<unsigned char *>(&Ires[i][j])[c] = vpMath::saturate<unsigned char>(value);
  }
}

template <class Type>
void vpImageTools::resizeBilinear(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i,
                                  const unsigned int j, const float u, const float v, const float xFrac,
                                  const float yFrac)
{
  unsigned int u0 = static_cast<unsigned int>(u);
  unsigned int v0 = static_cast<unsigned int>(v);

  unsigned int u1 = (std::min)(I.getWidth() - 1, static_cast<unsigned int>(u) + 1);
  unsigned int v1 = v0;

  unsigned int u2 = u0;
  unsigned int v2 = (std::min)(I.getHeight() - 1, static_cast<unsigned int>(v) + 1);

  unsigned int u3 = u1;
  unsigned int v3 = v2;

  float col0 = lerp(I[v0][u0], I[v1][u1], xFrac);
  float col1 = lerp(I[v2][u2], I[v3][u3], xFrac);
  float value = lerp(col0, col1, yFrac);

  Ires[i][j] = vpMath::saturate<Type>(value);
}

template <>
inline void vpImageTools::resizeBilinear(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, const unsigned int i,
                                         const unsigned int j, const float u, const float v, const float xFrac,
                                         const float yFrac)
{
  unsigned int u0 = static_cast<unsigned int>(u);
  unsigned int v0 = static_cast<unsigned int>(v);

  unsigned int u1 = (std::min)(I.getWidth() - 1, static_cast<unsigned int>(u) + 1);
  unsigned int v1 = v0;

  unsigned int u2 = u0;
  unsigned int v2 = (std::min)(I.getHeight() - 1, static_cast<unsigned int>(v) + 1);

  unsigned int u3 = (std::min)(I.getWidth() - 1, static_cast<unsigned int>(u) + 1);
  unsigned int v3 = (std::min)(I.getHeight() - 1, static_cast<unsigned int>(v) + 1);

  for (int c = 0; c < 3; c++) {
    float col0 = lerp(static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v0][u0])[c]),
                      static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v1][u1])[c]), xFrac);
    float col1 = lerp(static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v2][u2])[c]),
                      static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v3][u3])[c]), xFrac);
    float value = lerp(col0, col1, yFrac);

    reinterpret_cast<unsigned char *>(&Ires[i][j])[c] = vpMath::saturate<unsigned char>(value);
  }
}

template <class Type>
void vpImageTools::resizeNearest(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int i,
                                 const unsigned int j, const float u, const float v)
{
  Ires[i][j] = getPixelClamped(I, u, v);
}

/*!
  Resize the image using one interpolation method (by default it uses the
  nearest neighbor interpolation).

  \param I : Input image.
  \param Ires : Output image resized to \e width, \e height.
  \param width : Resized width.
  \param height : Resized height.
  \param method : Interpolation method.
  \param nThreads : Number of threads to use if OpenMP is available.

  \warning The input \e I and output \e Ires images must be different.
*/
template <class Type>
void vpImageTools::resize(const vpImage<Type> &I, vpImage<Type> &Ires, const unsigned int width,
                          const unsigned int height, const vpImageInterpolationType &method,
                          unsigned int nThreads)
{
  Ires.resize(height, width);

  vpImageTools::resize(I, Ires, method, nThreads);
}

/*!
  Resize the image using one interpolation method (by default it uses the
  nearest neighbor interpolation).

  \param I : Input image.
  \param Ires : Output image resized (you have to init the image \e Ires at
  the desired size).
  \param method : Interpolation method.
  \param nThreads : Number of threads to use if OpenMP is available.

  \warning The input \e I and output \e Ires images must be different.
*/
template <class Type>
void vpImageTools::resize(const vpImage<Type> &I, vpImage<Type> &Ires, const vpImageInterpolationType &method,
                          unsigned int
                        #if defined _OPENMP
                          nThreads
                        #endif
                          )
{
  if (I.getWidth() < 2 || I.getHeight() < 2 || Ires.getWidth() < 2 || Ires.getHeight() < 2) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  float scaleY = (I.getHeight() - 1) / static_cast<float>(Ires.getHeight() - 1);
  float scaleX = (I.getWidth() - 1) / static_cast<float>(Ires.getWidth() - 1);

  if (method == INTERPOLATION_NEAREST) {
    scaleY = I.getHeight() / static_cast<float>(Ires.getHeight() - 1);
    scaleX = I.getWidth() / static_cast<float>(Ires.getWidth() - 1);
  }

#if defined _OPENMP
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
  #pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < static_cast<int>(Ires.getHeight()); i++) {
    float v = i * scaleY;
    float yFrac = v - static_cast<int>(v);

    for (unsigned int j = 0; j < Ires.getWidth(); j++) {
      float u = j * scaleX;
      float xFrac = u - static_cast<int>(u);

      if (method == INTERPOLATION_NEAREST) {
        resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
      } else if (method == INTERPOLATION_LINEAR) {
        resizeBilinear(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
      } else if (method == INTERPOLATION_CUBIC) {
        resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
      }
    }
  }
}

template <> inline
void vpImageTools::resize(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires,
                          const vpImageInterpolationType &method, unsigned int
                                                                  #if defined _OPENMP
                                                                    nThreads
                                                                  #endif
                          )
{
  if (I.getWidth() < 2 || I.getHeight() < 2 || Ires.getWidth() < 2 || Ires.getHeight() < 2) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  if (method == INTERPOLATION_NEAREST || method == INTERPOLATION_CUBIC) {
    float scaleY = (I.getHeight() - 1) / static_cast<float>(Ires.getHeight() - 1);
    float scaleX = (I.getWidth() - 1) / static_cast<float>(Ires.getWidth() - 1);

    if (method == INTERPOLATION_NEAREST) {
      scaleY = I.getHeight() / static_cast<float>(Ires.getHeight() - 1);
      scaleX = I.getWidth() / static_cast<float>(Ires.getWidth() - 1);
    }

  #if defined _OPENMP
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
    #pragma omp parallel for schedule(dynamic)
  #endif
    for (int i = 0; i < static_cast<int>(Ires.getHeight()); i++) {
      float v = i * scaleY;
      float yFrac = v - static_cast<int>(v);

      for (unsigned int j = 0; j < Ires.getWidth(); j++) {
        float u = j * scaleX;
        float xFrac = u - static_cast<int>(u);

        if (method == INTERPOLATION_NEAREST) {
          resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
        } else if (method == INTERPOLATION_CUBIC) {
          resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
        }
      }
    }
  } else if (method == INTERPOLATION_LINEAR) {
    const int precision = 1 << 16;
    int64_t scaleY = static_cast<int64_t>((I.getHeight() - 1) / static_cast<float>(Ires.getHeight() - 1) * precision);
    int64_t scaleX = static_cast<int64_t>((I.getWidth() - 1) / static_cast<float>(Ires.getWidth() - 1) * precision);

#if defined _OPENMP
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < static_cast<int>(Ires.getHeight()); i++) {
      int64_t v = i * scaleY;
      int64_t vround = v & (~0xFFFF);
      int64_t rratio = v - vround;
      int64_t y_ = v >> 16;
      int64_t rfrac = precision - rratio;

      for (unsigned int j = 0; j < Ires.getWidth(); j++) {
        int64_t u = j * scaleX;
        int64_t uround = u & (~0xFFFF);
        int64_t cratio = u - uround;
        int64_t x_ = u >> 16;
        int64_t cfrac = precision - cratio;

        if (y_ + 1 < static_cast<int64_t>(I.getHeight()) && x_ + 1 < static_cast<int64_t>(I.getWidth())) {
          int64_t up = *reinterpret_cast<uint16_t *>(I.bitmap + y_ * I.getWidth() + x_);
          int64_t down = *reinterpret_cast<uint16_t *>(I.bitmap + (y_ + 1) * I.getWidth() + x_);

          Ires[i][j] = static_cast<unsigned char>((((up & 0x00FF) * rfrac + (down & 0x00FF) * rratio) * cfrac +
                                                  ((up >> 8) * rfrac + (down >> 8) * rratio) * cratio) >> 32);
        } else if (y_ + 1 < static_cast<int64_t>(I.getHeight())) {
          Ires[i][j] = static_cast<unsigned char>(((*(I.bitmap + y_ * I.getWidth() + x_)
                                                  * rfrac + *(I.bitmap + (y_ + 1) * I.getWidth() + x_) * rratio)) >> 16);
        } else if (x_ + 1 < static_cast<int64_t>(I.getWidth())) {
          uint16_t up = *reinterpret_cast<uint16_t *>(I.bitmap + y_ * I.getWidth() + x_);
          Ires[i][j] = static_cast<unsigned char>(((up & 0x00FF) * cfrac + (up >> 8) * cratio) >> 16);
        } else {
          Ires[i][j] = *(I.bitmap + y_ * I.getWidth() + x_);
        }
      }
    }
  }
}

template <> inline
void vpImageTools::resize(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires,
                          const vpImageInterpolationType &method, unsigned int
                                                                  #if defined _OPENMP
                                                                    nThreads
                                                                  #endif
                          )
{
  if (I.getWidth() < 2 || I.getHeight() < 2 || Ires.getWidth() < 2 || Ires.getHeight() < 2) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  if (method == INTERPOLATION_NEAREST || method == INTERPOLATION_CUBIC) {
    float scaleY = (I.getHeight() - 1) / static_cast<float>(Ires.getHeight() - 1);
    float scaleX = (I.getWidth() - 1) / static_cast<float>(Ires.getWidth() - 1);

    if (method == INTERPOLATION_NEAREST) {
      scaleY = I.getHeight() / static_cast<float>(Ires.getHeight() - 1);
      scaleX = I.getWidth() / static_cast<float>(Ires.getWidth() - 1);
    }

  #if defined _OPENMP
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
    #pragma omp parallel for schedule(dynamic)
  #endif
    for (int i = 0; i < static_cast<int>(Ires.getHeight()); i++) {
      float v = i * scaleY;
      float yFrac = v - static_cast<int>(v);

      for (unsigned int j = 0; j < Ires.getWidth(); j++) {
        float u = j * scaleX;
        float xFrac = u - static_cast<int>(u);

        if (method == INTERPOLATION_NEAREST) {
          resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
        } else if (method == INTERPOLATION_CUBIC) {
          resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
        }
      }
    }
  } else {
    const int precision = 1 << 16;
    int64_t scaleY = static_cast<int64_t>((I.getHeight() - 1) / static_cast<float>(Ires.getHeight() - 1) * precision);
    int64_t scaleX = static_cast<int64_t>((I.getWidth() - 1) / static_cast<float>(Ires.getWidth() - 1) * precision);

#if defined _OPENMP
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < static_cast<int>(Ires.getHeight()); i++) {
      int64_t v = i * scaleY;
      int64_t vround = v & (~0xFFFF);
      int64_t rratio = v - vround;
      int64_t y_ = v >> 16;
      int64_t rfrac = precision - rratio;

      for (unsigned int j = 0; j < Ires.getWidth(); j++) {
        int64_t u = j * scaleX;
        int64_t uround = u & (~0xFFFF);
        int64_t cratio = u - uround;
        int64_t x_ = u >> 16;
        int64_t cfrac = precision - cratio;

        if (y_ + 1 < static_cast<int64_t>(I.getHeight()) && x_ + 1 < static_cast<int64_t>(I.getWidth())) {
          int64_t col0 = lerp2((I.bitmap + y_ * I.getWidth() + x_)->R, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->R, rratio, rfrac);
          int64_t col1 = lerp2((I.bitmap + y_ * I.getWidth() + x_ + 1)->R, (I.bitmap + (y_ + 1) * I.getWidth() + x_ + 1)->R, rratio, rfrac);
          int64_t valueR = lerp2(col0, col1, cratio, cfrac);

          col0 = lerp2((I.bitmap + y_ * I.getWidth() + x_)->G, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->G, rratio, rfrac);
          col1 = lerp2((I.bitmap + y_ * I.getWidth() + x_ + 1)->G, (I.bitmap + (y_ + 1) * I.getWidth() + x_ + 1)->G, rratio, rfrac);
          int64_t valueG = lerp2(col0, col1, cratio, cfrac);

          col0 = lerp2((I.bitmap + y_ * I.getWidth() + x_)->B, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->B, rratio, rfrac);
          col1 = lerp2((I.bitmap + y_ * I.getWidth() + x_ + 1)->B, (I.bitmap + (y_ + 1) * I.getWidth() + x_ + 1)->B, rratio, rfrac);
          int64_t valueB = lerp2(col0, col1, cratio, cfrac);

          Ires[i][j] = vpRGBa(static_cast<unsigned char>(valueR >> 32),
                              static_cast<unsigned char>(valueG >> 32),
                              static_cast<unsigned char>(valueB >> 32));
        } else if (y_ + 1 < static_cast<int64_t>(I.getHeight())) {
          int64_t valueR = lerp2((I.bitmap + y_ * I.getWidth() + x_)->R, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->R, rratio, rfrac);
          int64_t valueG = lerp2((I.bitmap + y_ * I.getWidth() + x_)->G, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->G, rratio, rfrac);
          int64_t valueB = lerp2((I.bitmap + y_ * I.getWidth() + x_)->B, (I.bitmap + (y_ + 1) * I.getWidth() + x_)->B, rratio, rfrac);

          Ires[i][j] = vpRGBa(static_cast<unsigned char>(valueR >> 16),
                              static_cast<unsigned char>(valueG >> 16),
                              static_cast<unsigned char>(valueB >> 16));
        } else if (x_ + 1 < static_cast<int64_t>(I.getWidth())) {
          int64_t valueR = lerp2((I.bitmap + x_)->R, (I.bitmap + x_ + 1)->R, cratio, cfrac);
          int64_t valueG = lerp2((I.bitmap + x_)->G, (I.bitmap + x_ + 1)->G, cratio, cfrac);
          int64_t valueB = lerp2((I.bitmap + x_)->B, (I.bitmap + x_ + 1)->B, cratio, cfrac);

          Ires[i][j] = vpRGBa(static_cast<unsigned char>(valueR >> 16),
                              static_cast<unsigned char>(valueG >> 16),
                              static_cast<unsigned char>(valueB >> 16));
        } else {
          Ires[i][j] = *(I.bitmap + y_ * I.getWidth() + x_);
        }
      }
    }
  }
}

/*!
  Apply a warping (affine or perspective) transformation to an image.

  \param src : Input image.
  \param T : Transformation / warping matrix, a `2x3` matrix for an affine transformation
  or a `3x3` matrix for a perspective transformation (homography).
  \param dst : Output image, if empty it will be of the same size than src and zero-initialized.
  \param interpolation : Interpolation method (only INTERPOLATION_NEAREST and INTERPOLATION_LINEAR
  are accepted, if INTERPOLATION_CUBIC is passed, INTERPOLATION_NEAREST will be used instead).
  \param fixedPointArithmetic : If true and if `pixelCenter` is false, fixed-point arithmetic is used if
  possible. Otherwise (e.g. the input image is too big) it fallbacks to the default implementation.
  \param pixelCenter : If true, pixel coordinates are at (0.5, 0.5), otherwise at (0,0). Fixed-point
  arithmetic cannot be used with `pixelCenter` option.
*/
template <class Type>
void vpImageTools::warpImage(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst,
                             const vpImageInterpolationType &interpolation,
                             bool fixedPointArithmetic, bool pixelCenter)
{
  if ((T.getRows() != 2 && T.getRows() != 3) || T.getCols() != 3) {
    std::cerr << "Input transformation must be a (2x3) or (3x3) matrix." << std::endl;
    return;
  }

  if (src.getSize() == 0) {
    return;
  }

  const bool affine = (T.getRows() == 2);
  const bool interp_NN = (interpolation == INTERPOLATION_NEAREST) || (interpolation == INTERPOLATION_CUBIC);

  if (dst.getSize() == 0) {
    dst.resize(src.getHeight(), src.getWidth(), Type(0));
  }

  vpMatrix M = T;
  if (affine) {
    double D = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    D = !vpMath::nul(D, std::numeric_limits<double>::epsilon()) ? 1.0 / D : 0;
    double A11 = M[1][1] * D, A22 = M[0][0] * D;
    M[0][0] = A11; M[0][1] *= -D;
    M[1][0] *= -D; M[1][1] = A22;
    double b1 = -M[0][0] * M[0][2] - M[0][1] * M[1][2];
    double b2 = -M[1][0] * M[0][2] - M[1][1] * M[1][2];
    M[0][2] = b1; M[1][2] = b2;
  } else {
    M = T.inverseByLU();
  }

  if (fixedPointArithmetic && !pixelCenter) {
    fixedPointArithmetic = checkFixedPoint(0, 0, M, affine) &&
                           checkFixedPoint(dst.getWidth()-1, 0, M, affine) &&
                           checkFixedPoint(0, dst.getHeight()-1, M, affine) &&
                           checkFixedPoint(dst.getWidth() - 1, dst.getHeight() - 1, M, affine);
  }

  if (interp_NN) {
    //nearest neighbor interpolation
    warpNN(src, M, dst, affine, pixelCenter, fixedPointArithmetic);
  } else {
    //bilinear interpolation
    warpLinear(src, M, dst, affine, pixelCenter, fixedPointArithmetic);
  }
}

template <class Type>
void vpImageTools::warpNN(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine,
                          bool centerCorner, bool fixedPoint)
{
  if (fixedPoint && !centerCorner) {
    const int nbits = 16;
    const int32_t precision = 1 << nbits;
    const float precision_1 = 1 / static_cast<float>(precision);

    int32_t a0_i32 = static_cast<int32_t>(T[0][0] * precision);
    int32_t a1_i32 = static_cast<int32_t>(T[0][1] * precision);
    int32_t a2_i32 = static_cast<int32_t>(T[0][2] * precision);
    int32_t a3_i32 = static_cast<int32_t>(T[1][0] * precision);
    int32_t a4_i32 = static_cast<int32_t>(T[1][1] * precision);
    int32_t a5_i32 = static_cast<int32_t>(T[1][2] * precision);
    int32_t a6_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[2][0] * precision) : 0;
    int32_t a7_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[2][1] * precision) : 0;
    int32_t a8_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[2][2] * precision) : 1;

    int32_t height_1_i32 = static_cast<int32_t>((src.getHeight() - 1) * precision) + 0x8000;
    int32_t width_1_i32 = static_cast<int32_t>((src.getWidth() - 1) * precision) + 0x8000;

    if (affine) {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int32_t xi = a2_i32;
        int32_t yi = a5_i32;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (yi >= 0 && yi < height_1_i32 && xi >= 0 && xi < width_1_i32) {
            float x_ = (xi >> nbits) + (xi & 0xFFFF) * precision_1;
            float y_ = (yi >> nbits) + (yi & 0xFFFF) * precision_1;

            int x = vpMath::round(x_);
            int y = vpMath::round(y_);
            dst[i][j] = src[y][x];
          }

          xi += a0_i32;
          yi += a3_i32;
        }

        a2_i32 += a1_i32;
        a5_i32 += a4_i32;
      }
    } else {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int64_t xi = a2_i32;
        int64_t yi = a5_i32;
        int64_t wi = a8_i32;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (wi != 0 && yi >= 0 && yi <= (static_cast<int>(src.getHeight()) - 1)*wi &&
              xi >= 0 && xi <= (static_cast<int>(src.getWidth()) - 1)*wi) {
            float w_ = (wi >> nbits) + (wi & 0xFFFF) * precision_1;
            float x_ = ((xi >> nbits) + (xi & 0xFFFF) * precision_1) / w_;
            float y_ = ((yi >> nbits) + (yi & 0xFFFF) * precision_1) / w_;

            int x = vpMath::round(x_);
            int y = vpMath::round(y_);

            dst[i][j] = src[y][x];
          }

          xi += a0_i32;
          yi += a3_i32;
          wi += a6_i32;
        }

        a2_i32 += a1_i32;
        a5_i32 += a4_i32;
        a8_i32 += a7_i32;
      }
    }
  } else {
    double a0 = T[0][0];  double a1 = T[0][1];  double a2 = T[0][2];
    double a3 = T[1][0];  double a4 = T[1][1];  double a5 = T[1][2];
    double a6 = affine ? 0.0 : T[2][0];
    double a7 = affine ? 0.0 : T[2][1];
    double a8 = affine ? 1.0 : T[2][2];

    for (unsigned int i = 0; i < dst.getHeight(); i++) {
      for (unsigned int j = 0; j < dst.getWidth(); j++) {
        double x = a0 * (centerCorner ? j + 0.5 : j) + a1 * (centerCorner ? i + 0.5 : i) + a2;
        double y = a3 * (centerCorner ? j + 0.5 : j) + a4 * (centerCorner ? i + 0.5 : i) + a5;
        double w = a6 * (centerCorner ? j + 0.5 : j) + a7 * (centerCorner ? i + 0.5 : i) + a8;

        if (vpMath::nul(w, std::numeric_limits<double>::epsilon())) {
          w = 1.0;
        }

        int x_ = centerCorner ? coordCast(x / w) : vpMath::round(x / w);
        int y_ = centerCorner ? coordCast(y / w) : vpMath::round(y / w);

        if (x_ >= 0 && x_ < static_cast<int>(src.getWidth()) &&
            y_ >= 0 && y_ < static_cast<int>(src.getHeight())) {
          dst[i][j] = src[y_][x_];
        }
      }
    }
  }
}

template <class Type>
void vpImageTools::warpLinear(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine,
                              bool centerCorner, bool fixedPoint)
{
  if (fixedPoint && !centerCorner) {
    const int nbits = 16;
    const int64_t precision = 1 << nbits;
    const float precision_1 = 1 / static_cast<float>(precision);
    const int64_t precision2 = 1ULL << (2 * nbits);
    const float precision_2 = 1 / static_cast<float>(precision2);

    int64_t a0_i64 = static_cast<int64_t>(T[0][0] * precision);
    int64_t a1_i64 = static_cast<int64_t>(T[0][1] * precision);
    int64_t a2_i64 = static_cast<int64_t>(T[0][2] * precision);
    int64_t a3_i64 = static_cast<int64_t>(T[1][0] * precision);
    int64_t a4_i64 = static_cast<int64_t>(T[1][1] * precision);
    int64_t a5_i64 = static_cast<int64_t>(T[1][2] * precision);
    int64_t a6_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[2][0] * precision) : 0;
    int64_t a7_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[2][1] * precision) : 0;
    int64_t a8_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[2][2] * precision) : 1;

    int64_t height_i64 = static_cast<int64_t>(src.getHeight() * precision);
    int64_t width_i64 = static_cast<int64_t>(src.getWidth() * precision);

    if (affine) {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int64_t xi_ = a2_i64;
        int64_t yi_ = a5_i64;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (yi_ >= 0 && yi_ < height_i64 && xi_ >= 0 && xi_ < width_i64) {
            const int64_t xi_lower = xi_ & (~0xFFFF);
            const int64_t yi_lower = yi_ & (~0xFFFF);

            const int64_t t = yi_ - yi_lower;
            const int64_t t_1 = precision - t;
            const int64_t s = xi_ - xi_lower;
            const int64_t s_1 = precision - s;

            const int x_ = static_cast<int>(xi_ >> nbits);
            const int y_ = static_cast<int>(yi_ >> nbits);

            if (y_ < static_cast<int>(src.getHeight())-1 && x_ < static_cast<int>(src.getWidth())-1) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_+1];
              const Type val10 = src[y_+1][x_];
              const Type val11 = src[y_+1][x_+1];
              const int64_t interp_i64 = static_cast<int64_t>(s_1*t_1*val00 + s*t_1*val01 + s_1*t*val10 + s*t*val11);
              const float interp = (interp_i64 >> (nbits*2)) + (interp_i64 & 0xFFFFFFFF) * precision_2;
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else if (y_ < static_cast<int>(src.getHeight())-1) {
              const Type val00 = src[y_][x_];
              const Type val10 = src[y_+1][x_];
              const int64_t interp_i64 = static_cast<int64_t>(t_1*val00 + t*val10);
              const float interp = (interp_i64 >> nbits) + (interp_i64 & 0xFFFF) * precision_1;
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else if (x_ < static_cast<int>(src.getWidth())-1) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_+1];
              const int64_t interp_i64 = static_cast<int64_t>(s_1*val00 + s*val01);
              const float interp = (interp_i64 >> nbits) + (interp_i64 & 0xFFFF) * precision_1;
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi_ += a0_i64;
          yi_ += a3_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
      }
    } else {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int64_t xi = a2_i64;
        int64_t yi = a5_i64;
        int64_t wi = a8_i64;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (wi != 0 && yi >= 0 && yi <= (static_cast<int>(src.getHeight()) - 1)*wi &&
              xi >= 0 && xi <= (static_cast<int>(src.getWidth()) - 1)*wi) {
            const float wi_ = (wi >> nbits) + (wi & 0xFFFF) * precision_1;
            const float xi_ = ((xi >> nbits) + (xi & 0xFFFF) * precision_1) / wi_;
            const float yi_ = ((yi >> nbits) + (yi & 0xFFFF) * precision_1) / wi_;

            const int x_ = static_cast<int>(xi_);
            const int y_ = static_cast<int>(yi_);

            const float t = yi_ - y_;
            const float s = xi_ - x_;

            if (y_ < static_cast<int>(src.getHeight()) - 1 && x_ < static_cast<int>(src.getWidth()) - 1) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_ + 1];
              const Type val10 = src[y_ + 1][x_];
              const Type val11 = src[y_ + 1][x_ + 1];
              const float col0 = lerp(val00, val01, s);
              const float col1 = lerp(val10, val11, s);
              const float interp = lerp(col0, col1, t);
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else if (y_ < static_cast<int>(src.getHeight()) - 1) {
              const Type val00 = src[y_][x_];
              const Type val10 = src[y_ + 1][x_];
              const float interp = lerp(val00, val10, t);
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else if (x_ < static_cast<int>(src.getWidth()) - 1) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_ + 1];
              const float interp = lerp(val00, val01, s);
              dst[i][j] = vpMath::saturate<Type>(interp);
            } else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi += a0_i64;
          yi += a3_i64;
          wi += a6_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
        a8_i64 += a7_i64;
      }
    }
  } else {
    double a0 = T[0][0];  double a1 = T[0][1];  double a2 = T[0][2];
    double a3 = T[1][0];  double a4 = T[1][1];  double a5 = T[1][2];
    double a6 = affine ? 0.0 : T[2][0];
    double a7 = affine ? 0.0 : T[2][1];
    double a8 = affine ? 1.0 : T[2][2];

    for (unsigned int i = 0; i < dst.getHeight(); i++) {
      for (unsigned int j = 0; j < dst.getWidth(); j++) {
        double x = a0 * (centerCorner ? j + 0.5 : j) + a1 * (centerCorner ? i + 0.5 : i) + a2;
        double y = a3 * (centerCorner ? j + 0.5 : j) + a4 * (centerCorner ? i + 0.5 : i) + a5;
        double w = a6 * (centerCorner ? j + 0.5 : j) + a7 * (centerCorner ? i + 0.5 : i) + a8;
        if (vpMath::nul(w, std::numeric_limits<double>::epsilon())) {
          w = 1;
        }

        x = x / w - (centerCorner ? 0.5 : 0);
        y = y / w - (centerCorner ? 0.5 : 0);

        int x_lower = static_cast<int>(x);
        int y_lower = static_cast<int>(y);

        if (y_lower >= static_cast<int>(src.getHeight()) || x_lower >= static_cast<int>(src.getWidth()) ||
            y < 0 || x < 0) {
          continue;
        }

        double s = x - x_lower;
        double t = y - y_lower;

        if (y_lower < static_cast<int>(src.getHeight())-1 && x_lower < static_cast<int>(src.getWidth())-1) {
          const Type val00 = src[y_lower][x_lower];
          const Type val01 = src[y_lower][x_lower + 1];
          const Type val10 = src[y_lower + 1][x_lower];
          const Type val11 = src[y_lower + 1][x_lower + 1];
          const double col0 = lerp(val00, val01, s);
          const double col1 = lerp(val10, val11, s);
          const double interp = lerp(col0, col1, t);
          dst[i][j] = vpMath::saturate<Type>(interp);
        } else if (y_lower < static_cast<int>(src.getHeight())-1) {
          const Type val00 = src[y_lower][x_lower];
          const Type val10 = src[y_lower + 1][x_lower];
          const double interp = lerp(val00, val10, t);
          dst[i][j] = vpMath::saturate<Type>(interp);
        } else if (x_lower < static_cast<int>(src.getWidth())-1) {
          const Type val00 = src[y_lower][x_lower];
          const Type val01 = src[y_lower][x_lower + 1];
          const double interp = lerp(val00, val01, s);
          dst[i][j] = vpMath::saturate<Type>(interp);
        } else {
          dst[i][j] = src[y_lower][x_lower];
        }
      }
    }
  }
}

template <> inline
void vpImageTools::warpLinear(const vpImage<vpRGBa> &src, const vpMatrix &T, vpImage<vpRGBa> &dst, bool affine,
                              bool centerCorner, bool fixedPoint)
{
  if (fixedPoint && !centerCorner) {
    const int nbits = 16;
    const int64_t precision = 1 << nbits;
    const float precision_1 = 1 / static_cast<float>(precision);
    const int64_t precision2 = 1ULL << (2 * nbits);
    const float precision_2 = 1 / static_cast<float>(precision2);

    int64_t a0_i64 = static_cast<int64_t>(T[0][0] * precision);
    int64_t a1_i64 = static_cast<int64_t>(T[0][1] * precision);
    int64_t a2_i64 = static_cast<int64_t>(T[0][2] * precision);
    int64_t a3_i64 = static_cast<int64_t>(T[1][0] * precision);
    int64_t a4_i64 = static_cast<int64_t>(T[1][1] * precision);
    int64_t a5_i64 = static_cast<int64_t>(T[1][2] * precision);
    int64_t a6_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[2][0] * precision) : 0;
    int64_t a7_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[2][1] * precision) : 0;
    int64_t a8_i64 = precision;

    int64_t height_i64 = static_cast<int64_t>(src.getHeight() * precision);
    int64_t width_i64 = static_cast<int64_t>(src.getWidth() * precision);

    if (affine) {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int64_t xi = a2_i64;
        int64_t yi = a5_i64;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (yi >= 0 && yi < height_i64 && xi >= 0 && xi < width_i64) {
            const int64_t xi_lower = xi & (~0xFFFF);
            const int64_t yi_lower = yi & (~0xFFFF);

            const int64_t t = yi - yi_lower;
            const int64_t t_1 = precision - t;
            const int64_t s = xi - xi_lower;
            const int64_t s_1 = precision - s;

            const int x_ = static_cast<int>(xi >> nbits);
            const int y_ = static_cast<int>(yi >> nbits);

            if (y_ < static_cast<int>(src.getHeight())-1 && x_ < static_cast<int>(src.getWidth())-1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val01 = src[y_][x_+1];
              const vpRGBa val10 = src[y_+1][x_];
              const vpRGBa val11 = src[y_+1][x_+1];
              const int64_t interpR_i64 = static_cast<int64_t>(s_1*t_1*val00.R + s * t_1*val01.R + s_1 * t*val10.R + s * t*val11.R);
              const float interpR = (interpR_i64 >> (nbits*2)) + (interpR_i64 & 0xFFFFFFFF) * precision_2;

              const int64_t interpG_i64 = static_cast<int64_t>(s_1*t_1*val00.G + s * t_1*val01.G + s_1 * t*val10.G + s * t*val11.G);
              const float interpG = (interpG_i64 >> (nbits * 2)) + (interpG_i64 & 0xFFFFFFFF) * precision_2;

              const int64_t interpB_i64 = static_cast<int64_t>(s_1*t_1*val00.B + s * t_1*val01.B + s_1 * t*val10.B + s * t*val11.B);
              const float interpB = (interpB_i64 >> (nbits * 2)) + (interpB_i64 & 0xFFFFFFFF) * precision_2;

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else if (y_ < static_cast<int>(src.getHeight())-1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val10 = src[y_+1][x_];
              const int64_t interpR_i64 = static_cast<int64_t>(t_1*val00.R + t*val10.R);
              const float interpR = (interpR_i64 >> nbits) + (interpR_i64 & 0xFFFF) * precision_1;

              const int64_t interpG_i64 = static_cast<int64_t>(t_1*val00.G + t * val10.G);
              const float interpG = (interpG_i64 >> nbits) + (interpG_i64 & 0xFFFF) * precision_1;

              const int64_t interpB_i64 = static_cast<int64_t>(t_1*val00.B + t * val10.B);
              const float interpB = (interpB_i64 >> nbits) + (interpB_i64 & 0xFFFF) * precision_1;

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else if (x_ < static_cast<int>(src.getWidth())-1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val01 = src[y_][x_+1];
              const int64_t interpR_i64 = static_cast<int64_t>(s_1*val00.R + s*val01.R);
              const float interpR = (interpR_i64 >> nbits) + (interpR_i64 & 0xFFFF) * precision_1;

              const int64_t interpG_i64 = static_cast<int64_t>(s_1*val00.G + s * val01.G);
              const float interpG = (interpG_i64 >> nbits) + (interpG_i64 & 0xFFFF) * precision_1;

              const int64_t interpB_i64 = static_cast<int64_t>(s_1*val00.B + s * val01.B);
              const float interpB = (interpB_i64 >> nbits) + (interpB_i64 & 0xFFFF) * precision_1;

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi += a0_i64;
          yi += a3_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
      }
    } else {
      for (unsigned int i = 0; i < dst.getHeight(); i++) {
        int64_t xi = a2_i64;
        int64_t yi = a5_i64;
        int64_t wi = a8_i64;

        for (unsigned int j = 0; j < dst.getWidth(); j++) {
          if (yi >= 0 && yi <= (static_cast<int>(src.getHeight()) - 1)*wi &&
              xi >= 0 && xi <= (static_cast<int>(src.getWidth()) - 1)*wi) {
            const float wi_ = (wi >> nbits) + (wi & 0xFFFF) * precision_1;
            const float xi_ = ((xi >> nbits) + (xi & 0xFFFF) * precision_1) / wi_;
            const float yi_ = ((yi >> nbits) + (yi & 0xFFFF) * precision_1) / wi_;

            const int x_ = static_cast<int>(xi_);
            const int y_ = static_cast<int>(yi_);

            const float t = yi_ - y_;
            const float s = xi_ - x_;

            if (y_ < static_cast<int>(src.getHeight()) - 1 && x_ < static_cast<int>(src.getWidth()) - 1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val01 = src[y_][x_ + 1];
              const vpRGBa val10 = src[y_ + 1][x_];
              const vpRGBa val11 = src[y_ + 1][x_ + 1];
              const float colR0 = lerp(val00.R, val01.R, s);
              const float colR1 = lerp(val10.R, val11.R, s);
              const float interpR = lerp(colR0, colR1, t);

              const float colG0 = lerp(val00.G, val01.G, s);
              const float colG1 = lerp(val10.G, val11.G, s);
              const float interpG = lerp(colG0, colG1, t);

              const float colB0 = lerp(val00.B, val01.B, s);
              const float colB1 = lerp(val10.B, val11.B, s);
              const float interpB = lerp(colB0, colB1, t);

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else if (y_ < static_cast<int>(src.getHeight()) - 1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val10 = src[y_ + 1][x_];
              const float interpR = lerp(val00.R, val10.R, t);
              const float interpG = lerp(val00.G, val10.G, t);
              const float interpB = lerp(val00.B, val10.B, t);

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else if (x_ < static_cast<int>(src.getWidth()) - 1) {
              const vpRGBa val00 = src[y_][x_];
              const vpRGBa val01 = src[y_][x_ + 1];
              const float interpR = lerp(val00.R, val01.R, s);
              const float interpG = lerp(val00.G, val01.G, s);
              const float interpB = lerp(val00.B, val01.B, s);

              dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                                 vpMath::saturate<unsigned char>(interpG),
                                 vpMath::saturate<unsigned char>(interpB),
                                 255);
            } else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi += a0_i64;
          yi += a3_i64;
          wi += a6_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
        a8_i64 += a7_i64;
      }
    }
  } else {
    double a0 = T[0][0];  double a1 = T[0][1];  double a2 = T[0][2];
    double a3 = T[1][0];  double a4 = T[1][1];  double a5 = T[1][2];
    double a6 = affine ? 0.0 : T[2][0];
    double a7 = affine ? 0.0 : T[2][1];
    double a8 = affine ? 1.0 : T[2][2];

    for (unsigned int i = 0; i < dst.getHeight(); i++) {
      for (unsigned int j = 0; j < dst.getWidth(); j++) {
        double x = a0 * (centerCorner ? j + 0.5 : j) + a1 * (centerCorner ? i + 0.5 : i) + a2;
        double y = a3 * (centerCorner ? j + 0.5 : j) + a4 * (centerCorner ? i + 0.5 : i) + a5;
        double w = a6 * (centerCorner ? j + 0.5 : j) + a7 * (centerCorner ? i + 0.5 : i) + a8;

        x = x / w - (centerCorner ? 0.5 : 0);
        y = y / w - (centerCorner ? 0.5 : 0);

        int x_lower = static_cast<int>(x);
        int y_lower = static_cast<int>(y);

        if (y_lower >= static_cast<int>(src.getHeight()) || x_lower >= static_cast<int>(src.getWidth()) ||
            y < 0 || x < 0) {
          continue;
        }

        double s = x - x_lower;
        double t = y - y_lower;

        if (y_lower < static_cast<int>(src.getHeight())-1 && x_lower < static_cast<int>(src.getWidth())-1) {
          const vpRGBa val00 = src[y_lower][x_lower];
          const vpRGBa val01 = src[y_lower][x_lower +1];
          const vpRGBa val10 = src[y_lower +1][x_lower];
          const vpRGBa val11 = src[y_lower +1][x_lower +1];
          const double colR0 = lerp(val00.R, val01.R, s);
          const double colR1 = lerp(val10.R, val11.R, s);
          const double interpR = lerp(colR0, colR1, t);

          const double colG0 = lerp(val00.G, val01.G, s);
          const double colG1 = lerp(val10.G, val11.G, s);
          const double interpG = lerp(colG0, colG1, t);

          const double colB0 = lerp(val00.B, val01.B, s);
          const double colB1 = lerp(val10.B, val11.B, s);
          const double interpB = lerp(colB0, colB1, t);

          dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                             vpMath::saturate<unsigned char>(interpG),
                             vpMath::saturate<unsigned char>(interpB),
                             255);
        } else if (y_lower < static_cast<int>(src.getHeight())-1) {
          const vpRGBa val00 = src[y_lower][x_lower];
          const vpRGBa val10 = src[y_lower +1][x_lower];
          const double interpR = lerp(val00.R, val10.R, t);
          const double interpG = lerp(val00.G, val10.G, t);
          const double interpB = lerp(val00.B, val10.B, t);

          dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                              vpMath::saturate<unsigned char>(interpG),
                              vpMath::saturate<unsigned char>(interpB),
                             255);
        } else if (x_lower < static_cast<int>(src.getWidth())-1) {
          const vpRGBa val00 = src[y_lower][x_lower];
          const vpRGBa val01 = src[y_lower][x_lower +1];
          const double interpR = lerp(val00.R, val01.R, s);
          const double interpG = lerp(val00.G, val01.G, s);
          const double interpB = lerp(val00.B, val01.B, s);

          dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR),
                             vpMath::saturate<unsigned char>(interpG),
                             vpMath::saturate<unsigned char>(interpB),
                             255);
        } else {
          dst[i][j] = src[y_lower][x_lower];
        }
      }
    }
  }
}

#endif
