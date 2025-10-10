/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Image tools.
 */

/*!
  \file vpImageTools.h

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...
*/

#ifndef VP_IMAGE_TOOLS_H
#define VP_IMAGE_TOOLS_H

#ifdef VISP_HAVE_THREADS
#include <thread>
#endif

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRect.h>
#include <visp3/core/vpRectOriented.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>
#include <cmath>

#if defined(_OPENMP)
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpImageTools

  \ingroup group_core_image

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...
*/
class VISP_EXPORT vpImageTools
{
public:
  enum vpImageInterpolationType
  {
    INTERPOLATION_NEAREST, /*!< Nearest neighbor interpolation. */
    INTERPOLATION_LINEAR,  /*!< Bi-linear interpolation (optimized by SIMD lib if enabled). */
    INTERPOLATION_CUBIC,   /*!< Bi-cubic interpolation. */
    INTERPOLATION_AREA     /*!< Area interpolation (optimized by SIMD lib if enabled). */
  };

  template <class Type>
  static inline void binarise(vpImage<Type> &I, Type threshold1, Type threshold2, Type value1, Type value2, Type value3,
                              bool useLUT = true);
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

  static void extract(const vpImage<unsigned char> &src, vpImage<unsigned char> &dst, const vpRectOriented &r);
  static void extract(const vpImage<unsigned char> &src, vpImage<double> &dst, const vpRectOriented &r);

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
                       bool saturate = false);

  static void imageSubtract(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            vpImage<unsigned char> &Ires, bool saturate = false);

  /*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have value equal to true.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  inline static int inMask(const vpImage<vpRGBa> &I, const vpImage<bool> &mask, vpImage<vpRGBa> &I_mask)
  {
    return inMask(I, mask, I_mask, true, vpRGBa(0, 0, 0));
  }

  /*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have values that differ from 0.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  inline static int inMask(const vpImage<vpRGBa> &I, const vpImage<unsigned char> &mask, vpImage<vpRGBa> &I_mask)
  {
    const unsigned char inRangeVal = 255;
    return inMask(I, mask, I_mask, inRangeVal, vpRGBa(0, 0, 0));
  }

  /*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have value equal to true.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  inline static int inMask(const vpImage<unsigned char> &I, const vpImage<bool> &mask, vpImage<unsigned char> &I_mask)
  {
    return inMask(I, mask, I_mask, true, static_cast<unsigned char>(0));
  }

  /*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have values that differ from 0.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  inline static int inMask(const vpImage<unsigned char> &I, const vpImage<unsigned char> &mask, vpImage<unsigned char> &I_mask)
  {
    const unsigned char inRangeVal = 255;
    return inMask(I, mask, I_mask, inRangeVal, static_cast<unsigned char>(0));
  }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have value equal to true.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  template <typename ArithmeticType, bool useFullScale>
  inline static int inMask(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, const vpImage<bool> &mask, vpImage<vpHSV<ArithmeticType, useFullScale>> &I_mask)
  {
    vpHSV<ArithmeticType, useFullScale> black(static_cast<ArithmeticType>(0), static_cast<ArithmeticType>(0), static_cast<ArithmeticType>(0));
    return inMask(I, mask, I_mask, true, black);
  }

  /*!
  * Keep the part of an image that is in the mask.
  *
  * \param[in] I : Input image.
  * \param[in] mask : Mask where pixels to consider have values that differ from 0.
  * \param[out] I_mask : Resulting image where pixels that are in the mask are kept.
  * \return The number of pixels that are in the mask.
  *
  * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro
  */
  template <typename ArithmeticType, bool useFullScale>
  inline static int inMask(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, const vpImage<unsigned char> &mask, vpImage<vpHSV<ArithmeticType, useFullScale>> &I_mask)
  {
    const unsigned char inRangeVal = 255;
    vpHSV<ArithmeticType, useFullScale> black(static_cast<ArithmeticType>(0), static_cast<ArithmeticType>(0), static_cast<ArithmeticType>(0));
    return inMask(I, mask, I_mask, inRangeVal, black);
  }
#endif

  static int inRange(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                     const vpColVector &hsv_range, unsigned char *mask, unsigned int size);
  static int inRange(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                     const std::vector<int> &hsv_range, unsigned char *mask, unsigned int size);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /**
   * \brief Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
   *
   * \tparam ArithmeticType The arithmetic type used to encode the Hue, Saturation and Value channels.
   * \tparam useFullScale If ArithmeticType is unsigned char, true means that Hue is encoded on the full
   * range [0;255] and false means it is encoded in a limited range as defined in the vpHSV documentation.
   * \tparam RangeType The arithmetic type used to encode the ranges. Be careful that the validity of the range values is
   * not checked.
   * \param[in] Iin The input image.
   * \param[in] hsv_range 6-dim vector that contains the low/high range values for each HSV channel respectively.
   * Each element of this vector should be in the range defined by the ArithmeticType and useFullRange template parameters.
   * Note that there is also tutorial-hsv-tuner.cpp that may help to determine low/high HSV values.
   * \param[in] out The output mask encoded as booleans. True means that the pixel is in range and false that it
   * is not in range.
   * \return int The number of pixels that are in the HSV range.
   *
   * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro or \ref tutorial-hsv-range-tuner
   * \sa To see how to use it to perform color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
   */
  template <typename ArithmeticType, bool useFullScale, typename RangeType>
  static int inRange(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Iin,
                      const std::vector<RangeType> &hsv_range, vpImage<bool> &out)
  {
    return inRange(Iin, hsv_range, out, true, false);
  }

  /**
   * \brief Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
   *
   * \tparam ArithmeticType The arithmetic type used to encode the Hue, Saturation and Value channels.
   * \tparam useFullScale If ArithmeticType is unsigned char, true means that Hue is encoded on the full
   * range [0;255] and false means it is encoded in a limited range as defined in the vpHSV documentation.
   * \tparam RangeType The arithmetic type used to encode the ranges. Be careful that the validity of the range values is
   * not checked.
   * \param[in] Iin The input image.
   * \param[in] hsv_range 6-dim vector that contains the low/high range values for each HSV channel respectively.
   * Each element of this vector should be in the range defined by the ArithmeticType and useFullRange template parameters.
   * Note that there is also tutorial-hsv-tuner.cpp that may help to determine low/high HSV values.
   * \param[in] out The output mask encoded as unsigned char. 255 means that the pixel is in range and 0 that it
   * is not in range.
   * \return int The number of pixels that are in the HSV range.
   *
   * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro or \ref tutorial-hsv-range-tuner
   * \sa To see how to use it to perform color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
   */
  template <typename ArithmeticType, bool useFullScale, typename RangeType>
  static int inRange(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Iin,
                      const std::vector<RangeType> &hsv_range, vpImage<unsigned char> &out)
  {
    const unsigned char inRangeVal = 255;
    return inRange(Iin, hsv_range, out, inRangeVal, static_cast<unsigned char>(0));
  }

  /**
   * \brief Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
   *
   * \tparam ArithmeticType The arithmetic type used to encode the Hue, Saturation and Value channels.
   * \tparam useFullScale If ArithmeticType is unsigned char, true means that Hue is encoded on the full
   * range [0;255] and false means it is encoded in a limited range as defined in the vpHSV documentation.
   * \param[in] Iin The input image.
   * \param[in] hsv_range 6-dim vector that contains the low/high range values for each HSV channel respectively.
   * Each element of this vector should be in the range defined by the ArithmeticType and useFullRange template parameters.
   * Note that there is also tutorial-hsv-tuner.cpp that may help to determine low/high HSV values.
   * \warning The range values will be converted in ArithmeticType without checking the validity of the values.
   * \param[in] out The output mask encoded as booleans. True means that the pixel is in range and false that it
   * is not in range.
   * \return int The number of pixels that are in the HSV range.
   *
   * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro or \ref tutorial-hsv-range-tuner
   * \sa To see how to use it to perform color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
   */
  template <typename ArithmeticType, bool useFullScale>
  static int inRange(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Iin,
                      const vpColVector &hsv_range, vpImage<bool> &out)
  {
    const unsigned int nbItems = hsv_range.getRows();
    std::vector<ArithmeticType> range(nbItems);
    for (unsigned int r = 0; r < nbItems; ++r) {
      range[r] = static_cast<ArithmeticType>(hsv_range[r]);
    }
    return inRange(Iin, range, out, true, false);
  }

  /**
   * \brief Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
   *
   * \tparam ArithmeticType The arithmetic type used to encode the Hue, Saturation and Value channels.
   * \tparam useFullScale If ArithmeticType is unsigned char, true means that Hue is encoded on the full
   * range [0;255] and false means it is encoded in a limited range as defined in the vpHSV documentation.
   * \param[in] Iin The input image.
   * \param[in] hsv_range 6-dim vector that contains the low/high range values for each HSV channel respectively.
   * Each element of this vector should be in the range defined by the ArithmeticType and useFullRange template parameters.
   * Note that there is also tutorial-hsv-tuner.cpp that may help to determine low/high HSV values.
   * \warning The range values will be converted in ArithmeticType without checking the validity of the values.
   * \param[in] out The output mask encoded as unsigned char. 255 means that the pixel is in range and 0 that it
   * is not in range.
   * \return int The number of pixels that are in the HSV range.
   *
   * \sa To see how to use it to perform color segmentation on an image, \ref tutorial-hsv-segmentation-intro or \ref tutorial-hsv-range-tuner
   * \sa To see how to use it to perform color segmentation on a point-cloud , \ref tutorial-hsv-segmentation-pcl
   */
  template <typename ArithmeticType, bool useFullScale>
  static int inRange(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Iin,
                      const vpColVector &hsv_range, vpImage<unsigned char> &out)
  {
    const unsigned char inRangeVal = 255;
    const unsigned int nbItems = hsv_range.getRows();
    std::vector<ArithmeticType> range(nbItems);
    for (unsigned int r = 0; r < nbItems; ++r) {
      range[r] = static_cast<ArithmeticType>(hsv_range[r]);
    }
    return inRange(Iin, range, out, inRangeVal, static_cast<unsigned char>(0));
  }
#endif



  static void initUndistortMap(const vpCameraParameters &cam, unsigned int width, unsigned int height,
                               vpArray2D<int> &mapU, vpArray2D<int> &mapV, vpArray2D<float> &mapDu,
                               vpArray2D<float> &mapDv);

  static double interpolate(const vpImage<unsigned char> &I, const vpImagePoint &point,
                            const vpImageInterpolationType &method = INTERPOLATION_NEAREST);

  static void integralImage(const vpImage<unsigned char> &I, vpImage<double> &II, vpImage<double> &IIsq);

  static double normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2, bool useOptimized = true);

  static void normalize(vpImage<double> &I);

  static void remap(const vpImage<unsigned char> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                    const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<unsigned char> &Iundist);
  static void remap(const vpImage<vpRGBa> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                    const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<vpRGBa> &Iundist);

  template <class Type>
  static void resize(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int width, unsigned int height,
                     const vpImageInterpolationType &method = INTERPOLATION_NEAREST, unsigned int nThreads = 0);

  template <class Type>
  static void resize(const vpImage<Type> &I, vpImage<Type> &Ires,
                     const vpImageInterpolationType &method = INTERPOLATION_NEAREST, unsigned int nThreads = 0);

  static void templateMatching(const vpImage<unsigned char> &I, const vpImage<unsigned char> &I_tpl,
                               vpImage<double> &I_score, unsigned int step_u, unsigned int step_v,
                               bool useOptimized = true);

  template <class Type>
  static void undistort(const vpImage<Type> &I, const vpCameraParameters &cam, vpImage<Type> &newI,
                        unsigned int nThreads = 2);

  template <class Type>
  static void undistort(const vpImage<Type> &I, vpArray2D<int> mapU, vpArray2D<int> mapV, vpArray2D<float> mapDu,
                        vpArray2D<float> mapDv, vpImage<Type> &newI);

  template <class Type>
  static void warpImage(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst,
                        const vpImageInterpolationType &interpolation = INTERPOLATION_NEAREST,
                        bool fixedPointArithmetic = true, bool pixelCenter = false);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  template <class Type>
  VP_DEPRECATED static void createSubImage(const vpImage<Type> &I, unsigned int i_sub, unsigned int j_sub,
                                           unsigned int nrow_sub, unsigned int ncol_sub, vpImage<Type> &S);

  template <class Type>
  VP_DEPRECATED static void createSubImage(const vpImage<Type> &I, const vpRect &rect, vpImage<Type> &S);
  //@}
#endif

private:
  // Cubic interpolation
  static float cubicHermite(const float A, const float B, const float C, const float D, const float t);

  template <class Type> static Type getPixelClamped(const vpImage<Type> &I, float u, float v);

  static int coordCast(double x);

  // Linear interpolation
  static double lerp(double A, double B, double t);
  static float lerp(float A, float B, float t);
  static int64_t lerp2(int64_t A, int64_t B, int64_t t, int64_t t_1);

  static double normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2, const vpImage<double> &II,
                                      const vpImage<double> &IIsq, const vpImage<double> &II_tpl,
                                      const vpImage<double> &IIsq_tpl, unsigned int i0, unsigned int j0);

  template <class Type>
  static void resizeBicubic(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                            float v, float xFrac, float yFrac);

  template <class Type>
  static void resizeBilinear(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                             float v, float xFrac, float yFrac);

  template <class Type>
  static void resizeNearest(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                            float v);

#if defined(VISP_HAVE_SIMDLIB)
  static void resizeSimdlib(const vpImage<vpRGBa> &Isrc, unsigned int resizeWidth, unsigned int resizeHeight,
                            vpImage<vpRGBa> &Idst, int method);
  static void resizeSimdlib(const vpImage<unsigned char> &Isrc, unsigned int resizeWidth, unsigned int resizeHeight,
                            vpImage<unsigned char> &Idst, int method);
#endif

  template <class Type>
  static void warpNN(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine, bool centerCorner,
                     bool fixedPoint);

  template <class Type>
  static void warpLinear(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine,
                         bool centerCorner, bool fixedPoint);

  static bool checkFixedPoint(unsigned int x, unsigned int y, const vpMatrix &T, bool affine);

  static void warpLinearFixedPointNotCenter(const vpImage<vpRGBa> &src, const vpMatrix &T, vpImage<vpRGBa> &dst, bool affine);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename ArithmeticType, bool useFullScale, typename RangeType, typename OutType>
  static int inRange(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Iin,
                      const std::vector<RangeType> &hsv_range, vpImage<OutType> &mask, const OutType &valueInRange, const OutType &valueOutRange)
  {
    const std::size_t val_6 = 6;
    if (hsv_range.size() != val_6) {
      throw(vpImageException(vpImageException::notInitializedError,
                             "Error in vpImageTools::inRange(): wrong values vector size (%d)", hsv_range.size()));
    }
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int index_4 = 4;
    const unsigned int index_5 = 5;
    ArithmeticType h_low = static_cast<ArithmeticType>(hsv_range[index_0]);
    ArithmeticType h_high = static_cast<ArithmeticType>(hsv_range[index_1]);
    ArithmeticType s_low = static_cast<ArithmeticType>(hsv_range[index_2]);
    ArithmeticType s_high = static_cast<ArithmeticType>(hsv_range[index_3]);
    ArithmeticType v_low = static_cast<ArithmeticType>(hsv_range[index_4]);
    ArithmeticType v_high = static_cast<ArithmeticType>(hsv_range[index_5]);
    int size_ = Iin.getSize();
    mask.resize(Iin.getRows(), Iin.getCols());
    int cpt_in_range = 0;

#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_range)
#endif
    for (int i = 0; i < size_; ++i) {
      bool check_h_low_high_hue = (h_low <= Iin.bitmap[i].H) && (Iin.bitmap[i].H <= h_high);
      bool check_s_low_high_saturation = (s_low <= Iin.bitmap[i].S) && (Iin.bitmap[i].S <= s_high);
      bool check_v_low_high_value = (v_low <= Iin.bitmap[i].V) && (Iin.bitmap[i].V <= v_high);
      if (check_h_low_high_hue && check_s_low_high_saturation && check_v_low_high_value) {
        mask.bitmap[i] = valueInRange;
        ++cpt_in_range;
      }
      else {
        mask.bitmap[i] = valueOutRange;
      }
    }
    return cpt_in_range;
  }
#endif

  template <typename ImageType, typename MaskType>
  static int inMask(const vpImage<ImageType> &I, const vpImage<MaskType> &mask, vpImage<ImageType> &I_mask
            , const MaskType &inRangeCheck, const ImageType &outRangeValue)
  {
    if ((I.getHeight() != mask.getHeight()) || (I.getWidth() != mask.getWidth())) {
      throw(vpImageException(vpImageException::incorrectInitializationError,
                             "Error in vpImageTools::inMask(): image (%dx%d) and mask (%dx%d) size doesn't match",
                             I.getWidth(), I.getHeight(), mask.getWidth(), mask.getHeight()));
    }

    I_mask.resize(I.getHeight(), I.getWidth());
    int cpt_in_mask = 0;
    int size_ = static_cast<int>(I.getSize());
#if defined(_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_mask)
#endif
    for (int i = 0; i < size_; ++i) {
      if (mask.bitmap[i] == inRangeCheck) {
        I_mask.bitmap[i] = I.bitmap[i];
        ++cpt_in_mask;
      }
      else {
        I_mask.bitmap[i] = outRangeValue;
      }
    }
    return cpt_in_mask;
  }
#endif
};

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  Crop a region of interest (ROI) in an image.

  \deprecated This function is deprecated. You should rather use
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

  \deprecated This function is deprecated. You should rather use
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

  \param[in] I : Input image from which a sub image will be extracted.
  \param[in] roi_top : ROI vertical position of the upper/left corner in the input
  image.
  \param[in] roi_left : ROI  horizontal position of the upper/left corner
  in the input image.
  \param[in] roi_height : Cropped image height corresponding
  to the ROI height.
  \param[in] roi_width : Cropped image width corresponding to
  the ROI height.
  \param[out] crop : Cropped image.
  \param[in] v_scale : Vertical subsampling factor applied to the ROI.
  \param[in] h_scale : Horizontal subsampling factor applied to the ROI.

  \sa crop(const vpImage<Type> &, const vpRect &, vpImage<Type> &)
*/
template <class Type>
void vpImageTools::crop(const vpImage<Type> &I, double roi_top, double roi_left, unsigned int roi_height,
                        unsigned int roi_width, vpImage<Type> &crop, unsigned int v_scale, unsigned int h_scale)
{
  int i_min = std::max<int>(static_cast<int>(ceil(roi_top / v_scale)), 0);
  int j_min = std::max<int>(static_cast<int>(ceil(roi_left / h_scale)), 0);
  int i_max = std::min<int>(static_cast<int>(ceil(roi_top + roi_height) / v_scale), static_cast<int>(I.getHeight() / v_scale));
  int j_max = std::min<int>(static_cast<int>(ceil(roi_left + roi_width) / h_scale), static_cast<int>(I.getWidth() / h_scale));

  unsigned int i_min_u = static_cast<unsigned int>(i_min);
  unsigned int j_min_u = static_cast<unsigned int>(j_min);

  unsigned int r_width = static_cast<unsigned int>(j_max - j_min);
  unsigned int r_height = static_cast<unsigned int>(i_max - i_min);

  crop.resize(r_height, r_width);

  if ((v_scale == 1) && (h_scale == 1)) {
    for (unsigned int i = 0; i < r_height; ++i) {
      void *src = (void *)(I[i + i_min_u] + j_min_u);
      void *dst = static_cast<void *>(crop[i]);
      memcpy(dst, src, r_width * sizeof(Type));
    }
  }
  else if (h_scale == 1) {
    for (unsigned int i = 0; i < r_height; ++i) {
      void *src = (void *)(I[(i + i_min_u) * v_scale] + j_min_u);
      void *dst = static_cast<void *>(crop[i]);
      memcpy(dst, src, r_width * sizeof(Type));
    }
  }
  else {
    for (unsigned int i = 0; i < r_height; ++i) {
      for (unsigned int j = 0; j < r_width; ++j) {
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
  vpImageTools::crop(I, roi.getTop(), roi.getLeft(), static_cast<unsigned int>(roi.getHeight()), static_cast<unsigned int>(roi.getWidth()), crop,
                     v_scale, h_scale);
}

/*!
  Crop a region of interest (ROI) in an image. The ROI coordinates and
  dimension are defined in the original image.

  Setting \e v_scale and \e h_scale to values different from 1 allows also to
  subsample the cropped image.

  \param[in] bitmap : Pointer to the input image from which a sub image will be extracted.
  \param[in] width : Width of the input image.
  \param[in] height : Height of the input image.

  \param[in] roi : Region of interest corresponding to the cropped part of the image.

  \param[out] crop : Cropped image.
  \param[in] v_scale [in] : Vertical subsampling factor applied to the ROI.
  \param[in] h_scale [in] : Horizontal subsampling factor applied to the ROI.
*/
template <class Type>
void vpImageTools::crop(const unsigned char *bitmap, unsigned int width, unsigned int height, const vpRect &roi,
                        vpImage<Type> &crop, unsigned int v_scale, unsigned int h_scale)
{
  int i_min = std::max<int>(static_cast<int>(ceil(roi.getTop() / v_scale)), 0);
  int j_min = std::max<int>(static_cast<int>(ceil(roi.getLeft() / h_scale)), 0);
  int i_max = std::min<int>(static_cast<int>(ceil((roi.getTop() + roi.getHeight()) / v_scale)), static_cast<int>(height / v_scale));
  int j_max = std::min<int>(static_cast<int>(ceil((roi.getLeft() + roi.getWidth()) / h_scale)), static_cast<int>(width / h_scale));

  unsigned int i_min_u = static_cast<unsigned int>(i_min);
  unsigned int j_min_u = static_cast<unsigned int>(j_min);

  unsigned int r_width = static_cast<unsigned int>(j_max - j_min);
  unsigned int r_height = static_cast<unsigned int>(i_max - i_min);

  crop.resize(r_height, r_width);

  if (v_scale == 1 && h_scale == 1) {
    for (unsigned int i = 0; i < r_height; ++i) {
      void *src = (void *)(bitmap + ((((i + i_min_u) * width) + j_min_u) * sizeof(Type)));
      void *dst = (void *)(crop[i]);
      memcpy(dst, src, r_width * sizeof(Type));
    }
  }
  else if (h_scale == 1) {
    for (unsigned int i = 0; i < r_height; ++i) {
      void *src = (void *)(bitmap + (((((i + i_min_u) * width) * v_scale) + j_min_u) * sizeof(Type)));
      void *dst = (void *)(crop[i]);
      memcpy(dst, src, r_width * sizeof(Type));
    }
  }
  else {
    for (unsigned int i = 0; i < r_height; ++i) {
      unsigned int i_src = (((i + i_min_u) * width) * v_scale) + (j_min_u * h_scale);
      for (unsigned int j = 0; j < r_width; ++j) {
        void *src = (void *)(bitmap + ((i_src + (j * h_scale)) * sizeof(Type)));
        void *dst = (void *)(&crop[i][j]);
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
                                   Type value3, bool useLUT)
{
  if (useLUT) {
    std::cerr << "LUT not available for this type ! Will use the iteration method." << std::endl;
  }

  Type v;
  Type *p = I.bitmap;
  Type *pend = I.bitmap + (I.getWidth() * I.getHeight());
  for (; p < pend; ++p) {
    v = *p;
    if (v < threshold1) {
      *p = value1;
    }
    else if (v > threshold2) {
      *p = value3;
    }
    else {
      *p = value2;
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
template <>
inline void vpImageTools::binarise(vpImage<unsigned char> &I, unsigned char threshold1, unsigned char threshold2,
                                   unsigned char value1, unsigned char value2, unsigned char value3, bool useLUT)
{
  if (useLUT) {
    // Construct the LUT
    const unsigned int sizeLut = 256;
    unsigned char lut[sizeLut];
    for (unsigned int i = 0; i < sizeLut; ++i) {
      lut[i] = i < threshold1 ? value1 : (i > threshold2 ? value3 : value2);
    }

    I.performLut(lut);
  }
  else {
    unsigned char *p = I.bitmap;
    unsigned char *pend = I.bitmap + (I.getWidth() * I.getHeight());
    for (; p < pend; ++p) {
      unsigned char v = *p;
      if (v < threshold1) {
        *p = value1;
      }
      else if (v > threshold2) {
        *p = value3;
      }
      else {
        *p = value2;
      }
    }
  }
}

#ifdef VISP_HAVE_THREADS

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
  vpUndistortInternalType() : src(nullptr), dst(nullptr), width(0), height(0), cam(), nthreads(0), threadid(0) { }

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

  static void vpUndistort_threaded(vpUndistortInternalType<Type> &undistortSharedData);
};

template <class Type> void vpUndistortInternalType<Type>::vpUndistort_threaded(vpUndistortInternalType<Type> &undistortSharedData)
{
  int offset = static_cast<int>(undistortSharedData.threadid);
  int width = static_cast<int>(undistortSharedData.width);
  int height = static_cast<int>(undistortSharedData.height);
  int nthreads = static_cast<int>(undistortSharedData.nthreads);

  double u0 = undistortSharedData.cam.get_u0();
  double v0 = undistortSharedData.cam.get_v0();
  double px = undistortSharedData.cam.get_px();
  double py = undistortSharedData.cam.get_py();
  double kud = undistortSharedData.cam.get_kud();

  double invpx = 1.0 / px;
  double invpy = 1.0 / py;

  double kud_px2 = kud * invpx * invpx;
  double kud_py2 = kud * invpy * invpy;

  Type *dst = undistortSharedData.dst + (height / nthreads * offset) * width;
  Type *src = undistortSharedData.src;

  for (double v = height / nthreads * offset; v < height / nthreads * (offset + 1); ++v) {
    double deltav = v - v0;
    // double fr1 = 1.0 + kd * (vpMath::sqr(deltav * invpy));
    double fr1 = 1.0 + kud_py2 * deltav * deltav;

    for (double u = 0; u < width; ++u) {
      // computation of u,v : corresponding pixel coordinates in I.
      double deltau = u - u0;
      // double fr2 = fr1 + kd * (vpMath::sqr(deltau * invpx));
      double fr2 = fr1 + kud_px2 * deltau * deltau;

      double u_double = deltau * fr2 + u0;
      double v_double = deltav * fr2 + v0;

      // computation of the bilinear interpolation

      // declarations
      int u_round = static_cast<int>(u_double);
      int v_round = static_cast<int>(v_double);
      if (u_round < 0) {
        u_round = -1;
      }
      if (v_round < 0) {
        v_round = -1;
      }
      double du_double = (u_double)-static_cast<double>(u_round);
      double dv_double = (v_double)-static_cast<double>(v_round);
      Type v01;
      Type v23;
      if ((0 <= u_round) && (0 <= v_round) && (u_round < (width-1)) && (v_round < (height-1))) {
        // process interpolation
        const Type *_mp = &src[v_round * width + u_round];
        v01 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        _mp += width;
        v23 = (Type)(_mp[0] + ((_mp[1] - _mp[0]) * du_double));
        *dst = (Type)(v01 + ((v23 - v01) * dv_double));
      }
      else {
        *dst = 0;
      }
      dst++;
    }
  }
}
#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // VISP_HAVE_THREADS

/*!
  Undistort an image

  \param I : Input image to undistort.

  \param cam : Parameters of the camera causing distortion.

  \param undistI : Undistorted output image. The size of this image
  will be the same than the input image \e I. If the distortion
  parameter \f$k_{ud}\f$ is null, meaning that `cam.get_kud() == 0`, \e undistI is
  just a copy of \e I.

  \param nThreads : Number of threads to use if pthreads library is available.

  \warning This function works only with Types authorizing "+,-,
  multiplication by a scalar" operators.

  Since this function is time consuming, if you want to undistort multiple images, you should rather
  call initUndistortMap() once and then remap() to undistort the images.
  This will be less time consuming.

  \sa initUndistortMap(), remap()
*/
template <class Type>
void vpImageTools::undistort(const vpImage<Type> &I, const vpCameraParameters &cam, vpImage<Type> &undistI,
                             unsigned int nThreads)
{
#if defined(VISP_HAVE_THREADS)
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
  std::vector<std::thread *> threadpool;

  vpUndistortInternalType<Type> *undistortSharedData = new vpUndistortInternalType<Type>[nthreads];

  for (unsigned int i = 0; i < nthreads; ++i) {
    // Each thread works on a different set of data.
    undistortSharedData[i].src = I.bitmap;
    undistortSharedData[i].dst = undistI.bitmap;
    undistortSharedData[i].width = I.getWidth();
    undistortSharedData[i].height = I.getHeight();
    undistortSharedData[i].cam = cam;
    undistortSharedData[i].nthreads = nthreads;
    undistortSharedData[i].threadid = i;
    std::thread *undistort_thread = new std::thread(&vpUndistortInternalType<Type>::vpUndistort_threaded, std::ref(undistortSharedData[i]));
    threadpool.push_back(undistort_thread);
  }
  /* Wait on the other threads */

  for (unsigned int i = 0; i < nthreads; ++i) {
    threadpool[i]->join();
  }

  for (unsigned int i = 0; i < nthreads; ++i) {
    delete threadpool[i];
  }

  delete[] undistortSharedData;
#else  // VISP_HAVE_THREADS
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

  /*
  // if (kud == 0) {
  */
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
  for (double v = 0; v < height; ++v) {
    double deltav = v - v0;
    /*
    // double fr1 = 1.0 + kd * (vpMath::sqr(deltav * invpy));
    */
    double fr1 = 1.0 + (kud_py2 * deltav * deltav);

    for (double u = 0; u < width; ++u) {
      /*
      // computation of u,v : corresponding pixel coordinates in I.
      */
      double deltau = u - u0;
      /*
      // double fr2 = fr1 + kd * (vpMath::sqr(deltau * invpx));
      */
      double fr2 = fr1 + (kud_px2 * deltau * deltau);

      double u_double = (deltau * fr2) + u0;
      double v_double = (deltav * fr2) + v0;

      // printf("[%g][%g] %g %g : ", u, v, u_double, v_double );

      // computation of the bilinear interpolation

      // declarations
      int u_round = static_cast<int>(u_double);
      int v_round = static_cast<int>(v_double);
      if (u_round < 0.f) {
        u_round = -1;
      }
      if (v_round < 0.f) {
        v_round = -1;
      }
      double du_double = u_double-static_cast<double>(u_round);
      double dv_double = v_double-static_cast<double>(v_round);
      Type v01;
      Type v23;
      if ((0 <= u_round) && (0 <= v_round) && (u_round < ((static_cast<int>(width)) - 1)) && (v_round < ((static_cast<int>(height)) - 1))) {
        // process interpolation
        const Type *v_mp = &I[static_cast<unsigned int>(v_round)][static_cast<unsigned int>(u_round)];
        v01 = static_cast<Type>(v_mp[0] + ((v_mp[1] - v_mp[0]) * du_double));
        v_mp += width;
        v23 = static_cast<Type>(v_mp[0] + ((v_mp[1] - v_mp[0]) * du_double));
        *dst = static_cast<Type>(v01 + ((v23 - v01) * dv_double));
        /*
        // printf("R %d G %d B %d\n", dst->R, dst->G, dst->B);
        */
      }
      else {
        *dst = 0;
      }
      ++dst;
    }
  }
#endif // VISP_HAVE_THREADS
}

/*!
  Undistort an image.

  \param I       : Input image to undistort.
  \param mapU    : Map that contains at each destination coordinate the u-coordinate in the source image.
  \param mapV    : Map that contains at each destination coordinate the v-coordinate in the source image.
  \param mapDu   : Map that contains at each destination coordinate the \f$ \Delta u \f$ for the interpolation.
  \param mapDv   : Map that contains at each destination coordinate the \f$ \Delta v \f$ for the interpolation.
  \param newI    : Undistorted output image. The size of this image will be the same as the input image \e I.

  \note To undistort a fisheye image, you have to first call initUndistortMap() function to calculate maps and then
  call undistort() with input maps.

 */
template <class Type>
void vpImageTools::undistort(const vpImage<Type> &I, vpArray2D<int> mapU, vpArray2D<int> mapV, vpArray2D<float> mapDu,
                             vpArray2D<float> mapDv, vpImage<Type> &newI)
{
  remap(I, mapU, mapV, mapDu, mapDv, newI);
}

/*!
  Flip vertically the input image and give the result in the output image.

  \param I : Input image to flip.
  \param newI : Output image which is the flipped input image.
*/
template <class Type> void vpImageTools::flip(const vpImage<Type> &I, vpImage<Type> &newI)
{
  unsigned int height = I.getHeight(), width = I.getWidth();
  newI.resize(height, width);

  for (unsigned int i = 0; i < height; ++i) {
    memcpy(newI.bitmap + (i * width), I.bitmap + ((height - 1 - i) * width), width * sizeof(Type));
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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<vpRGBa> I;
  #ifdef _WIN32
    std::string filename("C:/Temp/visp-images/Klimt/Klimt.ppm");
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
  unsigned int height = I.getHeight(), width = I.getWidth();
  vpImage<Type> Ibuf;
  Ibuf.resize(1, width);

  const unsigned int halfHeight = height / 2;
  for (unsigned int i = 0; i < halfHeight; ++i) {
    memcpy(Ibuf.bitmap, I.bitmap + (i * width), width * sizeof(Type));

    memcpy(I.bitmap + (i * width), I.bitmap + ((height - 1 - i) * width), width * sizeof(Type));
    memcpy(I.bitmap + ((height - 1 - i) * width), Ibuf.bitmap, width * sizeof(Type));
  }
}

template <class Type> Type vpImageTools::getPixelClamped(const vpImage<Type> &I, float u, float v)
{
  int x = vpMath::round(static_cast<double>(u));
  int y = vpMath::round(static_cast<double>(v));
  x = std::max<int>(0, std::min<int>(x, static_cast<int>(I.getWidth()) - 1));
  y = std::max<int>(0, std::min<int>(y, static_cast<int>(I.getHeight()) - 1));

  return I[y][x];
}

// Reference:
// http://blog.demofox.org/2015/08/15/resizing-images-with-bicubic-interpolation/
template <class Type>
void vpImageTools::resizeBicubic(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                                 float v, float xFrac, float yFrac)
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
inline void vpImageTools::resizeBicubic(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, unsigned int i, unsigned int j,
                                        float u, float v, float xFrac, float yFrac)
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

  const int nbChannels = 3;
  for (int c = 0; c < nbChannels; ++c) {
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
void vpImageTools::resizeBilinear(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                                  float v, float xFrac, float yFrac)
{
  int u0 = static_cast<int>(u);
  int v0 = static_cast<int>(v);

  int u1 = std::min<int>(static_cast<int>(I.getWidth()) - 1, u0 + 1);
  int v1 = v0;

  int u2 = u0;
  int v2 = std::min<int>(static_cast<int>(I.getHeight()) - 1, v0 + 1);

  int u3 = u1;
  int v3 = v2;

  float col0 = lerp(I[v0][u0], I[v1][u1], xFrac);
  float col1 = lerp(I[v2][u2], I[v3][u3], xFrac);
  float value = lerp(col0, col1, yFrac);

  Ires[i][j] = vpMath::saturate<Type>(value);
}

template <>
inline void vpImageTools::resizeBilinear(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, unsigned int i,
                                         unsigned int j, float u, float v, float xFrac, float yFrac)
{
  int u0 = static_cast<int>(u);
  int v0 = static_cast<int>(v);

  int u1 = std::min<int>(static_cast<int>(I.getWidth()) - 1, u0 + 1);
  int v1 = v0;

  int u2 = u0;
  int v2 = std::min<int>(static_cast<int>(I.getHeight()) - 1, v0 + 1);

  int u3 = u1;
  int v3 = v2;

  const int nbChannels = 3;
  for (int c = 0; c < nbChannels; ++c) {
    float col0 = lerp(static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v0][u0])[c]),
                      static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v1][u1])[c]), xFrac);
    float col1 = lerp(static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v2][u2])[c]),
                      static_cast<float>(reinterpret_cast<const unsigned char *>(&I[v3][u3])[c]), xFrac);
    float value = lerp(col0, col1, yFrac);

    reinterpret_cast<unsigned char *>(&Ires[i][j])[c] = vpMath::saturate<unsigned char>(value);
  }
}

template <class Type>
void vpImageTools::resizeNearest(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int i, unsigned int j, float u,
                                 float v)
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
  \param nThreads : Number of threads to use if OpenMP is available
  (zero will let OpenMP uses the optimal number of threads).

  \warning The input \e I and output \e Ires images must be different objects.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture for:
    - unsigned char and vpRGBa image types
    - and only with INTERPOLATION_AREA and INTERPOLATION_LINEAR methods
*/
template <class Type>
void vpImageTools::resize(const vpImage<Type> &I, vpImage<Type> &Ires, unsigned int width, unsigned int height,
                          const vpImageInterpolationType &method, unsigned int nThreads)
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
  \param nThreads : Number of threads to use if OpenMP is available
  (zero will let OpenMP uses the optimal number of threads). Unused if OpenMP is not enabled.

  \warning The input \e I and output \e Ires images must be different objects.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture for:
    - unsigned char and vpRGBa image types
    - and only with INTERPOLATION_AREA and INTERPOLATION_LINEAR methods
*/
template <class Type>
void vpImageTools::resize(const vpImage<Type> &I, vpImage<Type> &Ires, const vpImageInterpolationType &method,
                          unsigned int nThreads)
{
#if !defined(_OPENMP)
  (void)nThreads;
#endif
  const unsigned int minWidth = 2, minHeight = 2;
  if ((I.getWidth() < minWidth) || (I.getHeight() < minHeight) || (Ires.getWidth() < minWidth) || (Ires.getHeight() < minHeight)) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  if (method == INTERPOLATION_AREA) {
    std::cerr << "INTERPOLATION_AREA is not implemented for this type." << std::endl;
    return;
  }

  const float scaleY = I.getHeight() / static_cast<float>(Ires.getHeight());
  const float scaleX = I.getWidth() / static_cast<float>(Ires.getWidth());
  const float half = 0.5f;
  const int ires_height = static_cast<int>(Ires.getHeight());
#if defined(_OPENMP)
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < ires_height; ++i) {
    const float v = ((i + half) * scaleY) - half;
    const float v0 = std::floor(v);
    const float yFrac = v - v0;

    unsigned int ires_width = static_cast<unsigned int>(Ires.getWidth());
    for (unsigned int j = 0; j < ires_width; ++j) {
      const float u = ((j + half) * scaleX) - half;
      const float u0 = std::floor(u);
      const float xFrac = u - u0;

      if (method == INTERPOLATION_NEAREST) {
        resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
      }
      else if (method == INTERPOLATION_LINEAR) {
        resizeBilinear(I, Ires, static_cast<unsigned int>(i), j, u0, v0, xFrac, yFrac);
      }
      else if (method == INTERPOLATION_CUBIC) {
        resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
      }
    }
  }
}

#if defined(VISP_HAVE_SIMDLIB)
template <>
inline void vpImageTools::resize(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires,
                                 const vpImageInterpolationType &method,
                                 unsigned int
#if defined(_OPENMP)
                                 nThreads
#endif
)
{
  const unsigned int minWidth = 2, minHeight = 2;

  if ((I.getWidth() < minWidth) || (I.getHeight() < minHeight) || (Ires.getWidth() < minWidth) || (Ires.getHeight() < minHeight)) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  if (method == INTERPOLATION_AREA) {
    resizeSimdlib(I, Ires.getWidth(), Ires.getHeight(), Ires, INTERPOLATION_AREA);
  }
  else if (method == INTERPOLATION_LINEAR) {
    resizeSimdlib(I, Ires.getWidth(), Ires.getHeight(), Ires, INTERPOLATION_LINEAR);
  }
  else {
    const float scaleY = static_cast<float>(I.getHeight()) / static_cast<float>(Ires.getHeight());
    const float scaleX = static_cast<float>(I.getWidth()) / static_cast<float>(Ires.getWidth());
    const float half = 0.5f;
    const int ires_height = static_cast<int>(Ires.getHeight());
#if defined(_OPENMP)
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < ires_height; ++i) {
      float v = ((static_cast<float>(i) + half) * scaleY) - half;
      float yFrac = static_cast<float>(v - static_cast<float>(static_cast<int>(v)));

      unsigned int ires_width = static_cast<unsigned int>(Ires.getWidth());
      for (unsigned int j = 0; j < ires_width; ++j) {
        float u = ((static_cast<float>(j) + half) * scaleX) - half;
        float xFrac = static_cast<float>(u - static_cast<float>(static_cast<int>(u)));

        if (method == INTERPOLATION_NEAREST) {
          resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
        }
        else if (method == INTERPOLATION_CUBIC) {
          resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
        }
      }
    }
  }
}

template <>
inline void vpImageTools::resize(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires,
                                 const vpImageInterpolationType &method,
                                 unsigned int
#if defined(_OPENMP)
                                 nThreads
#endif
)
{
  const unsigned int minWidth = 2, minHeight = 2;

  if ((I.getWidth() < minWidth) || (I.getHeight() < minHeight) || (Ires.getWidth() < minWidth) || (Ires.getHeight() < minHeight)) {
    std::cerr << "Input or output image is too small!" << std::endl;
    return;
  }

  if (method == INTERPOLATION_AREA) {
    resizeSimdlib(I, Ires.getWidth(), Ires.getHeight(), Ires, INTERPOLATION_AREA);
  }
  else if (method == INTERPOLATION_LINEAR) {
    resizeSimdlib(I, Ires.getWidth(), Ires.getHeight(), Ires, INTERPOLATION_LINEAR);
  }
  else {
    const float scaleY = static_cast<float>(I.getHeight()) / static_cast<float>(Ires.getHeight());
    const float scaleX = static_cast<float>(I.getWidth()) / static_cast<float>(Ires.getWidth());
    const float half = 0.5f;
    const int ires_height = static_cast<int>(Ires.getHeight());
#if defined(_OPENMP)
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < ires_height; ++i) {
      float v = ((static_cast<float>(i) + half) * scaleY) - half;
      float yFrac = static_cast<float>(v - static_cast<float>(static_cast<int>(v)));

      unsigned int ires_width = static_cast<unsigned int>(Ires.getWidth());
      for (unsigned int j = 0; j < ires_width; ++j) {
        float u = ((static_cast<float>(j) + half) * scaleX) - half;
        float xFrac = static_cast<float>(u - static_cast<float>(static_cast<int>(u)));

        if (method == INTERPOLATION_NEAREST) {
          resizeNearest(I, Ires, static_cast<unsigned int>(i), j, u, v);
        }
        else if (method == INTERPOLATION_CUBIC) {
          resizeBicubic(I, Ires, static_cast<unsigned int>(i), j, u, v, xFrac, yFrac);
        }
      }
    }
  }
}
#endif

#ifdef ENABLE_IMAGE_TOOLS_WARP
#include <visp3/core/vpImageTools_warp.h>
#endif

END_VISP_NAMESPACE
#endif
