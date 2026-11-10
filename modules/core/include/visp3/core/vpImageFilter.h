/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 * Various image tools, convolution, ...
 */

/*!
 * \file vpImageFilter.h
 * \brief  Various image filter, convolution, etc...
 */

#ifndef VP_IMAGE_FILTER_H
#define VP_IMAGE_FILTER_H

#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColorGetter.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#if (VISP_HAVE_OPENCV_VERSION < 0x050000)
#include <opencv2/imgproc/imgproc_c.h>
#endif
#endif

#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdocumentation"
#endif

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \namespace vpImageFilter
 *
 * \ingroup group_core_image
 *
 * \brief  Various image filter, convolution, etc...
 *
 * <b>Tutorials</b><br>
  <span style="margin-left:2em"> If you are interested in applying various filters on an image, you may have a look at:</span><br>

  - \ref tutorial-image-filtering
*/
namespace vpImageFilter
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace detail
{
/**
 * \brief Apply a filter at a given image location
 *
 * \tparam FilterType Image and filter types: double or float
 * \param I The input image
 * \param row The row coordinate where the filter should be applied
 * \param col The column coordinate where the filter should be applied
 * \param M the filter
 */
template <typename FilterType>
FilterType filter(const vpImage<FilterType> &I, const vpArray2D<FilterType> &M, unsigned int row, unsigned int col);

/*!
* Apply a 5x5 Gaussian filter to an image pixel.
*
* \param fr : Image to filter
* \param r : coordinates (row) of the pixel
* \param c : coordinates (column) of the pixel
*/
template <class T> double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c);

/**
 * \brief Resize the image \b I to the desired size and, if \b p_mask is different from nullptr, initialize
 * \b I with 0s.
 *
 * \tparam ImageType Any numerical type (int, float, ...)
 * \param p_mask If different from nullptr, a boolean mask that tells which pixels must be computed.
 * \param height The desired height.
 * \param width The desired width.
 * \param I The image that must be resized and potentially initialized.
 */
template<typename ImageType>
void resizeAndInitializeIfNeeded(const vpImage<bool> *p_mask, const unsigned int height, const unsigned int width, vpImage<ImageType> &I);

/**
 * \brief Indicates if the boolean mask is true at the desired coordinates.
 *
 * \param[in] p_mask Pointer towards the boolean mask if any or nullptr.
 * \param[in] r The row index in the boolean mask.
 * \param[in] c The column index in the boolean mask.
 * \return true If the boolean mask is true at the desired coordinates or if \b p_mask is equal to \b nullptr.
 * \return false False otherwise.
 */
bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c);

/**
 * \brief Indicates if the boolean mask is true at the desired coordinates.
 *
 * \param[in] p_mask Pointer towards the boolean mask if any or nullptr.
 * \param[in] id The index in the boolean mask bitmap.
 * \return true If the boolean mask is true at the desired coordinates or if \b p_mask is equal to \b nullptr.
 * \return false False otherwise.
 */
bool checkBooleanMask(const vpImage<bool> *p_mask, const int &id);

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
  // Helper to apply the scale to the raw values of the filters
template <typename FilterType>
void scaleFilter(vpArray2D<FilterType> &filter, const float &scale);
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  /**
   * \brief Return true if the distance must be computed because at least one point in its neighborhood
   * needs to compute a gradient.
   *
   * \param[in] p_mask The boolean mask that indicates for which pixels we must compute the gradient.
   * \param[in] iter The iterator that indicates which pixel of the image is currently considered.
   * \param[in] c The column counter.
   * \param[in] h The height of the image.
   * \param[in] w The width of the image.
   * \param[in] isGradientX If true, it means that we are investigating an horizontal gradient. If false, it is a
   * vertical gradient.
   * \return true
   * \return false
   */
bool checkBooleanPatch(const vpImage<bool> *p_mask, const int &iter, const int &c,
                       const int &h, const int &w,
                       const bool &isGradientX);


template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageX(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff
);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMonothread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const vpImage<bool> *p_mask = nullptr
);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageY(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff
);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMonothread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const vpImage<bool> *p_mask = nullptr
);

#ifdef VISP_HAVE_OPENMP

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageX(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff,
  const int &istart, const int &iam
);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMultithread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const int &maxNbThread, const vpImage<bool> *p_mask = nullptr);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageY(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff,
  const int &istart
);

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMultithread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const int &maxNbThread, const vpImage<bool> *p_mask = nullptr);
#endif
#endif

unsigned char filterGaussXPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j);
unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j);

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
template<typename ImageType, typename FilterType>
FilterType filterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

void filterXR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);
void filterXG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);
void filterXB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);

double filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);

double filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);

template <typename ImageType, typename FilterType>
FilterType filterXLeftBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                           const vpArray2D<FilterType> &filter);

template <typename ImageType, typename FilterType>
FilterType filterXRightBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const vpArray2D<FilterType> &filter);
#else
  /**
   * \brief Apply separately a filter to all the channels of a vpHSV.
   *
   * \tparam A specialization of a vpHSV color.
   * \tparam FilterType An arithmetic type.
   * \param[in] in The vpHSV that is filtered.
   * \param[in] out The result of the filtering, stored in a vector of doubles to limit rounding errors.
   * \param[in] coeff The filter coefficient to apply.
   */
template<class Color, typename FilterType>
typename std::enable_if<!std::is_same<Color, vpRGBa>::value, void>::type
filterChannel(const Color &in, vpColVector &out, const FilterType &coeff);

/**
 * \brief Apply separately a filter to all the channels of a vpRGBa.
 *
 * \tparam FilterType An arithmetic type.
 * \param[in] in The RGBa that is filtered.
 * \param[in] out The result of the filtering, stored in a vector of doubles to limit rounding errors.
 * \param[in] coeff The filter coefficient to apply.
 */
template<class Color, typename FilterType>
typename std::enable_if<std::is_same<Color, vpRGBa>::value, void>::type
filterChannel(const Color &in, vpColVector &out, const FilterType &coeff);

/**
 * \brief Apply separately a filter to all the channels of two colors.
 *
 * \tparam Color A color class, such as vpHSV or vpRGBa.
 * \tparam FilterType An arithmetic type.
 * \param[in] in1 The first RGBa that is filtered.
 * \param[in] in2 The second RGBa that is filtered with the same coefficient.
 * \param[in] out The result of the filtering, stored in a vector of doubles to limit rounding errors.
 * \param[in] coeff The filter coefficient to apply.
 */
template<class Color, typename FilterType>
void
filterChannel(const Color &in1, const Color &in2, vpColVector &out, const FilterType &coeff);

/**
 * \brief Filter along the horizontal direction "in the middle" of the image (i.e not on a border).
 *
 * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
 * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the horizontal direction "in the middle" of the image (i.e not on a border).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

  /**
   * \brief Filter along the horizontal direction "on the left border" of the image (the width of the border depends on
   * the filter length).
   *
   * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
   * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] result The pixel resulting from the filtering operation.
   * \param[in] r The row index.
   * \param[in] c The column index.
   * \param[in] filter The coefficients of the filter.
   * \return The method is enabled only for arithmetic input type.
   */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the horizontal direction "on the left border" of the image (the width of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the horizontal direction "on the right border" of the image (the width of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
 * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the horizontal direction "on the right border" of the image (the width of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);
#endif

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98

template<typename ImageType, typename FilterType>
FilterType filterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

void filterYR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);
void filterYG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);
void filterYB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter);

double filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);

double filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);
double filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const vpArray2D<double> &filter);

template<typename ImageType, typename FilterType>
FilterType filterYTopBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                          const vpArray2D<FilterType> &filter);

template<typename ImageType, typename FilterType>
FilterType filterYBottomBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const vpArray2D<FilterType> &filter);
#else

/**
 * \brief Filter along the vertical direction "in the middle" of the image (i.e not on a border).
 *
 * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
 * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the vertical direction "in the middle" of the image (i.e not on a border).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

  /**
   * \brief Filter along the vertical direction "on the top border" of the image (the height of the border depends on
   * the filter length).
   *
   * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
   * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] result The pixel resulting from the filtering operation.
   * \param[in] r The row index.
   * \param[in] c The column index.
   * \param[in] filter The coefficients of the filter.
   * \return The method is enabled only for arithmetic input type.
   */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the vertical direction "on the top border" of the image (the height of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the vertical direction "on the bottom border" of the image (the height of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be an arithmetic type.
 * \tparam OutputType The type of pixels in the resulting image. In this case, it must be an arithmetic type.
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                            const vpArray2D<FilterType> &filter);

/**
 * \brief Filter along the vertical direction "on the bottom border" of the image (the height of the border depends on
 * the filter length).
 *
 * \tparam ImageType The type of pixels. In this case, it must be a color type (vpRGBa, vpHSV).
 * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
 * \tparam FilterType An arithmetic type.
 * \param[in] I The image that must be filtered.
 * \param[in] result The pixel resulting from the filtering operation.
 * \param[in] r The row index.
 * \param[in] c The column index.
 * \param[in] filter The coefficients of the filter.
 * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);
#endif

/*!
   * Apply a 1x3 derivative filter to an image pixel.
   *
   * \param I : Image to filter
   * \param r : coordinates (row) of the pixel
   * \param c : coordinates (column) of the pixel
   */
template <class ImageType> double derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c);

/*!
 * Apply a 3x1 derivative filter to an image pixel.
 *
 * \param I : Image to filter
 * \param r : coordinates (row) of the pixel
 * \param c : coordinates (column) of the pixel
 */
template <class ImageType> double derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c);

/*!
 * Apply a 1 x size Derivative Filter in X to an image pixel.
 *
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param I : Image to filter
 * \param r : Coordinates(row) of the pixel
 * \param c : Coordinates(column) of the pixel
 * \param filter : Coefficients of the filter to be initialized using
 * getGaussianDerivativeKernel().
 *
 * \sa getGaussianDerivativeKernel()
 */
template <class ImageType, typename FilterType>
FilterType derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);

/*!
 * Apply a size x 1 Derivative Filter in Y to an image pixel.
 *
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param I : Image to filter.
 * \param r : Coordinates (row) of the pixel.
 * \param c : Coordinates (column) of the pixel.
 * \param filter : Coefficients of the filter to be initialized using
 * getGaussianDerivativeKernel().
 *
 * \sa getGaussianDerivativeKernel()
 */
template <class ImageType, typename FilterType>
FilterType derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const vpArray2D<FilterType> &filter);
}
#endif

  //! Canny filter backends for the edge detection operations
typedef enum vpCannyBackendType
{
  CANNY_OPENCV_BACKEND = 0,     //!< Use OpenCV
  CANNY_VISP_BACKEND = 1,     //!< Use ViSP
  CANNY_COUNT_BACKEND = 2 //! Number of supported backends
} vpCannyBackendType;

VISP_EXPORT std::string vpCannyBackendTypeList(const std::string &pref = "<", const std::string &sep = " , ",
                                          const std::string &suf = ">");

VISP_EXPORT std::string vpCannyBackendTypeToString(const vpCannyBackendType &type);

VISP_EXPORT vpCannyBackendType vpCannyBackendTypeFromString(const std::string &name);

//! Canny filter and gradient operators to apply on the image before the edge detection stage
typedef enum vpCannyFilteringAndGradientType
{
  CANNY_GBLUR_SOBEL_FILTERING = 0, //!< Apply Gaussian blur + Sobel operator on the input image
  CANNY_GBLUR_SCHARR_FILTERING = 1, //!< Apply Gaussian blur + Scharr operator on the input image
  CANNY_COUNT_FILTERING = 2 //! Number of supported backends
} vpCannyFilteringAndGradientType;

VISP_EXPORT std::string vpGetCannyFiltAndGradTypes(const std::string &pref = "<", const std::string &sep = " , ",
                                                       const std::string &suf = ">");

VISP_EXPORT std::string vpCannyFiltAndGradTypeToStr(const vpCannyFilteringAndGradientType &type);

VISP_EXPORT vpCannyFilteringAndGradientType vpCannyFiltAndGradTypeFromStr(const std::string &name);

VISP_EXPORT void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                  const float &thresholdCanny, const unsigned int &apertureSobel);

VISP_EXPORT void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                  const float &lowerThresholdCanny, const float &higherThresholdCanny,
                  const unsigned int &apertureSobel);

VISP_EXPORT void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                  const float &lowerThresholdCanny, const float &higherThresholdCanny,
                  const unsigned int &apertureSobel, const float &gaussianStdev, const float &lowerThresholdRatio,
                  const float &upperThresholdRatio, const bool &normalizeGradients,
                  const vpCannyBackendType &cannyBackend, const vpCannyFilteringAndGradientType &cannyFilteringSteps,
                  const vpImage<bool> *p_mask = nullptr);

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
VISP_EXPORT float computeCannyThreshold(const cv::Mat &cv_I, const cv::Mat *p_cv_dIx, const cv::Mat *p_cv_dIy,
                                   float &lowerThresh, const unsigned int &gaussianKernelSize = 5,
                                   const float &gaussianStdev = 2.f, const unsigned int &apertureGradient = 3,
                                   const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                                   const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING);

VISP_EXPORT void computePartialDerivatives(const cv::Mat &cv_I,
                                      cv::Mat &cv_dIx, cv::Mat &cv_dIy,
                                      const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                      const unsigned int &gaussianKernelSize = 5, const float &gaussianStdev = 2.f,
                                      const unsigned int &apertureGradient = 3,
                                      const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING);
#endif

  /**
   * \brief Compute the partial derivatives (i.e. horizontal and vertical gradients) of the input image.
   *
   * \tparam ImageType Either unsigned char, float or double
   * \tparam FilterType Either float or double.
   * \param[in] I The input image we want the partial derivatives.
   * \param[out] dIx The horizontal partial derivative, i.e. horizontal gradient.
   * \param[out] dIy The vertical partial derivative, i.e. vertical gradient.
   * \param[in] computeDx Indicate if we must compute the horizontal gradient.
   * \param[in] computeDy Indicate if we must compute  the vertical gradient.
   * \param[in] normalize Indicate if we must normalize the gradient filters.
   * \param[in] gaussianKernelSize The size of the kernel of the Gaussian filter used to blur the image.
   * If it is non-positive, it is computed from kernel size (`gaussianKernelSize` parameter) as
   * \f$\sigma = 0.3*((gaussianKernelSize-1)*0.5 - 1) + 0.8\f$.
   * \param[in] gaussianStdev The standard deviation of the Gaussian filter used to blur the image.
   * \param[in] apertureGradient The size of the kernel of the gradient filter.
   * \param[in] filteringType The type of filters to apply to compute the gradients.
   * \param[in] backend The type of backend to use to compute the gradients.
   * \param[in] p_mask If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
   */
template <typename ImageType, typename FilterType>
void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                             const unsigned int &apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND,
                                             const vpImage<bool> *p_mask = nullptr);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename FilterType>
void computePartialDerivatives(const vpImage<vpRGBa> &I,
                                             vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                             const unsigned int &apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;

template <typename ImageType>
void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<unsigned char> &dIx, vpImage<unsigned char> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int &gaussianKernelSize = 5, const unsigned char &gaussianStdev = 2.f,
                                             const unsigned int &apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;

template <typename ImageType>
void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<vpRGBa> &dIx, vpImage<vpRGBa> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int gaussianKernelSize = 5, const vpRGBa gaussianStdev = vpRGBa(),
                                             const unsigned int apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;
#else
template <typename FilterType>
void computePartialDerivatives(const vpImage<vpRGBa> &I,
                                             vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                             const unsigned int &apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);

template <typename ImageType>
void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<unsigned char> &dIx, vpImage<unsigned char> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int &gaussianKernelSize = 5, const unsigned char &gaussianStdev = 2.f,
                                             const unsigned int &apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);

template <typename ImageType>
void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<vpRGBa> &dIx, vpImage<vpRGBa> &dIy,
                                             const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                             const unsigned int gaussianKernelSize = 5, const vpRGBa gaussianStdev = vpRGBa(),
                                             const unsigned int apertureGradient = 3,
                                             const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                             const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);
#endif
#endif

  /**
   * \brief Compute the upper Canny edge filter threshold, using Gaussian blur + Sobel or + Scharr operators to compute
   * the gradient of the image.
   *
   * \tparam OutType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param[in] I : The gray-scale image, in ViSP format.
   * \param[in] p_dIx : If different from nullptr, must contain the gradient of the image with regard to the horizontal axis.
   * \param[in] p_dIy : If different from nullptr, must contain the gradient of the image with regard to the vertical axis.
   * \param[in] lowerThresh : Canny lower threshold.
   * \param[in] gaussianKernelSize : The size of the mask of the Gaussian filter to apply (an odd number).
   * \param[in] gaussianStdev : The standard deviation of the Gaussian filter to apply.
   * \param[in] apertureGradient : Size of the mask for the Sobel operator (odd number).
   * \param[in] lowerThresholdRatio : The ratio of the upper threshold the lower threshold must be equal to.
   * \param[in] upperThresholdRatio : The ratio of pixels whose absolute gradient Gabs is lower or equal to define
   * the upper threshold.
   * \param[in] filteringType : The gradient filter to apply to compute the gradient, if \b p_dIx and \b p_dIy are
   * nullptr.
   * \param[in] p_mask : If different from  \b nullptr , only the pixels for which \b p_mask is true will be considered.
   * \return The upper Canny edge filter threshold.
   */
template<typename OutType>
float computeCannyThreshold(const vpImage<unsigned char> &I, float &lowerThresh,
                                          const vpImage<OutType> *p_dIx = nullptr, const vpImage<OutType> *p_dIy = nullptr,
                                          const unsigned int &gaussianKernelSize = 5,
                                          const OutType &gaussianStdev = 2.f, const unsigned int &apertureGradient = 3,
                                          const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                                          const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                          const vpImage<bool> *p_mask = nullptr);

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  /**
   * \brief Compute the upper Canny edge filter threshold for a HSV image.
   *
   * \tparam ArithmeticType : The type of encoding of the channels of the HSV image.
   * \tparam useFullScale : When using unsigned char, true means that Hue is encoded on the range [0; 255], otherwise it
   * uses the limited range as defined in the vpHSV class.
   * \tparam OutType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param[in] I : The HSV image.
   * \param[in] p_dIx : If different from nullptr, must contain the gradient of the image with regard to the horizontal axis.
   * \param[in] p_dIy : If different from nullptr, must contain the gradient of the image with regard to the vertical axis.
   * \param[in] lowerThresh : Canny lower threshold.
   * \param[in] gaussianKernelSize : The size of the mask of the Gaussian filter to apply (an odd number).
   * \param[in] gaussianStdev : The standard deviation of the Gaussian filter to apply.
   * \param[in] lowerThresholdRatio : The ratio of the upper threshold the lower threshold must be equal to.
   * \param[in] upperThresholdRatio : The ratio of pixels whose absolute gradient Gabs is lower or equal to define
   * the upper threshold.
   * \param[in] filteringType : Canny filter type.
   * \param[in] p_mask : If different from  \b nullptr , only the pixels for which \b p_mask is true will be considered.
   * \return The upper Canny edge filter threshold.
   */
template<typename ArithmeticType, bool useFullScale, typename OutType>
float computeCannyThreshold(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, float &lowerThresh,
                                          const vpImage<OutType> *p_dIx = nullptr, const vpImage<OutType> *p_dIy = nullptr,
                                          const unsigned int &gaussianKernelSize = 5,
                                          const OutType &gaussianStdev = 2.f,
                                          const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                                          const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                          const vpImage<bool> *p_mask = nullptr);
#endif

/*!
  Apply a filter to an image.
  \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
  \param I : Image to filter
  \param If : Filtered image.
  \param M : Filter kernel.
  \param convolve : If true, perform a convolution otherwise a correlation.
  \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).

  \note By default it performs a correlation:
  \f[
    \textbf{I\_filtered} \left( u,v \right) =
    \sum_{y=0}^{\textbf{kernel\_h}}
    \sum_{x=0}^{\textbf{kernel\_w}}
    \textbf{M} \left( x,y \right ) \times
    \textbf{I} \left(
    u-\frac{\textbf{kernel\_w}}{2}+x,v-\frac{\textbf{kernel\_h}}{2}+y \right)
  \f]
  The convolution is almost the same operation:
  \f[
    \textbf{I\_filtered} \left( u,v \right) =
    \sum_{y=0}^{\textbf{kernel\_h}}
    \sum_{x=0}^{\textbf{kernel\_w}}
   \textbf{M} \left( x,y \right ) \times
    \textbf{I} \left(
    u+\frac{\textbf{kernel\_w}}{2}-x,v+\frac{\textbf{kernel\_h}}{2}-y \right)
  \f]
  Only pixels in the input image fully covered by the kernel are considered.
*/
template <typename ImageType, typename FilterType>
void filter(const vpImage<ImageType> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false,
                   const vpImage<bool> *p_mask = nullptr);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename FilterType>
void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false) = delete;
#else
template <typename FilterType>
void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false);
#endif
#endif

  /*!
   * Apply a filter to an image:
   * \f[
   *   \textbf{I}_u = \textbf{M} \ast \textbf{I} \textbf{ and } \textbf{I}_v =
   * \textbf{M}^t \ast \textbf{I} \f]
   * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param I : Image to filter
   * \param Iu : Filtered image along the horizontal axis (u = columns).
   * \param Iv : Filtered image along the vertical axis (v = rows).
   * \param M : Filter kernel.
   * \param convolve : If true, perform a convolution otherwise a correlation.
   * \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
   */
template <typename ImageType, typename FilterType>
void filter(const vpImage<ImageType> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M,
                   bool convolve = false, const vpImage<bool> *p_mask = nullptr);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template<typename FilterType>
void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M, bool convolve) = delete;

template<typename ImageType>
void filter(const vpImage<ImageType> &I, vpImage<ImageType> &Iu, vpImage<ImageType> &Iv, const vpArray2D<vpRGBa> &M, bool convolve) = delete;
#else
template<typename FilterType>
void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M, bool convolve);

template<typename ImageType>
void filter(const vpImage<ImageType> &I, vpImage<ImageType> &Iu, vpImage<ImageType> &Iv, const vpArray2D<vpRGBa> &M, bool convolve);
#endif
#endif

VISP_EXPORT void separableFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH, const vpColVector &kernelV);

/*!
 * Apply a symmetric separable filter, first along the horizontal direction and then along the vertical one.
 * \tparam ImageType : The type of pixels of the input image.
 * \tparam OutputType : The type of pixels of the filtered image.
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param I : The original image.
 * \param GI : The filtered image.
 * \param filter : The half size separable filter, as a 1 x (size+1)/2 array. The first value refers to the central
 * coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
 * \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
 */
template <typename ImageType, typename OutputType, typename FilterType>
void separableFilter(const vpImage<ImageType> &I, vpImage<OutputType> &GI, const vpArray2D<FilterType> &filter, const vpImage<bool> *p_mask = nullptr);

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
template <typename ImageType, typename FilterType>
void filterX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const vpArray2D<FilterType> &filter,
                    const vpImage<bool> *p_mask = nullptr);

void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter, const vpImage<bool> *p_mask = nullptr);
#else
  /**
   * \brief Filter along the horizontal direction.
   *
   * \tparam ImageType The type of pixels. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] dIx The image filtered along the horizontal direction.
   * \param[in] filter The half size filter coefficients, as a 1 x (size+1)/2 array. The first value refers to the central
   * coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
   * \param[in] p_mask A boolean mask that permits to select the pixels that must be filtered if different from nullptr,
   * unused otherwise.
   */
template<typename ImageType, typename OutputType, typename FilterType>
void filterX(const vpImage<ImageType> &I, vpImage<OutputType> &dIx, const vpArray2D<FilterType> &filter,
                    const vpImage<bool> *p_mask = nullptr);
#endif


// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
VISP_EXPORT void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter, const vpImage<bool> *p_mask = nullptr);

template<typename ImageType, typename FilterType>
void filterY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const vpArray2D<FilterType> &filter,
                    const vpImage<bool> *p_mask = nullptr);

#else

  /**
   * \brief Filter along the vertical direction.
   *
   * \tparam ImageType The type of pixels. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] dIy The image filtered along the vertical direction.
   * \param[in] filter The half size filter coefficients, as a 1 x (size+1)/2 array. The first value refers to the central
   * coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
   * \param[in] p_mask A boolean mask that permits to select the pixels that must be filtered if different from nullptr,
   * unused otherwise.
   */
template<typename ImageType, typename OutputType, typename FilterType>
void filterY(const vpImage<ImageType> &I, vpImage<OutputType> &dIy, const vpArray2D<FilterType> &filter,
                    const vpImage<bool> *p_mask = nullptr);
#endif

  /*!
   * Apply a Gaussian blur to an image.
   * \tparam ImageType : Either an arithmetic type or a color image.
   * \tparam OutputType : Either the same type than the ImageType, for color images, or an arithmetic type.
   * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param I : Input image.
   * \param GI : Filtered image.
   * \param size : Filter size. This value should be odd.
   * \param sigma : Gaussian standard deviation. If it is equal to zero or
   * negative, it is computed from filter size as sigma = (size-1)/6.
   * \param normalize : Flag indicating whether to normalize the filter coefficients or not.
   * \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
   *
   * \sa getGaussianKernel() to know which kernel is used.
   */
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename ImageType, typename OutputType, typename FilterType = float>
void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size = 7, FilterType sigma = 0., bool normalize = true,
                        const vpImage<bool> *p_mask = nullptr);
#else
template <typename ImageType, typename OutputType>
void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size, float sigma, bool normalize = true,
                        const vpImage<bool> *p_mask = nullptr);

template <typename ImageType, typename OutputType, typename FilterType >
void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size = 7, FilterType sigma = 0., bool normalize = true,
                        const vpImage<bool> *p_mask = nullptr);
#endif

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
VISP_EXPORT void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size = 7, double sigma = 0., bool normalize = true,
                    const vpImage<bool> *p_mask = nullptr);
#endif

// Gaussian pyramid operation
VISP_EXPORT void getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
VISP_EXPORT void getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
VISP_EXPORT void getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);

/*!
* Return the coefficients \f$G_i\f$ of a Gaussian filter.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param[out] filter : vpArray2D that will be resize to match half size filter kernel, i.e. it will be a
* (1, (size+1)/2) array. The first value refers to the central coefficient, the
* next one to the right coefficients. Left coefficients could be deduced by
* symmetry.
* \param[in] size : Filter size. This value should be odd and positive.
* \param[in] sigma : Gaussian standard deviation \f$ \sigma \f$. If it is equal to zero or negative, it is
* computed from filter size as sigma = (size-1)/6.
* \param[in] normalize : Flag indicating whether to normalize the filter coefficients or not. In that case
* \f$\Sigma G_i = 1 \f$.
*
* The function computes the \e (size+1)/2 values of the Gaussian filter coefficients \f$ G_i \f$ as:
* \f[ G_i = \frac{1}{\sigma  \sqrt{2 \pi}} \exp{(-i^2 / (2. * \sigma^2))}\f]
*/
template<typename FilterType>
void getGaussianKernel(vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma = 0., bool normalize = true);

/*!
* Return the coefficients of a Gaussian derivative filter that may be used to
* compute spatial image derivatives after applying a Gaussian blur.
*
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter :vpArray2D that will be resize to match half size filter kernel, i.e. it will be a
* (1, (size+1)/2) array. The first value refers to the central coefficient, the
* next one to the right coefficients. Left coefficients could be deduced by
* symmetry.
* \param size : Filter size. This value should be odd.
* \param sigma : Gaussian standard deviation. If it is equal to zero or negative, it is
* computed from filter size as sigma = (size-1)/6.
* \param normalize : Flag indicating whether to normalize the filter coefficients or not.
*/
template <typename FilterType>
void getGaussianDerivativeKernel(vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma = 0., bool normalize = true);

// Gradient along X
template<typename FilterType>
void getGradX(const vpImage<unsigned char> &I, vpImage<FilterType> &dIx, const vpImage<bool> *p_mask = nullptr);

template <typename ImageType, typename FilterType>
void getGradX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const vpArray2D<FilterType> &filter, const vpImage<bool> *p_mask = nullptr);

/*!
* Compute the gradient along X after applying a gaussian filter along Y.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param I : Input image
* \param dIx : Gradient along X.
* \param gaussianKernel : Gaussian kernel which values should be computed using getGaussianKernel().
* \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
* getGaussianDerivativeKernel().
* \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
*/
template <typename ImageType, typename FilterType>
void getGradXGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const vpArray2D<FilterType> &gaussianKernel,
                            const vpArray2D<FilterType> &gaussianDerivativeKernel, const vpImage<bool> *p_mask = nullptr);

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
/**
 * \brief Gradient filter along the X-axis for HSV images.
 *
 * \tparam ArithmeticType The encoding of the HSV channels.
 * \tparam FilterType The type to use for the filter and the type of the resulting gradient image.
 * \tparam useFullScale See the documentation of the vpHSV class to have details on this template parameter.
 * \param[in] I The input image.
 * \param[in] GIx The resulting gradient image along the X-axis.
 * \param[in] nbThread The number of threads to use to compute the gradient. Setting -1 uses the maximum number of threads.
 * \param[in] p_mask Optional boolean mask to indicate which pixels should be considered and which should be ignored.
 * \param[in] type The type of the gradient filter.
 */
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterX(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, const int &nbThread, const vpImage<bool> *p_mask, const vpCannyFilteringAndGradientType &type);
#endif

  // Gradient along Y
template <typename FilterType>
void getGradY(const vpImage<unsigned char> &I, vpImage<FilterType> &dIy, const vpImage<bool> *p_mask = nullptr);

template <typename ImageType, typename FilterType>
void getGradY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const vpArray2D<FilterType> &filter, const vpImage<bool> *p_mask = nullptr);

/*!
* Compute the gradient along Y after applying a gaussian filter along X.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param I : Input image
* \param dIy : Gradient along Y.
* \param gaussianKernel : Gaussian kernel which values should be computed  using getGaussianKernel().
* \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
* getGaussianDerivativeKernel().
* \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
*/
template <typename ImageType, typename FilterType>
void getGradYGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const vpArray2D<FilterType> &gaussianKernel,
                            const vpArray2D<FilterType> &gaussianDerivativeKernel, const vpImage<bool> *p_mask = nullptr);

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  /**
   * \brief Gradient filter along the Y-axis for HSV images.
   *
   * \tparam ArithmeticType The encoding of the HSV channels.
   * \tparam FilterType The type to use for the filter and the type of the resulting gradient image.
   * \tparam useFullScale See the documentation of the vpHSV class to have details on this template parameter.
   * \param[in] I The input image.
   * \param[in] GIy The resulting gradient image along the Y-axis.
   * \param[in] nbThread The number of threads to use to compute the gradient. Setting -1 uses the maximum number of threads.
   * \param[in] p_mask Optional boolean mask to indicate which pixels should be considered and which should be ignored.
   * \param[in] type The type of the gradient filter.
   */
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterY(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIy, const int &nbThread, const vpImage<bool> *p_mask, const vpCannyFilteringAndGradientType &type);
#endif

  /*!
    Get Scharr kernel for X-direction.
    \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Array that will be resized if needed.
    \param size : Filter size, which must be odd. The filter will be an array of size rows and size columns. For the moment, only 3 x 3 filter is handled.
    \return Scaling factor to normalize the Scharr kernel.
  */
template <typename FilterType>
FilterType getScharrKernelX(vpArray2D<FilterType> &filter, unsigned int size);

/*!
  Get Scharr kernel for Y-direction.
  \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
  \param filter : Array that will be resized if needed.
  \param size : Filter size, which must be odd. The filter will be an array of size rows and size columns. For the moment, only 3 x 3 filter is handled.
  \return Scaling factor to normalize the Scharr kernel.
*/
template <typename FilterType>
FilterType getScharrKernelY(vpArray2D<FilterType> &filter, unsigned int size);

/*!
* Get Sobel kernel for X-direction.
* \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter : Array that will be resized if needed.
* \param size : Filter size, which must be odd. The filter will be an array of size rows and size columns. Max size is 41.
* \return Scaling factor to normalize the Sobel kernel.
*/
template <typename FilterType>
FilterType getSobelKernelX(vpArray2D<FilterType> &filter, unsigned int size);

/*!
* Get Sobel kernel for Y-direction.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter : Array that will be resized if needed.
* \param size : Filter size, which must be odd. The filter will be an array of size rows and size columns. Max size is 41.
* \return Scaling factor to normalize the Sobel kernel.
*/
template <typename FilterType>
FilterType getSobelKernelY(vpArray2D<FilterType> &filter, unsigned int size);

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
VISP_EXPORT float median(const cv::Mat &cv_I);
VISP_EXPORT float median(const vpImage<unsigned char> &Isrc);
VISP_EXPORT std::vector<float> median(const vpImage<vpRGBa> &Isrc);
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  /**
   * \brief Compute the horizontal and vertical gradients for HSV images.
   *
   * \tparam ArithmeticType The encoding of the HSV channels.
   * \tparam FilterType The type to use for the filter and the type of the resulting gradient image.
   * \tparam useFullScale See the documentation of the vpHSV class to have details on this template parameter.
   * \param[in] I The input image.
   * \param[in] GIx The resulting gradient image along the X-axis.
   * \param[in] GIy The resulting gradient image along the Y-axis.
   * \param[in] nbThread The number of threads to use to compute the gradient. Setting -1 use the maximum number of thread.
   * \param[in] p_mask Optional boolean mask to indicate which pixels should be considered and which should be ignored.
   * \param[in] type The type of the gradient filter.
   */
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilter(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, vpImage<FilterType> &GIy, const int &nbThread = -1, const vpImage<bool> *p_mask = nullptr, const vpCannyFilteringAndGradientType &type = CANNY_GBLUR_SCHARR_FILTERING);
#endif
}
END_VISP_NAMESPACE

#include <visp3/core/detail/vpImageFilter_impl.h>
#include <visp3/core/detail/vpImageFilter_deprecated.h>
#endif
