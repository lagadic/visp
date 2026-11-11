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
 * \file vpImageFilter_deprecated.h
 * \brief  Deprecated methods of vpImageFilter
 */

#ifndef VP_IMAGE_FILTER_DEPRECATED_H
#define VP_IMAGE_FILTER_DEPRECATED_H

#include <visp3/core/vpConfig.h>
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

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
 * <span style="margin-left:2em"> If you are interested in applying various filters on an image, you may have a look at:</span><br>
 *
 * - \ref tutorial-image-filtering
 */
  namespace vpImageFilter
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace detail
{
  /*!
   * Apply a 1x3 derivative filter to an image pixel.
   *
   * \param I : Image to filter
   * \param r : coordinates (row) of the pixel
   * \param c : coordinates (column) of the pixel
   * \param filter : Coefficients of the filter to be initialized using
   * getGaussianDerivativeKernel(). Only the right half coefficients, so the array is of
   * size (size - 1) / 2 .
   * \param size : Size of the filter.
   */
template <class ImageType, typename FilterType>
FilterType derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  unsigned int i;
  FilterType result;

  result = 0;

  for (i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r][c + i] - I[r][c - i]);
  }
  return result;
}

/*!
 * Apply a 3x1 derivative filter to an image pixel.
 *
 * \param I : Image to filter
 * \param r : coordinates (row) of the pixel
 * \param c : coordinates (column) of the pixel
 * \param filter : Coefficients of the filter to be initialized using
 * getGaussianDerivativeKernel(). Only the right half coefficients, so the array is of
 * size (size - 1) / 2 .
 * \param size : Size of the filter.
 */
template <class ImageType, typename FilterType>
FilterType derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  unsigned int i;
  FilterType result;

  result = 0;

  for (i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r + i][c] - I[r - i][c]);
  }
  return result;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) // Check if cxx11 or higher
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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
  }
  result = static_cast<FilterType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
  }
  result = OutputType(res);
}

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
   * \param[in] size The size of the filter.
   * \return The method is enabled only for arithmetic input type.
   */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][i - c]);
    }
  }
  result = static_cast<OutputType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
    }
    else {
      filterChannel(I[r][c + i], I[r][i - c], res, filter[i]);
    }
  }
  result = OutputType(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  const unsigned int twice = 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r][((twice * width) - c) - i - 1] + I[r][c - i]);
    }
  }
  result = static_cast<FilterType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  const unsigned int twice = 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
    }
    else {
      filterChannel(I[r][((twice * width) - c) - i - 1], I[r][c - i], res, filter[i]);
    }
  }
  result = OutputType(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));

  for (unsigned int i = 1; i <= stop; ++i) {
    res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
  }
  result = static_cast<OutputType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
  }
  result = OutputType(res);
}

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
   * \param[in] size The size of the filter.
   * \return The method is enabled only for arithmetic input type.
   */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[i - r][c]);
    }
  }
  result = static_cast<OutputType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
    }
    else {
      filterChannel(I[r + i][c], I[i - r][c], res, filter[i]);
    }
  }
  result = OutputType(res);
}

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
 * \param[in] size The size of the filter.
 * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  const unsigned int twiceHeight = 2 * height;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));
  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[(twiceHeight - r) - i - 1][c] + I[r - i][c]);
    }
  }
  result = static_cast<OutputType>(res);
}

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
 * \param[in] size The size of the filter.
 * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
 */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int height = I.getHeight();
  const unsigned int twiceHeight = 2 * height;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
    }
    else {
      filterChannel(I[(twiceHeight - r) - i - 1][c], I[r - i][c], res, filter[i]);
    }
  }
  result = OutputType(res);
}
#else
template<typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

VP_DEPRECATED void filterXR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
VP_DEPRECATED void filterXG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
VP_DEPRECATED void filterXB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

VP_DEPRECATED double filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

VP_DEPRECATED double filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

template <typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterXLeftBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                           const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][i - c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterXRightBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twice = 2;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][((twice * width) - c) - i - 1] + I[r][c - i]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

VP_DEPRECATED void filterYR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
VP_DEPRECATED void filterYG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
VP_DEPRECATED void filterYB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

VP_DEPRECATED double filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

VP_DEPRECATED double filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
VP_DEPRECATED double filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

template<typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterYTopBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                          const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[i - r][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
VP_DEPRECATED inline FilterType filterYBottomBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twiceHeight = 2 * height;
  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[(twiceHeight - r) - i - 1][c] + I[r - i][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}
#endif
}
#endif

/*!
 * @name Deprecated functions
 */
//@{

/*!
 * \deprecated You should rather use separableFilter() that does the same.
 *
 * Apply a filter to an image using two separable kernels.
 *
 * \param[in] I : Image to filter
 * \param[out] If : Filtered image.
 * \param[in] kernelH : Separable kernel (performed first).
 * \param[in] kernelV : Separable kernel (performed last).
 * \note Only pixels in the input image fully covered by the kernel are considered.
 */
VP_DEPRECATED inline void sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
                                    const vpColVector &kernelV)
{
  separableFilter(I, If, kernelH, kernelV);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather apply a filter to the whole image using for example gaussianBlur().
 *
 * Apply a 5x5 Gaussian filter to an image pixel.
 *
 * \param[in] fr : Image to filter
 * \param[in] r : Coordinates (row) of the pixel
 * \param[in] c : Coordinates (column) of the pixel
 * \return The filtered pixel value.
 */
template <class T> VP_DEPRECATED inline double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c)
{
  return detail::gaussianFilter(fr, r, c);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather compute the gradient of the whole image using getGradX().
 *
 * Apply a 1x3 derivative filter to an image pixel.
 *
 * \param[in] I : Image to filter
 * \param[in] r : Coordinates (row) of the pixel
 * \param[in] c : Coordinates (column) of the pixel
 * \return The filtered pixel value.
 */
template <class ImageType> VP_DEPRECATED inline double derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
{
  return detail::derivativeFilterX(I, r, c);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather compute the gradient of the whole image using getGradY().
 *
 * Apply a 3x1 derivative filter to an image pixel.
 *
 * \param[in] I : Image to filter
 * \param[in] r : Coordinates (row) of the pixel
 * \param[in] c : Coordinates (column) of the pixel
 * \return The filtered pixel value.
 */
template <class ImageType> VP_DEPRECATED inline double derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
{
  return detail::derivativeFilterY(I, r, c);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather compute the gradient of the whole image using getGradX() with a vpArray2D
 * filter.
 *
 * Apply a 1 x size derivative filter to an image pixel.
 *
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param[in] I : Image to filter
 * \param[in] r : Coordinates (row) of the pixel
 * \param[in] c : Coordinates (column) of the pixel
 * \param[in] filter : Coefficients of the filter to be initialized using getGaussianDerivativeKernel().
 * \param[in] size : Size of the filter.
 * \return The filtered pixel value.
 */
template <class ImageType, typename FilterType>
VP_DEPRECATED inline FilterType derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                                  const FilterType *filter, unsigned int size)
{
  return detail::derivativeFilterX<ImageType, FilterType>(I, r, c, filter, size);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather compute the gradient of the whole image using getGradY() with a vpArray2D
 * filter.
 *
 * Apply a size x 1 derivative filter to an image pixel.
 *
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param[in] I : Image to filter
 * \param[in] r : Coordinates (row) of the pixel
 * \param[in] c : Coordinates (column) of the pixel
 * \param[in] filter : Coefficients of the filter to be initialized using getGaussianDerivativeKernel().
 * \param[in] size : Size of the filter.
 * \return The filtered pixel value.
 */
template <class ImageType, typename FilterType>
VP_DEPRECATED inline FilterType derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                                  const FilterType *filter, unsigned int size)
{
  return detail::derivativeFilterY<ImageType, FilterType>(I, r, c, filter, size);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather use getGaussXPyramidal() on the whole image.
 *
 * Apply a 1x5 Gaussian filter to an image pixel, as used to build a Gaussian pyramid.
 *
 * \param[in] I : Image to filter
 * \param[in] i : Coordinates (row) of the pixel
 * \param[in] j : Coordinates (column) of the pixel
 * \return The filtered pixel value.
 */
VP_DEPRECATED inline unsigned char filterGaussXPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
{
  return detail::filterGaussXPyramidal(I, i, j);
}

/*!
 * \deprecated This low-level function is kept for compatibility with previous ViSP releases but is no more part of
 * the public API. You should rather use getGaussYPyramidal() on the whole image.
 *
 * Apply a 5x1 Gaussian filter to an image pixel, as used to build a Gaussian pyramid.
 *
 * \param[in] I : Image to filter
 * \param[in] i : Coordinates (row) of the pixel
 * \param[in] j : Coordinates (column) of the pixel
 * \return The filtered pixel value.
 */
VP_DEPRECATED inline unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
{
  return detail::filterGaussYPyramidal(I, i, j);
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) // Check if cxx11 or higher
/**
   * \brief Filter along the horizontal direction.
   *
   * \tparam ImageType The type of pixels. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] dIx The image filtered along the horizontal direction.
   * \param[in] filter The coefficients of the filter.
   * \param[in] size The size of the filter.
   * \param[in] p_mask A boolean mask that permits to select the pixels that must be filtered if different from nullptr,
   * unused otherwise.
   */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED void filterX(const vpImage<ImageType> &I, vpImage<OutputType> &dIx, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight();
  const unsigned int width = I.getWidth();
  const int stop1J = static_cast<int>((size - 1) / 2);
  const int stop2J = static_cast<int>(width - ((size - 1) / 2));
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  int istart = 0;
  int istop = static_cast<int>(height);
#ifdef VISP_HAVE_OPENMP
  int iam, nt, ipoints, npoints(static_cast<int>(height));
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart, istop)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    ipoints = npoints / nt;
    // size of partition
    istart = iam * ipoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      ipoints = npoints - istart;
    }
    istop = istart + ipoints;
#endif
    for (int i = istart; i < istop; ++i) {
      for (int j = 0; j < stop1J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterXLeftBorder(I, dIx[static_cast<unsigned int>(i)][static_cast<unsigned int>(j)],
                                           static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }

      for (int j = stop1J; j < stop2J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterX(I, dIx[i][j], static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }
      for (int j = stop2J; j < static_cast<int>(width); ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterXRightBorder(I, dIx[static_cast<unsigned int>(i)][static_cast<unsigned int>(j)],
                                            static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }
    }
#ifdef VISP_HAVE_OPENMP
  }
#endif
}

/**
   * \brief Filter along the vertical direction.
   *
   * \tparam ImageType The type of pixels. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam OutputType The type of pixels in the resulting image. It can be an arithmetic type or a color type (vpRGBa, vpHSV).
   * \tparam FilterType An arithmetic type.
   * \param[in] I The image that must be filtered.
   * \param[in] dIy The image filtered along the vertical direction.
   * \param[in] filter The coefficients of the filter.
   * \param[in] size The size of the filter.
   * \param[in] p_mask A boolean mask that permits to select the pixels that must be filtered if different from nullptr,
   * unused otherwise.
   */
template<typename ImageType, typename OutputType, typename FilterType>
VP_DEPRECATED void filterY(const vpImage<ImageType> &I, vpImage<OutputType> &dIy, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  unsigned int jstart = 0;
  unsigned int jstop = width;
#ifdef VISP_HAVE_OPENMP
  unsigned int iam, nt, jpoints, npoints(width);
#pragma omp parallel default(shared) private(iam, nt, jpoints, jstart, jstop)
  {
    iam = static_cast<unsigned int>(omp_get_thread_num());
    nt = static_cast<unsigned int>(omp_get_num_threads());
    jpoints = npoints / nt;
    // size of partition
    jstart = iam * jpoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      jpoints = npoints - jstart;
    }
    jstop = jstart + jpoints;
#endif
    for (unsigned int i = 0; i < stop1I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterYTopBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop1I; i < stop2I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterY(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop2I; i < height; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterYBottomBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
#ifdef VISP_HAVE_OPENMP
    }
#endif
  }
}
#else
template <typename ImageType, typename FilterType>
VP_DEPRECATED void filterX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight();
  const unsigned int width = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = width - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterXLeftBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterX<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop2J; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterXRightBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}

VP_DEPRECATED void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

VP_DEPRECATED void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

template<typename ImageType, typename FilterType>
VP_DEPRECATED void filterY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterYTopBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop2I; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterYBottomBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}
#endif

/*!
 * Apply a separable filter.
 * \tparam ImageType : The type of pixels of the input image.
 * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
 * \param I : The original image.
 * \param GI : The filtered image.
 * \param filter : The separable filter.
 * \param size : The size of the filter.
 * \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
 * \deprecated Use rather separableFilter(const vpImage<ImageType> &, vpImage<OutputType> &, const vpArray2D<FilterType> &, const vpImage<bool> *).
 */
template <typename ImageType, typename FilterType>
VP_DEPRECATED void filter(const vpImage<ImageType> &I, vpImage<FilterType> &GI, const FilterType *filter, unsigned int size,
                          const vpImage<bool> *p_mask = nullptr)
{
  vpImage<FilterType> GIx;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) // Check if cxx11 or higher
  filterX<ImageType, FilterType, FilterType>(I, GIx, filter, size, p_mask);
  filterY<FilterType, FilterType, FilterType>(GIx, GI, filter, size, p_mask);
#else
  filterX<ImageType, FilterType>(I, GIx, filter, size, p_mask);
  filterY<FilterType, FilterType>(GIx, GI, filter, size, p_mask);
#endif
  GIx.destroy();
}

/*!
* Return the coefficients \f$G_i\f$ of a Gaussian filter.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param[out] filter : Pointer to the half size filter kernel that should refer to a
* (size+1)/2 array. The first value refers to the central coefficient, the
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
VP_DEPRECATED void getGaussianKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
{
  vpArray2D<FilterType> filter_as_array = vpArray2D<FilterType>::view(filter, 1, (size+1)/2);
  getGaussianKernel(filter_as_array, size, sigma, normalize);
}

/*!
* Return the coefficients of a Gaussian derivative filter that may be used to
* compute spatial image derivatives after applying a Gaussian blur.
*
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter : Pointer to the filter kernel that should refer to a
* (size+1)/2 array. The first value refers to the central coefficient, the
* next one to the right coefficients. Left coefficients could be deduced by
* symmetry.
* \param size : Filter size. This value should be odd.
* \param sigma : Gaussian standard deviation. If it is equal to zero or negative, it is
* computed from filter size as sigma = (size-1)/6.
* \param normalize : Flag indicating whether to normalize the filter coefficients or not.
*/
template <typename FilterType>
VP_DEPRECATED void getGaussianDerivativeKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
{
  vpArray2D<FilterType> filter_as_array = vpArray2D<FilterType>::view(filter, 1, (size+1)/2);
  getGaussianDerivativeKernel(filter_as_array, size, sigma, normalize);
}

template <typename ImageType, typename FilterType>
void getGradX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = width - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::derivativeFilterX<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop2J; j < width; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

/*!
* Compute the gradient along X after applying a gaussian filter along Y.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param I : Input image
* \param dIx : Gradient along X.
* \param gaussianKernel : Gaussian kernel which values should be computed using vpImageFilter::getGaussianKernel().
* \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
* vpImageFilter::getGaussianDerivativeKernel().
* \param size : Size of the Gaussian and Gaussian derivative kernels.
* \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
*/
template <typename ImageType, typename FilterType>
VP_DEPRECATED void getGradXGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *gaussianKernel,
                            const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask = nullptr)
{
  vpImage<FilterType> GIy;
  vpImageFilter::filterY<ImageType, FilterType>(I, GIy, gaussianKernel, size, p_mask);
  vpImageFilter::getGradX<FilterType, FilterType>(GIy, dIx, gaussianDerivativeKernel, size, p_mask);
}

template <typename ImageType, typename FilterType>
void getGradY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::derivativeFilterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop2I; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

/*!
* Compute the gradient along Y after applying a gaussian filter along X.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param I : Input image
* \param dIy : Gradient along Y.
* \param gaussianKernel : Gaussian kernel which values should be computed  using vpImageFilter::getGaussianKernel().
* \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
* vpImageFilter::getGaussianDerivativeKernel().
* \param size : Size of the Gaussian and Gaussian derivative kernels.
* \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
*/
template <typename ImageType, typename FilterType>
VP_DEPRECATED void getGradYGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *gaussianKernel,
                            const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask = nullptr)
{
  vpImage<FilterType> GIx;
  vpImageFilter::filterX<ImageType, FilterType>(I, GIx, gaussianKernel, size, p_mask);
  vpImageFilter::getGradY<FilterType, FilterType>(GIx, dIy, gaussianDerivativeKernel, size, p_mask);
}

/*!
  Get Scharr kernel for Y-direction.
  \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
  \param filter : Pointer to a double array already allocated.
  \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
  \return Scaling factor to normalize the Scharr kernel.
*/
template <typename FilterType>
VP_DEPRECATED FilterType getScharrKernelY(FilterType *filter, unsigned int size)
{
  const unsigned int kernel_size = (size * 2) + 1;

  if (size != 1) {
    // Size = 1 => kernel_size = 2*1 + 1 = 3
    std::stringstream errMsg;
    errMsg << "Cannot get Scharr kernel of size " << kernel_size << " != 3";
    throw vpException(vpException::dimensionError, errMsg.str());
  }

  vpArray2D<FilterType> filter_as_array = vpArray2D<FilterType>::view(filter, kernel_size, kernel_size);
  FilterType scale = getScharrKernelY(filter_as_array, kernel_size);

  return scale;
}

  /*!
    Get Scharr kernel for X-direction.
    \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Pointer to a double array already allocated.
    \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
    \return Scaling factor to normalize the Scharr kernel.
  */
template <typename FilterType>
VP_DEPRECATED static FilterType getScharrKernelX(FilterType *filter, unsigned int size)
{
  const unsigned int kernel_size = (size * 2) + 1;

  vpArray2D<FilterType> ScharrY(kernel_size, kernel_size);
  FilterType norm = getScharrKernelY<FilterType>(ScharrY.data, size);
  memcpy(filter, ScharrY.t().data, ScharrY.getRows() * ScharrY.getCols() * sizeof(FilterType));
  return norm;
}

/*!
* Get Sobel kernel for Y-direction.
* \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter : Pointer to a double array already allocated.
* \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
* \return Scaling factor to normalize the Sobel kernel.
*/
template <typename FilterType>
VP_DEPRECATED FilterType getSobelKernelY(FilterType *filter, unsigned int size)
{
  const unsigned int maxSize = 20;
  if (size == 0) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
  }
  if (size > maxSize) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");
  }

  const unsigned int kernel_size = (size * 2) + 1;
  vpArray2D<FilterType> filter_as_array = vpArray2D<FilterType>::view(filter, kernel_size, kernel_size);
  FilterType scale = getSobelKernelY(filter_as_array, kernel_size);

  return scale;
}

/*!
* Get Sobel kernel for X-direction.
* \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
* \param filter : Pointer to a double array already allocated.
* \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
* \return Scaling factor to normalize the Sobel kernel.
*/
template <typename FilterType>
VP_DEPRECATED FilterType getSobelKernelX(FilterType *filter, unsigned int size)
{
  const unsigned int kernel_size = (size * 2) + 1;
  vpArray2D<FilterType> SobelY(kernel_size, kernel_size);
  FilterType norm = getSobelKernelY<FilterType>(SobelY.data, size);
  memcpy(filter, SobelY.t().data, SobelY.getRows() * SobelY.getCols() * sizeof(FilterType));
  return norm;
}
//@}
}
END_VISP_NAMESPACE
#endif
#endif
