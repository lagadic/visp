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
 * Various image tools, convolution, ...
 */

/*!
 * \file vpImageFilter.h
 * \brief  Various image filter, convolution, etc...
 */

#ifndef VP_IMAGE_FILTER_H
#define VP_IMAGE_FILTER_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
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
#include <opencv2/imgproc/imgproc_c.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpImageFilter
 *
 * \ingroup group_core_image
 *
 * \brief  Various image filter, convolution, etc...
*/
class VISP_EXPORT vpImageFilter
{
public:
  //! Canny filter backends for the edge detection operations
  typedef enum vpCannyBackendType
  {
    CANNY_OPENCV_BACKEND = 0,     //!< Use OpenCV
    CANNY_VISP_BACKEND = 1,     //!< Use ViSP
    CANNY_COUNT_BACKEND = 2 //! Number of supported backends
  } vpCannyBackendType;

  static std::string vpCannyBackendTypeList(const std::string &pref = "<", const std::string &sep = " , ",
                                            const std::string &suf = ">");

  static std::string vpCannyBackendTypeToString(const vpCannyBackendType &type);

  static vpCannyBackendType vpCannyBackendTypeFromString(const std::string &name);

  //! Canny filter and gradient operators to apply on the image before the edge detection stage
  typedef enum vpCannyFilteringAndGradientType
  {
    CANNY_GBLUR_SOBEL_FILTERING = 0, //!< Apply Gaussian blur + Sobel operator on the input image
    CANNY_GBLUR_SCHARR_FILTERING = 1, //!< Apply Gaussian blur + Scharr operator on the input image
    CANNY_COUNT_FILTERING = 2 //! Number of supported backends
  } vpCannyFilteringAndGradientType;

  static std::string vpGetCannyFiltAndGradTypes(const std::string &pref = "<", const std::string &sep = " , ",
                                                         const std::string &suf = ">");

  static std::string vpCannyFiltAndGradTypeToStr(const vpCannyFilteringAndGradientType &type);

  static vpCannyFilteringAndGradientType vpCannyFiltAndGradTypeFromStr(const std::string &name);

  static void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                    const float &thresholdCanny, const unsigned int &apertureSobel);

  static void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                    const float &lowerThresholdCanny, const float &higherThresholdCanny,
                    const unsigned int &apertureSobel);

  static void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int &gaussianFilterSize,
                    const float &lowerThresholdCanny, const float &higherThresholdCanny,
                    const unsigned int &apertureSobel, const float &gaussianStdev, const float &lowerThresholdRatio,
                    const float &upperThresholdRatio, const bool &normalizeGradients,
                    const vpCannyBackendType &cannyBackend, const vpCannyFilteringAndGradientType &cannyFilteringSteps,
                    const vpImage<bool> *p_mask = nullptr);

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
  static float computeCannyThreshold(const cv::Mat &cv_I, const cv::Mat *p_cv_dIx, const cv::Mat *p_cv_dIy,
                                     float &lowerThresh, const unsigned int &gaussianKernelSize = 5,
                                     const float &gaussianStdev = 2.f, const unsigned int &apertureGradient = 3,
                                     const float &lowerThresholdRatio = 0.6, const float &upperThresholdRatio = 0.8,
                                     const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING);

  static void computePartialDerivatives(const cv::Mat &cv_I,
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
  inline static void computePartialDerivatives(const vpImage<ImageType> &I,
                                               vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                               const unsigned int &apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND,
                                               const vpImage<bool> *p_mask = nullptr)
  {
    if (backend == CANNY_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
      cv::Mat cv_I, cv_dIx, cv_dIy;
      vpImageConvert::convert(I, cv_I);
      computePartialDerivatives(cv_I, cv_dIx, cv_dIy, computeDx, computeDy, normalize, gaussianKernelSize,
                                static_cast<float>(gaussianStdev), apertureGradient, filteringType);
      if (computeDx) {
        vpImageConvert::convert(cv_dIx, dIx);
      }
      if (computeDy) {
        vpImageConvert::convert(cv_dIy, dIy);
      }
#else
      throw(vpException(vpException::badValue, "You need to compile ViSP with OpenCV to use CANNY_OPENCV_BACKEND"));
#endif
    }
    else {
      if ((filteringType == CANNY_GBLUR_SCHARR_FILTERING) || (filteringType == CANNY_GBLUR_SOBEL_FILTERING)) {
        dIx.resize(I.getHeight(), I.getWidth());
        dIy.resize(I.getHeight(), I.getWidth());

        // Computing the Gaussian blur + gradients of the image
        vpImage<FilterType> Iblur;
        vpImageFilter::gaussianBlur(I, Iblur, gaussianKernelSize, gaussianStdev, true, p_mask);

        vpArray2D<FilterType> gradientFilterX(apertureGradient, apertureGradient); // Gradient filter along the X-axis
        vpArray2D<FilterType> gradientFilterY(apertureGradient, apertureGradient); // Gradient filter along the Y-axis

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
        // Helper to apply the scale to the raw values of the filters
        auto scaleFilter = [](vpArray2D<FilterType> &filter, const float &scale) {
          const unsigned int nbRows = filter.getRows();
          const unsigned int nbCols = filter.getCols();
          for (unsigned int r = 0; r < nbRows; ++r) {
            for (unsigned int c = 0; c < nbCols; ++c) {
              filter[r][c] = filter[r][c] * scale;
            }
          }
          };
#endif

        // Scales to apply to the filters to get a normalized gradient filter that gives a gradient
        // between 0 and 255 for an vpImage<uchar>
        float scaleX = 1.f;
        float scaleY = 1.f;

        if (filteringType == CANNY_GBLUR_SOBEL_FILTERING) {
          if (computeDx) {
            scaleX = static_cast<float>(vpImageFilter::getSobelKernelX(gradientFilterX.data, (apertureGradient - 1) / 2));
          }
          if (computeDy) {
            scaleY = static_cast<float>(vpImageFilter::getSobelKernelY(gradientFilterY.data, (apertureGradient - 1) / 2));
          }
        }
        else if (filteringType == CANNY_GBLUR_SCHARR_FILTERING) {
          if (computeDx) {
            scaleX = static_cast<float>(vpImageFilter::getScharrKernelX(gradientFilterX.data, (apertureGradient - 1) / 2));
          }
          if (computeDy) {
            scaleY = static_cast<float>(vpImageFilter::getScharrKernelY(gradientFilterY.data, (apertureGradient - 1) / 2));
          }
        }

        // Scale the gradient filters to have a normalized gradient filter
        if (normalize) {
          if (computeDx) {
            scaleFilter(gradientFilterX, scaleX);
          }
          if (computeDy) {
            scaleFilter(gradientFilterY, scaleY);
          }
        }

        // Apply the gradient filters to get the gradients
        if (computeDx) {
          vpImageFilter::filter(Iblur, dIx, gradientFilterX, true, p_mask);
        }

        if (computeDy) {
          vpImageFilter::filter(Iblur, dIy, gradientFilterY, true, p_mask);
        }
      }
      else {
        std::string errMsg = "[vpImageFilter::computePartialDerivatives] Filtering + gradient method \"";
        errMsg += vpCannyFiltAndGradTypeToStr(filteringType);
        errMsg += "\" is not implemented yet\n";
        throw(vpException(vpException::notImplementedError, errMsg));
      }
    }
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  template <typename FilterType>
  inline static void computePartialDerivatives(const vpImage<vpRGBa> &I,
                                               vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                               const unsigned int &apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;

  template <typename ImageType>
  inline static void computePartialDerivatives(const vpImage<ImageType> &I,
                                               vpImage<unsigned char> &dIx, vpImage<unsigned char> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int &gaussianKernelSize = 5, const unsigned char &gaussianStdev = 2.f,
                                               const unsigned int &apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;

  template <typename ImageType>
  inline static void computePartialDerivatives(const vpImage<ImageType> &I,
                                               vpImage<vpRGBa> &dIx, vpImage<vpRGBa> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int gaussianKernelSize = 5, const vpRGBa gaussianStdev = vpRGBa(),
                                               const unsigned int apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr) = delete;
#else
  template <typename FilterType>
  inline static void computePartialDerivatives(const vpImage<vpRGBa> &I,
                                               vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int &gaussianKernelSize = 5, const FilterType &gaussianStdev = 2.f,
                                               const unsigned int &apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);

  template <typename ImageType>
  inline static void computePartialDerivatives(const vpImage<ImageType> &I,
                                               vpImage<unsigned char> &dIx, vpImage<unsigned char> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int &gaussianKernelSize = 5, const unsigned char &gaussianStdev = 2.f,
                                               const unsigned int &apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);

  template <typename ImageType>
  inline static void computePartialDerivatives(const vpImage<ImageType> &I,
                                               vpImage<vpRGBa> &dIx, vpImage<vpRGBa> &dIy,
                                               const bool &computeDx = true, const bool &computeDy = true, const bool &normalize = true,
                                               const unsigned int gaussianKernelSize = 5, const vpRGBa gaussianStdev = vpRGBa(),
                                               const unsigned int apertureGradient = 3,
                                               const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                               const vpCannyBackendType &backend = CANNY_VISP_BACKEND, const vpImage<bool> *p_mask = nullptr);
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
  inline static float computeCannyThreshold(const vpImage<unsigned char> &I, float &lowerThresh,
                                            const vpImage<OutType> *p_dIx = nullptr, const vpImage<OutType> *p_dIy = nullptr,
                                            const unsigned int &gaussianKernelSize = 5,
                                            const OutType &gaussianStdev = 2.f, const unsigned int &apertureGradient = 3,
                                            const float &lowerThresholdRatio = 0.6, const float &upperThresholdRatio = 0.8,
                                            const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                            const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int w = I.getWidth();
    const unsigned int h = I.getHeight();

    vpImage<unsigned char> dI(h, w);
    vpImage<OutType> dIx(h, w), dIy(h, w);
    if ((p_dIx != nullptr) && (p_dIy != nullptr)) {
      dIx = *p_dIx;
      dIy = *p_dIy;
    }
    else {
      computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
                                apertureGradient, filteringType, vpImageFilter::CANNY_VISP_BACKEND, p_mask);
    }

    // Computing the absolute gradient of the image G = |dIx| + |dIy|
    for (unsigned int r = 0; r < h; ++r) {
      for (unsigned int c = 0; c < w; ++c) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, r, c);

        if (computeVal) {
          float dx = static_cast<float>(dIx[r][c]);
          float dy = static_cast<float>(dIy[r][c]);
          float gradient = std::abs(dx) + std::abs(dy);
          float gradientClamped = std::min<float>(gradient, static_cast<float>(std::numeric_limits<unsigned char>::max()));
          dI[r][c] = static_cast<unsigned char>(gradientClamped);
        }
      }
    }

    // Compute the histogram
    vpHistogram hist;
    hist.setMask(p_mask);
    const unsigned int nbBins = 256;
    hist.calculate(dI, nbBins);
    float totalNbPixels = static_cast<float>(hist.getTotal());
    float accu = 0;
    float t = upperThresholdRatio * totalNbPixels;
    float bon = 0;
    for (unsigned int i = 0; i < nbBins; ++i) {
      float tf = static_cast<float>(hist[i]);
      accu = accu + tf;
      if (accu > t) {
        bon = static_cast<float>(i);
        break;
      }
    }
    float upperThresh = std::max<float>(bon, 1.f);
    lowerThresh = lowerThresholdRatio * bon;
    return upperThresh;
  }

  /*!
   * Apply a 1x3 derivative filter to an image pixel.
   *
   * \param I : Image to filter
   * \param r : coordinates (row) of the pixel
   * \param c : coordinates (column) of the pixel
   */
  template <class ImageType> static double derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
  {
    return ((2047.0 * static_cast<double>(I[r][c + 1] - I[r][c - 1])) + (913.0 * static_cast<double>(I[r][c + 2] - I[r][c - 2])) +
            (112.0 * static_cast<double>(I[r][c + 3] - I[r][c - 3]))) / 8418.0;
  }

  /*!
   * Apply a 3x1 derivative filter to an image pixel.
   *
   * \param I : Image to filter
   * \param r : coordinates (row) of the pixel
   * \param c : coordinates (column) of the pixel
   */
  template <class ImageType> static double derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
  {
    return ((2047.0 * static_cast<double>(I[r + 1][c] - I[r - 1][c])) + (913.0 * static_cast<double>(I[r + 2][c] - I[r - 2][c])) +
            (112.0 * static_cast<double>(I[r + 3][c] - I[r - 3][c]))) / 8418.0;
  }

  /*!
   * Apply a 1 x size Derivative Filter in X to an image pixel.
   *
   * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param I : Image to filter
   * \param r : Coordinates(row) of the pixel
   * \param c : Coordinates(column) of the pixel
   * \param filter : Coefficients of the filter to be initialized using
   * vpImageFilter::getGaussianDerivativeKernel().
   * \param size : Size of the filter.
   *
   * \sa vpImageFilter::getGaussianDerivativeKernel()
   */
  template <class ImageType, typename FilterType>
  static FilterType derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
   * Apply a size x 1 Derivative Filter in Y to an image pixel.
   *
   * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param I : Image to filter.
   * \param r : Coordinates (row) of the pixel.
   * \param c : Coordinates (column) of the pixel.
   * \param filter : Coefficients of the filter to be initialized using
   * vpImageFilter::getGaussianDerivativeKernel().
   * \param size : Size of the filter.
   *
   * \sa vpImageFilter::getGaussianDerivativeKernel()
   */
  template <class ImageType, typename FilterType>
  static FilterType derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
  static void filter(const vpImage<ImageType> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false,
                     const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int size_y = M.getRows(), size_x = M.getCols();
    const unsigned int half_size_y = size_y / 2, half_size_x = size_x / 2;

    const unsigned int inputHeight = I.getHeight(), inputWidth = I.getWidth();
    If.resize(inputHeight, inputWidth, 0.0);

    if (convolve) {
      const unsigned int stopHeight = inputHeight - half_size_y;
      const unsigned int stopWidth = inputWidth - half_size_x;
      for (unsigned int i = half_size_y; i < stopHeight; ++i) {
        for (unsigned int j = half_size_x; j < stopWidth; ++j) {
          // We have to compute the value for each pixel if we don't have a mask or for
          // pixels for which the mask is true otherwise
          bool computeVal = checkBooleanMask(p_mask, i, j);
          if (computeVal) {
            FilterType conv = 0;

            for (unsigned int a = 0; a < size_y; ++a) {
              for (unsigned int b = 0; b < size_x; ++b) {
                FilterType val = static_cast<FilterType>(I[(i + half_size_y) - a][(j + half_size_x) - b]); // Convolution
                conv += M[a][b] * val;
              }
            }
            If[i][j] = conv;
          }
        }
      }
    }
    else {
      const unsigned int stopHeight = inputHeight - half_size_y;
      const unsigned int stopWidth = inputWidth - half_size_x;
      for (unsigned int i = half_size_y; i < stopHeight; ++i) {
        for (unsigned int j = half_size_x; j < stopWidth; ++j) {
          // We have to compute the value for each pixel if we don't have a mask or for
          // pixels for which the mask is true otherwise
          bool computeVal = checkBooleanMask(p_mask, i, j);
          if (computeVal) {
            FilterType corr = 0;

            for (unsigned int a = 0; a < size_y; ++a) {
              for (unsigned int b = 0; b < size_x; ++b) {
                FilterType val = static_cast<FilterType>(I[(i - half_size_y) + a][(j - half_size_x) + b]); // Correlation
                corr += M[a][b] * val;
              }
            }
            If[i][j] = corr;
          }
        }
      }
    }
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  template <typename FilterType>
  static void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false) = delete;
#else
  template <typename FilterType>
  static void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false);
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
  static void filter(const vpImage<ImageType> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M,
                     bool convolve = false, const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int size = M.getRows();
    const unsigned int half_size = size / 2;
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stopV = height - half_size;
    const unsigned int stopU = width - half_size;

    Iu.resize(height, width, 0.0);
    Iv.resize(height, width, 0.0);

    if (convolve) {
      for (unsigned int v = half_size; v < stopV; ++v) {
        for (unsigned int u = half_size; u < stopU; ++u) {
          // We have to compute the value for each pixel if we don't have a mask or for
          // pixels for which the mask is true otherwise
          bool computeVal = checkBooleanMask(p_mask, v, u);
          if (computeVal) {
            FilterType conv_u = 0;
            FilterType conv_v = 0;

            for (unsigned int a = 0; a < size; ++a) {
              for (unsigned int b = 0; b < size; ++b) {
                FilterType val = static_cast<FilterType>(I[(v + half_size) - a][(u + half_size) - b]); // Convolution
                conv_u += M[a][b] * val;
                conv_v += M[b][a] * val;
              }
            }
            Iu[v][u] = conv_u;
            Iv[v][u] = conv_v;
          }
        }
      }
    }
    else {
      for (unsigned int v = half_size; v < stopV; ++v) {
        for (unsigned int u = half_size; u < stopU; ++u) {
          // We have to compute the value for each pixel if we don't have a mask or for
          // pixels for which the mask is true otherwise
          bool computeVal = checkBooleanMask(p_mask, v, u);

          if (computeVal) {
            FilterType conv_u = 0;
            FilterType conv_v = 0;

            for (unsigned int a = 0; a < size; ++a) {
              for (unsigned int b = 0; b < size; ++b) {
                FilterType val = static_cast<FilterType>(I[(v - half_size) + a][(u - half_size) + b]); // Correlation
                conv_u += M[a][b] * val;
                conv_v += M[b][a] * val;
              }
            }
            Iu[v][u] = conv_u;
            Iv[v][u] = conv_v;
          }
        }
      }
    }
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  template<typename FilterType>
  static void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M, bool convolve) = delete;

  template<typename ImageType>
  static void filter(const vpImage<ImageType> &I, vpImage<ImageType> &Iu, vpImage<ImageType> &Iv, const vpArray2D<vpRGBa> &M, bool convolve) = delete;
#else
  template<typename FilterType>
  static void filter(const vpImage<vpRGBa> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M, bool convolve);

  template<typename ImageType>
  static void filter(const vpImage<ImageType> &I, vpImage<ImageType> &Iu, vpImage<ImageType> &Iv, const vpArray2D<vpRGBa> &M, bool convolve);
#endif

  static void sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH, const vpColVector &kernelV);

  /*!
   * Apply a separable filter.
   * \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
   * \param I: The original image.
   * \param GI: The filtered image.
   * \param filter: The separable filter.
   * \param size: The size of the filter.
   * \param p_mask: If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
   */
  template <typename ImageType, typename FilterType>
  static void filter(const vpImage<ImageType> &I, vpImage<FilterType> &GI, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr)
  {
    vpImage<FilterType> GIx;
    filterX<ImageType, FilterType>(I, GIx, filter, size, p_mask);
    filterY<FilterType, FilterType>(GIx, GI, filter, size, p_mask);
    GIx.destroy();
  }

  static inline unsigned char filterGaussXPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
  {
    return static_cast<unsigned char>(((1. * I[i][j - 2]) + (4. * I[i][j - 1]) + (6. * I[i][j]) + (4. * I[i][j + 1]) + (1. * I[i][j + 2])) / 16.);
  }
  static inline unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
  {
    return static_cast<unsigned char>(((1. * I[i - 2][j]) + (4. * I[i - 1][j]) + (6. * I[i][j]) + (4. * I[i + 1][j]) + (1. * I[i + 2][j])) / 16.);
  }

  template <typename ImageType, typename FilterType>
  static void filterX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size,
                      const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight();
    const unsigned int width = I.getWidth();
    const unsigned int stop1J = (size - 1) / 2;
    const unsigned int stop2J = width - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < stop1J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIx[i][j] = vpImageFilter::filterXLeftBorder<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
      for (unsigned int j = stop1J; j < stop2J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIx[i][j] = vpImageFilter::filterX<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
      for (unsigned int j = stop2J; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIx[i][j] = vpImageFilter::filterXRightBorder<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
    }
  }

  static void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

  template<typename ImageType, typename FilterType>
  static inline FilterType filterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
  {
    const unsigned int stop = (size - 1) / 2;
    FilterType result = static_cast<FilterType>(0.);

    for (unsigned int i = 1; i <= stop; ++i) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    return result + (filter[0] * static_cast<FilterType>(I[r][c]));
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  static void filterXR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterXG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterXB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

  static double filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

  static double filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

  template <typename ImageType, typename FilterType>
  static inline FilterType filterXLeftBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
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
  static inline FilterType filterXRightBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                              const FilterType *filter, unsigned int size)
  {
    const unsigned int stop = (size - 1) / 2;
    const unsigned int width = I.getWidth();
    FilterType result = static_cast<FilterType>(0.);

    for (unsigned int i = 1; i <= stop; ++i) {
      if ((c + i) < width) {
        result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
      }
      else {
        result += filter[i] * static_cast<FilterType>(I[r][((2 * width) - c) - i - 1] + I[r][c - i]);
      }
    }
    return result + (filter[0] * static_cast<FilterType>(I[r][c]));
  }
#endif

  static void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

  template<typename ImageType, typename FilterType>
  static void filterY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size,
                      const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stop1I = (size - 1) / 2;
    const unsigned int stop2I = height - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

    for (unsigned int i = 0; i < stop1I; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = vpImageFilter::filterYTopBorder<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop1I; i < stop2I; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = vpImageFilter::filterY<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop2I; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = vpImageFilter::filterYBottomBorder<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
    }
  }

  template<typename ImageType, typename FilterType>
  static inline FilterType filterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
  {
    const unsigned int stop = (size - 1) / 2;
    FilterType result = static_cast<FilterType>(0.);

    for (unsigned int i = 1; i <= stop; ++i) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    return result + (filter[0] * static_cast<FilterType>(I[r][c]));
  }
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  static void filterYR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterYG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterYB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

  static double filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

  static double filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);
  static double filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size);

  template<typename ImageType, typename FilterType>
  static inline FilterType filterYTopBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
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
  static inline FilterType filterYBottomBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                               const FilterType *filter, unsigned int size)
  {
    const unsigned int height = I.getHeight();
    const unsigned int stop = (size - 1) / 2;
    FilterType result = static_cast<FilterType>(0.);

    for (unsigned int i = 1; i <= stop; ++i) {
      if ((r + i) < height) {
        result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
      }
      else {
        result += filter[i] * static_cast<FilterType>(I[((2 * height) - r) - i - 1][c] + I[r - i][c]);
      }
    }
    return result + (filter[0] * static_cast<FilterType>(I[r][c]));
  }
#endif


  /*!
   * Apply a Gaussian blur to an image.
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
  template <typename ImageType, typename FilterType>
  static void gaussianBlur(const vpImage<ImageType> &I, vpImage<FilterType> &GI, unsigned int size = 7, FilterType sigma = 0., bool normalize = true,
                           const vpImage<bool> *p_mask = nullptr)
  {
    FilterType *fg = new FilterType[(size + 1) / 2];
    vpImageFilter::getGaussianKernel<FilterType>(fg, size, sigma, normalize);
    vpImage<FilterType> GIx;
    vpImageFilter::filterX<ImageType, FilterType>(I, GIx, fg, size, p_mask);
    vpImageFilter::filterY<FilterType, FilterType>(GIx, GI, fg, size, p_mask);
    GIx.destroy();
    delete[] fg;
  }

  static void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size = 7, double sigma = 0., bool normalize = true,
                           const vpImage<bool> *p_mask = nullptr);

  /*!
   * Apply a 5x5 Gaussian filter to an image pixel.
   *
   * \param fr : Image to filter
   * \param r : coordinates (row) of the pixel
   * \param c : coordinates (column) of the pixel
   */
  template <class T> static double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c)
  {
    return ((15.0 * fr[r][c]) +
            (12.0 * (fr[r - 1][c] + fr[r][c - 1] + fr[r + 1][c] + fr[r][c + 1])) +
            (9.0 * (fr[r - 1][c - 1] + fr[r + 1][c - 1] + fr[r - 1][c + 1] + fr[r + 1][c + 1])) +
            (5.0 * (fr[r - 2][c] + fr[r][c - 2] + fr[r + 2][c] + fr[r][c + 2])) +
            (4.0 * (fr[r - 2][c + 1] + fr[r - 2][c - 1] + fr[r - 1][c - 2] + fr[r + 1][c - 2] + fr[r + 2][c - 1] +
                    fr[r + 2][c + 1] + fr[r - 1][c + 2] + fr[r + 1][c + 2])) +
            (2.0 * (fr[r - 2][c - 2] + fr[r + 2][c - 2] + fr[r - 2][c + 2] + fr[r + 2][c + 2]))) / 159.0;
  }
  // Gaussian pyramid operation
  static void getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);

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
  static void getGaussianKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
  {
    if ((size % 2) != 1) {
      throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));
    }

    if (sigma <= 0) {
      sigma = static_cast<FilterType>((size - 1) / 6.0);
    }

    int middle = (static_cast<int>(size) - 1) / 2;
    FilterType sigma2 = static_cast<FilterType>(vpMath::sqr(sigma));
    FilterType coef1 = static_cast<FilterType>(1. / (sigma * sqrt(2. * M_PI)));
    FilterType v_2_sigma2 = static_cast<FilterType>(2. * sigma2);
    for (int i = 0; i <= middle; ++i) {
      filter[i] = coef1 * static_cast<FilterType>(exp(-(i * i) / v_2_sigma2));
    }
    if (normalize) {
      // renormalization
      FilterType sum = 0;
      for (int i = 1; i <= middle; ++i) {
        sum += 2 * filter[i];
      }
      sum += filter[0];

      for (int i = 0; i <= middle; ++i) {
        filter[i] = filter[i] / sum;
      }
    }
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
  static void getGaussianDerivativeKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
  {
    if ((size % 2) != 1) {
      throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));
    }

    if (sigma <= 0) {
      sigma = static_cast<FilterType>((size - 1) / 6.0);
    }

    int middle = (static_cast<int>(size) - 1) / 2;
    FilterType sigma2 = static_cast<FilterType>(vpMath::sqr(sigma));
    FilterType coef_1 = static_cast<FilterType>(1. / (sigma * sqrt(2. * M_PI)));
    FilterType coef_1_over_2 = coef_1 / static_cast<FilterType>(2.);
    FilterType v_2_coef_1 = static_cast<FilterType>(2.) * coef_1;
    FilterType v_2_sigma2 = static_cast<FilterType>(2. * sigma2);
    filter[0] = 0.;
    for (int i = 1; i <= middle; ++i) {
      filter[i] = -coef_1_over_2 * (static_cast<FilterType>(exp(-((i + 1) * (i + 1)) / v_2_sigma2)) - static_cast<FilterType>(exp(-((i - 1) * (i - 1)) / v_2_sigma2)));
    }

    if (normalize) {
      FilterType sum = 0;
      for (int i = 1; i <= middle; ++i) {
        sum += v_2_coef_1 * static_cast<FilterType>(exp(-(i * i) / v_2_sigma2));
      }
      sum += coef_1;

      for (int i = 1; i <= middle; ++i) {
        filter[i] = filter[i] / sum;
      }
    }
  }

  // Gradient along X
  template<typename FilterType>
  static void getGradX(const vpImage<unsigned char> &I, vpImage<FilterType> &dIx, const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stopJ = width - 3;
    const unsigned int val_3 = 3;
    resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < val_3; ++j) {
        // If a mask is used, the image is already initialized with 0s
        bool computeVal = (p_mask == nullptr);
        if (computeVal) {
          dIx[i][j] = static_cast<FilterType>(0);
        }
      }
      for (unsigned int j = 3; j < stopJ; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIx[i][j] = static_cast<FilterType>(vpImageFilter::derivativeFilterX(I, i, j));
        }
      }
      for (unsigned int j = stopJ; j < width; ++j) {
        // If a mask is used, the image is already initialized with 0s
        bool computeVal = (p_mask == nullptr);
        if (computeVal) {
          dIx[i][j] = static_cast<FilterType>(0);
        }
      }
    }
  }

  template <typename ImageType, typename FilterType>
  static void getGradX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stop1J = (size - 1) / 2;
    const unsigned int stop2J = width - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

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
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIx[i][j] = vpImageFilter::derivativeFilterX<ImageType, FilterType>(I, i, j, filter, size);
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
  static void getGradXGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *gaussianKernel,
                              const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask = nullptr)
  {
    vpImage<FilterType> GIy;
    vpImageFilter::filterY<ImageType, FilterType>(I, GIy, gaussianKernel, size, p_mask);
    vpImageFilter::getGradX<FilterType, FilterType>(GIy, dIx, gaussianDerivativeKernel, size, p_mask);
  }

  // Gradient along Y
  template <typename FilterType>
  static void getGradY(const vpImage<unsigned char> &I, vpImage<FilterType> &dIy, const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stopI = height - 3;
    resizeAndInitializeIfNeeded(p_mask, height, width, dIy);
    const unsigned int val_3 = 3;
    for (unsigned int i = 0; i < val_3; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = static_cast<FilterType>(0);
        }
      }
    }
    for (unsigned int i = 3; i < stopI; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = static_cast<FilterType>(vpImageFilter::derivativeFilterY(I, i, j));
        }
      }
    }
    for (unsigned int i = stopI; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = static_cast<FilterType>(0);
        }
      }
    }
  }

  template <typename ImageType, typename FilterType>
  static void getGradY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stop1I = (size - 1) / 2;
    const unsigned int stop2I = height - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

    for (unsigned int i = 0; i < stop1I; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = static_cast<FilterType>(0);
        }
      }
    }
    for (unsigned int i = stop1I; i < stop2I; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          dIy[i][j] = vpImageFilter::derivativeFilterY<ImageType, FilterType>(I, i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop2I; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
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
  static void getGradYGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *gaussianKernel,
                              const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask = nullptr)
  {
    vpImage<FilterType> GIx;
    vpImageFilter::filterX<ImageType, FilterType>(I, GIx, gaussianKernel, size, p_mask);
    vpImageFilter::getGradY<FilterType, FilterType>(GIx, dIy, gaussianDerivativeKernel, size, p_mask);
  }

  /*!
    Get Scharr kernel for X-direction.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Pointer to a double array already allocated.
    \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
    \return Scaling factor to normalize the Scharr kernel.
  */
  template <typename FilterType>
  inline static FilterType getScharrKernelX(FilterType *filter, unsigned int size)
  {
    if (size != 1) {
      // Size = 1 => kernel_size = 2*1 + 1 = 3
      std::stringstream errMsg;
      errMsg << "Cannot get Scharr kernel of size " << ((size * 2) + 1) << " != 3";
      throw vpException(vpException::dimensionError, errMsg.str());
    }

    vpArray2D<FilterType> ScharrY((size * 2) + 1, (size * 2) + 1);
    FilterType norm = getScharrKernelY<FilterType>(ScharrY.data, size);
    memcpy(filter, ScharrY.t().data, ScharrY.getRows() * ScharrY.getCols() * sizeof(FilterType));
    return norm;
  }

  /*!
    Get Scharr kernel for Y-direction.
    \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Pointer to a double array already allocated.
    \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
    \return Scaling factor to normalize the Scharr kernel.
  */
  template <typename FilterType>
  inline static FilterType getScharrKernelY(FilterType *filter, unsigned int size)
  {
    // Scharr kernel pre-computed for the usual size
    static const FilterType ScharrY3x3[9] = { -3.0, -10.0, -3.0, 0.0, 0.0, 0.0, 3.0, 10.0, 3.0 };

    if (size != 1) {
      // Size = 1 => kernel_size = 2*1 + 1 = 3
      std::stringstream errMsg;
      errMsg << "Cannot get Scharr kernel of size " << ((size * 2) + 1) << " != 3";
      throw vpException(vpException::dimensionError, errMsg.str());
    }

    const unsigned int kernel_size = (size * 2) + 1;
    if (kernel_size == 3) {
      memcpy(filter, ScharrY3x3, kernel_size * kernel_size * sizeof(FilterType));
      return static_cast<FilterType>(1.0 / 32.0);
    }

    return static_cast<FilterType>(0.);
  }

  /*!
   * Get Sobel kernel for X-direction.
   * \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
   * \param filter : Pointer to a double array already allocated.
   * \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
   * \return Scaling factor to normalize the Sobel kernel.
   */
  template <typename FilterType>
  inline static FilterType getSobelKernelX(FilterType *filter, unsigned int size)
  {
    if (size == 0) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    }
    if (size > 20) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");
    }

    vpArray2D<FilterType> SobelY((size * 2) + 1, (size * 2) + 1);
    FilterType norm = getSobelKernelY<FilterType>(SobelY.data, size);
    memcpy(filter, SobelY.t().data, SobelY.getRows() * SobelY.getCols() * sizeof(FilterType));
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
  inline static FilterType getSobelKernelY(FilterType *filter, unsigned int size)
  {
    // Sobel kernel pre-computed for the usual size
    static const FilterType SobelY3x3[9] = { -1.0, -2.0, -1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0 };
    static const FilterType SobelY5x5[25] = { -1.0, -4.0, -6.0, -4.0, -1.0, -2.0, -8.0, -12.0, -8.0, -2.0, 0.0, 0.0, 0.0,
                                        0.0,  0.0,  2.0,  8.0,  12.0, 8.0,  2.0,  1.0,   4.0,  6.0,  4.0, 1.0 };
    static const FilterType SobelY7x7[49] = { -1,   -6,  -15, -20, -15, -6, -1, -4, -24, -60, -80, -60, -24, -4, -5,  -30, -75,
                                        -100, -75, -30, -5,  0,   0,  0,  0,  0,   0,   0,   5,   30,  75, 100, 75,  30,
                                        5,    4,   24,  60,  80,  60, 24, 4,  1,   6,   15,  20,  15,  6,  1 };
    const vpArray2D<FilterType> smoothingKernel(3, 3);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    smoothingKernel[index_0][index_0] = 1.0;
    smoothingKernel[index_0][index_1] = 2.0;
    smoothingKernel[index_0][index_2] = 1.0;
    smoothingKernel[index_1][index_0] = 2.0;
    smoothingKernel[index_1][index_1] = 4.0;
    smoothingKernel[index_1][index_2] = 2.0;
    smoothingKernel[index_2][index_0] = 1.0;
    smoothingKernel[index_2][index_1] = 2.0;
    smoothingKernel[index_2][index_2] = 1.0;

    if (size == 0) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    }
    if (size > 20) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");
    }

    const unsigned int kernel_size = (size * 2) + 1;
    FilterType scale = static_cast<FilterType>(1. / 8.); // Scale to normalize Sobel3x3
    if (kernel_size == 3) {
      memcpy(filter, SobelY3x3, kernel_size * kernel_size * sizeof(FilterType));
      return scale;
    }
    scale *= static_cast<FilterType>(1. / 16.); // Sobel5x5 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel3x3
    if (kernel_size == 5) {
      memcpy(filter, SobelY5x5, kernel_size * kernel_size * sizeof(FilterType));
      return scale;
    }
    scale *= static_cast<FilterType>(1. / 16.); // Sobel7x7 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel5x5
    if (kernel_size == 7) {
      memcpy(filter, SobelY7x7, kernel_size * kernel_size * sizeof(FilterType));
      return scale;
    }

    vpArray2D<FilterType> sobelY(7, 7);
    memcpy(sobelY.data, SobelY7x7, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));
    for (unsigned int i = 4; i <= size; ++i) {
      sobelY = vpArray2D<FilterType>::conv2(sobelY, smoothingKernel, "full");
      // Sobel(N+1)x(N+1) is the convolution of smoothingKernel, which needs 1/16 scale factor, with SobelNxN
      scale *= static_cast<FilterType>(1. / 16.);
    }

    memcpy(filter, sobelY.data, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));

    return scale;
  }

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
  static float median(const cv::Mat &cv_I);
  static float median(const vpImage<unsigned char> &Isrc);
  static std::vector<float> median(const vpImage<vpRGBa> &Isrc);
#endif

private:
  /**
   * \brief Resize the image \b I to the desired size and, if \b p_mask is different from nullptr, initialize
   * \b I with 0s.
   *
   * @tparam ImageType Any numerical type (int, float, ...)
   * @param p_mask If different from nullptr, a boolean mask that tells which pixels must be computed.
   * @param height The desired height.
   * @param width The desired width.
   * @param I The image that must be resized and potentially initialized.
   */
  template<typename ImageType>
  static void resizeAndInitializeIfNeeded(const vpImage<bool> *p_mask, const unsigned int height, const unsigned int width, vpImage<ImageType> &I)
  {
    if (p_mask == nullptr) {
      // Just need to resize the output image, values will be computed and overwrite what is inside the image
      I.resize(height, width);
    }
    else {
      // Need to reset the image because some points will not be computed
      I.resize(height, width, static_cast<ImageType>(0));
    }
  }

  /**
   * \brief Indicates if the boolean mask is true at the desired coordinates.
   *
   * \param[in] p_mask Pointer towards the boolean mask if any or nullptr.
   * \param[in] r The row index in the boolean mask.
   * \param[in] c The column index in the boolean mask.
   * \return true If the boolean mask is true at the desired coordinates or if \b p_mask is equal to \b nullptr.
   * \return false False otherwise.
   */
  static bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c)
  {
    bool computeVal = true;
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    if (p_mask != nullptr)
#else
    if (p_mask != NULL)
#endif
    {
      computeVal = (*p_mask)[r][c];
    }
    return computeVal;
  }

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
  // Helper to apply the scale to the raw values of the filters
  template <typename FilterType>
  static void scaleFilter(vpArray2D<FilterType> &filter, const float &scale)
  {
    const unsigned int nbRows = filter.getRows();
    const unsigned int nbCols = filter.getCols();
    for (unsigned int r = 0; r < nbRows; ++r) {
      for (unsigned int c = 0; c < nbCols; ++c) {
        filter[r][c] = filter[r][c] * scale;
      }
    }
  }
#endif

};
END_VISP_NAMESPACE
#endif
