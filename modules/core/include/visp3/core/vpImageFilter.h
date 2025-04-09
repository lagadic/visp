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
                                     const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
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
              filter[r][c] = static_cast<FilterType>(filter[r][c] * scale);
            }
          }
          };
#endif

        // Scales to apply to the filters to get a normalized gradient filter that gives a gradient
        // between 0 and 255 for an vpImage<uchar>
        float scaleX = 1.f;
        float scaleY = 1.f;
        const unsigned int val2 = 2U;

        if (filteringType == CANNY_GBLUR_SOBEL_FILTERING) {
          if (computeDx) {
            scaleX = static_cast<float>(vpImageFilter::getSobelKernelX(gradientFilterX.data, (apertureGradient - 1) / val2));
          }
          if (computeDy) {
            scaleY = static_cast<float>(vpImageFilter::getSobelKernelY(gradientFilterY.data, (apertureGradient - 1) / val2));
          }
        }
        else if (filteringType == CANNY_GBLUR_SCHARR_FILTERING) {
          if (computeDx) {
            scaleX = static_cast<float>(vpImageFilter::getScharrKernelX(gradientFilterX.data, (apertureGradient - 1) / val2));
          }
          if (computeDy) {
            scaleY = static_cast<float>(vpImageFilter::getScharrKernelY(gradientFilterY.data, (apertureGradient - 1) / val2));
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
                                            const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                                            const vpCannyFilteringAndGradientType &filteringType = CANNY_GBLUR_SOBEL_FILTERING,
                                            const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int w = I.getWidth();
    const unsigned int h = I.getHeight();

    if ((lowerThresholdRatio <= 0.f) || (lowerThresholdRatio >= 1.f)) {
      std::stringstream errMsg;
      errMsg << "Lower ratio (" << lowerThresholdRatio << ") " << (lowerThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    if ((upperThresholdRatio <= 0.f) || (upperThresholdRatio >= 1.f)) {
      std::stringstream errMsg;
      errMsg << "Upper ratio (" << upperThresholdRatio << ") " << (upperThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    if (lowerThresholdRatio  >= upperThresholdRatio) {
      std::stringstream errMsg;
      errMsg << "Lower ratio (" << lowerThresholdRatio << ") should be lower than the upper ratio (" << upperThresholdRatio << ")";
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

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
    unsigned int i = 0;
    bool notFound = true;
    while ((i < nbBins) && notFound) {
      float tf = static_cast<float>(hist[static_cast<unsigned char>(i)]);
      accu = accu + tf;
      if (accu > t) {
        bon = static_cast<float>(i);
        notFound = false;
      }
      ++i;
    }
    if (notFound) {
      std::stringstream errMsg;
      errMsg << "Could not find a bin for which " << upperThresholdRatio * 100.f << " percents of the pixels had a gradient lower than the upper threshold.";
      throw(vpException(vpException::fatalError, errMsg.str()));
    }
    float upperThresh = std::max<float>(bon, 1.f);
    lowerThresh = lowerThresholdRatio * bon;
    lowerThresh = std::max<float>(lowerThresh, std::numeric_limits<float>::epsilon());
    return upperThresh;
  }


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
   * \param[in] p_mask : If different from  \b nullptr , only the pixels for which \b p_mask is true will be considered.
   * \return The upper Canny edge filter threshold.
   */
  template<typename ArithmeticType, bool useFullScale, typename OutType>
  inline static float computeCannyThreshold(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, float &lowerThresh,
                                            const vpImage<OutType> *p_dIx = nullptr, const vpImage<OutType> *p_dIy = nullptr,
                                            const unsigned int &gaussianKernelSize = 5,
                                            const OutType &gaussianStdev = 2.f,
                                            const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                                            const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int w = I.getWidth();
    const unsigned int h = I.getHeight();

    if ((lowerThresholdRatio <= 0.f) || (lowerThresholdRatio >= 1.f)) {
      std::stringstream errMsg;
      errMsg << "Lower ratio (" << lowerThresholdRatio << ") " << (lowerThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    if ((upperThresholdRatio <= 0.f) || (upperThresholdRatio >= 1.f)) {
      std::stringstream errMsg;
      errMsg << "Upper ratio (" << upperThresholdRatio << ") " << (upperThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    if (lowerThresholdRatio  >= upperThresholdRatio) {
      std::stringstream errMsg;
      errMsg << "Lower ratio (" << lowerThresholdRatio << ") should be lower than the upper ratio (" << upperThresholdRatio << ")";
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    vpImage<unsigned char> dI(h, w);
    vpImage<OutType> dIx(h, w), dIy(h, w);
    if ((p_dIx != nullptr) && (p_dIy != nullptr)) {
      dIx = *p_dIx;
      dIy = *p_dIy;
    }
    else {
      vpImage<vpHSV<ArithmeticType, useFullScale>> Iblur;
      gaussianBlur(I, Iblur, gaussianKernelSize, gaussianStdev, true, p_mask);
      int nbThread = 1;
#ifdef VISP_HAVE_OPENMP
      nbThread = omp_get_max_threads();
#endif
      gradientFilter(Iblur, dIx, dIy, nbThread, p_mask);
    }

    // Computing the absolute gradient of the image G = |dIx| + |dIy|
    const float dIMax = 2.f * vpHSV<ArithmeticType, useFullScale>::maxGradValue;
    const float step = dIMax / 256.;
    for (unsigned int r = 0; r < h; ++r) {
      for (unsigned int c = 0; c < w; ++c) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, r, c);

        if (computeVal) {
          float dx = static_cast<float>(dIx[r][c]);
          float dy = static_cast<float>(dIy[r][c]);
          float gradient = std::abs(dx) + std::abs(dy);
          float encodedGradient = gradient / step;
          dI[r][c] = static_cast<unsigned char>(encodedGradient);
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
    unsigned int i = 0;
    bool notFound = true;
    while ((i < nbBins) && notFound) {
      float tf = static_cast<float>(hist[i]);
      accu = accu + tf;
      if (accu > t) {
        bon = static_cast<float>(i);
        notFound = false;
      }
      ++i;
    }
    if (notFound) {
      std::stringstream errMsg;
      errMsg << "Could not find a bin for which " << upperThresholdRatio * 100.f << " percents of the pixels had a gradient lower than the upper threshold.";
      throw(vpException(vpException::fatalError, errMsg.str()));
    }
    float upperThresh = bon * step;
    lowerThresh = lowerThresholdRatio * upperThresh;
    lowerThresh = std::max<float>(lowerThresh, std::numeric_limits<float>::epsilon());
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
    const int val1 = 1, val2 = 2, val3 = 3;
    return ((2047.0 * static_cast<double>(I[r][c + val1] - I[r][c - val1])) + (913.0 * static_cast<double>(I[r][c + val2] - I[r][c - val2])) +
            (112.0 * static_cast<double>(I[r][c + val3] - I[r][c - val3]))) / 8418.0;
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
    const int val1 = 1, val2 = 2, val3 = 3;
    return ((2047.0 * static_cast<double>(I[r + val1][c] - I[r - val1][c])) + (913.0 * static_cast<double>(I[r + val2][c] - I[r - val2][c])) +
            (112.0 * static_cast<double>(I[r + val3][c] - I[r - val3][c]))) / 8418.0;
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
  static FilterType filter(const vpImage<FilterType> &I, const vpArray2D<FilterType> &M, unsigned int row, unsigned int col)
  {
    const unsigned int size_y = M.getRows(), size_x = M.getCols();
    const unsigned int half_size_y = size_y / 2, half_size_x = size_x / 2;
    FilterType corr = 0;

    for (unsigned int a = 0; a < size_y; ++a) {
      for (unsigned int b = 0; b < size_x; ++b) {
        FilterType val = static_cast<FilterType>(I[row - half_size_y + a][col - half_size_x + b]); // Correlation
        corr += M[a][b] * val;
      }
    }
    return corr;
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
   * \param I : The original image.
   * \param GI : The filtered image.
   * \param filter : The separable filter.
   * \param size : The size of the filter.
   * \param p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).
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
    const int val2 = 2;
    return static_cast<unsigned char>(((1. * I[i][j - val2]) + (4. * I[i][j - 1]) + (6. * I[i][j]) + (4. * I[i][j + 1]) + (1. * I[i][j + val2])) / 16.);
  }
  static inline unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
  {
    const int val2 = 2;
    return static_cast<unsigned char>(((1. * I[i - val2][j]) + (4. * I[i - 1][j]) + (6. * I[i][j]) + (4. * I[i + 1][j]) + (1. * I[i + val2][j])) / 16.);
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
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
#endif
#else
#ifndef DOXYGEN_SHOULD_SKIP_THIS
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
  static inline typename std::enable_if<!std::is_same<Color, vpRGBa>::value, void>::type
    filterChannel(const Color &in, vpColVector &out, const FilterType &coeff)
  {
    out[0] = coeff * vpColorGetter<0>::get(in);
    out[1] = coeff * vpColorGetter<1>::get(in);
    out[2] = coeff * vpColorGetter<2>::get(in);
  }

/**
 * \brief Apply separately a filter to all the channels of a vpRGBa.
 *
 * \tparam FilterType An arithmetic type.
 * \param[in] in The RGBa that is filtered.
 * \param[in] out The result of the filtering, stored in a vector of doubles to limit rounding errors.
 * \param[in] coeff The filter coefficient to apply.
 */
  template<class Color, typename FilterType>
  static inline typename std::enable_if<std::is_same<Color, vpRGBa>::value, void>::type
    filterChannel(const Color &in, vpColVector &out, const FilterType &coeff)
  {
    out[0] = coeff * vpColorGetter<0>::get(in);
    out[1] = coeff * vpColorGetter<1>::get(in);
    out[2] = coeff * vpColorGetter<2>::get(in);
    out[3] = vpRGBa::alpha_default;
  }

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
  inline static void
    filterChannel(const Color &in1, const Color &in2, vpColVector &out, const FilterType &coeff)
  {
    out[0] += coeff * (vpColorGetter<0>::get(in1) + vpColorGetter<0>::get(in2));
    out[1] += coeff * (vpColorGetter<1>::get(in1) + vpColorGetter<1>::get(in2));
    out[2] += coeff * (vpColorGetter<2>::get(in1) + vpColorGetter<2>::get(in2));
  }
#endif

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
  static void filterX(const vpImage<ImageType> &I, vpImage<OutputType> &dIx, const FilterType *filter, unsigned int size,
                      const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight();
    const unsigned int width = I.getWidth();
    const unsigned int stop1J = (size - 1) / 2;
    const unsigned int stop2J = width - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

#ifdef VISP_HAVE_OPENMP
    unsigned int iam, nt, ipoints, istart, istop, npoints(height);
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
#else
    unsigned int istart = 0;
    unsigned int istop = height;
#endif
    for (unsigned int i = istart; i < istop; ++i) {
      for (unsigned int j = 0; j < stop1J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterXLeftBorder(I, dIx[i][j], i, j, filter, size);
        }
      }

      for (unsigned int j = stop1J; j < stop2J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterX(I, dIx[i][j], i, j, filter, size);
        }
      }
      for (unsigned int j = stop2J; j < width; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterXRightBorder(I, dIx[i][j], i, j, filter, size);
        }
      }
    }
#ifdef VISP_HAVE_OPENMP
    }
#endif
  }

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
   * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
   * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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

#ifndef DOXYGEN_SHOULD_SKIP_THIS
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
   * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                              const FilterType *filter, unsigned int size)
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
   * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
   * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                              const FilterType *filter, unsigned int size)
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
   * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
#endif
#endif


#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
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
#else

  /**
   * \brief Filter along the vertical direction.
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
  static void filterY(const vpImage<ImageType> &I, vpImage<OutputType> &dIy, const FilterType *filter, unsigned int size,
                      const vpImage<bool> *p_mask = nullptr)
  {
    const unsigned int height = I.getHeight(), width = I.getWidth();
    const unsigned int stop1I = (size - 1) / 2;
    const unsigned int stop2I = height - ((size - 1) / 2);
    resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  #ifdef VISP_HAVE_OPENMP
    unsigned int iam, nt, jpoints, jstart, jstop, npoints(width);
  #pragma omp parallel default(shared) private(iam, nt, jpoints, jstart, jstop)
    {
      iam = omp_get_thread_num();
      nt = omp_get_num_threads();
      jpoints = npoints / nt;
      // size of partition
      jstart = iam * jpoints; // starting array index
      if (iam == nt-1) {
        // last thread may do more
        jpoints = npoints - jstart;
      }
      jstop = jstart + jpoints;
  #else
    unsigned int jstart = 0;
    unsigned int jstop = width;
  #endif
    for (unsigned int i = 0; i < stop1I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterYTopBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop1I; i < stop2I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterY(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop2I; i < height; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          vpImageFilter::filterYBottomBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
  #ifdef VISP_HAVE_OPENMP
    }
  #endif
    }
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
   * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type  filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
   * \return std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for color input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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

#ifndef DOXYGEN_SHOULD_SKIP_THIS
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
   * \return std::enable_if<std::is_arithmetic<ImageType>::value, void>::type The method is enabled only for arithmetic input type.
   */
  template<typename ImageType, typename OutputType, typename FilterType>
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                              const FilterType *filter, unsigned int size)
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
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
  static inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
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
  static inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
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
#endif
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
  template <typename ImageType, typename OutputType, typename FilterType>
  static inline void
  gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size = 7, FilterType sigma = 0., bool normalize = true,
                          const vpImage<bool> *p_mask = nullptr)
  {
    FilterType *fg = new FilterType[(size + 1) / 2];
    vpImageFilter::getGaussianKernel<FilterType>(fg, size, sigma, normalize);
    vpImage<OutputType> GIx;
    vpImageFilter::filterX<ImageType, OutputType>(I, GIx, fg, size, p_mask);
    vpImageFilter::filterY<OutputType, OutputType>(GIx, GI, fg, size, p_mask);
    GIx.destroy();
    delete[] fg;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  static void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size = 7, double sigma = 0., bool normalize = true,
                          const vpImage<bool> *p_mask = nullptr);
#endif

  /*!
  * Apply a 5x5 Gaussian filter to an image pixel.
  *
  * \param fr : Image to filter
  * \param r : coordinates (row) of the pixel
  * \param c : coordinates (column) of the pixel
  */
  template <class T> static double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c)
  {
    const int val2 = 2;
    return ((15.0 * fr[r][c]) +
            (12.0 * (fr[r - 1][c] + fr[r][c - 1] + fr[r + 1][c] + fr[r][c + 1])) +
            (9.0 * (fr[r - 1][c - 1] + fr[r + 1][c - 1] + fr[r - 1][c + 1] + fr[r + 1][c + 1])) +
            (5.0 * (fr[r - val2][c] + fr[r][c - val2] + fr[r + val2][c] + fr[r][c + val2])) +
            (4.0 * (fr[r - val2][c + 1] + fr[r - val2][c - 1] + fr[r - 1][c - val2] + fr[r + 1][c - val2] + fr[r + val2][c - 1] +
                    fr[r + val2][c + 1] + fr[r - 1][c + val2] + fr[r + 1][c + val2])) +
            (2.0 * (fr[r - val2][c - val2] + fr[r + val2][c - val2] + fr[r - val2][c + val2] + fr[r + val2][c + val2]))) / 159.0;
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
    const unsigned int mod2 = 2;
    if ((size % mod2) != 1) {
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
      const unsigned int val2 = 2U;
      for (int i = 1; i <= middle; ++i) {
        sum += val2 * filter[i];
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
    const unsigned int mod2 = 2;
    if ((size % mod2) != 1) {
      throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));
    }

    if (sigma <= 0) {
      sigma = static_cast<FilterType>((size - 1) / 6.0);
    }

    const int half = 2;
    int middle = (static_cast<int>(size) - 1) / half;
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterX(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GIx,
    const int nbThread = 1, const vpImage<bool> *p_mask = nullptr, const vpCannyFilteringAndGradientType &type = CANNY_COUNT_FILTERING
  )
  {
    const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
    GIx.resize(nbRows, nbCols, 0.);
    std::vector<OutputType> filter(3);
    OutputType scale;
    switch (type) {
    case CANNY_COUNT_FILTERING:
      // Prewitt case
      filter = { 1., 1., 1. };
      scale = 6.;
      break;
    case CANNY_GBLUR_SOBEL_FILTERING:
      filter = { 1., 2., 1. };
      scale = 8.;
      break;
    case CANNY_GBLUR_SCHARR_FILTERING:
      filter = { 3., 10., 3. };
      scale = 32.;
      break;
    default:
      throw(vpException(vpException::badValue, "Wrong type of filtering"));
    }

    for (unsigned char i = 0; i < 3; ++i) {
      filter[i] = filter[i] / scale;
    }
  #ifdef VISP_HAVE_OPENMP
    if (nbThread == 1) {
      gradientFilterXMonothread(I, GIx, filter, p_mask);
    }
    else {
      gradientFilterXMultithread(I, GIx, filter, nbThread, p_mask);
    }
  #else
    gradientFilterXMonothread(I, GIx, filter, p_mask);
  #endif
  }
#endif

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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterY(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GIy,
  const int nbThread = 1, const vpImage<bool> *p_mask = nullptr, const vpCannyFilteringAndGradientType &type = CANNY_COUNT_FILTERING
  )
  {
    const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
    GIy.resize(nbRows, nbCols, 0.);
    std::vector<OutputType> filter(3);
    OutputType scale;
    switch (type) {
    case CANNY_COUNT_FILTERING:
      // Prewitt case
      filter = { 1., 1., 1. };
      scale = 6.;
      break;
    case CANNY_GBLUR_SOBEL_FILTERING:
      filter = { 1., 2., 1. };
      scale = 8.;
      break;
    case CANNY_GBLUR_SCHARR_FILTERING:
      filter = { 3., 10., 3. };
      scale = 32.;
      break;
    default:
      throw(vpException(vpException::badValue, "Wrong type of filtering"));
    }
    for (unsigned char i = 0; i < 3; ++i) {
      filter[i] = filter[i] / scale;
    }
  #ifdef VISP_HAVE_OPENMP
    if (nbThread == 1) {
      gradientFilterYMonothread(I, GIy, filter, p_mask);
    }
    else {
      gradientFilterYMultithread(I, GIy, filter, nbThread, p_mask);
    }
  #else
    gradientFilterYMonothread(I, GIy, filter, p_mask);
  #endif
  }

  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilter(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GIx, vpImage<OutputType> &GIy,
    const int nbThread = 1, const vpImage<bool> *p_mask = nullptr, const vpCannyFilteringAndGradientType &type = CANNY_COUNT_FILTERING
  )
  {
    const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
    GIx.resize(nbRows, nbCols, 0.);
    GIy.resize(nbRows, nbCols, 0.);
    gradientFilterX(I, GIx, nbThread, p_mask, type);
    gradientFilterY(I, GIy, nbThread, p_mask, type);
  }
#endif

  /*!
    Get Scharr kernel for X-direction.
    \tparam FilterType : Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Pointer to a double array already allocated.
    \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
    \return Scaling factor to normalize the Scharr kernel.
  */
  template <typename FilterType>
  inline static FilterType getScharrKernelX(FilterType *filter, unsigned int size)
  {
    const unsigned int actualKernelSize = (size * 2) + 1;
    if (size != 1) {
      // Size = 1 => kernel_size = 2*1 + 1 = 3
      std::stringstream errMsg;
      errMsg << "Cannot get Scharr kernel of size " << actualKernelSize << " != 3";
      throw vpException(vpException::dimensionError, errMsg.str());
    }

    vpArray2D<FilterType> ScharrY(actualKernelSize, actualKernelSize);
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
    const unsigned int kernel3 = 3;
    if (kernel_size == kernel3) {
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
    const unsigned int maxSize = 20;
    if (size == 0) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    }
    if (size > maxSize) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");
    }

    const unsigned int kernel_size = (size * 2) + 1;
    vpArray2D<FilterType> SobelY(kernel_size, kernel_size);
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

    const unsigned int maxSize = 20;
    if (size == 0) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    }
    if (size > maxSize) {
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");
    }

    const unsigned int kernel_size = (size * 2) + 1;
    FilterType scale = static_cast<FilterType>(1. / 8.); // Scale to normalize Sobel3x3
    const unsigned int kernel3 = 3, kernel5 = 5, kernel7 = 7;
    if (kernel_size == kernel3) {
      memcpy(filter, SobelY3x3, kernel_size * kernel_size * sizeof(FilterType));
      return scale;
    }
    scale *= static_cast<FilterType>(1. / 16.); // Sobel5x5 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel3x3
    if (kernel_size == kernel5) {
      memcpy(filter, SobelY5x5, kernel_size * kernel_size * sizeof(FilterType));
      return scale;
    }
    scale *= static_cast<FilterType>(1. / 16.); // Sobel7x7 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel5x5
    if (kernel_size == kernel7) {
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
   * \tparam ImageType Any numerical type (int, float, ...)
   * \param p_mask If different from nullptr, a boolean mask that tells which pixels must be computed.
   * \param height The desired height.
   * \param width The desired width.
   * \param I The image that must be resized and potentially initialized.
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

  /**
   * \brief Indicates if the boolean mask is true at the desired index.
   *
   * \param[in] p_mask Pointer towards the boolean mask if any or nullptr.
   * \param[in] iter The index in the boolean mask bitmap.
   * \return true If the boolean mask is true at the desired index or if \b p_mask is equal to \b nullptr.
   * \return false False otherwise.
   */
  static inline bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &iter)
  {
    if (!p_mask) {
      return true;
    }
    return p_mask->bitmap[iter];
  };

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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImage(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &Idiff
  )
  {
    const unsigned int nbCols = I.getCols();
    vpColVector diff(3);

    // Computing the difference and sign for row 0 column 0
    vpColVector diffPrevRow0, diffPrevRow1;
    bool isPositivePrevRow0, isPositivePrevRow1;
    Idiff.bitmap[0] = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[0], I.bitmap[1], diff);
    vpColVector current = I.bitmap[0].toColVector();
    isPositivePrevRow0 = (vpColVector::dotProd(diff, current) >= 0.);
    diffPrevRow0 = diff;

    // Computing the difference and sign for row 1 column 0
    Idiff.bitmap[nbCols] = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[nbCols], I.bitmap[nbCols + 1], diff);
    current = I.bitmap[nbCols].toColVector();
    isPositivePrevRow1 = (vpColVector::dotProd(diff, current) >= 0.);
    diffPrevRow1 = diff;

    for (unsigned int iter = 1; iter < nbCols - 1; ++iter) {
      // Computing the difference and sign for row 0
      OutputType distanceRow0 = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[iter], I.bitmap[iter + 1], diff);
      if (vpColVector::dotProd(diff, diffPrevRow0) < 0.) {
        // We change the sign of the difference only if the cosine distance is negative
        isPositivePrevRow0 = !isPositivePrevRow0;
      }
      diffPrevRow0 = diff;

      // Assigning the signed distance for the row 0
      if (isPositivePrevRow0) {
        Idiff.bitmap[iter] = distanceRow0;
      }
      else {
        Idiff.bitmap[iter] = -distanceRow0;
      }

      // Computing the difference and sign for row 1
      OutputType distanceRow1 = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[nbCols + iter], I.bitmap[nbCols + iter + 1], diff);
      if (vpColVector::dotProd(diff, diffPrevRow1) < 0.) {
        // We change the sign of the difference only if the cosine distance is negative
        isPositivePrevRow1 = !isPositivePrevRow1;
      }
      diffPrevRow1 = diff;

      // Assigning the signed distance for the row 1
      if (isPositivePrevRow1) {
        Idiff.bitmap[nbCols + iter] = distanceRow1;
      }
      else {
        Idiff.bitmap[nbCols + iter] = -distanceRow1;
      }
    }
  }

  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMonothread(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
    const vpImage<bool> *p_mask = nullptr
  )
  {
    const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
    const unsigned int size = I.getSize();
    const unsigned int offsetIdiff = nbCols;

    auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &iter, const unsigned int &c, const unsigned int &h, const unsigned int &w)
      {
        if (!p_mask) {
          return true;
        }
        static const unsigned int maxIter = (h - 1) * w;
        static const unsigned int minIter = w;
        bool hasToCompute = p_mask->bitmap[iter];
        if (c < w - 1) { // We do not compute gradient on the last column
          hasToCompute |= p_mask->bitmap[iter + 1]; // I[r][c + 1];
          if (iter < maxIter) { // We do not compute gradient on the last row
            hasToCompute |= p_mask->bitmap[iter + w + 1]; // I[r + 1][c + 1];
          }
        }

        if (iter < maxIter) { // We do not compute gradient on the last row
          hasToCompute |= p_mask->bitmap[iter + w]; // I[r + 1][c];
        }

        if (iter > minIter) { // We do not compute gradient on the first row
          hasToCompute |= p_mask->bitmap[iter - w]; // I[r - 1][c];
          if (c < w - 1) { // We do not compute gradient on the last column
            hasToCompute |= p_mask->bitmap[iter - w + 1]; // I[r - 1][c + 1];
          }
        }
        return hasToCompute;
      };

    vpImage<OutputType> Idiff(nbRows, nbCols);
    initGradientFilterDifferenceImage(I, Idiff);
    const unsigned int resetCounter = nbCols - 1;
    const unsigned int stopIter = size - (nbCols + 1);
    unsigned int counter = resetCounter, idCol = 0;
    vpColVector diff(3), diffPrev(3);
    bool isPrevPositive = true;
    for (unsigned int iter = nbCols; iter < stopIter; ++iter) {
      if (counter) {
        // Computing the amplitude of the difference
        OutputType futureDiff = 0.;
        if (checkBooleanPatch(p_mask, iter + offsetIdiff, idCol, nbRows, nbCols)) {
          futureDiff = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[iter + offsetIdiff], I.bitmap[iter + nbCols +1], diff);
        }
        else {
          diff = I.bitmap[iter + nbCols +1] - I.bitmap[iter + offsetIdiff];
        }
        if (idCol) {
          if (vpColVector::dotProd(diff, diffPrev) < 0.) {
            // We change the sign of the difference only if the cosine distance is negative
            isPrevPositive = !isPrevPositive;
          }
        }
        else {
          vpColVector colFirstCol = I.bitmap[iter + offsetIdiff].toColVector();
          // The first sign depends on the positiveness of the cosine distance between the difference and first pixel of a row
          isPrevPositive = (vpColVector::dotProd(diff, colFirstCol) >= 0.);
        }
        diffPrev = diff;

        // The sign of the difference is deduced by the sign of the cosine distance between the successive difference vectors
        if (isPrevPositive) {
          Idiff.bitmap[iter + offsetIdiff] = futureDiff;
        }
        else {
          Idiff.bitmap[iter + offsetIdiff] = -futureDiff;
        }
      }
      if (counter) {
        if ((counter != resetCounter)) {
          if (checkBooleanMask(p_mask, iter)) {
            OutputType gradient = 0.;
            int offset = iter - nbCols; // Looking in the row above first
            for (int i = -1; i <= 1; ++i) {
              // Kind of  +/- (I[r + i][c + 1] - I[r + i][c]) +/- (I[r + i][c] - I[r + i][c - 1])
              gradient += filter[i + 1] * (Idiff.bitmap[offset] + Idiff.bitmap[offset - 1]);
              offset += nbCols; // Preparing to look in the next row
            }
            GI.bitmap[iter] = gradient;
          }
        }
        --counter;
      }
      else {
        counter = resetCounter;
      }

      if (idCol < resetCounter) {
        ++idCol;
      }
      else {
        idCol = 0;
      }
    }
  }

  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageY(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &Idiff, std::vector<bool> &isPrevRowPositive,
    std::vector<vpColVector> &diffPrevRow
  )
  {
    const unsigned int nbCols = I.getCols();
    vpColVector diff(3), diff0(3); // Difference vector for I[0][0]
    // Computing the sign and distance for the first row
    for (unsigned int iter = 0; iter < nbCols; ++iter) {
      OutputType distance = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[iter], I.bitmap[iter + nbCols], diff);
      diffPrevRow[iter] = diff;
      vpColVector current = I.bitmap[iter].toColVector();
      // Checking the signeness of the distance using the sign of the cosine distance
      isPrevRowPositive[iter] = (vpColVector::dotProd(diff, current) >= 0.);
      // We set the signed distance in the difference map
      if (isPrevRowPositive[iter]) {
        Idiff.bitmap[iter] = distance;
      }
      else {
        Idiff.bitmap[iter] = -distance;
      }
      if (iter == 0) {
        diff0 = diff;
      }
    }
    // Computing the distance and sign for I[1][0]
    OutputType distance = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[nbCols], I.bitmap[nbCols + nbCols], diff);
    diffPrevRow[0] = diff;
    if (vpColVector::dotProd(diff, diff0) < 0.) {
      // If the cosine distance changes sign, we invert the sign of the difference map
      isPrevRowPositive[0] = !isPrevRowPositive[0];
    }
    // We set the signed distance in the difference map
    if (isPrevRowPositive[0]) {
      Idiff.bitmap[nbCols] = distance;
    }
    else {
      Idiff.bitmap[nbCols] = -distance;
    }
  }

  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMonothread(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
    const vpImage<bool> *p_mask = nullptr
  )
  {
    const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
    const unsigned int size = I.getSize();
    const unsigned int offsetIdiff = 1;

    auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &iter,
      const unsigned int &c /*iterSign*/,
      const unsigned int &resetCounter, const unsigned int &h, const unsigned int &w)
      {
        if (!p_mask) {
          return true;
        }

        static const unsigned int maxIter = (h - 1) * w;

        bool hasToCompute = p_mask->bitmap[iter];

        // unsigned int c;
        // if (iterSign) {
        //   c = iterSign - 1;
        // }
        // else {
        //   c = resetCounter;
        // }

        if (c < w - 1) { // We do not compute gradient on the last column
          hasToCompute |= p_mask->bitmap[iter + 1]; // Checking mask[r][c + 1]
          if (iter < maxIter) { // We do not compute gradient on the last row
            hasToCompute |= p_mask->bitmap[iter + w + 1]; // Checking mask[r + 1][c + 1]
          }
        }

        if (iter < maxIter) { // We do not compute gradient on the last row
          hasToCompute |= p_mask->bitmap[iter + w]; // Checking mask[r + 1][c]
        }

        if (c > 1) { // We do not compute gradient on the first column
          hasToCompute |= p_mask->bitmap[iter - 1]; // Checking mask[r][c - 1]
          if (iter < maxIter) { // We do not compute gradient on the last row
            hasToCompute |= p_mask->bitmap[iter + w - 1]; // Checking mask[r + 1][c - 1]
          }
        }
        return hasToCompute;
      };

    vpImage<OutputType> Idiff(nbRows, nbCols);
    std::vector<bool> isPrevRowPositive(nbCols, true);
    std::vector<vpColVector> diffRowPrev(nbCols);
    initGradientFilterDifferenceImageY(I, Idiff, isPrevRowPositive, diffRowPrev);
    const unsigned int resetCounter = nbCols - 1;
    const unsigned int stopIter = size - (nbCols + 1);
    unsigned int counter = resetCounter, iterSign = offsetIdiff;
    vpColVector diff(3);
    for (unsigned int iter = nbCols; iter < stopIter; ++iter) {
      // Computing the amplitude of the difference
      OutputType futureDiff = 0.;

      if (checkBooleanPatch(p_mask, iter + offsetIdiff, iterSign, resetCounter, nbRows, nbCols)) {
        futureDiff = vpHSV<HSVType, useFullScale>::template mahalanobisDistance<OutputType>(I.bitmap[iter + offsetIdiff], I.bitmap[iter + nbCols +1], diff);
      }
      else {
        diff = I.bitmap[iter + nbCols +1] - I.bitmap[iter + offsetIdiff];
      }

      // Computing the sign of the difference from the sign of the cosine distance
      if (vpColVector::dotProd(diff, diffRowPrev[iterSign]) < 0.) {
        isPrevRowPositive[iterSign] = !isPrevRowPositive[iterSign]; // Changing sign only if the cosine distance is negative
      }

      // Saving the difference vector
      diffRowPrev[iterSign] = diff;

      // The sign of the difference is deduced by the sign of the cosine distance between the successive difference vectors
      if (isPrevRowPositive[iterSign]) {
        Idiff.bitmap[iter + offsetIdiff] = futureDiff;
      }
      else {
        Idiff.bitmap[iter + offsetIdiff] = -futureDiff;
      }

      if (counter) {
        if ((counter != resetCounter)) {
          if (checkBooleanMask(p_mask, iter)) {
            OutputType gradient = 0.;
            for (int i = -1; i <= 1; ++i) {
              // Kind of +/- (I[r + 1][c + i] - I[r][c + 1]) +/- (I[r][c + i] - I[r - 1][c + 1])
              gradient += filter[i + 1] * (Idiff.bitmap[iter + i] + Idiff.bitmap[iter - nbCols + i]);
            }
            GI.bitmap[iter] = gradient;
          }
        }
        --counter;
      }
      else {
        counter = resetCounter;
      }
      if (iterSign < resetCounter) {
        ++iterSign;
      }
      else {
        iterSign = 0;
      }
    }
  }

#ifdef VISP_HAVE_OPENMP
  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMultithread(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
    const int &nbThread, const vpImage<bool> *p_mask = nullptr)
  {

  }

  template <typename HSVType, bool useFullScale, typename OutputType>
  static typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMultithread(
    const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
    const int &nbThread, const vpImage<bool> *p_mask = nullptr)
  {

  }
#endif
#endif
};
#if defined(__clang__)
#  pragma clang diagnostic pop
#endif
END_VISP_NAMESPACE
#endif

