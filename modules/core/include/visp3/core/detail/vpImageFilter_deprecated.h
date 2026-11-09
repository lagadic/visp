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
 * \class vpImageFilter
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
}
END_VISP_NAMESPACE
#endif
#endif
