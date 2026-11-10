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
 * Python bindings.
 */

#ifndef VISP_PYTHON_CORE_IMAGE_FILTER_HPP
#define VISP_PYTHON_CORE_IMAGE_FILTER_HPP
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRGBa.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <optional>
#include <sstream>

/*
 * Custom bindings for the templated functions of the vpImageFilter namespace, which is exposed as the
 * visp.core.ImageFilter Python submodule.
 * Functions that are deprecated or that are part of the vpImageFilter::detail namespace are not exposed.
 */

namespace
{
/*!
 * Convert an optional Python mask into a pointer that can be passed to the vpImageFilter functions.
 */
inline const VISP_NAMESPACE_ADDRESSING vpImage<bool> *
toMaskPtr(const std::optional<VISP_NAMESPACE_ADDRESSING vpImage<bool>> &mask)
{
  return mask ? &(mask.value()) : nullptr;
}
}

template<typename FilterType>
void define_getGaussianKernels(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  pyImageFilter.def("getGaussianKernel", [](vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    vpImageFilter::getGaussianKernel(filter, size, sigma, normalize);
  }, R"doc(
Return the coefficients G_i of a Gaussian filter.

:param filter: Array that will be resized to rows = 1, cols = (size+1)/2. The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core import ImageFilter, ArrayDouble2D
  size = 7
  filter = ArrayDouble2D()
  ImageFilter.getGaussianKernel(filter, size, sigma=0., normalize=True)
  assert filter.getRows() == 1 and filter.getCols() == (size + 1) // 2
  assert filter.numpy().any()
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);

  pyImageFilter.def("getGaussianDerivativeKernel", [](vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    vpImageFilter::getGaussianDerivativeKernel(filter, size, sigma, normalize);
  }, R"doc(
Return the coefficients of a Gaussian derivative filter that may be used to compute spatial image derivatives after applying a Gaussian blur.

:param filter: Array that will be resized to rows = 1, cols = (size+1)/2. The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core import ImageFilter, ArrayDouble2D
  size = 7
  filter = ArrayDouble2D()
  ImageFilter.getGaussianDerivativeKernel(filter, size, sigma=0., normalize=True)
  assert filter.getRows() == 1 and filter.getCols() == (size + 1) // 2
  assert filter.numpy().any()
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);
}

template<typename FilterType>
void define_simple_getGradXY(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def("getGradX", [](const vpImage<unsigned char> &input, vpImage<FilterType> &output, const std::optional<vpImage<bool>> &mask) -> void {
    vpImageFilter::getGradX(input, output, toMaskPtr(mask));
  }, R"doc(
Compute the gradient along the X-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the X-axis.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  Iout = ImageDouble()
  ImageFilter.getGradX(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradX(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
    )doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyImageFilter.def("getGradY", [](const vpImage<unsigned char> &input, vpImage<FilterType> &output, const std::optional<vpImage<bool>> &mask) -> void {
    vpImageFilter::getGradY(input, output, toMaskPtr(mask));
  }, R"doc(
Compute the gradient along the Y-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the Y-axis.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  Iout = ImageDouble()
  ImageFilter.getGradY(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradY(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename FilterType>
void define_complex_getGradXY(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def("getGradX", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const vpArray2D<FilterType> &filter, const std::optional<vpImage<bool>> &mask) -> void {
    vpImageFilter::getGradX(input, output, filter, toMaskPtr(mask));
  }, R"doc(
Compute the gradient along the X-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the X-axis.
:param filter: The half size derivative filter, as a 1 x (size+1)/2 array, e.g. computed with ImageFilter.getGaussianDerivativeKernel().
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter, ArrayDouble2D
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  filter = ArrayDouble2D()
  ImageFilter.getGaussianDerivativeKernel(filter, 7)
  Iout = ImageDouble()
  ImageFilter.getGradX(Iin, Iout, filter, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradX(Iin, Iout, filter, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyImageFilter.def("getGradY", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const vpArray2D<FilterType> &filter, const std::optional<vpImage<bool>> &mask) -> void {
    vpImageFilter::getGradY(input, output, filter, toMaskPtr(mask));
  }, R"doc(
Compute the gradient along the Y-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the Y-axis.
:param filter: The half size derivative filter, as a 1 x (size+1)/2 array, e.g. computed with ImageFilter.getGaussianDerivativeKernel().
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter, ArrayDouble2D
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  filter = ArrayDouble2D()
  ImageFilter.getGaussianDerivativeKernel(filter, 7)
  Iout = ImageDouble()
  ImageFilter.getGradY(Iin, Iout, filter, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradY(Iin, Iout, filter, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename FilterType>
void define_gaussianBlur(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "gaussianBlur",
    [](const vpImage<ImageType> &input,
       vpImage<FilterType> &output,
       unsigned int size,
       double sigma,
       bool normalize,
       const std::optional<vpImage<bool>> &mask) -> void {
         vpImageFilter::gaussianBlur(input, output, size, sigma, normalize, toMaskPtr(mask));
    },
    R"doc(
Apply a Gaussian blur to an image.

:param input: The input image.
:param output: The resulting blurred image.
:param size: Filter size. This value should be odd.
:param sigma: Gaussian standard deviation. If it is zero or negative, it is computed from the filter size as ``(size - 1) / 6``.
:param normalize: If True, normalize the Gaussian filter coefficients.
:param mask: Optional mask indicating which pixels to consider. True means that the pixel is considered, while False means that it is ignored.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter

  Iin = ImageGray(100, 100, 0)
  Iin[25:75, 25:75] = 50

  Iout = ImageDouble()
  ImageFilter.gaussianBlur(Iin, Iout)

  assert Iout.getRows() == Iin.getRows()
  assert Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()

  mask = ImageBool(100, 100, True)

  Iout = ImageDouble()
  ImageFilter.gaussianBlur(Iin, Iout, size=7, sigma=0.0, normalize=True, mask=mask)

  assert Iout.getRows() == Iin.getRows()
  assert Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("size") = 7,
    py::arg("sigma") = 0.,
    py::arg("normalize") = true,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename FilterType>
void define_partialDerivative(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "computePartialDerivatives",
    [](const vpImage<ImageType> &I,
       vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
       const bool &computeDx, const bool &computeDy, const bool &normalize,
       const unsigned int &gaussianKernelSize, const FilterType &gaussianStdev,
       const unsigned int &apertureGradient,
       const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
       const vpImageFilter::vpCannyBackendType &backend,
       const std::optional<vpImage<bool>> &mask) -> void {
         vpImageFilter::computePartialDerivatives(I, dIx, dIy, computeDx, computeDy, normalize, gaussianKernelSize, gaussianStdev,
                                                  apertureGradient, filteringType, backend, toMaskPtr(mask));
    },
    R"doc(
Compute the partial derivatives (i.e. horizontal and vertical gradients) of the input image.

:param I: The input image.
:param dIx: The horizontal partial derivative, i.e. horizontal gradient.
:param dIy: The vertical partial derivative, i.e. vertical gradient.
:param computeDx: Indicate if we must compute  the horizontal gradient.
:param computeDy: Indicate if we must compute  the vertical gradient.
:param normalize: If True, normalize the Gaussian filter coefficients.
:param gaussianKernelSize: Gaussian filter size. This value should be odd.
:param gaussianStdev: Gaussian standard deviation. If it is zero or negative, it is computed from the filter size as ``(size - 1) / 6``.
:param apertureGradient: Gradient filter size. This value should be odd.
:param filteringType: The type of filters to apply to compute the gradients.
:param backend: The type of backend to use to compute the gradients.
:param mask: Optional mask indicating which pixels to consider. True means that the pixel is considered, while False means that it is ignored.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter

  Iin = ImageGray(100, 100, 0)
  Iin[25:75, 25:75] = 50

  dIx = ImageDouble()
  dIy = ImageDouble()
  ImageFilter.computePartialDerivatives(Iin, dIx, dIy)

  assert dIx.getRows() == Iin.getRows()
  assert dIx.getCols() == Iin.getCols()
  assert dIx.numpy().any()
  assert dIy.getRows() == Iin.getRows()
  assert dIy.getCols() == Iin.getCols()
  assert dIy.numpy().any()

  mask = ImageBool(100, 100, True)

  dIx = ImageDouble()
  dIy = ImageDouble()
  ImageFilter.computePartialDerivatives(
    Iin,
    dIx, dIy,
    computeDx=True, computeDy=True,
    normalize=True, gaussianKernelSize=5, gaussianStdev=2.,
    apertureGradient=3,
    filteringType=ImageFilter.CANNY_GBLUR_SOBEL_FILTERING,
    backend=ImageFilter.CANNY_VISP_BACKEND,
    mask=mask
  )

  assert dIx.getRows() == Iin.getRows()
  assert dIx.getCols() == Iin.getCols()
  assert dIx.numpy().any()
  assert dIy.getRows() == Iin.getRows()
  assert dIy.getCols() == Iin.getCols()
  assert dIy.numpy().any()
)doc",
    py::arg("I"),
    py::arg("dIx"),
    py::arg("dIy"),
    py::arg("computeDx") = true,
    py::arg("computeDy") = true,
    py::arg("normalize") = true,
    py::arg("gaussianKernelSize") = 5,
    py::arg("gaussianStdev") = 2.,
    py::arg("apertureGradient") = 3,
    py::arg("filteringType") = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING,
    py::arg("backend") = vpImageFilter::CANNY_VISP_BACKEND,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ArithmeticType, bool useFullScale, typename FilterType>
void define_computeCannyThresholdHSV(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "computeCannyThreshold",
    [](const vpImage<vpHSV<ArithmeticType, useFullScale>> &I,
       const std::optional<vpImage<FilterType>> &dIx, const std::optional<vpImage<FilterType>> &dIy,
       const unsigned int &gaussianKernelSize,
       const FilterType &gaussianStdev,
       const float &lowerThresholdRatio, const float &upperThresholdRatio,
       const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
       const std::optional<vpImage<bool>> &mask) {
         const vpImage<FilterType> *p_dIx = (dIx ? &(dIx.value()) : nullptr);
         const vpImage<FilterType> *p_dIy = (dIy ? &(dIy.value()) : nullptr);
         float lowerThresh = 0.;
         float upperThresh = vpImageFilter::computeCannyThreshold(I, lowerThresh, p_dIx, p_dIy, gaussianKernelSize, gaussianStdev,
                                                                  lowerThresholdRatio, upperThresholdRatio, filteringType, toMaskPtr(mask));
         return py::make_tuple(lowerThresh, upperThresh);
    },
    R"doc(
Compute the lower and upper Canny edge filter thresholds of a HSV image, using Gaussian blur + Sobel or + Scharr operators to compute the gradient of the image.

:param I: The input HSV image.
:param dIx: If different from None, must contain the gradient of the image with regard to the horizontal axis.
:param dIy: If different from None, must contain the gradient of the image with regard to the vertical axis.
:param gaussianKernelSize: Gaussian filter size. This value should be odd.
:param gaussianStdev: Gaussian standard deviation. If it is zero or negative, it is computed from the filter size as ``(size - 1) / 6``.
:param lowerThresholdRatio: The ratio of the upper threshold the lower threshold must be equal to.
:param upperThresholdRatio: The ratio of pixels whose absolute gradient Gabs is lower or equal to define the upper threshold.
:param filteringType: The type of filters to apply to compute the gradients.
:param mask: Optional mask indicating which pixels to consider. True means that the pixel is considered, while False means that it is ignored.

:return: A tuple (lower threshold, upper threshold) for the Canny filter.
)doc",
    py::arg("I"),
    py::arg("dIx").none(true) = static_cast<std::optional<vpImage<FilterType>>>(std::nullopt),
    py::arg("dIy").none(true) = static_cast<std::optional<vpImage<FilterType>>>(std::nullopt),
    py::arg("gaussianKernelSize") = 5,
    py::arg("gaussianStdev") = 2.,
    py::arg("lowerThresholdRatio") = 0.6,
    py::arg("upperThresholdRatio") = 0.8,
    py::arg("filteringType") = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename FilterType>
void define_computeCannyThreshold(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "computeCannyThreshold",
    [](const vpImage<unsigned char> &I,
       const std::optional<vpImage<FilterType>> &dIx, const std::optional<vpImage<FilterType>> &dIy,
       const unsigned int &gaussianKernelSize,
       const FilterType &gaussianStdev, const unsigned int &apertureGradient,
       const float &lowerThresholdRatio, const float &upperThresholdRatio,
       const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
       const std::optional<vpImage<bool>> &mask) {
         const vpImage<FilterType> *p_dIx = (dIx ? &(dIx.value()) : nullptr);
         const vpImage<FilterType> *p_dIy = (dIy ? &(dIy.value()) : nullptr);
         float lowerThresh = 0.;
         float upperThresh = vpImageFilter::computeCannyThreshold(I, lowerThresh, p_dIx, p_dIy, gaussianKernelSize, gaussianStdev,
                                                                  apertureGradient, lowerThresholdRatio, upperThresholdRatio,
                                                                  filteringType, toMaskPtr(mask));
         return py::make_tuple(lowerThresh, upperThresh);
    },
    R"doc(
Compute the lower and upper Canny edge filter thresholds, using Gaussian blur + Sobel or + Scharr operators to compute the gradient of the image.

:param I: The input image.
:param dIx: If different from None, must contain the gradient of the image with regard to the horizontal axis.
:param dIy: If different from None, must contain the gradient of the image with regard to the vertical axis.
:param gaussianKernelSize: Gaussian filter size. This value should be odd.
:param gaussianStdev: Gaussian standard deviation. If it is zero or negative, it is computed from the filter size as ``(size - 1) / 6``.
:param apertureGradient: Gradient filter size. This value should be odd.
:param lowerThresholdRatio: The ratio of the upper threshold the lower threshold must be equal to.
:param upperThresholdRatio: The ratio of pixels whose absolute gradient Gabs is lower or equal to define the upper threshold.
:param filteringType: The type of filters to apply to compute the gradients.
:param mask: Optional mask indicating which pixels to consider. True means that the pixel is considered, while False means that it is ignored.

:return: A tuple (lower threshold, upper threshold) for the Canny filter.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageBool, ImageFilter
  import numpy as np

  Iin = ImageGray(100, 100, 0)
  Iin.numpy()[25:75, 25:75] = np.random.randint(0,256,size=(50,50))

  (lowerThresh, upperThresh) = ImageFilter.computeCannyThreshold(Iin)

  assert lowerThresh != 0.
  assert upperThresh != 0.

  mask = ImageBool(100, 100, True)

  (lowerThresh, upperThresh) = ImageFilter.computeCannyThreshold(
    Iin,
    dIx=None, dIy=None,
    gaussianKernelSize=5, gaussianStdev=2.,
    apertureGradient=3,
    lowerThresholdRatio=0.6,
    upperThresholdRatio=0.8,
    filteringType=ImageFilter.CANNY_GBLUR_SOBEL_FILTERING,
    mask=mask
  )

  assert lowerThresh != 0.
  assert upperThresh != 0.
)doc",
    py::arg("I"),
    py::arg("dIx").none(true) = static_cast<std::optional<vpImage<FilterType>>>(std::nullopt),
    py::arg("dIy").none(true) = static_cast<std::optional<vpImage<FilterType>>>(std::nullopt),
    py::arg("gaussianKernelSize") = 5,
    py::arg("gaussianStdev") = 2.,
    py::arg("apertureGradient") = 3,
    py::arg("lowerThresholdRatio") = 0.6,
    py::arg("upperThresholdRatio") = 0.8,
    py::arg("filteringType") = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

inline void define_canny(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "canny",
    [](const vpImage<unsigned char> &input,
       vpImage<unsigned char> &output,
       const unsigned int &gaussianFilterSize,
       const float &lowerThreshold,
       const float &upperThreshold,
       const unsigned int &apertureGradient,
       const float &gaussianStdev,
       const float &lowerThresholdRatio,
       const float &upperThresholdRatio,
       const bool &normalizeGradients,
       const vpImageFilter::vpCannyBackendType &cannyBackend,
       const vpImageFilter::vpCannyFilteringAndGradientType &cannyFilteringSteps,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::canny(input, output, gaussianFilterSize, lowerThreshold, upperThreshold, apertureGradient, gaussianStdev,
                             lowerThresholdRatio, upperThresholdRatio, normalizeGradients, cannyBackend, cannyFilteringSteps,
                             toMaskPtr(mask));
      },
    R"doc(
Apply the Canny edge detector to a grayscale image.

:param input: The input grayscale image.
:param output: The resulting edge image. Edge pixels have value 255, non-edge pixels have value 0.
:param gaussianFilterSize: Size of the Gaussian filter. Must be odd.
:param lowerThreshold: The lower threshold for the Canny operator. Values lower than this value are rejected. If negative, it will be set to one third of the upperThreshold.
:param upperThreshold: The upper threshold for the Canny operator. Only value greater than this value are marked as an edge. If negative, it will be automatically computed, along with the lower threshold. Otherwise, the lower threshold will be set to one third of the upper threshold.
:param apertureGradient: Size of the Sobel or Scharr gradient mask. Must be odd.
:param gaussianStdev: The standard deviation of the Gaussian filter to apply. If it is non-positive, it is computed from kernel size (gaussianKernelSize parameter) as σ=0.3∗((gaussianKernelSize−1)∗0.5−1)+0.8.
:param lowerThresholdRatio: The ratio of the upper threshold the lower threshold must be equal to. It is used only if the user asks to compute the Canny thresholds.
:param upperThresholdRatio: The ratio of pixels whose absolute gradient is lower or equal to define the upper threshold. It is used only if the user asks to compute the Canny thresholds.
:param normalizeGradients: Needs to be true if asking to compute the upperThreshold, otherwise it depends on the user application and user-defined thresholds.
:param cannyBackend: The backend to use to perform the Canny edge filtering.
:param cannyFilteringSteps: The filtering + gradient operators to apply to compute the gradient in the early stage of the Canny algorithm.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("gaussianFilterSize"),
    py::arg("lowerThreshold"),
    py::arg("upperThreshold"),
    py::arg("apertureGradient"),
    py::arg("gaussianStdev"),
    py::arg("lowerThresholdRatio"),
    py::arg("upperThresholdRatio"),
    py::arg("normalizeGradients"),
    py::arg("cannyBackend"),
    py::arg("cannyFilteringSteps"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename OutputType>
void define_filter(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "filter",
    [](const vpImage<ImageType> &input,
       vpImage<OutputType> &output,
       const vpArray2D<OutputType> &filter,
       bool convolve,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::filter(input, output, filter, convolve, toMaskPtr(mask));
      },
    R"doc(
Apply a 2D filter to an image.

:param input: The original image.
:param output: The filtered image.
:param filter: The filter kernel.
:param convolve: If true, perform a convolution otherwise a correlation.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("filter"),
    py::arg("convolve") = false,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyImageFilter.def(
    "filter",
    [](const vpImage<ImageType> &input,
       vpImage<OutputType> &Iu,
       vpImage<OutputType> &Iv,
       const vpArray2D<OutputType> &filter,
       bool convolve,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::filter(input, Iu, Iv, filter, convolve, toMaskPtr(mask));
      },
    R"doc(
Apply a 2D filter to an image, along the horizontal axis with the filter kernel and along the vertical axis with its transpose.

:param input: The original image.
:param Iu: Filtered image along the horizontal axis (u = columns).
:param Iv: Filtered image along the vertical axis (v = rows).
:param filter: The filter kernel.
:param convolve: If true, perform a convolution otherwise a correlation.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("Iu"),
    py::arg("Iv"),
    py::arg("filter"),
    py::arg("convolve") = false,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename OutputType, typename FilterType>
void define_separableFilter(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "separableFilter",
    [](const vpImage<ImageType> &input,
       vpImage<OutputType> &output,
       const vpArray2D<FilterType> &filter,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::separableFilter(input, output, filter, toMaskPtr(mask));
      },
    R"doc(
Apply a symmetric separable filter, first along the horizontal direction and then along the vertical one.

:param input: The original image.
:param output: The filtered image.
:param filter: The half size separable filter, as a 1 x (size+1)/2 array. The first value refers to the central coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageFilter, ArrayDouble2D

  Iin = ImageGray(100, 100, 0)
  Iin[25:75, 25:75] = 50
  filter = ArrayDouble2D()
  ImageFilter.getGaussianKernel(filter, 7)
  Iout = ImageDouble()
  ImageFilter.separableFilter(Iin, Iout, filter)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("filter"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename OutputType, typename FilterType>
void define_directional_filter(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "filterX",
    [](const vpImage<ImageType> &input,
       vpImage<OutputType> &output,
       const vpArray2D<FilterType> &filter,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::filterX<ImageType, OutputType, FilterType>(input, output, filter, toMaskPtr(mask));
      },
    R"doc(
Filter along the horizontal direction.

:param input: The original image.
:param output: The filtered image.
:param filter: The half size filter coefficients, as a 1 x (size+1)/2 array. The first value refers to the central coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("filter"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyImageFilter.def(
    "filterY",
    [](const vpImage<ImageType> &input,
       vpImage<OutputType> &output,
       const vpArray2D<FilterType> &filter,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::filterY<ImageType, OutputType, FilterType>(input, output, filter, toMaskPtr(mask));
      },
    R"doc(
Filter along the vertical direction.

:param input: The original image.
:param output: The filtered image.
:param filter: The half size filter coefficients, as a 1 x (size+1)/2 array. The first value refers to the central coefficient, the next ones to the right coefficients. Left coefficients are deduced by symmetry.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("filter"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename FilterType>
void define_getKernels(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyImageFilter.def(
    "getScharrKernelX",
    [](vpArray2D<FilterType> &filter, unsigned int size) -> FilterType
      {
        return vpImageFilter::getScharrKernelX(filter, size);
      },
    R"doc(
Get Scharr kernel for X-direction.

:param filter: Array that will be resized if needed to size x size.
:param size: Filter size, which must be odd. For the moment, only 3 x 3 filter is handled.
:return: Scaling factor to normalize the Scharr kernel.

Example usage:

.. testcode::

  from visp.core import ImageFilter, ArrayDouble2D
  filter = ArrayDouble2D()
  scale = ImageFilter.getScharrKernelX(filter, 3)
  assert filter.getRows() == 3 and filter.getCols() == 3
)doc",
    py::arg("filter"),
    py::arg("size"));

  pyImageFilter.def(
    "getScharrKernelY",
    [](vpArray2D<FilterType> &filter, unsigned int size) -> FilterType
      {
        return vpImageFilter::getScharrKernelY(filter, size);
      },
    R"doc(
Get Scharr kernel for Y-direction.

:param filter: Array that will be resized if needed to size x size.
:param size: Filter size, which must be odd. For the moment, only 3 x 3 filter is handled.
:return: Scaling factor to normalize the Scharr kernel.
)doc",
    py::arg("filter"),
    py::arg("size"));

  pyImageFilter.def(
    "getSobelKernelX",
    [](vpArray2D<FilterType> &filter, unsigned int size) -> FilterType
      {
        return vpImageFilter::getSobelKernelX(filter, size);
      },
    R"doc(
Get Sobel kernel for X-direction.

:param filter: Array that will be resized if needed to size x size.
:param size: Filter size, which must be odd. Max size is 41.
:return: Scaling factor to normalize the Sobel kernel.

Example usage:

.. testcode::

  from visp.core import ImageFilter, ArrayDouble2D
  filter = ArrayDouble2D()
  scale = ImageFilter.getSobelKernelX(filter, 5)
  assert filter.getRows() == 5 and filter.getCols() == 5
)doc",
    py::arg("filter"),
    py::arg("size"));

  pyImageFilter.def(
    "getSobelKernelY",
    [](vpArray2D<FilterType> &filter, unsigned int size) -> FilterType
      {
        return vpImageFilter::getSobelKernelY(filter, size);
      },
    R"doc(
Get Sobel kernel for Y-direction.

:param filter: Array that will be resized if needed to size x size.
:param size: Filter size, which must be odd. Max size is 41.
:return: Scaling factor to normalize the Sobel kernel.
)doc",
    py::arg("filter"),
    py::arg("size"));
}

template<typename ImageType, typename FilterType>
void define_getGradXYGauss2D(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  pyImageFilter.def(
    "getGradXGauss2D",
    [](const vpImage<ImageType> &input,
       vpImage<FilterType> &output,
       const vpArray2D<FilterType> &gaussianKernel,
       const vpArray2D<FilterType> &gaussianDerivativeKernel,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::getGradXGauss2D(input, output, gaussianKernel, gaussianDerivativeKernel, toMaskPtr(mask));
      },
    R"doc(
Compute the gradient along X after applying a gaussian filter along Y.

:param input: The original image.
:param output: Gradient along X.
:param gaussianKernel: Gaussian kernel which values should be computed using ImageFilter.getGaussianKernel().
:param gaussianDerivativeKernel: Gaussian derivative kernel which values should be computed using ImageFilter.getGaussianDerivativeKernel().
:param mask: Optional mask. True pixels are processed and False pixels are ignored.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageFilter, ArrayDouble2D
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  size = 7
  gaussianKernel = ArrayDouble2D()
  gaussianDerivativeKernel = ArrayDouble2D()
  ImageFilter.getGaussianKernel(gaussianKernel, size)
  ImageFilter.getGaussianDerivativeKernel(gaussianDerivativeKernel, size)
  Iout = ImageDouble()
  ImageFilter.getGradXGauss2D(Iin, Iout, gaussianKernel, gaussianDerivativeKernel)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  assert Iout.numpy().any()
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("gaussianKernel"),
    py::arg("gaussianDerivativeKernel"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyImageFilter.def(
    "getGradYGauss2D",
    [](const vpImage<ImageType> &input,
       vpImage<FilterType> &output,
       const vpArray2D<FilterType> &gaussianKernel,
       const vpArray2D<FilterType> &gaussianDerivativeKernel,
       const std::optional<vpImage<bool>> &mask) -> void
      {
        vpImageFilter::getGradYGauss2D(input, output, gaussianKernel, gaussianDerivativeKernel, toMaskPtr(mask));
      },
    R"doc(
Compute the gradient along Y after applying a gaussian filter along X.

:param input: The original image.
:param output: Gradient along Y.
:param gaussianKernel: Gaussian kernel which values should be computed using ImageFilter.getGaussianKernel().
:param gaussianDerivativeKernel: Gaussian derivative kernel which values should be computed using ImageFilter.getGaussianDerivativeKernel().
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("gaussianKernel"),
    py::arg("gaussianDerivativeKernel"),
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ArithmeticType, bool useFullScale, typename FilterType>
void define_gradientFilterHSV(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  pyImageFilter.def(
    "gradientFilterX",
    [](const vpImage<vpHSV<ArithmeticType, useFullScale>> &input,
       vpImage<FilterType> &output,
       const int &nbThread,
       const std::optional<vpImage<bool>> &mask,
       const vpImageFilter::vpCannyFilteringAndGradientType &filterType) -> void
      {
        vpImageFilter::gradientFilterX(input, output, nbThread, toMaskPtr(mask), filterType);
      },
    R"doc(
Gradient filter along the X-axis for HSV images.

:param input: The original image.
:param output: Gradient along X.
:param nbThread: The number of threads to use to compute the gradient. Setting -1 use the maximum number of thread.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
:param filterType: The type of the gradient filter.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("nbThread"),
    py::arg("mask").none(true),
    py::arg("filterType"));

  pyImageFilter.def(
    "gradientFilterY",
    [](const vpImage<vpHSV<ArithmeticType, useFullScale>> &input,
       vpImage<FilterType> &output,
       const int &nbThread,
       const std::optional<vpImage<bool>> &mask,
       const vpImageFilter::vpCannyFilteringAndGradientType &filterType) -> void
      {
        vpImageFilter::gradientFilterY(input, output, nbThread, toMaskPtr(mask), filterType);
      },
    R"doc(
Gradient filter along the Y-axis for HSV images.

:param input: The original image.
:param output: Gradient along Y.
:param nbThread: The number of threads to use to compute the gradient. Setting -1 use the maximum number of thread.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
:param filterType: The type of the gradient filter.
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("nbThread"),
    py::arg("mask").none(true),
    py::arg("filterType"));

  pyImageFilter.def(
    "gradientFilter",
    [](const vpImage<vpHSV<ArithmeticType, useFullScale>> &input,
       vpImage<FilterType> &GIx,
       vpImage<FilterType> &GIy,
       const int &nbThread,
       const std::optional<vpImage<bool>> &mask,
       const vpImageFilter::vpCannyFilteringAndGradientType &filterType) -> void
      {
        vpImageFilter::gradientFilter(input, GIx, GIy, nbThread, toMaskPtr(mask), filterType);
      },
    R"doc(
Compute the horizontal and vertical gradients for HSV images.

:param input: The original image.
:param GIx: Gradient along X.
:param GIy: Gradient along Y.
:param nbThread: The number of threads to use to compute the gradient. Setting -1 use the maximum number of thread.
:param mask: Optional mask. True pixels are processed and False pixels are ignored.
:param filterType: The type of the gradient filter.
)doc",
    py::arg("input"),
    py::arg("GIx"),
    py::arg("GIy"),
    py::arg("nbThread") = -1,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt),
    py::arg("filterType") = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING);
}

/*
 * vpImageFilter
 */
inline void bindings_vpImageFilter(py::module_ &pyImageFilter)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  define_getGaussianKernels<float>(pyImageFilter);
  define_getGaussianKernels<double>(pyImageFilter);

  define_simple_getGradXY<float>(pyImageFilter);
  define_simple_getGradXY<double>(pyImageFilter);
  define_complex_getGradXY<unsigned char, float>(pyImageFilter);
  define_complex_getGradXY<unsigned char, double>(pyImageFilter);
  define_complex_getGradXY<float, float>(pyImageFilter);
  define_complex_getGradXY<double, double>(pyImageFilter);

  define_gaussianBlur<unsigned char, float>(pyImageFilter);
  define_gaussianBlur<unsigned char, double>(pyImageFilter);
  define_gaussianBlur<float, float>(pyImageFilter);
  define_gaussianBlur<double, double>(pyImageFilter);
  define_gaussianBlur<vpRGBa, vpRGBa>(pyImageFilter);

  define_partialDerivative<unsigned char, float>(pyImageFilter);
  define_partialDerivative<unsigned char, double>(pyImageFilter);
  define_partialDerivative<float, float>(pyImageFilter);
  define_partialDerivative<double, double>(pyImageFilter);

  define_filter<unsigned char, float>(pyImageFilter);
  define_filter<unsigned char, double>(pyImageFilter);
  define_filter<float, float>(pyImageFilter);
  define_filter<double, double>(pyImageFilter);

  define_separableFilter<unsigned char, float, float>(pyImageFilter);
  define_separableFilter<unsigned char, double, double>(pyImageFilter);
  define_separableFilter<float, float, float>(pyImageFilter);
  define_separableFilter<double, double, double>(pyImageFilter);

  define_directional_filter<unsigned char, float, float>(pyImageFilter);
  define_directional_filter<unsigned char, float, double>(pyImageFilter);
  define_directional_filter<unsigned char, double, float>(pyImageFilter);
  define_directional_filter<unsigned char, double, double>(pyImageFilter);
  define_directional_filter<float, float, float>(pyImageFilter);
  define_directional_filter<float, float, double>(pyImageFilter);
  define_directional_filter<float, double, float>(pyImageFilter);
  define_directional_filter<float, double, double>(pyImageFilter);
  define_directional_filter<double, float, float>(pyImageFilter);
  define_directional_filter<double, float, double>(pyImageFilter);
  define_directional_filter<double, double, float>(pyImageFilter);
  define_directional_filter<double, double, double>(pyImageFilter);
  define_directional_filter<vpRGBa, vpRGBa, float>(pyImageFilter);
  define_directional_filter<vpRGBa, vpRGBa, double>(pyImageFilter);
  define_directional_filter<vpHSV<unsigned char, false>, vpHSV<unsigned char, false>, float>(pyImageFilter);
  define_directional_filter<vpHSV<unsigned char, false>, vpHSV<unsigned char, false>, double>(pyImageFilter);
  define_directional_filter<vpHSV<unsigned char, true>, vpHSV<unsigned char, true>, float>(pyImageFilter);
  define_directional_filter<vpHSV<unsigned char, true>, vpHSV<unsigned char, true>, double>(pyImageFilter);
  define_directional_filter<vpHSV<double, true>, vpHSV<double, true>, float>(pyImageFilter);
  define_directional_filter<vpHSV<double, true>, vpHSV<double, true>, double>(pyImageFilter);

  define_getKernels<float>(pyImageFilter);
  define_getKernels<double>(pyImageFilter);

  define_getGradXYGauss2D<unsigned char, float>(pyImageFilter);
  define_getGradXYGauss2D<unsigned char, double>(pyImageFilter);
  define_getGradXYGauss2D<float, float>(pyImageFilter);
  define_getGradXYGauss2D<double, double>(pyImageFilter);

  define_gradientFilterHSV<unsigned char, false, float>(pyImageFilter);
  define_gradientFilterHSV<unsigned char, false, double>(pyImageFilter);
  define_gradientFilterHSV<unsigned char, true, float>(pyImageFilter);
  define_gradientFilterHSV<unsigned char, true, double>(pyImageFilter);
  define_gradientFilterHSV<double, true, float>(pyImageFilter);
  define_gradientFilterHSV<double, true, double>(pyImageFilter);

  define_canny(pyImageFilter);
  define_computeCannyThreshold<float>(pyImageFilter);
  define_computeCannyThreshold<double>(pyImageFilter);
  define_computeCannyThresholdHSV<unsigned char, false, float>(pyImageFilter);
  define_computeCannyThresholdHSV<unsigned char, false, double>(pyImageFilter);
  define_computeCannyThresholdHSV<unsigned char, true, float>(pyImageFilter);
  define_computeCannyThresholdHSV<unsigned char, true, double>(pyImageFilter);
  define_computeCannyThresholdHSV<double, true, float>(pyImageFilter);
  define_computeCannyThresholdHSV<double, true, double>(pyImageFilter);
}
#endif
