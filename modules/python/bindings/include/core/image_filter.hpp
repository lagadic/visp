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
#include <sstream>

template<typename FilterType>
void define_getGaussianKernels(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  pyClass.def_static("getGaussianKernel", [](py::array_t<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    filter.resize({ static_cast<py::ssize_t>(1), static_cast<py::ssize_t>((size+1)/2) });
    py::buffer_info buf_filter = filter.request();
    FilterType *filter_ptr = static_cast<FilterType *>(buf_filter.ptr);
    vpImageFilter::getGaussianKernel(filter_ptr, size, sigma, normalize);
  }, R"doc(
Return the coefficients G_i of a Gaussian filter.

:param filter: Array that will have a dimension rows = 1, cols = (size+1)/2.The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: normalize Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core ImageFilter
  import numpy as np
  size = 7
  normalize = True
  filter = np.zeros((1,1))
  ImageFilter.getGaussianKernel(filter, size, sigma, normalize)
  assert filter.shape[0] == 1 and filter.shape[1] == ((size + 1)/2)
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);

  pyClass.def_static("getGaussianKernel", [](vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    filter.resize(1, (size+1)/2, true);
    vpImageFilter::getGaussianKernel(filter.data, size, sigma, normalize);
  }, R"doc(
Return the coefficients G_i of a Gaussian filter.

:param filter: Array that will have a dimension rows = 1, cols = (size+1)/2.The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: normalize Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core ImageFilter, ArrayDouble2D
  import numpy as np
  size = 7
  normalize = True
  filter = ArrayDouble2D(1,1)
  ImageFilter.getGaussianKernel(filter, size, sigma, normalize)
  assert filter.getRows() == 1 and filter.getCols() == ((size + 1)/2)
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);

  pyClass.def_static("getGaussianDerivativeKernel", [](py::array_t<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    filter.resize({ static_cast<py::ssize_t>(1), static_cast<py::ssize_t>((size+1)/2) });
    py::buffer_info buf_filter = filter.request();
    FilterType *filter_ptr = static_cast<FilterType *>(buf_filter.ptr);
    vpImageFilter::getGaussianDerivativeKernel(filter_ptr, size, sigma, normalize);
  }, R"doc(
Return the coefficients of a Gaussian derivative filter that may be used to compute spatial image derivatives after applying a Gaussian blur.

:param filter: Array that will have a dimension rows = 1, cols = (size+1)/2.The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: normalize Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core ImageFilter
  import numpy as np
  size = 7
  normalize = True
  filter = np.zeros((1,1))
  ImageFilter.getGaussianDerivativeKernel(filter, size, sigma, normalize)
  assert filter.shape[0] == 1 and filter.shape[1] == ((size + 1)/2)
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);

  pyClass.def_static("getGaussianDerivativeKernel", [](vpArray2D<FilterType> &filter, unsigned int size, FilterType sigma, bool normalize) -> void {
    filter.resize(1, (size+1)/2, true);
    vpImageFilter::getGaussianDerivativeKernel(filter.data, size, sigma, normalize);
  }, R"doc(
Return the coefficients of a Gaussian derivative filter that may be used to compute spatial image derivatives after applying a Gaussian blur.

:param filter: Array that will have a dimension rows = 1, cols = (size+1)/2.The first value refers to the central coefficient, the next one to the right coefficients. Left coefficients could be deduced by symmetry.
:param size: Filter size. This value should be odd and positive.
:param sigma: Gaussian standard deviation. If it is equal to zero or negative, it is computed from filter size as sigma = (size-1)/6.
:param normalize: normalize Flag indicating whether to normalize the filter coefficients or not. In that case, the sum of the coefficients equal one.

Example usage:

.. testcode::

  from visp.core ImageFilter, ArrayDouble2D
  import numpy as np
  size = 7
  normalize = True
  filter = ArrayDouble2D(1,1)
  ImageFilter.getGaussianDerivativeKernel(filter, size, sigma, normalize)
  assert filter.getRows() == 1 and filter.getCols() == ((size + 1)/2)
    )doc", py::arg("filter"), py::arg("size"), py::arg("sigma") = 0., py::arg("normalize") = true);
}

template<typename FilterType>
void define_simple_getGradXY(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyClass.def_static("getGradX", [](const vpImage<unsigned char> &input, vpImage<FilterType> &output, const std::optional<vpImage<bool>> &mask) -> void {
    if (!mask) {
      vpImageFilter::getGradX(input, output, nullptr);
    }
    else {
      vpImageFilter::getGradX(input, output, &(mask.value()));
    }
  }, R"doc(
Compute the gradient along the X-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the X-axis.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the X-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  Iout = ImageDouble()
  ImageFilter.getGradX(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradX(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
    )doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyClass.def_static("getGradY", [](const vpImage<unsigned char> &input, vpImage<FilterType> &output, const std::optional<vpImage<bool>> &mask) -> void {
    if (!mask) {
      vpImageFilter::getGradY(input, output, nullptr);
    }
    else {
      vpImageFilter::getGradY(input, output, &(mask.value()));
    }
}, R"doc(
Compute the gradient along the Y-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the Y-axis.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the Y-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  Iout = ImageDouble()
  ImageFilter.getGradY(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradY(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
)doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}

template<typename ImageType, typename FilterType>
void define_complex_getGradXY(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  pyClass.def_static("getGradX", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const py::array_t<FilterType> &filter, unsigned int size, const std::optional<vpImage<bool>> &mask) -> void {
    py::buffer_info buf_filter = filter.request();
    const unsigned int expected_dim = ((size+1)/2);
    if ((buf_filter.shape[1] != expected_dim) || (buf_filter.shape[0] != 1)) {
      std::stringstream ss;
      ss << "The filter must be of size (" << 1 << " , " << expected_dim << "), but got filter = (" << shape_to_string(buf_filter.shape);
      throw std::runtime_error(ss.str());
    }

    const FilterType *filter_ptr = static_cast<const FilterType *>(buf_filter.ptr);
    if (!mask) {
      vpImageFilter::getGradX(input, output, filter_ptr, size, nullptr);
    }
    else {
      vpImageFilter::getGradX(input, output, filter_ptr, size, &(mask.value()));
    }
  }, R"doc(
Compute the gradient along the X-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the X-axis.
:param filter: The filter to apply to compute the gradient.
:param size: The size of the filter.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the X-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  import numpy as np
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  size = 7
  filter = np.zeros((1,int((size + 1) / 2)))
  ImageFilter.getGaussianDerivativeKernel(filter, size)
  Iout = ImageDouble()
  ImageFilter.getGradX(Iin, Iout, filter, size, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradX(Iin, Iout, filter, size, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
    )doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("size"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyClass.def_static("getGradX", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const vpArray2D<FilterType> &filter, unsigned int size, const std::optional<vpImage<bool>> &mask) -> void {
    const unsigned int expected_dim = ((size+1)/2);
    if ((filter.getCols() != expected_dim) || (filter.getRows() != 1)) {
      std::stringstream ss;
      ss << "The filter must be of size (" << 1 << " , " << expected_dim << "), but got filter = (" << filter.getRows() << " , " << filter.getCols();
      throw std::runtime_error(ss.str());
    }

    const FilterType *filter_ptr = filter.data;
    if (!mask) {
      vpImageFilter::getGradX(input, output, filter_ptr, size, nullptr);
    }
    else {
      vpImageFilter::getGradX(input, output, filter_ptr, size, &(mask.value()));
    }
    }, R"doc(
Compute the gradient along the X-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the X-axis.
:param filter: The filter to apply to compute the gradient.
:param size: The size of the filter.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the Y-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter, ArrayDouble2D
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  size = 7
  filter = ArrayDouble2D()
  ImageFilter.getGaussianDerivativeKernel(filter, size)
  Iout = ImageDouble()
  ImageFilter.getGradX(Iin, Iout, filter, size, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradX(Iin, Iout, filter, size, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
)doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("size"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyClass.def_static("getGradY", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const py::array_t<FilterType> &filter, unsigned int size, const std::optional<vpImage<bool>> &mask) -> void {
    py::buffer_info buf_filter = filter.request();
    const unsigned int expected_dim = ((size+1)/2);
    if ((buf_filter.shape[1] != expected_dim) || (buf_filter.shape[0] != 1)) {
      std::stringstream ss;
      ss << "The filter must be of size (" << 1 << " , " << expected_dim << "), but got filter = (" << shape_to_string(buf_filter.shape);
      throw std::runtime_error(ss.str());
    }

    const FilterType *filter_ptr = static_cast<const FilterType *>(buf_filter.ptr);
    if (!mask) {
      vpImageFilter::getGradY(input, output, filter_ptr, size, nullptr);
    }
    else {
      vpImageFilter::getGradY(input, output, filter_ptr, size, &(mask.value()));
    }
  }, R"doc(
Compute the gradient along the Y-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the Y-axis.
:param filter: The filter to apply to compute the gradient.
:param size: The size of the filter.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the Y-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter
  import numpy as np
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  size = 7
  filter = np.zeros((1,int((size + 1) / 2)))
  ImageFilter.getGaussianDerivativeKernel(filter, size)
  Iout = ImageDouble()
  ImageFilter.getGradY(Iin, Iout, filter, size, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradY(Iin, Iout, filter, size, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
)doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("size"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));

  pyClass.def_static("getGradY", [](const vpImage<ImageType> &input, vpImage<FilterType> &output, const vpArray2D<FilterType> &filter, unsigned int size, const std::optional<vpImage<bool>> &mask) -> void {
    const unsigned int expected_dim = ((size+1)/2);
    if ((filter.getCols() != expected_dim) || (filter.getRows() != 1)) {
      std::stringstream ss;
      ss << "The filter must be of size (" << 1 << " , " << expected_dim << "), but got filter = (" << filter.getRows() << " , " << filter.getCols();
      throw std::runtime_error(ss.str());
    }

    const FilterType *filter_ptr = filter.data;
    if (!mask) {
      vpImageFilter::getGradY(input, output, filter_ptr, size, nullptr);
    }
    else {
      vpImageFilter::getGradY(input, output, filter_ptr, size, &(mask.value()));
    }
    }, R"doc(
Compute the gradient along the Y-axis.

:param input: The image that must be filtered.
:param output: The resulting image that contains the gradient along the Y-axis.
:param filter: The filter to apply to compute the gradient.
:param size: The size of the filter.
:param mask: mask where True means that the gradient must be computed for the pixel and False means that the computation must be skipped.

:return: An image that contains the gradient along the Y-axis.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter, ArrayDouble2D
  Iin = ImageGray(100,100,0)
  Iin[25:75,25:75] = 50
  size = 7
  filter = ArrayDouble2D()
  ImageFilter.getGaussianDerivativeKernel(filter, size)
  Iout = ImageDouble()
  ImageFilter.getGradY(Iin, Iout, filter, size, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  ImageFilter.getGradY(Iin, Iout, filter, size, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
)doc", py::arg("input"), py::arg("output"), py::arg("filter"), py::arg("size"), py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}



template<typename ImageType>
void define_gaussianFilter(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
{
#ifdef ENABLE_VISP_NAMESPACE
	using namespace VISP_NAMESPACE_NAME;
#endif

	pyClass.def_static(
		"gaussianFilter",
		[](const vpImage<ImageType> &input,
			unsigned int r,
			unsigned int c) -> double {
				return vpImageFilter::gaussianFilter(input, r, c);
			}, R"doc(
Apply a 5x5 Gaussian filter to one image pixel.

:param input: The image to filter.
:param r: Row coordinate of the pixel.
:param c: Column coordinate of the pixel.

:return: The filtered pixel value.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageFilter

  Iin = ImageGray(100, 100, 0)
  Iin[25:75, 25:75] = 50

  filtered_value = ImageFilter.gaussianFilter(Iin, 50, 50)
  assert isinstance(filtered_value, float)
)doc",
    py::arg("input"),
    py::arg("r"),
    py::arg("c"));
}

template<typename ImageType, typename FilterType>
void define_gaussianBlur(
	py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

	pyClass.def_static(
		"gaussianBlur",
		[](const vpImage<ImageType> &input,
			vpImage<FilterType> &output,
			unsigned int size,
			double sigma,
			bool normalize,
			const std::optional<vpImage<bool>> &mask) -> void {
      			if (!mask) {
        			vpImageFilter::gaussianBlur(input, output, size, sigma, normalize, nullptr);
				}
				else {
					vpImageFilter::gaussianBlur(
					input, output, size, sigma, normalize, &(mask.value()));
				}
    },
    R"doc(
Apply a Gaussian blur to an image.

:param input: The input image.
:param output: The resulting blurred image.
:param size: Filter size. This value should be odd.
:param sigma: Gaussian standard deviation. If it is zero or negative,
  it is computed from the filter size as ``(size - 1) / 6``.
:param normalize: If True, normalize the Gaussian filter coefficients.
:param mask: Optional mask indicating which pixels to consider. True
  means that the pixel is considered, while False means that it is ignored.

Example usage:

.. testcode::

  from visp.core import ImageGray, ImageDouble, ImageBool, ImageFilter

  Iin = ImageGray(100, 100, 0)
  Iin[25:75, 25:75] = 50

  Iout = ImageDouble()
  ImageFilter.gaussianBlur(Iin, Iout)

  assert Iout.getRows() == Iin.getRows()
  assert Iout.getCols() == Iin.getCols()

  mask = ImageBool(100, 100, True)

  Iout = ImageDouble()
  ImageFilter.gaussianBlur(
      Iin,
      Iout,
      size=7,
      sigma=0.0,
      normalize=True,
      mask=mask
  )

  assert Iout.getRows() == Iin.getRows()
  assert Iout.getCols() == Iin.getCols()
)doc",
    py::arg("input"),
    py::arg("output"),
    py::arg("size") = 7,
    py::arg("sigma") = 0.,
    py::arg("normalize") = true,
    py::arg("mask").none(true) = static_cast<std::optional<vpImage<bool>>>(std::nullopt));
}




  /*
   * vpImageFilter
   */
void
bindings_vpImageFilter(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyImageFilter)
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

  define_gaussianFilter<unsigned char>(pyImageFilter);
  define_gaussianFilter<double>(pyImageFilter);
  define_gaussianFilter<float>(pyImageFilter);

  define_gaussianBlur<unsigned char, float>(pyImageFilter);
  define_gaussianBlur<unsigned char, double>(pyImageFilter);
  define_gaussianBlur<float, float>(pyImageFilter);
  define_gaussianBlur<double, double>(pyImageFilter);
}
#endif
