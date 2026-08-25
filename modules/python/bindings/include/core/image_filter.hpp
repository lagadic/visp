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
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRGBa.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <sstream>

/*
 * Image 2D indexing
 */
template<typename FilterType>
void define_getGradXY(py::class_<VISP_NAMESPACE_ADDRESSING vpImageFilter, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImageFilter>> &pyClass)
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
  Iin[25:75][25:75] = 50
  Iout = ImageDouble()
  Iout = ImageFilter.getGradX(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  Iout = ImageFilter.getGradX(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
    )doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true));

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
  Iin[25:75][25:75] = 50
  Iout = ImageDouble()
  Iout = ImageFilter.getGradY(Iin, Iout, None)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
  Iout = ImageDouble()
  mask = ImageBool(100,100,True)
  Iout = ImageFilter.getGradY(Iin, Iout, mask)
  assert Iout.getRows() == Iin.getRows() and Iout.getCols() == Iin.getCols()
)doc", py::arg("input"), py::arg("output"), py::arg("mask").none(true));
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
  define_getGradXY<float>(pyImageFilter);
  define_getGradXY<double>(pyImageFilter);
}

// template<typename T>
// typename std::enable_if<std::is_same<VISP_NAMESPACE_ADDRESSING vpRGBa, T>::value, void>::type
// bindings_vpImage(py::class_<VISP_NAMESPACE_ADDRESSING vpImage<T>, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImage<T>>> &pyImage)
// {
// #ifdef ENABLE_VISP_NAMESPACE
//   using namespace VISP_NAMESPACE_NAME;
// #endif
//   using NpRep = unsigned char;
//   static_assert(sizeof(T) == 4 * sizeof(NpRep));
//   pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
//     return make_array_buffer<NpRep, 3>(reinterpret_cast<NpRep *>(image.bitmap), { image.getHeight(), image.getWidth(), 4 }, false);
//   });
//   pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<NpRep> {
//     return py::cast(self).template cast<np_array_cf<NpRep>>();
//   }, numpy_fn_doc_image, py::keep_alive<0, 1>());

//   pyImage.def(py::init([](np_array_cf<NpRep> &np_array) {
//     verify_array_shape_and_dims(np_array, 3, "ViSP RGBa image");
//     const std::vector<py::ssize_t> shape = np_array.request().shape;
//     if (shape[2] != 4) {
//       throw std::runtime_error("Tried to copy a 3D numpy array that does not have 4 elements per pixel into a ViSP RGBA image");
//     }
//     vpImage<T> result(static_cast<unsigned int>(shape[0]), static_cast<unsigned int>(shape[1]));
//     copy_data_from_np(np_array, (NpRep *)result.bitmap);
//     return result;
//                        }), R"doc(
// Construct an image by **copying** a 3D numpy array. this numpy array should be of the form :math:`H \times W \times 4`
// where the 4 denotes the red, green, blue and alpha components of the image.

// :param np_array: The numpy array to copy.

// )doc", py::arg("np_array"));
//   define_get_item_2d_image<T, NpRep>(pyImage);
//   define_set_item_2d_image<T, NpRep>(pyImage, sizeof(T) / sizeof(NpRep));


//   pyImage.def("__repr__", [](const vpImage<T> &self) -> std::string {
//     std::stringstream ss;
//     ss << "<RGBa Image (" << self.getHeight() << ", " << self.getWidth() << ")>";
//     return ss.str();
//   });

//   pyImage.def("_visp_repr", [](const vpImage<T> &self) -> std::string {
//     std::stringstream ss;
//     ss << self;
//     return ss.str();
//   }, R"doc(Get the full ViSP image string representation.)doc");

// }
// template<typename T>
// typename std::enable_if<std::is_same<VISP_NAMESPACE_ADDRESSING vpRGBf, T>::value, void>::type
// bindings_vpImage(py::class_<VISP_NAMESPACE_ADDRESSING vpImage<T>, std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpImage<T>>> &pyImage)
// {
// #ifdef ENABLE_VISP_NAMESPACE
//   using namespace VISP_NAMESPACE_NAME;
// #endif
//   using NpRep = float;
//   static_assert(sizeof(T) == 3 * sizeof(NpRep));
//   pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
//     return make_array_buffer<NpRep, 3>(reinterpret_cast<NpRep *>(image.bitmap), { image.getHeight(), image.getWidth(), 3 }, false);
//   });

//   pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<NpRep> {
//     return py::cast(self).template cast<np_array_cf<NpRep>>();
//   }, numpy_fn_doc_image, py::keep_alive<0, 1>());

//   pyImage.def(py::init([](np_array_cf<NpRep> &np_array) {
//     verify_array_shape_and_dims(np_array, 3, "ViSP RGBa image");
//     const std::vector<py::ssize_t> shape = np_array.request().shape;
//     if (shape[2] != 3) {
//       throw std::runtime_error("Tried to copy a 3D numpy array that does not have 3 elements per pixel into a ViSP RGBf image");
//     }
//     vpImage<T> result(static_cast<unsigned int>(shape[0]), static_cast<unsigned int>(shape[1]));
//     copy_data_from_np(np_array, (NpRep *)result.bitmap);
//     return result;
//                        }), R"doc(
// Construct an image by **copying** a 3D numpy array. this numpy array should be of the form :math:`H \times W \times 3`
// where the 3 denotes the red, green and blue components of the image.

// :param np_array: The numpy array to copy.

// )doc", py::arg("np_array"));

//   define_get_item_2d_image<T, NpRep>(pyImage);
//   define_set_item_2d_image<T, NpRep>(pyImage, sizeof(T) / sizeof(NpRep));

//   pyImage.def("__repr__", [](const vpImage<T> &self) -> std::string {
//     std::stringstream ss;
//     ss << "<RGBf Image (" << self.getHeight() << ", " << self.getWidth() << ")>";
//     return ss.str();
//   });

//   pyImage.def("_visp_repr", [](const vpImage<T> &self) -> std::string {
//     std::stringstream ss;
//     ss << self;
//     return ss.str();
//   }, R"doc(Get the full ViSP image string representation.)doc");
// }

#endif
