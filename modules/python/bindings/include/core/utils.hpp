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

#ifndef VISP_PYTHON_CORE_UTILS_HPP
#define VISP_PYTHON_CORE_UTILS_HPP

#include <sstream>
#include <cstring>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

template<typename Item>
using np_array_cf = py::array_t<Item, py::array::c_style | py::array::forcecast>;

template<typename Item>
using np_array_c = py::array_t<Item, py::array::c_style>;

/*
 * Create a buffer info for a row major array
 */
template<typename T, unsigned N>
py::buffer_info make_array_buffer(T *data, std::array<unsigned, N> dims, bool readonly)
{
  std::array<py::ssize_t, N> strides;
  for (unsigned i = 0; i < N; i++) {
    unsigned s = sizeof(T);
    for (unsigned j = i + 1; j < N; ++j) {
      s *= dims[j];
    }
    strides[i] = s;
  }
  return py::buffer_info(
      data,                             /* Pointer to data (nullptr -> ask NumPy to allocate!) */
      sizeof(T),                        /* Size of one item */
      py::format_descriptor<T>::value,  /* Buffer format */
      N,                                /* How many dimensions? */
      dims,                             /* Number of elements for each dimension */
      strides,                           /* Strides for each dimension */
      readonly
  );
}

std::string shape_to_string(const std::vector<py::ssize_t> &shape)
{
  std::stringstream ss;
  ss << "(";
  for (int i = 0; i < int(shape.size()) - 1; ++i) {
    ss << shape[i] << ",";
  }
  if (shape.size() > 0) {
    ss << shape[shape.size() - 1];
  }
  ss << ")";
  return ss.str();
}

template<typename Item>
void verify_array_shape_and_dims(np_array_cf<Item> np_array, unsigned dims, const char *class_name)
{
  py::buffer_info buffer = np_array.request();
  std::vector<py::ssize_t> shape = buffer.shape;
  if (shape.size() != dims) {
    std::stringstream ss;
    ss << "Tried to instanciate " << class_name
      << " that expects a " << dims << "D array but got a numpy array of shape "
      << shape_to_string(shape);

    throw std::runtime_error(ss.str());
  }
}

template<typename Item>
void verify_array_shape_and_dims(np_array_c<Item> np_array, unsigned dims, const char *class_name)
{
  py::buffer_info buffer = np_array.request();
  std::vector<py::ssize_t> shape = buffer.shape;
  if (shape.size() != dims) {
    std::stringstream ss;
    ss << "Tried to instanciate " << class_name
      << " that expects a " << dims << "D array but got a numpy array of shape "
      << shape_to_string(shape);

    throw std::runtime_error(ss.str());
  }
}

template<typename Item>
void verify_array_shape_and_dims(np_array_cf<Item> np_array, std::vector<py::ssize_t> expected_dims, const char *class_name)
{
  verify_array_shape_and_dims(np_array, static_cast<unsigned int>(expected_dims.size()), class_name);
  py::buffer_info buffer = np_array.request();
  std::vector<py::ssize_t> shape = buffer.shape;
  bool invalid_shape = false;
  for (unsigned int i = 0; i < expected_dims.size(); ++i) {
    if (shape[i] != expected_dims[i]) {
      invalid_shape = true;
      break;
    }
  }
  if (invalid_shape) {
    std::stringstream ss;
    ss << "Tried to instanciate " << class_name
      << " that expects an array of dimensions " << shape_to_string(expected_dims)
      << " but got a numpy array of shape " << shape_to_string(shape);

    throw std::runtime_error(ss.str());
  }
}
template<typename Item>
void copy_data_from_np(np_array_cf<Item> src, Item *dest)
{
  py::buffer_info buffer = src.request();
  std::vector<py::ssize_t> shape = buffer.shape;
  unsigned int elements = 1;
  for (py::ssize_t dim : shape) {
    elements *= static_cast<unsigned int>(dim);
  }
  const Item *data = (Item *)buffer.ptr;
  std::memcpy(dest, data, elements * sizeof(Item));

}

std::tuple<int, int, int, int> solveSliceIndices(py::slice slice, unsigned int size)
{

  py::handle start = slice.attr("start"), end = slice.attr("stop"), step = slice.attr("step");

  int startI = 0, endI = size, stepI = 1;
  if (!start.is(py::none())) {
    startI = py::cast<int>(start);
    if (startI < 0) {
      startI = size + startI;
    }

    if (startI >= static_cast<int>(size)) {
      throw std::runtime_error("Invalid slice indexing out of array");
    }
  }

  if (!end.is(py::none())) {
    endI = py::cast<int>(end);
    if (endI < 0) {
      endI = size + endI;
    }

    if (endI > static_cast<int>(size)) {
      throw std::runtime_error("Invalid slice indexing out of array");
    }
  }

  if (!step.is(py::none())) {
    stepI = py::cast<int>(step);
    if (stepI <= 0) {
      throw std::runtime_error("Slice indexing: negative or zero step not supported!");
    }
  }

  if (endI < startI) {
    throw std::runtime_error("Slice indexing: end index is lower than start index");
  }

  int count;

  if (stepI > endI - startI) {
    count = 1;
  }
  else {
    int t = (endI - startI) / stepI;
    int endS = startI + t * stepI;
    count = (endS == endI) ? t : t + 1;
  }

  return std::make_tuple(startI, endI, stepI, count);
}


#endif
