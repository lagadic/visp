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
 * Python bindings.
 */

#ifndef VISP_PYTHON_CORE_IMAGES_HPP
#define VISP_PYTHON_CORE_IMAGES_HPP
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBf.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <sstream>

namespace
{
const char *numpy_fn_doc_image = R"doc(
  Numpy view of the underlying image data.
  This numpy view can be used to directly modify the array.
)doc";
}

/*
 * Image 2D indexing
 */
template<typename T, typename NpRep>
void define_get_item_2d_image(py::class_<vpImage<T>, std::shared_ptr<vpImage<T>>> &pyClass)
{
  pyClass.def("__getitem__", [](const vpImage<T> &self, std::pair<int, int> pair) -> T {
    int i = pair.first, j = pair.second;
    const int rows = (int)self.getRows(), cols = (int)self.getCols();
    if (i >= rows || j >= cols || i < -rows || j < -cols) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D image: got indices " << shape_to_string({ i, j })
        << " but image has dimensions " << shape_to_string({ rows, cols });
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = rows + i;
    }
    if (j < 0) {
      j = cols + j;
    }
    return self[i][j];
  });
  pyClass.def("__getitem__", [](const vpImage<T> &self, int i) -> np_array_cf<NpRep> {
    const int rows = (int)self.getRows();
    if (i >= rows || i < -rows) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D image: got row index " << shape_to_string({ i })
        << " but array has " << rows << " rows";
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = rows + i;
    }
    return (py::cast(self).template cast<np_array_cf<NpRep> >())[py::cast(i)].template cast<np_array_cf<NpRep>>();
  });
  pyClass.def("__getitem__", [](const vpImage<T> &self, py::slice slice) -> py::array_t<NpRep> {
    return (py::cast(self).template cast<np_array_cf<NpRep> >())[slice].template cast<py::array_t<NpRep>>();
  }, py::keep_alive<0, 1>());
  pyClass.def("__getitem__", [](const vpImage<T> &self, py::tuple tuple) {
    return (py::cast(self).template cast<np_array_cf<NpRep> >())[tuple].template cast<py::array_t<NpRep>>();
  }, py::keep_alive<0, 1>());
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

    if (startI >= size) {
      throw std::runtime_error("Invalid slice indexing out of array");
    }
  }

  if (!end.is(py::none())) {
    endI = py::cast<int>(end);
    if (endI < 0) {
      endI = size + endI;
    }

    if (endI >= size) {
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
    count = (endI - startI) / stepI;
    int endS = startI + t * stepI;
    count = endS <= endI ? t : t - 1;
  }

  return std::make_tuple(startI, endI, stepI, count);
}

/*
 * Image 2D indexing
 */
template<typename T, typename NpRep>
void define_set_item_2d_image(py::class_<vpImage<T>, std::shared_ptr<vpImage<T>>> &pyClass, unsigned int componentsPerPixel)
{
  pyClass.def("__setitem__", [](vpImage<T> &self, std::pair<int, int> pair, const T &value) {
    int i = pair.first, j = pair.second;
    const int rows = (int)self.getRows(), cols = (int)self.getCols();
    if (i >= rows || j >= cols || i < -rows || j < -cols) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D image: got indices " << shape_to_string({ i, j })
        << " but image has dimensions " << shape_to_string({ rows, cols });
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = rows + i;
    }
    if (j < 0) {
      j = cols + j;
    }
    self[i][j] = value;
  });
  pyClass.def("__setitem__", [](vpImage<T> &self, int i, const T &value) {
    const int rows = (int)self.getRows();
    if (i >= rows || i < -rows) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D image: got indices (" << i << ", :)"
        << " but image has dimensions " << shape_to_string({ rows, self.getCols() });
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = rows + i;
    }
    T *row = self[i];
    for (int j = 0; j < self.getCols(); ++j) {
      row[j] = value;
    }
  });
  pyClass.def("__setitem__", [](vpImage<T> &self, py::slice slice, const T &value) {
    int rowStart, rowEnd, rowStep;
    std::tie(rowStart, rowEnd, rowStep) = solveSliceIndices(slice, self.getRows());
    for (int i = rowStart; i < rowEnd; i += rowStep) {
      T *row = self[i];
      for (int j = 0; j < self.getCols(); ++j) {
        row[j] = value;
      }
    }
  });
  pyClass.def("__setitem__", [](vpImage<T> &self, std::tuple<py::slice, py::slice> slices, const T &value) {
    py::slice sliceRows, sliceCols;
    int rowStart, rowEnd, rowStep;
    int colStart, colEnd, colStep;
    std::tie(sliceRows, sliceCols) = slices;
    std::tie(rowStart, rowEnd, rowStep) = solveSliceIndices(sliceRows, self.getRows());
    std::tie(colStart, colEnd, colStep) = solveSliceIndices(sliceCols, self.getCols());

    for (int i = rowStart; i < rowEnd; i += rowStep) {
      T *row = self[i];
      for (unsigned int j = colStart; j < colEnd; j += colStep) {
        row[j] = value;
      }
    }
  });


  if (componentsPerPixel == 1) {
    pyClass.def("__setitem__", [](vpImage<T> &self, py::slice sliceRows, py::array<NpRep, py::array::c_style> &values) {
      int rowStart, rowEnd, rowStep;
      std::tie(sliceRows, sliceCols) = slices;
      std::tie(rowStart, rowEnd, rowStep) = solveSliceIndices(sliceRows, self.getRows());

      py::buffer_info values_info = values.request();

      // Copy the array into each row (same values in each row)
      if (values_info.ndim == 1) {
        if (values_info.shape[0] != self.getCols()) {
          throw std::runtime_error("Number of image columns and NumPy array dimension do not match");
        }

        const NpRep *value_ptr = static_cast<NpRep *>(valuesInfo.ptr);

        for (int i = rowStart; i < rowEnd; i += rowStep) {
          T *row = self[i];
          unsigned int k = 0;
          for (unsigned int j = colStart; j < colEnd; j += colStep) {
            row[j] = value_ptr[k++];
          }
        }
      }
      // 2D array to 2D array
      else if (values_info.ndim == 2) {
        unsigned int numAssignedRows = 0;
        if (values_info.shape[0] != numAssignedRows || values_info.shape[1] != self.getCols()) {
          throw std::runtime_error("Indexing into 2D image: NumPy array has wrong size");
        }
        const NpRep *value_ptr = static_cast<NpRep *>(valuesInfo.ptr);

        unsigned int k = 0;
        for (int i = rowStart; i < rowEnd; i += rowStep) {
          T *row = self[i];
          for (unsigned int j = colStart; j < colEnd; j += colStep) {
            row[j] = value_ptr[k++];
          }
        }

      }
      else {
        throw std::runtime_error("Cannot write into 2D raw type image with multidimensional NumPy array that has more than 2 dimensions");
      }
    });

  }

  // Handle vprgba/vprgbf
  if (componentsPerPixel > 1) {
  //   pyClass.def("__setitem__", [](vpImage<T> &self, std::tuple<py::slice, py::slice> slices, const T &value) {
  //     py::slice sliceRows, sliceCols;
  //     int rowStart, rowEnd, rowStep;
  //     int colStart, colEnd, colStep;
  //     std::tie(sliceRows, sliceCols) = slices;
  //     std::tie(rowStart, rowEnd, rowStep) = solveSliceIndices(sliceRows, self.getRows());
  //     std::tie(colStart, colEnd, colStep) = solveSliceIndices(sliceCols, self.getCols());

  //     for (int i = rowStart; i < rowEnd; i += rowStep) {
  //       T *row = self[i];
  //       for (unsigned int j = colStart; j < colEnd; j += colStep) {
  //         row[j] = value;
  //       }
  //     }
  // });


  }
}


  /*
   * vpImage
   */
template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>, std::shared_ptr<vpImage<T>>> &pyImage)
{
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<T, 2>(image.bitmap, { image.getHeight(), image.getWidth() }, false);
  });
  pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<T> {
    return py::cast(self).template cast<np_array_cf<T>>();
  }, numpy_fn_doc_image, py::keep_alive<0, 1>());

  pyImage.def(py::init([](np_array_cf<T> &np_array) {
    verify_array_shape_and_dims(np_array, 2, "ViSP Image");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpImage<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, result.bitmap);
    return result;
                       }), R"doc(
Construct an image by **copying** a 2D numpy array.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));

  define_get_item_2d_image<T, T>(pyImage);
  define_set_item_2d_image<T, T>(pyImage, 1);

  pyImage.def("__repr__", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << "<Image (" << self.getHeight() << ", " << self.getWidth() << ")>";
    return ss.str();
  });

  pyImage.def("_visp_repr", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << self;
    return ss.str();
  }, R"doc(Get the full ViSP image string representation.)doc");

}

template<typename T>
typename std::enable_if<std::is_same<vpRGBa, T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>, std::shared_ptr<vpImage<T>>> &pyImage)
{
  using NpRep = unsigned char;
  static_assert(sizeof(T) == 4 * sizeof(NpRep));
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<NpRep, 3>(reinterpret_cast<NpRep *>(image.bitmap), { image.getHeight(), image.getWidth(), 4 }, false);
  });
  pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<NpRep> {
    return py::cast(self).template cast<np_array_cf<NpRep>>();
  }, numpy_fn_doc_image, py::keep_alive<0, 1>());

  pyImage.def(py::init([](np_array_cf<NpRep> &np_array) {
    verify_array_shape_and_dims(np_array, 3, "ViSP RGBa image");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    if (shape[2] != 4) {
      throw std::runtime_error("Tried to copy a 3D numpy array that does not have 4 elements per pixel into a ViSP RGBA image");
    }
    vpImage<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, (NpRep *)result.bitmap);
    return result;
                       }), R"doc(
Construct an image by **copying** a 3D numpy array. this numpy array should be of the form :math:`H \times W \times 4`
where the 4 denotes the red, green, blue and alpha components of the image.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));
  define_get_item_2d_image<T, NpRep>(pyImage);
  define_set_item_2d_image<T, NpRep>(pyImage, sizeof(T) / sizeof(NpRep));


  pyImage.def("__repr__", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << "<RGBa Image (" << self.getHeight() << ", " << self.getWidth() << ")>";
    return ss.str();
  });

  pyImage.def("_visp_repr", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << self;
    return ss.str();
  }, R"doc(Get the full ViSP image string representation.)doc");

}
template<typename T>
typename std::enable_if<std::is_same<vpRGBf, T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>, std::shared_ptr<vpImage<T>>> &pyImage)
{
  using NpRep = float;
  static_assert(sizeof(T) == 3 * sizeof(NpRep));
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<NpRep, 3>(reinterpret_cast<NpRep *>(image.bitmap), { image.getHeight(), image.getWidth(), 3 }, false);
  });

  pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<NpRep> {
    return py::cast(self).template cast<np_array_cf<NpRep>>();
  }, numpy_fn_doc_image, py::keep_alive<0, 1>());

  pyImage.def(py::init([](np_array_cf<NpRep> &np_array) {
    verify_array_shape_and_dims(np_array, 3, "ViSP RGBa image");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    if (shape[2] != 3) {
      throw std::runtime_error("Tried to copy a 3D numpy array that does not have 3 elements per pixel into a ViSP RGBf image");
    }
    vpImage<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, (NpRep *)result.bitmap);
    return result;
                       }), R"doc(
Construct an image by **copying** a 3D numpy array. this numpy array should be of the form :math:`H \times W \times 3`
where the 3 denotes the red, green and blue components of the image.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));

  define_get_item_2d_image<T, NpRep>(pyImage);
  define_set_item_2d_image<T, NpRep>(pyImage, sizeof(T) / sizeof(NpRep));

  pyImage.def("__repr__", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << "<RGBf Image (" << self.getHeight() << ", " << self.getWidth() << ")>";
    return ss.str();
  });

  pyImage.def("_visp_repr", [](const vpImage<T> &self) -> std::string {
    std::stringstream ss;
    ss << self;
    return ss.str();
  }, R"doc(Get the full ViSP image string representation.)doc");
}

#endif
