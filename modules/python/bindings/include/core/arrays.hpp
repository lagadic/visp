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

#ifndef VISP_PYTHON_CORE_ARRAYS_HPP
#define VISP_PYTHON_CORE_ARRAYS_HPP

#include "core/utils.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

/*
 * Array2D and its children.
 */

/*
 * Get buffer infos : used in def_buffer and the .numpy() function.
 */
template<typename T> py::buffer_info get_buffer_info(T &) = delete;
template<typename T,
  template <typename> class Array,
  typename std::enable_if<std::is_same<vpArray2D<T>, Array<T>>::value, bool>::type = true>
py::buffer_info get_buffer_info(Array<T> &array)
{
  return make_array_buffer<T, 2>(array.data, { array.getRows(), array.getCols() }, false);
}

template<>
py::buffer_info get_buffer_info(vpMatrix &array)
{
  return make_array_buffer<double, 2>(array.data, { array.getRows(), array.getCols() }, false);
}

template<>
py::buffer_info get_buffer_info(vpColVector &array)
{
  return make_array_buffer<double, 1>(array.data, { array.getRows() }, false);
}
template<>
py::buffer_info get_buffer_info(vpRowVector &array)
{
  return make_array_buffer<double, 1>(array.data, { array.getCols() }, false);
}
template<>
py::buffer_info get_buffer_info(vpTranslationVector &array)
{
  return make_array_buffer<double, 1>(array.data, { 3 }, false);
}
template<>
py::buffer_info get_buffer_info(vpRotationMatrix &array)
{
  return make_array_buffer<double, 2>(array.data, { array.getRows(), array.getCols() }, true);
}
template<>
py::buffer_info get_buffer_info(vpHomogeneousMatrix &array)
{
  return make_array_buffer<double, 2>(array.data, { array.getRows(), array.getCols() }, true);
}

/*
* Print helpers
*/

const char *matlab_str_help = R"doc(
  Returns the Matlab representation of this data array (see matlabPrint in the C++ documentation)
)doc";
const char *csv_str_help = R"doc(
  Returns the CSV representation of this data array (see csvPrint in the C++ documentation)
)doc";
const char *maple_str_help = R"doc(
  Returns the CSV representation of this data array (see maplePrint in the C++ documentation)
)doc";

const char *cpp_str_help = R"doc(
  Returns a C++ code representation of this data array (see cppPrint in the C++ documentation)

  :param name: variable name of the matrix.
  :param byte_per_byte: Whether to print byte per byte defaults to false.
)doc";

template<typename PybindClass, typename T, typename S>
void add_print_helper(PybindClass &pyCls, std::ostream &(T:: *fn)(std::ostream &) const, const S pythonName, const char *help)
{
  pyCls.def(pythonName, [fn](const T &self) -> std::string {
    std::stringstream ss;
    (self.*fn)(ss);
    return ss.str();
  }, help);
}

template<typename PybindClass, typename T>
void add_cpp_print_helper(PybindClass &pyCls, std::ostream &(T:: *fn)(std::ostream &, const std::string &, bool) const)
{
  pyCls.def("strCppCode", [fn](const T &self, const std::string &name = "A", bool byte_per_byte = false) -> std::string {
    std::stringstream ss;
    (self.*fn)(ss, name, byte_per_byte);
    return ss.str();
  }, cpp_str_help, py::arg("name"), py::arg("byte_per_byte") = false);
}




/*
 * Array 2D indexing
 */
template<typename PyClass, typename Class, typename Item>
void define_get_item_2d_array(PyClass &pyClass)
{
  pyClass.def("__getitem__", [](const Class &self, std::pair<int, int> pair) -> Item {
    int i = pair.first, j = pair.second;
    const int rows = (int)self.getRows(), cols = (int)self.getCols();
    if (i >= rows || j >= cols || i < -rows || j < -cols) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D array: got indices " << shape_to_string({ i, j })
        << " but array has dimensions " << shape_to_string({ rows, cols });
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
  pyClass.def("__getitem__", [](const Class &self, int i) -> np_array_cf<Item> {
    const int rows = (int)self.getRows();
    if (i >= rows || i < -rows) {
      std::stringstream ss;
      ss << "Invalid indexing into a 2D array: got row index " << shape_to_string({ i })
        << " but array has " << rows << " rows";
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = rows + i;
    }
    return (py::cast(self).template cast<np_array_cf<Item> >())[py::cast(i)].template cast<np_array_cf<Item>>();
  }, py::keep_alive<0, 1>());
  pyClass.def("__getitem__", [](const Class &self, py::slice slice) -> py::array_t<Item> {
    return (py::cast(self).template cast<np_array_cf<Item> >())[slice].template cast<np_array_cf<Item>>();
  }, py::keep_alive<0, 1>());
  pyClass.def("__getitem__", [](const Class &self, py::tuple tuple) {
    return (py::cast(self).template cast<np_array_cf<Item> >())[tuple].template cast<py::array_t<Item>>();
  }, py::keep_alive<0, 1>());
}

/*
 * Array 2D indexing
 */
template<typename PyClass, typename Class, typename Item>
void define_get_item_1d_array(PyClass &pyClass)
{
  pyClass.def("__getitem__", [](const Class &self, int i) -> Item {

    const int elems = (int)self.getRows() * (int)self.getCols();
    if (i >= elems || i < -elems) {
      std::stringstream ss;
      ss << "Invalid indexing into a 1D array: got indices " << shape_to_string({ i })
        << " but array has dimensions " << shape_to_string({ elems });
      throw std::runtime_error(ss.str());
    }
    if (i < 0) {
      i = elems + i;
    }
    return self[i];
  });
  pyClass.def("__getitem__", [](const Class &self, py::slice slice) -> py::array_t<Item> {
    return (py::cast(self).template cast<np_array_cf<Item> >())[slice].template cast<py::array_t<Item>>();
  }, py::keep_alive<0, 1>());
}

const char *numpy_fn_doc_writable = R"doc(
  Numpy view of the underlying array data.
  This numpy view can be used to directly modify the array.
)doc";

const char *numpy_fn_doc_nonwritable = R"doc(
  Numpy view of the underlying array data.
  This numpy view cannot be modified.
  If you try to modify the array, an exception will be raised.
)doc";

template<typename T>
void bindings_vpArray2D(py::class_<vpArray2D<T>, std::shared_ptr<vpArray2D<T>>> &pyArray2D)
{
  pyArray2D.def_buffer(&get_buffer_info<T, vpArray2D>);

  pyArray2D.def("numpy", [](vpArray2D<T> &self) -> np_array_cf<T> {
    return py::cast(self).template cast<np_array_cf<T> >();
  }, numpy_fn_doc_writable, py::keep_alive<0, 1>());

  pyArray2D.def(py::init([](np_array_cf<T> &np_array) {
    verify_array_shape_and_dims(np_array, 2, "ViSP 2D array");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpArray2D<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, result.data);
    return result;
                         }), R"doc(
Construct a 2D ViSP array by **copying** a 2D numpy array.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));

  define_get_item_2d_array<py::class_<vpArray2D<T>, std::shared_ptr<vpArray2D<T>>>, vpArray2D<T>, T>(pyArray2D);
}

void bindings_vpMatrix(py::class_<vpMatrix, std::shared_ptr<vpMatrix>, vpArray2D<double>> &pyMatrix)
{
  pyMatrix.def_buffer(&get_buffer_info<vpMatrix>);

  pyMatrix.def("numpy", [](vpMatrix &self) -> np_array_cf<double> {
    return py::cast(self).cast<np_array_cf<double>>();
  }, numpy_fn_doc_writable, py::keep_alive<0, 1>());

  pyMatrix.def(py::init([](np_array_cf<double> np_array) {
    verify_array_shape_and_dims(np_array, 2, "ViSP Matrix");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpMatrix result(shape[0], shape[1]);
    copy_data_from_np(np_array, result.data);
    return result;
                        }), R"doc(
Construct a matrix by **copying** a 2D numpy array.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));

  add_print_helper(pyMatrix, &vpMatrix::csvPrint, "strCsv", csv_str_help);
  add_print_helper(pyMatrix, &vpMatrix::maplePrint, "strMaple", maple_str_help);
  add_print_helper(pyMatrix, &vpMatrix::matlabPrint, "strMatlab", matlab_str_help);
  add_cpp_print_helper(pyMatrix, &vpMatrix::cppPrint);

  define_get_item_2d_array<py::class_<vpMatrix, std::shared_ptr<vpMatrix>, vpArray2D<double>>, vpMatrix, double>(pyMatrix);
}


void bindings_vpRotationMatrix(py::class_<vpRotationMatrix, std::shared_ptr<vpRotationMatrix>, vpArray2D<double>> &pyRotationMatrix)
{

  pyRotationMatrix.def_buffer(&get_buffer_info<vpRotationMatrix>);
  pyRotationMatrix.def("numpy", [](vpRotationMatrix &self) -> np_array_cf<double> {
    return py::cast(self).cast<np_array_cf<double>>();
  }, numpy_fn_doc_nonwritable, py::keep_alive<0, 1>());
  pyRotationMatrix.def(py::init([](np_array_cf<double> np_array) {
    verify_array_shape_and_dims(np_array, { 3, 3 }, "ViSP rotation matrix");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpRotationMatrix result;
    copy_data_from_np(np_array, result.data);
    if (!result.isARotationMatrix()) {
      throw std::runtime_error("Input numpy array is not a valid rotation matrix");
    }
    return result;
                                }), R"doc(
Construct a rotation matrix by **copying** a 2D numpy array.
This numpy array should be of dimensions :math:`3 \times 3` and be a valid rotation matrix.
If it is not a rotation matrix, an exception will be raised.

:param np_array: The numpy 1D array to copy.

)doc", py::arg("np_array"));
  define_get_item_2d_array<py::class_<vpRotationMatrix, std::shared_ptr<vpRotationMatrix>, vpArray2D<double>>, vpRotationMatrix, double>(pyRotationMatrix);
}

void bindings_vpHomogeneousMatrix(py::class_<vpHomogeneousMatrix, std::shared_ptr<vpHomogeneousMatrix>, vpArray2D<double>> &pyHomogeneousMatrix)
{
  pyHomogeneousMatrix.def_buffer(get_buffer_info<vpHomogeneousMatrix>);
  pyHomogeneousMatrix.def("numpy", [](vpHomogeneousMatrix &self) -> np_array_cf<double> {
    return py::cast(self).cast<np_array_cf<double>>();
  }, numpy_fn_doc_nonwritable, py::keep_alive<0, 1>());

  pyHomogeneousMatrix.def(py::init([](np_array_cf<double> np_array) {
    verify_array_shape_and_dims(np_array, { 4, 4 }, "ViSP homogeneous matrix");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpHomogeneousMatrix result;
    copy_data_from_np(np_array, result.data);
    if (!result.isAnHomogeneousMatrix()) {
      throw std::runtime_error("Input numpy array is not a valid homogeneous matrix");
    }
    return result;
                                   }), R"doc(
Construct a homogeneous matrix by **copying** a 2D numpy array.
This numpy array should be of dimensions :math:`4 \times 4` and be a valid homogeneous matrix.
If it is not a homogeneous matrix, an exception will be raised.

:param np_array: The numpy 1D array to copy.

)doc", py::arg("np_array"));
  define_get_item_2d_array<py::class_<vpHomogeneousMatrix, std::shared_ptr<vpHomogeneousMatrix>, vpArray2D<double>>, vpHomogeneousMatrix, double>(pyHomogeneousMatrix);
}



void bindings_vpTranslationVector(py::class_<vpTranslationVector, std::shared_ptr<vpTranslationVector>, vpArray2D<double>> &pyTranslationVector)
{
  pyTranslationVector.def_buffer(&get_buffer_info<vpTranslationVector>);

  pyTranslationVector.def("numpy", [](vpTranslationVector &self) -> np_array_cf<double> {
    return py::cast(self).cast<np_array_cf<double>>();
  }, numpy_fn_doc_writable, py::keep_alive<0, 1>());

  pyTranslationVector.def(py::init([](np_array_cf<double> np_array) {
    const std::vector<py::ssize_t> required_shape = { 3 };
    verify_array_shape_and_dims(np_array, required_shape, "ViSP translation vector");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpTranslationVector result;
    copy_data_from_np(np_array, result.data);
    return result;
                                   }), R"doc(
Construct a Translation vector by **copying** a 1D numpy array of size 3.

:param np_array: The numpy 1D array to copy.

)doc", py::arg("np_array"));
  define_get_item_1d_array<py::class_<vpTranslationVector, std::shared_ptr<vpTranslationVector>, vpArray2D<double>>, vpTranslationVector, double>(pyTranslationVector);
}


void bindings_vpColVector(py::class_<vpColVector, std::shared_ptr<vpColVector>, vpArray2D<double>> &pyColVector)
{
  pyColVector.def_buffer(&get_buffer_info<vpColVector>);

  pyColVector.def("numpy", [](vpColVector &self) -> np_array_cf<double> {
    return py::cast(self).cast<np_array_cf<double>>();
  }, numpy_fn_doc_writable, py::keep_alive<0, 1>());

  pyColVector.def(py::init([](np_array_cf<double> np_array) {
    verify_array_shape_and_dims(np_array, 1, "ViSP column vector");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpColVector result(shape[0]);
    copy_data_from_np(np_array, result.data);
    return result;
                           }), R"doc(
Construct a column vector by **copying** a 1D numpy array.

:param np_array: The numpy 1D array to copy.

)doc", py::arg("np_array"));
  define_get_item_1d_array<py::class_<vpColVector, std::shared_ptr<vpColVector>, vpArray2D<double>>, vpColVector, double>(pyColVector);

  add_print_helper(pyColVector, &vpColVector::csvPrint, "strCsv", csv_str_help);
  add_print_helper(pyColVector, &vpColVector::maplePrint, "strMaple", maple_str_help);
  add_print_helper(pyColVector, &vpColVector::matlabPrint, "strMatlab", matlab_str_help);
  add_cpp_print_helper(pyColVector, &vpColVector::cppPrint);

}

void bindings_vpRowVector(py::class_<vpRowVector, std::shared_ptr<vpRowVector>, vpArray2D<double>> &pyRowVector)
{
  pyRowVector.def_buffer(&get_buffer_info<vpRowVector>);
  pyRowVector.def("numpy", [](vpRowVector &self) -> np_array_cf<double> {
    return np_array_cf<double>(get_buffer_info<vpRowVector>(self), py::cast(self));
  }, numpy_fn_doc_writable, py::keep_alive<0, 1>());
  pyRowVector.def(py::init([](np_array_cf<double> np_array) {
    verify_array_shape_and_dims(np_array, 1, "ViSP row vector");
    const std::vector<py::ssize_t> shape = np_array.request().shape;
    vpRowVector result(shape[0]);
    copy_data_from_np(np_array, result.data);
    return result;
                           }), R"doc(
Construct a row vector by **copying** a 1D numpy array.

:param np_array: The numpy 1D array to copy.

)doc", py::arg("np_array"));
  define_get_item_1d_array<py::class_<vpRowVector, std::shared_ptr<vpRowVector>, vpArray2D<double>>, vpRowVector, double>(pyRowVector);
  add_print_helper(pyRowVector, &vpRowVector::csvPrint, "strCsv", csv_str_help);
  add_print_helper(pyRowVector, &vpRowVector::maplePrint, "strMaple", maple_str_help);
  add_print_helper(pyRowVector, &vpRowVector::matlabPrint, "strMatlab", matlab_str_help);
  add_cpp_print_helper(pyRowVector, &vpRowVector::cppPrint);
}


#endif
