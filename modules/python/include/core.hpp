#ifndef VISP_PYTHON_CORE_HPP
#define VISP_PYTHON_CORE_HPP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>

namespace py = pybind11;

template<typename T>
void bindings_vpArray2D(py::class_<vpArray2D<T>> &pyArray2D)
{
  pyArray2D.def_buffer([](vpArray2D<T> &array) -> py::buffer_info {
    return py::buffer_info(
        array.data,                             /* Pointer to buffer */
        sizeof(T),                              /* Size of one scalar */
        py::format_descriptor<T>::format(),     /* Python struct-style format descriptor */
        2,                                      /* Number of dimensions */
        py::detail::any_container<ssize_t>({ array.getRows(), array.getCols() }),   /* Buffer dimensions */
        py::detail::any_container<ssize_t>({ sizeof(T) * array.getCols(), sizeof(T) }),  /* Strides (in bytes) for each index */
        false /* Readonly? */
    );
 });
}


void bindings_vpColVector(py::class_<vpColVector, vpArray2D<double>> &pyColVector)
{
  pyColVector.def_buffer([](vpColVector &array) -> py::buffer_info {
    return py::buffer_info(
        array.data,                             /* Pointer to buffer */
        sizeof(double),                              /* Size of one scalar */
        py::format_descriptor<double>::format(),     /* Python struct-style format descriptor */
        1,                                      /* Number of dimensions */
        py::detail::any_container<ssize_t>({ array.getRows() }),   /* Buffer dimensions */
        py::detail::any_container<ssize_t>({ sizeof(double) }),  /* Strides (in bytes) for each index */
        false /* Readonly? */
    );
 });
}


void bindings_vpRotationMatrix(py::class_<vpRotationMatrix, vpArray2D<double>> &pyRotationMatrix)
{
  pyRotationMatrix.def_buffer([](vpRotationMatrix &array) -> py::buffer_info {
    return py::buffer_info(
        array.data,                             /* Pointer to buffer */
        sizeof(double),                              /* Size of one scalar */
        py::format_descriptor<double>::format(),     /* Python struct-style format descriptor */
        2,                                      /* Number of dimensions */
        py::detail::any_container<ssize_t>({ array.getRows(), array.getCols() }),   /* Buffer dimensions */
        py::detail::any_container<ssize_t>({ 3 * sizeof(double), sizeof(double) }),  /* Strides (in bytes) for each index */
        true /* Readonly? */
    );
 });
}


#endif