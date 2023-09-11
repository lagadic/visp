#ifndef VISP_PYTHON_CORE_HPP
#define VISP_PYTHON_CORE_HPP
#include <sstream>
#include <cstring>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>



namespace py = pybind11;

template<typename Item>
using np_array_cf = py::array_t<Item, py::array::c_style | py::array::forcecast>;

/*
Create a buffer info for a row major array
*/
template<typename T, unsigned N>
py::buffer_info make_array_buffer(T *data, std::array<unsigned, N> dims, bool readonly)
{
  std::array<ssize_t, N> strides;
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

std::string shape_to_string(const std::vector<ssize_t> &shape)
{
  std::stringstream ss;
  ss << "(";
  for (int i = 0; i < shape.size() - 1; ++i) {
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
  std::vector<ssize_t> shape = buffer.shape;
  if (shape.size() != dims) {
    std::stringstream ss;
    ss << "Tried to instanciate " << class_name
      << " that expects a " << dims << "D array but got a numpy array of shape "
      << shape_to_string(shape);

    throw std::runtime_error(ss.str());
  }
}
template<typename Item>
void verify_array_shape_and_dims(np_array_cf<Item> np_array, std::vector<ssize_t> expected_dims, const char *class_name)
{
  verify_array_shape_and_dims(np_array, expected_dims.size());
  py::buffer_info buffer = np_array.request();
  std::vector<ssize_t> shape = buffer.shape;
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
  std::vector<ssize_t> shape = buffer.shape;
  unsigned int elements = 1;
  for (ssize_t dim : shape) {
    elements *= dim;
  }
  const Item *data = (Item *)buffer.ptr;
  std::memcpy(dest, data, elements * sizeof(Item));

}

template<typename T>
void bindings_vpArray2D(py::class_<vpArray2D<T>> &pyArray2D)
{
  pyArray2D.def_buffer([](vpArray2D<T> &array) -> py::buffer_info {
    return make_array_buffer<T, 2>(array.data, { array.getRows(), array.getCols() }, false);
  });
  pyArray2D.def(py::init([](np_array_cf<T> np_array) {
    verify_array_shape_and_dims(np_array, 2, "ViSP 2D array");
    const std::vector<ssize_t> shape = np_array.request().shape;
    vpArray2D<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, result.data);
    return result;
                         }));
}


void bindings_vpColVector(py::class_<vpColVector, vpArray2D<double>> &pyColVector)
{
  pyColVector.def_buffer([](vpColVector &array) -> py::buffer_info {
    return make_array_buffer<double, 1>(array.data, { array.getRows() }, false);
  });
}

void bindings_vpRowVector(py::class_<vpRowVector, vpArray2D<double>> &pyRowVector)
{
  pyRowVector.def_buffer([](vpRowVector &array) -> py::buffer_info {
    return make_array_buffer<double, 1>(array.data, { array.getCols() }, false);
  });
}


void bindings_vpRotationMatrix(py::class_<vpRotationMatrix, vpArray2D<double>> &pyRotationMatrix)
{
  pyRotationMatrix.def_buffer([](vpRotationMatrix &array) -> py::buffer_info {
    return make_array_buffer<double, 2>(array.data, { array.getRows(), array.getCols() }, true);
  });
}

void bindings_vpHomogeneousMatrix(py::class_<vpHomogeneousMatrix, vpArray2D<double>> &pyHomogeneousMatrix)
{
  pyHomogeneousMatrix.def_buffer([](vpHomogeneousMatrix &array) -> py::buffer_info {
    return make_array_buffer<double, 2>(array.data, { array.getRows(), array.getCols() }, true);
  });
}

// template<typename T>
// void bindings_vpImage(py::class_<vpImage<T>> &pyImage) = delete;

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>> &pyImage)
{
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<T, 2>(image.bitmap, { image.getHeight(), image.getWidth() }, false);
  });
}
template<>
void bindings_vpImage(py::class_<vpImage<vpRGBa>> &pyImage)
{ }
template<>
void bindings_vpImage<vpRGBf>(py::class_<vpImage<vpRGBf>> &pyImage)
{ }




#endif