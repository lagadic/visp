#ifndef VISP_PYTHON_CORE_IMAGES_HPP
#define VISP_PYTHON_CORE_IMAGES_HPP


#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBf.h>


/*
  vpImage
*/
template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>> &pyImage)
{
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<T, 2>(image.bitmap, { image.getHeight(), image.getWidth() }, false);
  });
  pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<T> {
    return py::cast(self).template cast<np_array_cf<T>>();
  }, numpy_fn_doc_writable);

  pyImage.def(py::init([](np_array_cf<T> np_array) {
    verify_array_shape_and_dims(np_array, 2, "ViSP Image");
    const std::vector<ssize_t> shape = np_array.request().shape;
    vpImage<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, result.bitmap);
    return result;
  }), R"doc(
Construct an image by **copying** a 2D numpy array.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));
}

template<typename T>
typename std::enable_if<std::is_same<vpRGBa, T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>> &pyImage)
{
  static_assert(sizeof(T) == 4 * sizeof(unsigned char));
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<unsigned char, 3>(reinterpret_cast<unsigned char *>(image.bitmap), { image.getHeight(), image.getWidth(), 4 }, false);
  });
  pyImage.def("numpy", [](vpImage<T> &self) -> np_array_cf<unsigned char> {
    return py::cast(self).template cast<np_array_cf<unsigned char>>();
  }, numpy_fn_doc_writable);

  pyImage.def(py::init([](np_array_cf<unsigned char> np_array) {
    verify_array_shape_and_dims(np_array, 3, "ViSP RGBa image");
    const std::vector<ssize_t> shape = np_array.request().shape;
    if (shape[2] != 4) {
      throw std::runtime_error("Tried to copy a 3D numpy array that does not have 4 elements per pixel into a ViSP RGBA image");
    }
    vpImage<T> result(shape[0], shape[1]);
    copy_data_from_np(np_array, (unsigned char *)result.bitmap);
    return result;
  }), R"doc(
Construct an image by **copying** a 3D numpy array. this numpy array should be of the form :math:`H \times W \times 4`
where the 4 denotes the red, green, blue and alpha components of the image.

:param np_array: The numpy array to copy.

)doc", py::arg("np_array"));

}
template<typename T>
typename std::enable_if<std::is_same<vpRGBf, T>::value, void>::type
bindings_vpImage(py::class_<vpImage<T>> &pyImage)
{
  static_assert(sizeof(T) == 3 * sizeof(float));
  pyImage.def_buffer([](vpImage<T> &image) -> py::buffer_info {
    return make_array_buffer<float, 3>(reinterpret_cast<float *>(image.bitmap), { image.getHeight(), image.getWidth(), 3 }, false);
  });
}



#endif