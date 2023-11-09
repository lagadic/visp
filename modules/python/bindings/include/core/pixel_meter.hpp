#ifndef VISP_PYTHON_CORE_PIXEL_METER_HPP
#define VISP_PYTHON_CORE_PIXEL_METER_HPP


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>

#include "core/utils.hpp"

void bindings_vpPixelMeterConversion(py::class_<vpPixelMeterConversion> &pyPM)
{

  pyPM.def_static("convertPoints", [](const vpCameraParameters &cam, const py::array_t<double> &us, const py::array_t<double> &vs) {
    py::buffer_info bufu = us.request(), bufv = vs.request();
    if (bufu.ndim != bufv.ndim || bufu.shape != bufv.shape) {
      std::stringstream ss;
      ss << "us and vs must have the same number of dimensions and same number of elements, but got us = " << shape_to_string(bufu.shape);
      ss << "and vs = " << shape_to_string(bufv.shape);
      throw std::runtime_error(ss.str());
    }
    py::array_t<double> xs(bufu.shape);
    py::array_t<double> ys(bufv.shape);

    const double *u_ptr = static_cast<const double *>(bufu.ptr);
    const double *v_ptr = static_cast<const double *>(bufv.ptr);
    double *x_ptr = static_cast<double *>(xs.request().ptr);
    double *y_ptr = static_cast<double *>(ys.request().ptr);

    for (size_t i = 0; i < bufu.size; ++i) {
      vpPixelMeterConversion::convertPoint(cam, u_ptr[i], v_ptr[i], x_ptr[i], y_ptr[i]);
    }


    return std::make_tuple(std::move(xs), std::move(ys));

  }, R"doc(
Convert a set of 2D pixel coordinates to normalized coordinates.
:param cam: The camera intrinsics with which to convert pixels to normalized coordinates.
:param us: The pixel coordinates along the horizontal axis.
:param vs: The pixel coordinates along the vertical axis.

:raises RuntimeError: If us and vs do not have the same dimensions and shape.

:return: A tuple containing the x and y normalized coordinates of the input pixels.

)doc", py::arg("cam"), py::arg("us"), py::arg("vs"));

}


void bindings_vpMeterPixelConversion(py::class_<vpMeterPixelConversion> &pyMP)
{

}



#endif
