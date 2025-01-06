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

#ifndef VISP_PYTHON_CORE_DISPLAY_HPP
#define VISP_PYTHON_CORE_DISPLAY_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>

namespace py = pybind11;

void bindings_vpDisplay(py::class_<vpDisplay, std::shared_ptr<vpDisplay>> &pyDisplay)
{

  pyDisplay.def_static("displayCrosses",
  [](const vpImage<vpRGBa> &I,
     const py::array_t<int, py::array::c_style> &is,
     const py::array_t<int, py::array::c_style> &js,
     unsigned int size, const vpColor &color, unsigned int thickness = 1) {

       py::buffer_info bufi = is.request(), bufj = js.request();
       if (bufi.ndim != bufj.ndim || bufi.shape != bufj.shape) {
         std::stringstream ss;
         ss << "is and js must have the same number of dimensions and same number of elements, but got is = " << shape_to_string(bufi.shape);
         ss << "and js = " << shape_to_string(bufj.shape);
         throw std::runtime_error(ss.str());
       }

       const int *i_ptr = static_cast<const int *>(bufi.ptr);
       const int *j_ptr = static_cast<const int *>(bufj.ptr);

       for (py::ssize_t i = 0; i < bufi.size; ++i) {
         vpDisplay::displayCross(I, i_ptr[i], j_ptr[i], size, color, thickness);
       }

  });

  pyDisplay.def_static("displayLines",
  [](const vpImage<vpRGBa> &I,
     const py::array_t<int, py::array::c_style> &is1,
     const py::array_t<int, py::array::c_style> &js1,
     const py::array_t<int, py::array::c_style> &is2,
     const py::array_t<int, py::array::c_style> &js2,
     const vpColor &color, unsigned int thickness = 1, bool segment = true) {

       py::buffer_info bufi1 = is1.request(), bufj1 = js1.request();
       py::buffer_info bufi2 = is2.request(), bufj2 = js2.request();

       if (bufi1.shape != bufj1.shape || bufi1.shape != bufi2.shape || bufi1.shape != bufj2.shape) {
         std::stringstream ss;
         ss << "In display lines: numpy arrays must have same dimensions!";
         throw std::runtime_error(ss.str());
       }

       const int *i1_ptr = static_cast<const int *>(bufi1.ptr);
       const int *j1_ptr = static_cast<const int *>(bufj1.ptr);

       const int *i2_ptr = static_cast<const int *>(bufi2.ptr);
       const int *j2_ptr = static_cast<const int *>(bufj2.ptr);


       for (py::ssize_t i = 0; i < bufi1.size; ++i) {
         vpDisplay::displayLine(I, i1_ptr[i], j1_ptr[i], i2_ptr[i], j2_ptr[i], color, thickness, segment);
       }

  });

}

#endif
