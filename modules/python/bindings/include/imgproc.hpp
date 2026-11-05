/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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

#ifndef VISP_PYTHON_IMGPROC_HPP
#define VISP_PYTHON_IMGPROC_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/imgproc/vpImgproc.h>

namespace py = pybind11;

/*
 * Bindings for imgproc free functions that take an optional "const vpImage<bool> *p_mask" argument.
 * Raw pointer arguments are not handled by the generator. Here, the mask is exposed as an optional
 * ImageBool argument: passing None (the default) is equivalent to passing nullptr in C++.
 */
void bindings_imgproc(py::module_ &submodule)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  // equalizeHistogram, gray-level images
  submodule.def("equalizeHistogram", [](vpImage<unsigned char> &I, const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING equalizeHistogram(I, p_mask);
  }, R"doc(
Adjust the contrast of a grayscale image in place by histogram equalization.

:param I: The grayscale image to apply histogram equalization to.
:param p_mask: If not None, a boolean mask image (same size as I) that indicates which pixels are considered.
)doc", py::arg("I"), py::arg("p_mask") = py::none());

  submodule.def("equalizeHistogram", [](const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2,
                                        const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING equalizeHistogram(I1, I2, p_mask);
  }, R"doc(
Adjust the contrast of a grayscale image by histogram equalization.

:param I1: The input grayscale image.
:param I2: The output grayscale image after histogram equalization.
:param p_mask: If not None, a boolean mask image (same size as I1) that indicates which pixels are considered.
)doc", py::arg("I1"), py::arg("I2"), py::arg("p_mask") = py::none());

  // gammaCorrection, gray-level images
  submodule.def("gammaCorrection", [](vpImage<unsigned char> &I, float gamma, vpGammaMethod method,
                                      const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING gammaCorrection(I, gamma, method, p_mask);
  }, R"doc(
Apply gamma correction in place to a grayscale image.

:param I: The grayscale image to apply gamma correction to.
:param gamma: Gamma value. Only used with GAMMA_MANUAL method.
:param method: The gamma correction method to use.
:param p_mask: If not None, a boolean mask image (same size as I) that indicates which pixels are considered.
)doc", py::arg("I"), py::arg("gamma"), py::arg("method") = VISP_NAMESPACE_ADDRESSING GAMMA_MANUAL,
     py::arg("p_mask") = py::none());

  submodule.def("gammaCorrection", [](const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, float gamma,
                                      vpGammaMethod method, const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING gammaCorrection(I1, I2, gamma, method, p_mask);
  }, R"doc(
Apply gamma correction to a grayscale image.

:param I1: The input grayscale image.
:param I2: The output grayscale image.
:param gamma: Gamma value. Only used with GAMMA_MANUAL method.
:param method: The gamma correction method to use.
:param p_mask: If not None, a boolean mask image (same size as I1) that indicates which pixels are considered.
)doc", py::arg("I1"), py::arg("I2"), py::arg("gamma"), py::arg("method") = VISP_NAMESPACE_ADDRESSING GAMMA_MANUAL,
     py::arg("p_mask") = py::none());

  // gammaCorrection, color images
  submodule.def("gammaCorrection", [](vpImage<vpRGBa> &I, float gamma, vpGammaColorHandling colorHandling,
                                      vpGammaMethod method, const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING gammaCorrection(I, gamma, colorHandling, method, p_mask);
  }, R"doc(
Apply gamma correction in place to a color image.

:param I: The color image to apply gamma correction to.
:param gamma: Gamma value. Only used with GAMMA_MANUAL method.
:param colorHandling: How the color channels are handled (GAMMA_RGB or GAMMA_HSV).
:param method: The gamma correction method to use.
:param p_mask: If not None, a boolean mask image (same size as I) that indicates which pixels are considered.
)doc", py::arg("I"), py::arg("gamma"), py::arg("colorHandling") = VISP_NAMESPACE_ADDRESSING GAMMA_RGB,
     py::arg("method") = VISP_NAMESPACE_ADDRESSING GAMMA_MANUAL, py::arg("p_mask") = py::none());

  submodule.def("gammaCorrection", [](const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, float gamma,
                                      vpGammaColorHandling colorHandling, vpGammaMethod method,
                                      const vpImage<bool> *p_mask) {
    VISP_NAMESPACE_ADDRESSING gammaCorrection(I1, I2, gamma, colorHandling, method, p_mask);
  }, R"doc(
Apply gamma correction to a color image.

:param I1: The input color image.
:param I2: The output color image.
:param gamma: Gamma value. Only used with GAMMA_MANUAL method.
:param colorHandling: How the color channels are handled (GAMMA_RGB or GAMMA_HSV).
:param method: The gamma correction method to use.
:param p_mask: If not None, a boolean mask image (same size as I1) that indicates which pixels are considered.
)doc", py::arg("I1"), py::arg("I2"), py::arg("gamma"), py::arg("colorHandling") = VISP_NAMESPACE_ADDRESSING GAMMA_RGB,
     py::arg("method") = VISP_NAMESPACE_ADDRESSING GAMMA_MANUAL, py::arg("p_mask") = py::none());
}

#endif
