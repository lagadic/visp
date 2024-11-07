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

#ifndef VISP_PYTHON_BLOB_HPP
#define VISP_PYTHON_BLOB_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <visp3/core/vpConfig.h>
#include <visp3/blob/vpDot2.h>
#include <optional>

namespace py = pybind11;

void bindings_vpDot2(py::class_<vpDot2, std::shared_ptr<vpDot2>, vpTracker> &pyDot2)
{
  pyDot2.def_static("defineDots", [](std::vector<vpDot2> &dots,
                                     const std::string &dotFile,
                                     vpImage<unsigned char> &I,
                                     vpColor col = vpColor::blue,
                                     bool trackDot = true) {
                                       return vpDot2::defineDots(&dots[0], dots.size(), dotFile, I, col, trackDot);
  }, R"doc(
Wrapper for the defineDots method, see the C++ ViSP documentation.
)doc", py::arg("dots"), py::arg("dotFile"), py::arg("I"), py::arg("color"), py::arg("trackDot") = true);

  pyDot2.def_static("trackAndDisplay", [](std::vector<vpDot2> &dots,
                                          vpImage<unsigned char> &I,
                                          std::vector<vpImagePoint> &cogs,
                                          std::optional<std::vector<vpImagePoint>> cogStar) {
                                            vpImagePoint *desireds = cogStar ? &((*cogStar)[0]) : nullptr;
                                            vpDot2::trackAndDisplay(&dots[0], dots.size(), I, cogs, desireds);
  }, R"doc(
Wrapper for the trackAndDisplay method, see the C++ ViSP documentation.
)doc", py::arg("dots"), py::arg("I"), py::arg("cogs"), py::arg("desiredCogs"));
}

#endif
