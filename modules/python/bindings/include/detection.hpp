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


#ifndef VISP_PYTHON_DETECTION_HPP
#define VISP_PYTHON_DETECTION_HPP

#include <tuple>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>


#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>

namespace py = pybind11;

void bindings_vpDetectorAprilTag(py::class_<vpDetectorAprilTag, std::shared_ptr<vpDetectorAprilTag>, vpDetectorBase> &pyAprilTag)
{
  pyAprilTag.def("detect",
   [](vpDetectorAprilTag &self, const vpImage<unsigned char> &I, double tagSize, const vpCameraParameters &cam) -> std::tuple<bool, std::vector<vpHomogeneousMatrix>> {
     std::vector<vpHomogeneousMatrix> cMos;
     bool detected = self.detect(I, tagSize, cam, cMos);
     return std::make_tuple(detected, cMos);
  }, R"doc(
Run Apriltag detection and retrieve whether poses have detected and if so, the list of poses

:param I: The image in which to detect the tags.

:param tagSize: the size of apriltags, in meters

:param cam: The camera intrinsics, required to compute the pose

:return: A tuple containing whether any tag has been detected and the list of tag poses. Combine with getTagsId to associate the IDs to the poses.

)doc", py::arg("I"), py::arg("tag_size"), py::arg("cam"));

  pyAprilTag.def("detectWithAlternativesPoses",
   [](vpDetectorAprilTag &self, const vpImage<unsigned char> &I, double tagSize, const vpCameraParameters &cam) -> std::tuple<bool, std::vector<vpHomogeneousMatrix>, std::vector<vpHomogeneousMatrix>, std::vector<double>, std::vector<double>> {
     std::vector<vpHomogeneousMatrix> cMos, cMos2;
     std::vector<double> errors1, errors2;


     bool detected = self.detect(I, tagSize, cam, cMos, &cMos2, &errors1, &errors2);
     return std::make_tuple(detected, cMos, cMos2, errors1, errors2);
  }, R"doc(
Run Apriltag detection and retrieve whether poses have detected and if so, the list of poses. This version provides two poses, since tags are planar objects.

:param I: The image in which to detect the tags.

:param tagSize: the size of apriltags, in meters

:param cam: The camera intrinsics, required to compute the pose

:return: A tuple containing:
- Whether any tag has been detected
- The first list of detected tag poses
- The second list of detected tag poses
- The projections errors associated to the first list of poses
- The projections errors associated to the second list of poses
)doc", py::arg("I"), py::arg("tag_size"), py::arg("cam"));

}


#endif
