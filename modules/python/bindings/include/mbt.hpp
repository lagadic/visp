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

#ifndef VISP_PYTHON_MBT_HPP
#define VISP_PYTHON_MBT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbGenericTracker.h>

namespace py = pybind11;

void bindings_vpMbGenericTracker(py::class_<vpMbGenericTracker, std::shared_ptr<vpMbGenericTracker>, vpMbTracker> &pyMbGenericTracker)
{
  pyMbGenericTracker.def("track", [](vpMbGenericTracker &self, std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                     std::map<std::string, py::array_t<double, py::array::c_style>> &mapOfPointClouds) {
                                       std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
                                       std::map<std::string, vpMatrix> mapOfVectors;
                                       for (const auto &point_cloud_pair: mapOfPointClouds) {

                                         py::buffer_info buffer = point_cloud_pair.second.request();
                                         if (buffer.ndim != 3 && buffer.shape[2] != 3) {
                                           std::stringstream ss;
                                           ss << "Pointcloud error: pointcloud at key: " << point_cloud_pair.first <<
                                             " should be a 3D numpy array of dimensions H X W x 3";
                                           throw std::runtime_error(ss.str());
                                         }
                                         const auto shape = buffer.shape;
                                         mapOfHeights[point_cloud_pair.first] = shape[0];
                                         mapOfWidths[point_cloud_pair.first] = shape[1];
                                         vpMatrix pc(shape[0] * shape[1], 3);
                                         const double *data = point_cloud_pair.second.unchecked<3>().data(0, 0, 0);
                                         memcpy(pc.data, data, shape[0] * shape[1] * 3 * sizeof(double));
                                         mapOfVectors[point_cloud_pair.first] = std::move(pc);
                                       }
                                       std::map<std::string, const vpMatrix * > mapOfVectorPtrs;
                                       for (const auto &p: mapOfVectors) {
                                         mapOfVectorPtrs[p.first] = &(p.second);
                                       }
                                       self.track(mapOfImages, mapOfVectorPtrs, mapOfWidths, mapOfHeights);
  }, R"doc(
Perform tracking, with point clouds being represented as numpy arrays

:param mapOfImages: Dictionary mapping from a camera name to a grayscale image

:param: mapOfPointclouds: Dictionary mapping from a camera name to a point cloud.
A point cloud is represented as a H x W x 3 double NumPy array.

)doc", py::arg("mapOfImages"), py::arg("mapOfPointclouds"));
}

#endif
