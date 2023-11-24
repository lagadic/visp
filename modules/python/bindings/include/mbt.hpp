#ifndef VISP_PYTHON_MBT_HPP
#define VISP_PYTHON_MBT_HPP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <visp3/mbt/vpMbGenericTracker.h>

namespace py = pybind11;


void bindings_vpMbGenericTracker(py::class_<vpMbGenericTracker, vpMbTracker> &pyMbGenericTracker)
{
  pyMbGenericTracker.def("track", [](vpMbGenericTracker &self, std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                     std::map<std::string, py::array_t<double, py::array::c_style>> &mapOfPointClouds) {
    std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
    std::map<std::string, vpMatrix> mapOfVectors;
    for (const auto &point_cloud_pair: mapOfPointClouds) {

      py::buffer_info buffer = point_cloud_pair.second.request();
      if (buffer.ndim != 3 and buffer.shape[2] != 3) {
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


  });
}

#endif
