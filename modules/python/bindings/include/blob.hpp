#ifndef VISP_PYTHON_BLOB_HPP
#define VISP_PYTHON_BLOB_HPP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/blob/vpDot2.h>
#include <optional>

namespace py = pybind11;

void bindings_vpDot2(py::class_<vpDot2, vpTracker> &pyDot2)
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
