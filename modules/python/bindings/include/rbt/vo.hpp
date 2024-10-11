
#ifndef VISP_PYTHON_RBT_VO_HPP
#define VISP_PYTHON_RBT_VO_HPP

#include <visp3/rbt/vpRBVisualOdometry.h>
#include <pybind11/pybind11.h>


class TrampolineRBVisualOdometry : public vpRBVisualOdometry
{
public:
  using vpRBVisualOdometry::vpRBVisualOdometry;

  TrampolineRBVisualOdometry() : vpRBVisualOdometry() { }

  virtual void compute(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame) VP_OVERRIDE
  {
    pybind11::gil_scoped_acquire gil;  // Acquire the GIL while in this scope.
    // Try to look up the overridden method on the Python side.
    pybind11::function override = pybind11::get_override(this, "compute");
    if (override) {  // method is found
      // Pybind seems to copy the frames, so we pass the pointers
      override(&frame, &previousFrame);
    }
  }

  virtual vpHomogeneousMatrix getCameraMotion() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      vpHomogeneousMatrix,           /* Return type */
      vpRBVisualOdometry,     /* Parent class */
      getCameraMotion,       /* Name of function in C++ (must match Python name) */

      );
  }
  virtual vpHomogeneousMatrix getCameraPose() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      vpHomogeneousMatrix,           /* Return type */
      vpRBVisualOdometry,     /* Parent class */
      getCameraPose,       /* Name of function in C++ (must match Python name) */

      );
  }
};



#endif
