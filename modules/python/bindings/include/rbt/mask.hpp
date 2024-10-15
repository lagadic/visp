
#ifndef VISP_PYTHON_RBT_MASK_HPP
#define VISP_PYTHON_RBT_MASK_HPP

#include <visp3/rbt/vpObjectMask.h>
#include <visp3/core/vpImage.h>
#include <pybind11/pybind11.h>


class TrampolineObjectMask : public vpObjectMask
{
public:
  using vpObjectMask::vpObjectMask;

  TrampolineObjectMask() : vpObjectMask() { }

  virtual void updateMask(const vpRBFeatureTrackerInput &frame,
                          const vpRBFeatureTrackerInput &previousFrame,
                          vpImage<float> &mask) VP_OVERRIDE
  {
    pybind11::gil_scoped_acquire gil;  // Acquire the GIL while in this scope.
    // Try to look up the overridden method on the Python side.
    pybind11::function override = pybind11::get_override(this, "updateMask");
    if (override) {  // method is found
      // Pybind seems to copy the frames, so we pass the pointers
      override(&frame, &previousFrame, &mask);
    }
  }

  virtual void display(const vpImage<float> &mask, vpImage<unsigned char> &Imask) const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,             /* Return type */
      vpObjectMask,     /* Parent class */
      display,          /* Name of function in C++ (must match Python name) */
      mask, Imask
    );
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &) VP_OVERRIDE
  {

  }
#endif
};



#endif
